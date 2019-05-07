// test_perspective.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "lens.h"

using namespace cv;
using namespace std;

#define M_PI 3.141592
#define TO_RADIAN(x) ((x)/180.0 * 3.141592)

Point2f center;
Point2f image_start;
double fl;
vector<Point2f> object_points;

void getPoints(const char* filename, vector<Point2f>& points)
{
	FileStorage fs_P(filename, FileStorage::READ);
	CV_Assert(fs_P.isOpened());
	fs_P["points"] >> points;
	fs_P.release();
}

//센서상의 점들에서 왜곡을 제거한 p'를 구한다. 
vector<Point2f>& cal_points_(vector<Point2f>& src, vector<Point2f>&dst)
{
	float k;
	for (auto p : src) {
		Point2f p_0, p_;
		Point2f p0 = p - center;

		double length = norm(p0);
		double k = length / fl;
		float theta_;	//입사각
		findAngle(k, theta_);
		float length_ = fl * tan(theta_);
		p_0 = length_ / length * p0;
		p_ = p_0;
		dst.push_back(p_);
	}
	return dst;
}

Point2f& cal_points(Point2f& p_, Point2f& p)
{
	Point2f p0;
	Point2f p_0 = p_;
	float k;

	float length_ = norm(p_0);
	float theta_ = atan(length_ / fl);

	findk(theta_, k);
	float length = k * fl;
	p0 = length / length_ * p_0;
	p = p0 + center;

	return p;
}

//왜곡없는 점들에서 왜곡이 포함된 점들을 구한다.
vector<Point2f>& cal_points(vector<Point2f>& src, vector<Point2f>&dst)
{
	float k;
	for (auto p_ : src) {
		Point2f p0, p;
		Point2f p_0 = p_;

		float length_ = norm(p_0);
		float theta_ = atan(length_ / fl);

		findk(theta_, k);
		float length = k * fl;
		p0 = length / length_ * p_0;
		p = p0 + center;
		dst.push_back(p);

		return dst;
	}
}

void cal_points(Mat& src, Mat& dst)
{
	float k;

	for (int j = 0; j < dst.rows; j++) {
		for (int i = 0; i < dst.cols; i++) {
			Point2f temp;
			cal_points(Point2f(src.at<Vec2f>(j, i)), temp);
			dst.at<Vec2f>(j, i)[0] = temp.x;
			dst.at<Vec2f>(j, i)[1] = temp.y;
		}
	}
}
void calHomography(vector<Point2f>& P_points, vector<Point2f>& points, Mat& h)
{
	//cout << "points" << endl << points << endl;

	h = findHomography(P_points, points, NULL);

	vector<Point2f> dst;
	perspectiveTransform(P_points, dst, h);
	//cout << "dst" << endl << dst << endl;
	float sum = 0;
	for (int i = 0; i < points.size(); i++) {
		sum += norm(points[i] - dst[i]);
	}
	cout << "fl= " << fl << ", sum= " << sum << endl;
	//}
	//cout << dst << endl;
}

void makeImage(Mat& src_image, Mat& dst, Mat& h)
{
	int width = dst.cols;
	int height = dst.rows;

	Mat mat_dst_points(height, width, CV_32FC2);
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			mat_dst_points.at<Vec2f>(j, i)[0] = i;
			mat_dst_points.at<Vec2f>(j, i)[1] = j;
		}
	}
	Mat mat_points_(height, width, CV_32FC2);
	perspectiveTransform(mat_dst_points, mat_points_, h);
	Mat mat_p(height, width, CV_32FC2);
	cal_points(mat_points_, mat_p);
	int col, row;
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			dst.at<Vec3b>(j, i) = src_image.at<Vec3b>(Point2i(mat_p.at<Vec2f>(j,i)));

		}
	}
}

void draw_houghLines(Mat src, Mat&dst, vector<Vec2f>& lines, int nline)
{
	cvtColor(src, dst, CV_GRAY2BGR);
	for (size_t i = 0; i < min((int)lines.size(), nline); i++) {
		float rho = lines[i][0], theta = lines[i][1];
		double a = cos(theta), b = sin(theta);
		Point2d pt(a*rho, b* rho);
		Point2d delta(1000 * -b, 1000 * a);
		line(dst, pt + delta, pt - delta, Scalar(0, 255, 0), 1, LINE_AA);
	}

}

void homogenOp(Mat img, Mat& dst, int mask_size)
{
	dst = Mat(img.size(), CV_8U, Scalar(0));
	Point h_m(mask_size / 2, mask_size / 2);

	for (int i = h_m.y; i < img.rows - h_m.y; i++) {
		for (int j = h_m.x; j < img.cols - h_m.x; j++)
		{
			uchar max = 0;
			for (int u = 0; u < mask_size; u++) {
				for (int v = 0; v < mask_size; v++)
				{
					int y = i + u - h_m.y;
					int x = j + v - h_m.x;
					uchar difference = abs(img.at<uchar>(i, j) - img.at<uchar>(y, x));
					if (difference > max) max = difference;
				}
			}
			dst.at<uchar>(i, j) = max;
		}
	}
}

void homogenOp_edge_horizon(Mat img, Mat& dst, int mask_size)
{
	dst = Mat(img.size(), CV_8U, Scalar(0));
	Point h_m(mask_size / 2, mask_size / 2);

	for (int i = h_m.y; i < img.rows - h_m.y; i++) {
		for (int j = 0; j < img.cols; j++)
		{
			uchar max = 0;
			uchar difference = abs(img.at<uchar>(i, j) - img.at<uchar>(i-1, j));
			uchar difference2 = abs(img.at<uchar>(i, j) - img.at<uchar>(i + 1, j));

			dst.at<uchar>(i, j) = MAX(difference, difference2);
		}
	}
}

void homogenOp_edge_vertical(Mat img, Mat& dst, int mask_size)
{
	dst = Mat(img.size(), CV_8U, Scalar(0));
	Point h_m(mask_size / 2, mask_size / 2);

	for (int i = 0; i < img.rows; i++) {
		for (int j = h_m.x; j < img.cols - h_m.x; j++)
		{
			uchar max = 0;
			uchar difference = abs(img.at<uchar>(i, j) - img.at<uchar>(i, j-1));
			uchar difference2 = abs(img.at<uchar>(i, j) - img.at<uchar>(i, j+1));

			dst.at<uchar>(i, j) = MAX(difference, difference2);
		}
	}
}
int main()
{
	center.x = 2145;
	center.y = 1477;
	fl = 950;

	cv::Mat image; // create an empty image
	image = cv::imread("capture4.jpg");
	if (image.empty()) { // error handling
						 // no image has been created…
						 // possibly display an error message
						 // and quit the application
		cv::waitKey(0);
	}
	Mat homograpy;

	vector<Point2f> points;
	vector<Point2f> points_;
	vector<Point2f> dst_points;
	getPoints("points.xml", points);

	Size dst_size(450, 250);
	Mat dewarped(250, 450, CV_8UC3);
	dst_points.push_back(Point2f{ 0,0 });
	dst_points.push_back(Point2f{ (float)dst_size.width - 1,0 });
	dst_points.push_back(Point2f{ (float)dst_size.width -1, (float)dst_size.height -1 });
	dst_points.push_back(Point2f{ 0, (float)dst_size.height - 1 });
	calHomography(dst_points, cal_points_(points, points_), homograpy);
	cout << "homography" << endl << homograpy << endl;

	makeImage(image, dewarped, homograpy);
	cv::imshow("dewarped", dewarped);
	imwrite("result.jpg", dewarped);

	Mat hsv_img, hsv[3];
	cvtColor(dewarped, hsv_img, CV_BGR2HSV);
	split(hsv_img, hsv);
	cv::imshow("hsv0", hsv[0]);
	cv::imshow("hsv1", hsv[1]);
	cv::imshow("hsv2", hsv[2]);


	Mat dewarped2;
	cv::resize(dewarped, dewarped2, cv::Size(dewarped.cols / 2, dewarped.rows / 2), 0, 0, CV_INTER_NN);

	Mat edge;
#if 0
	//canny
	Canny(dewarped2, edge, 130, 300);
#else
	cv::imshow("dewarped2", dewarped2);
	Mat gray_dewarped2;
	cvtColor(dewarped2, gray_dewarped2, CV_BGR2GRAY);
	homogenOp_edge_vertical(gray_dewarped2, edge, 3);
#endif
	cv::imshow("edge", edge);

	//hough
	Mat hough;
	double delta_rho = 1, delta_theta = CV_PI / 180;
	std::vector<Vec2f> lines;
	HoughLines(edge, lines, delta_rho, delta_theta, 10);
	draw_houghLines(edge, hough, lines, 5);
	cv::imshow("hough", hough);


	cv::waitKey(0);
    return 0;
}

