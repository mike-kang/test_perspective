// test_perspective.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "lens.h"
#include <algorithm>
#include "HoughLines.h"

using namespace cv;
using namespace std;

#define M_PI 3.141592
#define TO_RADIAN(x) ((x)/180.0 * 3.141592)

//extern void houghLines(Mat src, vector<Vec3f>& s_lines, double rho, double theta, int thresh);

Point2f center;
Point2f image_start;
double fl;
vector<Point2f> object_points;

void getPoints(const char* filename, vector<Point2f>& points, Size& board_size, Size& plate_size)
{
	FileStorage fs_P(filename, FileStorage::READ);
	CV_Assert(fs_P.isOpened());
	fs_P["points"] >> points;
	fs_P["board_size"] >> board_size;
	fs_P["plate_size"] >> plate_size;
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
		//double a = cos(theta), b = sin(theta);
		//Point2d pt(a*rho, b* rho);
		//Point2d delta(1000 * -b, 1000 * a);
		//line(dst, pt + delta, pt - delta, Scalar(0, 255, 0), 1, LINE_AA);

		if (theta < 3.14 / 4. || theta > 3.*3.14 / 4.) { // 수직 행
			//cv::Point pt1(rho / cos(theta), 0); // 첫 행에서 해당 선의 교차점   
			//cv::Point pt2((rho - result.rows*sin(theta)) / cos(theta), result.rows);
			//// 마지막 행에서 해당 선의 교차점
			//cv::line(image, pt1, pt2, cv::Scalar(255), 1); // 하얀 선으로 그리기

		}
		else { // 수평 행
			cv::Point pt1(0, rho / sin(theta)); // 첫 번째 열에서 해당 선의 교차점  
			cv::Point pt2(dst.cols, (rho - dst.cols*cos(theta)) / sin(theta));
			// 마지막 열에서 해당 선의 교차점
			cv::line(dst, pt1, pt2, Scalar(0, 255, 0), 1); 
		}
	}

}


void draw_houghLines(Mat src, Mat&dst, vector<CHoughLines::Line>& lines, int nline)
{
	cvtColor(src, dst, CV_GRAY2BGR);
	for (size_t i = 0; i < min((int)lines.size(), nline); i++) {
		auto points = lines[i].points;
		for (auto point : *points) {
			dst.at<Vec3b>(point & 0x0000ffff, point >> 16) = Vec3b(0, 0, 255);
		}
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
void calc_histo(Mat image, Mat &hist, int bins, int range_max = 256)
{
	hist = Mat(bins, 1, CV_32F, Scalar(0));
	float gap = range_max / (float)bins;

	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++)
		{
			int idx = int(image.at<uchar>(i, j) / gap);
			hist.at<float>(idx)++;
		}
	}
}


void draw_histo(Mat hist, Mat &hist_img, Size size = Size(256, 200))
{
	hist_img = Mat(size, CV_8U, Scalar(255));
	float  bin = (float)hist_img.cols / hist.rows;
	normalize(hist, hist, 0, size.height, NORM_MINMAX);

	for (int i = 0; i<hist.rows; i++)
	{
		float  start_x = (i * bin);
		float  end_x = (i + 1) * bin;
		Point2f pt1(start_x, 0);
		Point2f pt2(end_x, hist.at <float>(i));

		if (pt2.y > 0)
			rectangle(hist_img, pt1, pt2, Scalar(0), -1);
	}
	flip(hist_img, hist_img, 0);
}

int main()
{
	//center.x = 2145;
	//center.y = 1477;
	center.x = 1937;
	center.y = 1488;
	int factor = 2;
	fl = 950;

	cv::Mat image; // create an empty image
	image = cv::imread("capture_211.jpg");
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
	Size board_size, plate_size;
	getPoints("points_211_2.xml", points, board_size, plate_size);

	//Size dst_size(450, 250);
	board_size /= factor;
	plate_size /= factor;
	int offset_x = 100;
	int offset_y = 100;
	Mat dewarped(board_size.height+ offset_y, board_size.width + offset_x, CV_8UC3);
	dst_points.push_back(Point2f{ (float)offset_x,(float)offset_y });
	dst_points.push_back(Point2f{ (float)board_size.width - 1 + offset_x, (float)offset_y });
	dst_points.push_back(Point2f{ (float)board_size.width -1 + offset_x, (float)board_size.height -1+ offset_y });
	dst_points.push_back(Point2f{ (float)offset_x, (float)board_size.height - 1 + offset_y });
	calHomography(dst_points, cal_points_(points, points_), homograpy);
	cout << "homography" << endl << homograpy << endl;

	makeImage(image, dewarped, homograpy);
	cv::imshow("dewarped", dewarped);
	imwrite("result.jpg", dewarped);

	//Mat hsv_img, hsv[3];
	//cvtColor(dewarped, hsv_img, CV_BGR2HSV);
	//split(hsv_img, hsv);
	//cv::imshow("hsv0", hsv[0]);
	//cv::imshow("hsv1", hsv[1]);
	//cv::imshow("hsv2", hsv[2]);


	Mat dewarped2;
	cv::resize(dewarped, dewarped2, cv::Size(dewarped.cols / 2, dewarped.rows / 2), 0, 0, CV_INTER_NN);
	cv::imshow("dewarped2", dewarped2);

	Mat gray_dewarped2;
	cvtColor(dewarped2, gray_dewarped2, CV_BGR2GRAY);
	cv::imshow("gray_dewarped2", gray_dewarped2);

#if 1
	//canny
	Mat canny;
	//Mat roi = edge_h(Range(std::max(band.start - 15, 0), std::min(band.end + 15, gray_dewarped2.rows)), Range(0, gray_dewarped2.cols));
	Canny(gray_dewarped2, canny, 200, 150);

#else
	Mat edge_v, edge_h, edge;
	Mat blur;
	GaussianBlur(gray_dewarped2, blur, Size(5,5), 0.3);
	cv::imshow("blur", blur);
	Sobel(blur, edge_v, CV_32F, 1, 0, 3);
	Sobel(blur, edge_h, CV_32F, 0, 1, 3);
	convertScaleAbs(edge_v, edge_v);
	threshold(edge_v, edge_v, 100, 255, THRESH_BINARY);
	convertScaleAbs(edge_h, edge_h);
	threshold(edge_h, edge_h, 100, 255, THRESH_BINARY);
	//homogenOp(blur, edge, 3);
	//edge = edge_v;
#endif
	cv::imshow("canny", canny);
	//cv::imshow("edge_h", edge_h);
	//imwrite("edge.bmp", edge);

	//Mat hist, hist_image;
	//calc_histo(edge, hist, 256);
	//draw_histo(hist, hist_image);
	//cv::imshow("hist_image", hist_image);
/*
	Mat graph_h(edge.rows, edge.cols, CV_8UC1);
	graph_h = 0;
	for (int c = 0; c < edge.cols; c++) {
		int sum = 0;
		for (int r = 0; r < edge.rows; r++) {
			if (edge.at<uchar>(r, c) > 50)
				sum++;
		}
		graph_h.at<uchar>(graph_h.rows - sum - 1, c) = 255;
	}
	cv::imshow("graph_h", graph_h);
	*/
	int max = 0;
	Mat graph_v(canny.rows, canny.cols+ 1, CV_8UC3);
	graph_v = 0;
	vector<Range> vec_bands;
	bool bInBand = false;
	int start_r;
	for (int r = 0; r < canny.rows; r++) {
		int count = 0;
		for (int c = 0; c < canny.cols; c++) {
			if (canny.at<uchar>(r, c) > 50)
				count++;
		}
		if (count > max)
			max = count;
		if (count > 25) {
			cv::line(graph_v, Point(0, r), Point(count, r), Scalar(255, 255, 255), 1);
			if (!bInBand) {
				start_r = r;
				bInBand = true;
			}
		}
		else {
			if (bInBand) {
				bInBand = false;
				if(r - start_r > 13)
					vec_bands.push_back(Range(start_r, r - 1));
			}
		}
		//graph_v.at<uchar>(r, sum) = 255;
	}
	cout << "vertical projection's max count is " << max << endl;
	cv::imshow("graph_v", graph_v);

	Mat bands(canny.rows, canny.cols + 1, CV_8UC3);
	bands = 0;
	for (auto band : vec_bands) {
		for (int r = band.start; r < band.end; r++) {
			int count = 0;
			for (int c = 0; c < canny.cols; c++) {
				if (canny.at<uchar>(r, c) > 70)
					count++;
			}
			if (count > 25) {
				cv::line(bands, Point(0, r), Point(count, r), Scalar(255, 255, 255), 1);
			}

		}
	}
	cv::imshow("bands", bands);
	int i = 0;
	for (auto band : vec_bands) {
		cout << "**band " << i << endl;
		//canny
		//Mat canny;
		Mat roi = canny(Range(std::max(band.start - 15, 0), std::min(band.end + 15, gray_dewarped2.rows)),Range(0, gray_dewarped2.cols));
		//Canny(roi, canny, 200, 200);
		cv::imshow("roi" + to_string(i), roi);

		//hough
		Mat hough;
		double delta_rho = 1, delta_theta = CV_PI / 180;
		CHoughLines hl(roi, delta_rho, delta_theta, 65);
		vector<CHoughLines::Line> vec_line = hl.cal();

		//houghLines(canny, lines, delta_rho, delta_theta, 60);
		std::vector<CHoughLines::Line> lines_;
		for (auto line : vec_line) {
			float rho = line.rho;
			float theta = line.theta;
			float count = line.points->size();
			if (theta > 1.57 - 0.2 && theta < 1.57 + 0.2) {
				cout << "line: rho:" << rho << ", theta:" << theta << "(" << theta*180/3.141592 << "), count:" << count << endl;
				lines_.push_back(CHoughLines::Line(rho, theta, line.points));
			}
		}

		draw_houghLines(roi, hough, lines_, 6);
		
		cv::imshow("hough" + to_string(i++), hough);
		//break;
	}
	

	cv::waitKey(0);
    return 0;
}

