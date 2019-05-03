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
		p_ = p_0 + center;
		dst.push_back(p_);
	}
	return dst;
}

Point2f& cal_points(Point2f& p_, Point2f& p)
{
	Point2f p0;
	Point2f p_0 = p_ - center;
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
		Point2f p_0 = p_ - center;

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
void makeImage(Mat& src, Mat& h, Mat& dst)
{
	Mat mat_obj_(rect.height, rect.width, CV_32FC2);
	Mat mat_p_(rect.height, rect.width, CV_32FC2);

}

int main()
{
	center.x = 2145;
	center.y = 1477;
	fl = 947.;

	cv::Mat image; // create an empty image
	image = cv::imread("capture.jpg");
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

	Size dst_size(150, 200);
	Mat dst(150, 200, CV_8UC3);
	dst_points.push_back(Point2f{ 0,0 });
	dst_points.push_back(Point2f{ dst_size.width - 1,0 });
	dst_points.push_back(Point2f{ dst_size.width -1, dst_size.height -1 });
	dst_points.push_back(Point2f{ 0, dst_size.height - 1 });
	calHomography(dst_points, cal_points_(points, points_), homograpy);
	cout << "homography" << endl << homograpy << endl;

	makeImage(image, dst, homograpy);
	//cv::imshow("hough2[" + to_string(i) + "]", dst2);

	cv::waitKey(0);
    return 0;
}

