#include <opencv2/opencv.hpp>
#include "Dewarp.h"
#include "lens.h"

using namespace cv;
using namespace std;

//센서상의 점에서 왜곡을 제거한 p'를 구한다. 
void Dewarp::cal_point_(Point2f& src, Point2f& dst)
{
	Point2f p_0, p_;
	Point2f p0 = src - m_cameraParam.center;

	double length = norm(p0);
	double k = length / m_cameraParam.fl;
	float theta_;	//입사각
	findAngle(k, theta_);
	float length_ = m_cameraParam.fl * tan(theta_);
	p_0 = length_ / length * p0;
	dst = p_0;
}
void Dewarp::cal_point_(Point2i& src, Point2f& dst)
{
	Point2f p_0, p_;
	Point2f p0 = (Point2f)src - m_cameraParam.center;

	double length = norm(p0);
	double k = length / m_cameraParam.fl;
	float theta_;	//입사각
	findAngle(k, theta_);
	float length_ = m_cameraParam.fl * tan(theta_);
	p_0 = length_ / length * p0;
	dst = p_0;
}
//센서상의 점들에서 왜곡을 제거한 p'를 구한다. 
vector<Point2f>& Dewarp::cal_points_(vector<Point2f>& src, vector<Point2f>&dst)
{
	for (auto p : src) {
		Point2f p_;
		cal_point_(p, p_);
		dst.push_back(p_);
	}
	return dst;
}
vector<Point2f>& Dewarp::cal_points_(vector<Point2i>& src, vector<Point2f>&dst)
{
	for (auto p : src) {
		Point2f p_;
		cal_point_(p, p_);
		dst.push_back(p_);
	}
	return dst;
}
Point2f& Dewarp::cal_point(Point2f& p_, Point2f& p)
{
	Point2f p0;
	Point2f p_0 = p_;
	float k;

	float length_ = norm(p_0);
	float theta_ = atan(length_ / m_cameraParam.fl);

	findk(theta_, k);
	float length = k * m_cameraParam.fl;
	p0 = length / length_ * p_0;
	p = p0 + m_cameraParam.center;

	return p;
}


//왜곡없는 점들에서 왜곡이 포함된 점들을 구한다. p_ -> p
vector<Point2f>& Dewarp::cal_points(vector<Point2f>& src, vector<Point2f>&dst)
{
	float k;
	for (auto p_ : src) {
		Point2f p0, p;
		Point2f p_0 = p_;

		float length_ = norm(p_0);
		float theta_ = atan(length_ / m_cameraParam.fl);

		findk(theta_, k);
		float length = k * m_cameraParam.fl;
		p0 = length / length_ * p_0;
		p = p0 + m_cameraParam.center;
		dst.push_back(p);
	}
	return dst;
}

void Dewarp::cal_points(Mat& src, Mat& dst)
{
	float k;

	for (int j = 0; j < dst.rows; j++) {
		for (int i = 0; i < dst.cols; i++) {
			Point2f temp;
			cal_point(Point2f(src.at<Vec2f>(j, i)), temp);
			dst.at<Vec2f>(j, i)[0] = temp.x;
			dst.at<Vec2f>(j, i)[1] = temp.y;
		}
	}
}

void Dewarp::calHomography(vector<Point2f>& frame, vector<Point2f>& image)
{
	//cout << "points" << endl << points << endl;
	vector<Point2f> points_;

	m_homo = findHomography(frame, cal_points_(image, points_), NULL);
	m_homo_inv = m_homo.inv(cv::DECOMP_LU);
	//cout << m_homo_inv << endl;
	/*
	m_homo32.create(m_homo.rows, m_homo.cols, CV_32F);
	for (int j = 0; j < m_homo.rows; j++) {
		for (int i = 0; i < m_homo.cols; i++) {
			m_homo32.at<float>(j, i) = m_homo.at<double>(j, i);
		}
	}
	*/
#if 0
	vector<Point2f> dst;
	perspectiveTransform(frame, dst, m_homo);
	//cout << "dst" << endl << dst << endl;
	float sum = 0;
	for (int i = 0; i < image.size(); i++) {
		sum += norm(image[i] - dst[i]);
	}
	cout << "fl= " << m_cameraParam.fl << ", sum= " << sum << endl;
#endif
	//cout << dst << endl;
}

void Dewarp::makeImage(Mat& src_image, Mat& dst)
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
	cv::perspectiveTransform(mat_dst_points, mat_points_, m_homo);
	Mat mat_p(height, width, CV_32FC2);
	cal_points(mat_points_, mat_p);
	int col, row;
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			dst.at<Vec3b>(j, i) = src_image.at<Vec3b>(Point2i(mat_p.at<Vec2f>(j, i)));
		}
	}
}

void Dewarp::calImagePoints(vector<cv::Point2i>& pointsInFrame, vector<cv::Point2i>& pointsInimage)
{
	int count = pointsInFrame.size();

	double* data = new double[count * 3];
	int i = 0;
	for (Point2f p : pointsInFrame) {
		data[i++] = p.x;
		data[i++] = p.y;
		data[i++] = 1.0;
	}
	Mat m(count, 3, CV_64F, data);
	
	Mat result = m_homo * m.t();
	cout << result << endl;

	vector<cv::Point2f> pointsInimage_;
	for (int i = 0; i < count; i++) {
		float k = result.at<double>(2, i);
		pointsInimage_.push_back(Point2f(result.at<double>(0, i)/k, result.at<double>(1, i)/k));
	}
	vector<cv::Point2f> tempfloat;
	cal_points(pointsInimage_, tempfloat);
	for (auto p : tempfloat) {
		pointsInimage.push_back(p);
	}
	delete[]data;

}

void Dewarp::calImagePoint(Point2i& pointInFrame, Point2i& pointInimage)
{
	int i = 0;

	Mat m(Matx31d(pointInFrame.x, pointInFrame.y, 1.));

	Mat result = m_homo * m;
	//cout << result << endl;

	float k = result.at<double>(2);
	Point2f pointInimage_(result.at<double>(0) / k, result.at<double>(1) / k);
	Point2f temp;
	cal_point(pointInimage_, temp);
	pointInimage = temp;

}

void Dewarp::calDewarpPoint(Point2i& src, Point2i& dst)
{
	Point2f points_;
	cal_point_(src, points_);
	Mat m(Matx31d(points_.x, points_.y, 1.));
	Mat result = m_homo_inv * m;
	float k = result.at<double>(2);
	dst.x = result.at<double>(0) / k;
	dst.y = result.at<double>(1) / k;
}

void Dewarp::calDewarpPoints(vector<cv::Point2i>& src, vector<cv::Point2i>&dst)
{
	for (auto p : src) {
		Point2i dewarp;
		calDewarpPoint(p, dewarp);
		dst.push_back(dewarp);
	}
}

