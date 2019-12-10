#pragma once
#include <opencv2/opencv.hpp>

using namespace std;

class Dewarp
{
public:
	struct CameraParam {
		CameraParam() {}
		CameraParam(double _fl, cv::Point2f c) :fl(_fl), center(c) {}
		double fl;
		cv::Point2f center;

	};
	Dewarp(CameraParam cp) {
		m_cameraParam = cp;
	}
	void calHomography(vector<cv::Point2f>& frame/*in*/, vector<cv::Point2f>& image/*in*/);
	void makeImage(cv::Mat& src_image, cv::Mat& dst);
	void calImagePoints(vector<cv::Point2i>& pointsInFrame/*in*/, vector<cv::Point2i>& pointsInimage/*out*/);
	void calImagePoint(cv::Point2i& pointInFrame, cv::Point2i& pointInimage);

private:
	CameraParam m_cameraParam;
	cv::Mat m_homo;
	cv::Mat m_homo32;

	vector<cv::Point2f>& cal_points_(vector<cv::Point2f>& src, vector<cv::Point2f>&dst);
	cv::Point2f& cal_point(cv::Point2f& p_, cv::Point2f& p);
	vector<cv::Point2f>& cal_points(vector<cv::Point2f>& src, vector<cv::Point2f>&dst);
	void cal_points(cv::Mat& src, cv::Mat& dst);
	//void calHomography(vector<cv::Point2f>& P_points, vector<cv::Point2f>& points);

};