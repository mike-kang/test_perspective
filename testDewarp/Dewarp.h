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
	void calHomography(vector<cv::Point2f>& frame, vector<cv::Point2f>& image);
	void makeImage(cv::Mat& src_image, cv::Mat& dst);

private:
	CameraParam m_cameraParam;
	cv::Mat m_homo;

	vector<cv::Point2f>& cal_points_(vector<cv::Point2f>& src, vector<cv::Point2f>&dst);
	cv::Point2f& cal_points(cv::Point2f& p_, cv::Point2f& p);
	vector<cv::Point2f>& cal_points(vector<cv::Point2f>& src, vector<cv::Point2f>&dst);
	void cal_points(cv::Mat& src, cv::Mat& dst);
	//void calHomography(vector<cv::Point2f>& P_points, vector<cv::Point2f>& points);

};