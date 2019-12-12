#pragma once
#include <opencv2/opencv.hpp>

using namespace std;

class Dewarp
{
public:
	struct CameraParam {
		CameraParam() {}
		CameraParam(double _fl, cv::Point2i c) :fl(_fl), center(c) {}
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
	void calDewarpPoints(vector<cv::Point2i>& src, vector<cv::Point2i>&dst);
	void calDewarpPoint(cv::Point2i& src, cv::Point2i& dst);
private:
	CameraParam m_cameraParam;
	cv::Mat m_homo;
	cv::Mat m_homo_inv;

	void Dewarp::cal_point_(cv::Point2f& src, cv::Point2f& dst);
	void Dewarp::cal_point_(cv::Point2i& src, cv::Point2f& dst);

	vector<cv::Point2f>& cal_points_(vector<cv::Point2f>& src, vector<cv::Point2f>&dst);
	vector<cv::Point2f>& cal_points_(vector<cv::Point2i>& src, vector<cv::Point2f>&dst);

	cv::Point2f& cal_point(cv::Point2f& p_, cv::Point2f& p);
	vector<cv::Point2f>& cal_points(vector<cv::Point2f>& src, vector<cv::Point2f>&dst);
	void cal_points(cv::Mat& src, cv::Mat& dst);


	//void calHomography(vector<cv::Point2f>& P_points, vector<cv::Point2f>& points);

};