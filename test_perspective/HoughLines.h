#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

using HoughElement = std::vector<unsigned int>*;

class CHoughLines
{
public:
	struct Line {
		Line(float _rho, float _theta, std::vector<unsigned int>* _points) {
			rho = _rho;
			theta = _theta;
			points = _points;
		}
		float rho;
		float theta;
		std::vector<unsigned int>* points;
	};

	CHoughLines(cv::Mat src, double rho, double theta, int thresh);
	~CHoughLines();
	std::vector<Line> cal();

private:
	void hough_coord();
	void acc_mask(cv::Size size, std::vector<Line>& vec_line);
	//void thres_lines(std::vector<Line>& lines, cv::Size size);
	void sort_lines(std::vector<Line>& vec_line);

	HoughElement* m_houghPlane;
	cv::Mat m_src;
	int m_width;
	int m_height;
	double m_unit_rho;
	double m_unit_theta;
	int m_thresh;
};
