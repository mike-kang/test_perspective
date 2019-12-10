#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class FloodFill {
public:
	struct FF_Point {
		Point2i seedPoint;
		Point2i pt[3];
		Point2i directPoint;
	};

	FloodFill(cv::Mat img, bool (*func)(cv::Vec3b), bool debug= false) 
		:m_img(img), m_func(func), m_debug(debug){
		m_checkMat = cv::Mat::zeros(m_img.size(), CV_8U);

	}
	void process(FF_Point ff_p, std::vector<cv::Point2i>& points) {
		points.clear();
		m_points = points;
		//float* data = new float[4];
		
		for (int i = 0; i < 2; i++) {
			Mat m1(Matx22f(ff_p.pt[i].x, 1, ff_p.pt[i + 1].x, 1));
			Mat m2(Matx12f(ff_p.pt[i].y, ff_p.pt[i + 1].y));
			Mat m2_t = m2.t();
			Mat m1_inv = m1.inv(DECOMP_LU);
			Mat ab = m1_inv * m2_t;
			m_a[i] = ab.at<float>(0);
			m_b[i] = ab.at<float>(1);
			if (checkfunc0(i, ff_p.directPoint))
				m_checkfunc[i] = checkfunc0;
			else
				m_checkfunc[i] = checkfunc1;
		}
		
		floodfill(ff_p.seedPoint, 0);
	}

private:
	bool checkfunc0(int i, cv::Point2i p) {
		return (m_a[i] * p.x + m_b[i] > p.y);
	}
	bool checkfunc1(int i, cv::Point2i p) {
		return (m_a[i] * p.x + m_b[i] < p.y);
	}
	void floodfill(cv::Point2i p, int depth) {
		if((this->*m_checkfunc[0])(0, p) && (this->*m_checkfunc[1])(1, p))
			m_points.push_back(p);
		m_checkMat.at<char>(p) = 1;
		//if (m_debug)
		//	m_img.at<cv::Vec3b>(p) = cv::Vec3b(0, 0, 255);
		cv::Point _p = p + cv::Point2i(0, -1);
		if (m_checkMat.at<char>(_p) == 0 && (*m_func)(m_img.at<cv::Vec3b>(_p))) {
			cout << "up:" << depth << endl;
			floodfill(_p, depth + 1);
		}
		_p = p + cv::Point2i(0, 1);
		if (m_checkMat.at<char>(_p) == 0 && (*m_func)(m_img.at<cv::Vec3b>(_p))) {
			cout << "down:" << depth << endl;
			floodfill(_p, depth + 1);
		}
		_p = p + cv::Point2i(-1, 0);
		if (m_checkMat.at<char>(_p) == 0 && (*m_func)(m_img.at<cv::Vec3b>(_p))) {
			cout << "left:" << depth << endl;
			floodfill(_p, depth + 1);
		}
		_p = p + cv::Point2i(1, 0);
		if (m_checkMat.at<char>(_p) == 0 && (*m_func)(m_img.at<cv::Vec3b>(_p))) {
			cout << "right:" <<depth << endl;
			floodfill(_p, depth + 1);
		}
	}

	cv::Mat m_img;
	bool(*m_func)(cv::Vec3b);
	bool m_debug;
	std::vector<cv::Point2i> m_points;
	cv::Mat m_checkMat;
	float m_a[2];
	float m_b[2];
	bool(FloodFill::*m_checkfunc[2])(int i, cv::Point2i p);

};