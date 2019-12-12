#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "colorfunc.h"
#include <functional>

class FloodFill {
public:
	struct FF_Point {
		cv::Point2i seedPoint;
		cv::Point2i pt[3];
		cv::Point2i directPoint;
		uchar v_threshold;
		void print() {
			std::cout << "seedPoint: " << seedPoint << std::endl;
			std::cout << "pt: " << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
			std::cout << "directPoint: " << directPoint << std::endl;
		}
	};

	FloodFill(cv::Mat img, bool (*func)(cv::Vec3b, uchar v_threshold), bool debug= false) 
		:m_img(img), m_func(func), m_debug(debug){

	}
	int process(FF_Point ff_p, std::vector<cv::Point2i>* points, uchar initial_v_threshold, uchar v_diff) {
		//points->clear();
		m_points = points;
		m_checkMat = cv::Mat::zeros(m_img.size(), CV_8U);

		//float* data = new float[4];
		m_v_threshold = initial_v_threshold;
		m_v_diff = v_diff;
		//cout << "v_threshold:" << (int)m_v_threshold << endl;
		for (int i = 0; i < 2; i++) {
			cv::Mat m1(cv::Matx22f(ff_p.pt[i].x, 1, ff_p.pt[i + 1].x, 1));
			cv::Mat m2(cv::Matx12f(ff_p.pt[i].y, ff_p.pt[i + 1].y));
			cv::Mat m2_t = m2.t();
			cv::Mat m1_inv = m1.inv(cv::DECOMP_LU);
			cv::Mat ab = m1_inv * m2_t;
			m_a[i] = ab.at<float>(0);
			m_b[i] = ab.at<float>(1);
			if (checkfunc0(i, ff_p.directPoint))
				m_checkfunc[i] = &FloodFill::checkfunc0;
			else
				m_checkfunc[i] = &FloodFill::checkfunc1;
		}
		if (!m_func(m_img.at<cv::Vec3b>(ff_p.seedPoint), m_v_threshold)) {
			cout << "seed point color error" << endl;
			printColor(m_img, ff_p.seedPoint);
			return 1;
		}
		if (!(this->*m_checkfunc[0])(0, ff_p.seedPoint) || !(this->*m_checkfunc[1])(1, ff_p.seedPoint)) {
			cout << "seed point position error" << endl;
			return 2;
		}

		cout << "seed Point " << ff_p.seedPoint << endl;
		if(!floodfill(ff_p.seedPoint, m_v_threshold, 0)) {
			//v_threshold가 너무 낮아서 생기는 문제.
			cout << "*********************v_threshold is low : " << (int)m_v_threshold << endl;
			return 3;
		}
		//cout << "*********************v_threshold:" << (int)m_v_threshold << endl;
		return 0;
	}

private:
	bool checkfunc0(int i, cv::Point2i p) {
		//std::cout <<"checkfunc0 " << m_a[i] * p.x + m_b[i] << " " << p.y << std::endl;
		return (m_a[i] * p.x + m_b[i] >= p.y);
	}
	bool checkfunc1(int i, cv::Point2i p) {
		//std::cout << "checkfunc1 " << m_a[i] * p.x + m_b[i] << " " << p.y << std::endl;
		return (m_a[i] * p.x + m_b[i] <= p.y);
	}
	bool floodfill(cv::Point2i p, uchar pre_v, int depth) {
		if (depth >100) {
			cout << "floodfill depth :" << depth << endl;
			return false;
		}
		if (m_checkMat.at<char>(p))
			return true;
		m_checkMat.at<char>(p) = 1;
		uchar v = get_v(m_img, p);
		if (pre_v - v > m_v_diff)
			return true;
		if ((this->*m_checkfunc[0])(0, p) && (this->*m_checkfunc[1])(1, p))
		{
			m_points->push_back(p);
			//m_checkMat.at<char>(p) = 1;
			//cout << "up:" << depth << endl;
			if (!floodfill(p + cv::Point2i(0, -1), v, depth + 1))
				return false;
			if (!floodfill(p + cv::Point2i(0, 1), v, depth + 1))
				return false;
			if (!floodfill(p + cv::Point2i(-1, 0), v, depth + 1))
				return false;
			if (!floodfill(p + cv::Point2i(1, 0), v, depth + 1))
				return false;
		}
		return true;
	}

	cv::Mat m_img;
	//bool(*m_func)(cv::Vec3b);
	std::function<bool(cv::Vec3b, uchar v_threshold)> m_func;
	bool m_debug;
	std::vector<cv::Point2i>* m_points;
	cv::Mat m_checkMat;
	float m_a[2];
	float m_b[2];
	bool(FloodFill::*m_checkfunc[2])(int i, cv::Point2i p);
	uchar m_v_threshold;
	uchar m_v_diff;
};