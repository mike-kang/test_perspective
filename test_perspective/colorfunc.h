#pragma once

#include <opencv2/opencv.hpp>
#include <vector>


bool isBG_GREEN_Color(cv::Vec3b color);
bool is_GREEN_FontColor(cv::Vec3b color);
bool is_GREEN_Color(cv::Vec3b color);
bool isBG_WHITE_Color(cv::Vec3b color);
void getColor(cv::Mat img, cv::Point2i point, int& h, float& s, uchar& v);
void printColor(cv::Mat img, cv::Point2i point);
bool isBG_WHITE_ColorEx(cv::Vec3b color, uchar v_threshold);
uchar get_v(cv::Mat img, cv::Point2i point);

