#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "Group.h"

class FindGroup {
public:

	FindGroup(std::vector<cv::Rect> rects/*, Position pos*/);
	void process(std::vector<Group>& groups, cv::Mat dewarped_img);

private:
	struct Element {
		Element(cv::Rect _rect, bool a = true) {
			avail = a;
			rect = _rect;
		}
		bool avail;
		cv::Rect rect;

	};

	void findLeft(cv::Rect range, Group& group);
	void findRight(cv::Rect range, Group& group);

	std::vector<Element> m_elements;
	int m_increase_range_width;
	int m_increase_range_height;
	int m_avg_width;
	int m_avg_height;
	int m_avail_count;

};