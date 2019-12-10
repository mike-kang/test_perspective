#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>

class Group {
public:
	enum BG_COLOR {
		BG_WHITE,
		BG_GREEN
	};
	Group():m_sorted(false){

	}
	void setLevel(int val) { m_level = val; }
	void add(cv::Rect rect) {
		m_members.push_back(rect);
	}
	int getMemberCount() { return m_members.size(); }
	int getLevel() { return m_level; }
	int optimize();

	cv::Rect getGroupRect() {
		cv::Rect sum;
		for (auto rect : m_members) {
			sum |= rect;
		}
		return sum;
	}
	void sort();	//left -> right
	cv::Rect getLeftMember() {
		return m_members[0];
	}
	cv::Rect getRightMember() {
		return m_members[m_members.size() - 1];
	}
	int getId() { return m_id; }
	void setId(int id) { m_id = id; }
	void add(Group group) {
		m_level = std::min(m_level, group.m_level);
		m_members.insert(m_members.end(), group.m_members.begin(), group.m_members.end());
		if (m_sorted)
			sort();
	}
	int m_score;
	std::vector<cv::Rect>& getMembers() {
		return m_members;
	}
	BG_COLOR m_bgColor;
	int getAvgWidth();
	int getMaxWidth();

private:
	int m_level;
	std::vector<cv::Rect> m_members;
	int m_id;
	bool m_sorted;
	std::vector<cv::Point2i> m_points;	//후에 floodfill에 사용할 points

};