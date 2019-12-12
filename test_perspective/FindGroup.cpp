#include "FindGroup.h"
#include <algorithm>

using namespace cv;
using namespace std;

#define FACTOR ((float)1.6)

bool isContain(Rect rect, Point p)
{
	return (p.x >= rect.x && p.x <= rect.x + rect.width && p.y >= rect.y && p.y <= rect.y + rect.height);
}

FindGroup::FindGroup(std::vector<cv::Rect> rects) 
{
	int sum_w = 0, sum_h = 0;
	int min = 300, max = 0;
	for (auto rect : rects) {
		m_elements.push_back(Element(rect));
		sum_w += rect.width;
		sum_h += rect.height;
		if (rect.width < min)
			min = rect.width;
		if (rect.width > max)
			max = rect.width;
	}
	
	m_avail_count = rects.size();

	if (m_avail_count > 3) {
		m_avg_width = (sum_w - min - max) / (m_avail_count - 2);
	}
	else
		m_avg_width = sum_w / m_avail_count;
	m_increase_range_width = m_avg_width * FACTOR;

	m_increase_range_height = sum_h / m_avail_count * ((float)0.5);
}

void FindGroup::findLeft(Rect range, Group& group)
{
	//cout << "findLeft" << endl;
	for (int i = 0; i < m_elements.size(); i++) {
		Element& elem = m_elements[i];
		if (!elem.avail) continue;
		Rect rect = elem.rect;
		if (isContain(range, rect.tl() + Point2i(rect.width, 0)) || isContain(range, rect.br())) {
			elem.avail = false;
			m_avail_count--;
			group.add(rect);
			//cout << "add rect:" << i << endl;

			findLeft(Rect(rect.tl() - Point2i(m_increase_range_width, m_increase_range_height), rect.br() + Point2i(0, m_increase_range_height)), group);
		}
	}
}

void FindGroup::findRight(Rect range, Group& group)
{
	//cout << "findRight" << endl;
	for (int i = 0; i < m_elements.size(); i++) {
		Element& elem = m_elements[i];
		if (!elem.avail) continue;
		Rect rect = elem.rect;
		if (isContain(range, rect.tl()) || isContain(range, rect.br() - Point2i(rect.width, 0))) {
			elem.avail = false;
			m_avail_count--;
			group.add(rect);
			//cout << "add rect:" << i << endl;

			findRight(Rect(rect.tl() - Point2i(0, m_increase_range_height), rect.br() + Point2i(m_increase_range_width, m_increase_range_height)), group);
		}
	}
}

bool isBG_GREEN_Color_hsv(Vec3b color)
{
	if (color[1] > 70 && color[0] >= 35 && color[0] <= 90) {
		return true;
	}
	return false;
}

void FindGroup::process(vector<Group>& groups, Mat dewarped_img)
{
	//vector<Group> groups;
	sort(m_elements.begin(), m_elements.end(), [](Element a, Element b) -> bool {
		return a.rect.br().y > b.rect.br().y;
	});
	//cout << "*********************sorted" << endl;
	//for (auto elem : m_elements) {
	//	cout << elem.rect << endl;
	//}
	int i = 0;
	while (m_avail_count) {
		Group group;
		Element& center = m_elements[i];
		center.avail = false;
		m_avail_count--;
		//cout << "center:" << i << endl;

		group.setLevel(center.rect.br().y);
		group.add(center.rect);
		Rect centerRect = center.rect;
		findLeft(Rect(centerRect.tl() - Point2i(m_increase_range_width, m_increase_range_height), centerRect.br() + Point2i(0, m_increase_range_height)), group);
		findRight(Rect(centerRect.tl() - Point2i(0, m_increase_range_height), centerRect.br() + Point2i(m_increase_range_width, m_increase_range_height)), group);

		groups.push_back(group);
		while (++i < m_elements.size()) {
			if (m_elements[i].avail) {
				break;
			}
			//cout << "skip idx:" << i << endl;
		}
	}

	//cout << "Groups count:" << groups.size() << endl;
	i = 0;
	vector<Group> vecGroup2;
	vector<Group> vecGroup4;

	for (auto& group : groups) {
		int op = group.optimize();	// member 겹치는 것 제거
		//cout << "optimize count: " << op << endl;
		group.setId(i++);
		if (group.getMemberCount() == 4) {
			group.sort();
			vecGroup4.push_back(group);
		}
		else if (group.getMemberCount() == 2) {
			group.sort();
			vecGroup2.push_back(group);
		}
	}
	//member가 4개인 group과 2개인 group이 있을 때, 2개인 group이 왼편에 있으면 둘을 하나의 group으로 만든다.
	vector<std::pair<int, int> > pairs;
	if (vecGroup4.size() && vecGroup2.size()) {
		for (Group group2 : vecGroup2) {
			Rect rect2 = group2.getRightMember();
			for (Group group4 : vecGroup4) {
				//group4의 가장 왼쪽 rect과 group2의 가장 오른쪽 rect의 높이가 비슷하고, 거리가 2.5*m_avg_width 이내이면, OK
				Rect rect4 = group4.getLeftMember();
				int y0 = min(rect2.tl().y, rect4.tl().y);
				int y1 = max(rect2.br().y, rect4.br().y);
				int height = y1 - y0;
				if ((height < max(rect2.height, rect4.height) + min(rect2.height, rect4.height)* (float)0.2)){
					//cout << rect4.tl().x - rect2.br().x << " " << group4.getMaxWidth() << endl;
					if (rect4.tl().x - rect2.br().x <= 2.5 * group4.getMaxWidth()){
						pairs.push_back(std::make_pair(group2.getId(), group4.getId()));
					}
				}
			}
		}
	}
	for (auto pair : pairs) {
		groups[pair.first].add(groups[pair.second]);
		groups.erase(groups.begin() + pair.second);
		//cout << "[" << pair.first << "] members: " << groups[pair.first].getMemberCount() << endl;
	}

	//배경색 찾기
	for (auto& group : groups) {
		group.m_bgColor = Group::BG_WHITE;
		vector<Rect>& rects = group.getMembers();
		for (Rect rect : rects) {
			Mat hsv_img;
			cvtColor(dewarped_img(rect), hsv_img, CV_BGR2HSV);
			int green_count = 0;
			for (int j = 0; j < hsv_img.rows; j++) {
				for (int i = 0; i < hsv_img.cols; i++) {
					Vec3b color = hsv_img.at<Vec3b>(j, i);
					if (isBG_GREEN_Color_hsv(color))
						green_count++;
				}
			}
			float rate = (float)green_count / hsv_img.rows / hsv_img.cols * 100;
			//cout << "green rate: " << rate << endl;
			if (rate > 20) {
				group.m_bgColor = Group::BG_GREEN;
				break;
			}
		}
		switch (group.m_bgColor) {
		case Group::BG_WHITE:
			cout << "bg color : WHITE" << endl;
			break;
		case Group::BG_GREEN:
			cout << "bg color : GREEN" << endl;
			break;
		}
		//white 번호판은 한줄인데, 맨왼쪽이나 오른쪽에 번호판 모서리가 Rect으로 잡히는 경우 제거.
		if ((group.m_bgColor == Group::BG_WHITE && rects.size() > 5) 
			|| (group.m_bgColor == Group::BG_GREEN && rects.size() > 2))
		{
			group.sort();
			int sum_y = 0;
			int sum_height = 0;
			for (int i = 1; i < rects.size() - 1; i++) {
				sum_y += rects[i].tl().y;
				sum_height += rects[i].height;
			}
			int avg_y = sum_y / (rects.size() - 2);
			int avg_height = sum_height / (rects.size() - 2);
			if (rects[0].tl().y < avg_y - avg_height * 0.2 && rects[0].height < avg_height *0.75)
				rects.erase(rects.begin());
			if (rects[rects.size() - 1].tl().y < avg_y - avg_height * 0.2 && rects[rects.size() - 1].height < avg_height *0.75)
				rects.erase(rects.begin() + rects.size() - 1);
			if (rects[rects.size() - 1].tl().y > avg_y + avg_height * 0.2 && rects[rects.size() - 1].height < avg_height *0.75)
				rects.erase(rects.begin() + rects.size() - 1);
		}
#if 0
		//명도 찾기
		int v_count[256] = { 0, };
		for (Rect rect : rects) {
			Mat hsv_img;
			cvtColor(dewarped_img(rect), hsv_img, CV_BGR2HSV);
			for (int j = 0; j < hsv_img.rows; j++) {
				for (int i = 0; i < hsv_img.cols; i++) {
					v_count[hsv_img.at<Vec3b>(j, i)[2]]++;
				}
			}
		}
		for (int i = 0; i < 256; i++) {
			cout << "v[" << i << "] = " << v_count[i] << endl;
		}
#endif

	}

	//score를 매기고, score 순으로 정렬한다.
	//member가 하나인 것은 score = 0;
	//member 수가 많을 수록 높으며, 흰색은 4개 부터는 동일하다.(score 10의 자리수를 결정)
	//흰색 member * 10, 녹색 member * 20
	//위치가 낮을 수록 높다. score 1의 자리 수를 결정(전체 height를 10으로 나눈다)
	float factor = (float)dewarped_img.rows / 10;
	for (auto& group : groups) {
		int member_count = group.getMemberCount();
		if (member_count == 1)
			group.m_score = 0;
		else {
			if(group.m_bgColor == Group::BG_WHITE)
				group.m_score = min(40, member_count * 10);
			else 
				group.m_score = min(40, member_count * 20);
			group.m_score += group.getLevel() / factor;
		}
	}
	sort(groups.begin(), groups.end(), [](Group a, Group b) {
		return a.m_score > b.m_score;
	});

	//score가 0인 것은 제거
	i = 0;
	for (auto& group : groups) {
		if (group.m_score == 0)
			break;
		i++;
	}
	if (i != groups.size()) {
		cout << "Groups resize:" << groups.size() << " -> " << i << endl;
		groups.resize(i);
	}




}