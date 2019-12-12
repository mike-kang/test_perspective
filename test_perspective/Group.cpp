#include "Group.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

using namespace std;
using namespace cv;

void Group::sort()
{
	if (m_members.size() == 1)
		return;
	std::sort(m_members.begin(), m_members.end(), [](Rect a, Rect b) -> bool {
		return a.tl().x < b.tl().x;
	});
}

int Group::optimize() 
{
	//std::cout << "optimize" << std::endl;
	std::vector<std::pair<int, int> > seq_pairs;
	for (int j = 0; j < m_members.size(); j++) {
		for (int i = j + 1; i < m_members.size(); i++) {
			seq_pairs.push_back(std::make_pair(j, i));
		}
	}
	std::vector<std::pair<int, int> > pairs;
	for (auto pair : seq_pairs) {
		cv::Rect rectA = m_members[pair.first];
		cv::Rect rectB = m_members[pair.second];
		if (abs(rectA.x - rectB.x) <= 2
			&& abs(rectA.x + rectA.width - rectB.x - rectB.width) <= 2
			&& (rectA & rectB).area() >= std::min(rectA.area(), rectB.area()) * (float)0.6)
		{
			pairs.push_back(pair);
		}
	}
	vector<int> vec_erase;
	for (auto pair : pairs) {
		vec_erase.push_back(pair.first);
		vec_erase.push_back(pair.second);
		cv::Rect sum = m_members[pair.first] | m_members[pair.second];
		m_members.push_back(sum);
	}
	std::sort(vec_erase.begin(), vec_erase.end(), [](int a, int b) -> bool {
		return a > b;
	});
	for (auto i : vec_erase) {
		m_members.erase(m_members.begin() + i);
	}

	if (m_sorted)
		sort();
	return pairs.size();
}

int Group::getAvgWidth()
{
	int avg_width;
	int count = m_members.size();

	int min = 100, max = 0;
	int sum_w = 0;
	for (auto rect : m_members) {
		sum_w += rect.width;
		if (rect.width < min)
			min = rect.width;
		if (rect.width > max)
			max = rect.width;
	}

	if (count > 3) {
		avg_width = (sum_w - min - max) / (count - 2);
	}
	else
		avg_width = sum_w / count;
	return avg_width;
}

int Group::getMaxWidth()
{
	int max = 0;
	for (auto rect : m_members) {
		if (rect.width > max)
			max = rect.width;
	}

	return max;
}