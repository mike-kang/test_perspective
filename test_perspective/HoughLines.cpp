#include "stdafx.h"
#include "HoughLines.h"

using namespace std;
using namespace cv;

CHoughLines::CHoughLines(Mat src, double rho, double theta, int thresh)
	:m_src(src), m_unit_rho(rho), m_unit_theta(theta), m_thresh(thresh)
{
	m_height = int((src.rows + src.cols) * 2 / rho);
	m_width = int(CV_PI / theta);
	m_houghPlane = new HoughElement[m_width * m_height];
	memset(m_houghPlane, 0x00, m_width * m_height * sizeof(HoughElement));
}

vector<CHoughLines::Line> CHoughLines::cal()
{
	hough_coord();
	Size mask(3, 7);
	vector<Line> vec_line;
	//vector<Line> vec_s_line;
	acc_mask(mask, vec_line);
	sort_lines(vec_line);
	return vec_line;
}


CHoughLines::~CHoughLines()
{
	for (int i = 0; i < m_width* m_height; i++) {
		auto p = m_houghPlane + i;
		if (*p)
			delete *p;
	}
}

void CHoughLines::hough_coord()
{
	for (int i = 0; i < m_src.rows; i++) {
		for (int j = 0; j < m_src.cols; j++)
		{
			Point pt(j, i);
			if (m_src.at<uchar>(pt) > 0) {

				for (int t = 0; t < m_width; t++)
				{
					double radian = t * m_unit_theta;
					double r = pt.x * cos(radian) + pt.y * sin(radian);
					r = cvRound(r / m_unit_rho + m_height / 2.0);
					HoughElement& p = m_houghPlane[m_width * (int)r + t];
					if(!p)
						p = new vector<unsigned int>;
					p->push_back((j << 16) | i);	// x<<16 | y
					//cout << ((i << 16) | j) << endl;
				}
			}
		}
	}
	/*
	for (int j = 0; j < m_height; j++) {
		for (int i = 0; i < m_width; i++)
		{
			HoughElement p = m_houghPlane[m_width * j + i];
			if(p)
				cout << "[" << i <<"," <<j<< "] " << p->size() << endl;
		}
	}
	*/
}

void CHoughLines::acc_mask(Size size, vector<Line>& vec_line)
{
	Point  h_m = size / 2;	// 마스크 크기 절반
	int i = 0;
	for (int r = h_m.y; r < m_height - h_m.y; r++) {
		for (int t = h_m.x; t < m_width - h_m.x; t++)
		{
			HoughElement& p = m_houghPlane[m_width * r + t];
			if (p) {
				int c_value = p->size();
				if (c_value < m_thresh) {
					delete p;
					p = nullptr;
				}
				else{
					//cout << "[" << t << "," << r << "] center " << c_value << endl;
					int maxVal = 0;
					for (int u = -h_m.y; u <= h_m.y; u++) {
						for (int v = -h_m.x; v <= h_m.x; v++) {
							//Point point = left_top + Point(v, u);
							if (u || v) {
								HoughElement tp = m_houghPlane[m_width * (r + u) + t + v];
								if (tp) {
									if(tp->size() > maxVal)
										maxVal = tp->size();
								}
							}
						}
					}
					//cout << "maxVal " << maxVal << endl;

					//Rect rect(left_top, size);
					if (c_value >= maxVal)
					{
						//cout << p << endl;
						for (int u = -h_m.y; u <= h_m.y; u++) {
							for (int v = -h_m.x; v <= h_m.x; v++) {
								if (u || v) {
									HoughElement& tp = m_houghPlane[m_width * (r + u) + t + v];
									if (tp) {
										//cout << "del " << tp->size() << endl;
										delete tp;
										tp = nullptr;
									}
								}
							}
						}
						float rho = (float)((r - m_height / 2) * m_unit_rho);		// 수직거리
						float radian = (float)(t * m_unit_theta);						// 각도

						vec_line.push_back(Line(rho, radian, p));

						//cout << "[" << i++ << "] "<< rho <<"," << radian <<"," << p->size() <<"," << p << endl;
					}

				}
			}
		}
	}
}

/*
void CHoughLines::thres_lines(vector<Line>& lines, Size size)
{
	Point  h_m = size / 2;	// 마스크 크기 절반
	for (int r = h_m.y; r < m_height - h_m.y; r++) {
		for (int t = h_m.x; t < m_width - h_m.x; t++)
		{
			HoughElement p = m_houghPlane[m_width * r + t];
			if(p){
				float rho = (float)((r - m_height / 2) * m_unit_rho);		// 수직거리
				float radian = (float)(t * m_unit_theta);						// 각도

				lines.push_back(Line(rho, radian, p));
			}
		}
	}
}
*/
void CHoughLines::sort_lines(vector<Line>& vec_line)
{
	sort(vec_line.begin(), vec_line.end(), [](const Line& a, const Line& b) {
		return a.points->size() > b.points->size();
	});
}