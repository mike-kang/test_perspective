// test_perspective.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include "Dewarp.h"
#include "FindGroup.h"
#include "FloodFill.h"
#include "colorfunc.h"

using namespace cv;
using namespace std;

#define M_PI 3.141592
#define TO_RADIAN(x) ((x)/180.0 * 3.141592)

#define GROUP_DEBUG
#define SHOW_BAND
//#define SHOW_GRID_DEBUG
//extern void houghLines(Mat src, vector<Vec3f>& s_lines, double rho, double theta, int thresh);

//Point2f center;
Point2f image_start;
//double fl;
vector<Point2f> object_points;

enum Position {
	POS_UL,
	POS_U,
	POS_UR,
	POS_DL,
	POS_D,
	POS_DR,
	POS_MAX
};
const char* str_pos[] = { "upleft", "up", "upright", "downleft", "down", "downright" };
void getPoints(const char* filename, vector<Point2f>& points, Size& board_size, Size& plate_size, Position& position)
{
	FileStorage fs_P(filename, FileStorage::READ);
	CV_Assert(fs_P.isOpened());
	fs_P["points"] >> points;
	fs_P["board_size"] >> board_size;
	//fs_P["plate_size"] >> plate_size;
	string pos;
	fs_P["position"] >> pos;
	for (int i = 0; i < POS_MAX; i++) {
		if (pos == str_pos[i]) {
			position = (Position)i;
			break;
		}
	}

	fs_P.release();
}

void draw_houghLines(Mat src, Mat&dst, vector<Vec2f>& lines, int nline)
{
	cvtColor(src, dst, CV_GRAY2BGR);
	for (size_t i = 0; i < min((int)lines.size(), nline); i++) {
		float rho = lines[i][0], theta = lines[i][1];
		//double a = cos(theta), b = sin(theta);
		//Point2d pt(a*rho, b* rho);
		//Point2d delta(1000 * -b, 1000 * a);
		//line(dst, pt + delta, pt - delta, Scalar(0, 255, 0), 1, LINE_AA);

		if (theta < 3.14 / 4. || theta > 3.*3.14 / 4.) { // 수직 행
			//cv::Point pt1(rho / cos(theta), 0); // 첫 행에서 해당 선의 교차점   
			//cv::Point pt2((rho - result.rows*sin(theta)) / cos(theta), result.rows);
			//// 마지막 행에서 해당 선의 교차점
			//cv::line(image, pt1, pt2, cv::Scalar(255), 1); // 하얀 선으로 그리기

		}
		else { // 수평 행
			cv::Point pt1(0, rho / sin(theta)); // 첫 번째 열에서 해당 선의 교차점  
			cv::Point pt2(dst.cols, (rho - dst.cols*cos(theta)) / sin(theta));
			// 마지막 열에서 해당 선의 교차점
			cv::line(dst, pt1, pt2, Scalar(0, 255, 0), 1); 
		}
	}

}

/*
void draw_houghLines(Mat src, Mat&dst, vector<CHoughLines::Line>& lines, int nline)
{
	cvtColor(src, dst, CV_GRAY2BGR);
	for (size_t i = 0; i < min((int)lines.size(), nline); i++) {
		auto points = lines[i].points;
		for (auto point : *points) {
			dst.at<Vec3b>(point & 0x0000ffff, point >> 16) = Vec3b(0, 0, 255);
		}
	}
}
*/

void homogenOp(Mat img, Mat& dst, int mask_size)
{
	dst = Mat(img.size(), CV_8U, Scalar(0));
	Point h_m(mask_size / 2, mask_size / 2);

	for (int i = h_m.y; i < img.rows - h_m.y; i++) {
		for (int j = h_m.x; j < img.cols - h_m.x; j++)
		{
			uchar max = 0;
			for (int u = 0; u < mask_size; u++) {
				for (int v = 0; v < mask_size; v++)
				{
					int y = i + u - h_m.y;
					int x = j + v - h_m.x;
					uchar difference = abs(img.at<uchar>(i, j) - img.at<uchar>(y, x));
					if (difference > max) max = difference;
				}
			}
			dst.at<uchar>(i, j) = max;
		}
	}
}

void homogenOp_edge_horizon(Mat img, Mat& dst, int mask_size)
{
	dst = Mat(img.size(), CV_8U, Scalar(0));
	Point h_m(mask_size / 2, mask_size / 2);

	for (int i = h_m.y; i < img.rows - h_m.y; i++) {
		for (int j = 0; j < img.cols; j++)
		{
			uchar max = 0;
			uchar difference = abs(img.at<uchar>(i, j) - img.at<uchar>(i-1, j));
			uchar difference2 = abs(img.at<uchar>(i, j) - img.at<uchar>(i + 1, j));

			dst.at<uchar>(i, j) = MAX(difference, difference2);
		}
	}
}

void homogenOp_edge_vertical(Mat img, Mat& dst, int mask_size)
{
	dst = Mat(img.size(), CV_8U, Scalar(0));
	Point h_m(mask_size / 2, mask_size / 2);

	for (int i = 0; i < img.rows; i++) {
		for (int j = h_m.x; j < img.cols - h_m.x; j++)
		{
			uchar max = 0;
			uchar difference = abs(img.at<uchar>(i, j) - img.at<uchar>(i, j-1));
			uchar difference2 = abs(img.at<uchar>(i, j) - img.at<uchar>(i, j+1));

			dst.at<uchar>(i, j) = MAX(difference, difference2);
		}
	}
}
void calc_histo(Mat image, Mat &hist, int bins, int range_max = 256)
{
	hist = Mat(bins, 1, CV_32F, Scalar(0));
	float gap = range_max / (float)bins;

	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++)
		{
			int idx = int(image.at<uchar>(i, j) / gap);
			hist.at<float>(idx)++;
		}
	}
}


void draw_histo(Mat hist, Mat &hist_img, Size size = Size(256, 200))
{
	hist_img = Mat(size, CV_8U, Scalar(255));
	float  bin = (float)hist_img.cols / hist.rows;
	normalize(hist, hist, 0, size.height, NORM_MINMAX);

	for (int i = 0; i<hist.rows; i++)
	{
		float  start_x = (i * bin);
		float  end_x = (i + 1) * bin;
		Point2f pt1(start_x, 0);
		Point2f pt2(end_x, hist.at <float>(i));

		if (pt2.y > 0)
			rectangle(hist_img, pt1, pt2, Scalar(0), -1);
	}
	flip(hist_img, hist_img, 0);
}
struct _Segmentation {
	_Segmentation(Point s, Point e, int l) {
		start = s;
		end = e;
		length = l;
	}
	Point start;
	Point end;
	int length;
};
Vec2i findLineSegment(Mat roi, Vec2f line, int thresh)
{
	float rho = line[0];
	float theta = line[1];
	int y;
	float slope = -1./tan(theta);
	float intercept_y = rho / sin(theta);
	vector<_Segmentation> vec_segmentation;
	Point start, end;
	int count = 0;
	int status = 0; //0:find start point, 1:find end point
	int disconnect = 0;
	int pre_y;
	for (int x = 0; x < roi.cols; x++) {
		y = slope * x + intercept_y;
		if (y < 0) {
			if (slope <= 0)
				break;
			continue;
		}
		else if (y >= roi.rows - 1) {
			if (slope >= 0)
				break;
			continue;
		}
		if (!status) {
			if (roi.at<uchar>(y, x) > 0 || roi.at<uchar>(std::max(y - 1, 0), x) > 0 || roi.at<uchar>(min(y + 1, roi.rows - 1), x) > 0) {
				start.x = x;
				start.y = y;
				status = 1;
				disconnect = 0;
			}
		}
		else {
			//find end point
			if (roi.at<uchar>(y, x) == 0 && roi.at<uchar>(std::max(y - 1, 0), x) == 0 && roi.at<uchar>(min(y + 1, roi.rows - 1), x) == 0) {
				disconnect++;
				if (disconnect == 1) {
					end.x = x - 1;
					end.y = pre_y;
				}
				else if (disconnect > 1) {
					status = 0;
					int len = sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
					if(len >= thresh)
						vec_segmentation.push_back(_Segmentation(start, end, len));
				}
			}

		}
		pre_y = y;
	}
	for (auto seg : vec_segmentation) {
		cout << "segmentation:" << seg.length << "    "  << seg.start.x <<","<< seg.start.y << ",  " << seg.end.x << "," << seg.end.y << endl;

	}
	return Vec2i(0, 0);
}

float calTriArea(Point2f p0, Point2f p1, Point2f p2)
{
	float a = sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2));
	float b = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	float c = sqrt(pow(p2.x - p0.x, 2) + pow(p2.y - p0.y, 2));
	float s = (a + b + c) / 2.;
	return sqrt(s*(s - a)*(s - b)*(s - c));
}
float calArea(vector<Point2f> points)
{
	return calTriArea(points[0], points[1], points[2]) + calTriArea(points[2], points[3], points[0]);
}

void drawGrid(Mat& grid, Size mask, vector<Point> points)
{
	for (auto p : points) {
		int i = p.x;
		int j = p.y;
		grid(Range(j * mask.height, mask.height *(j + 1)), Range(i * mask.width, mask.width *(i + 1))) = Scalar(255, 255, 255);
	}

}
struct detectedLine {
	float slope;
	float intercept_y;
	int start_x;
	int end_x;
};
bool detect_line(Mat& img, Rect rect, int delta, vector<detectedLine>& result_lines);
bool find_outline(Mat& img, Point center, vector<detectedLine>& detected_lines, vector<Point>& result_points, Mat& track);

#define TO_PIXEL(x, factor) (x * factor)

enum PointType {
	TL,
	TR,
	BR,
	BL
};
struct PointEx {
	PointEx(Point2i p, PointType pt) :m_point(p), m_pt(pt) {}
	Point2i m_point;
	PointType m_pt;
};


void pointsPushRectGreen(Mat img, vector<Point2i>& points, Rect rect, bool (*font_func)(Vec3b), bool(*bg_func)(Vec3b), Position pos)
{
	Point2i point_delta;
	int factor;
	if (pos == POS_U || pos == POS_UL || pos == POS_UR)
		factor = 1;
	else
		factor = -1;
	Point2i rectPoints[] = { rect.tl(), rect.tl() + Point2i(rect.width, 0), rect.br(), rect.br() - Point2i(rect.width, 0) };
	Point2i deltaPoints[] = {  Point2i(1,1), Point2i(-1,1), Point2i(-1,-1), Point2i(1, -1) };
	for (int i = 0; i < 4; i++) {
		Point2i p = rectPoints[i];
		point_delta = deltaPoints[i];
		while (!(*font_func)(img.at<Vec3b>(p))) {
			p += point_delta;
		}
		while (!(*bg_func)(img.at<Vec3b>(p))) {
			p -= point_delta;
		}
		if (!rect.contains(p)) {
			cout << "point not included" << endl;
		}
		else
			points.push_back(p);
	}
}



bool findPlate(Mat& image, const char* xml, int idx)
{
	Point2f center;
	//Point2f image_start;

	//center.x = 1937;
	//center.y = 1488;
	center.x = 1950;
	center.y = 1511;
	float factor; 
	Dewarp dewarpObj(Dewarp::CameraParam(950, center));
	cout << "*************************test " << idx << endl;
	//Mat homograpy;

	vector<Point2f> points;
	//vector<Point2f> points_;
	vector<Point2f> dst_points;
	Size board_size_mm/*(mm,mm)*/, plate_size;
	Size board_size;
	Position position;
	getPoints(xml, points, board_size_mm, plate_size, position);

	float effective_pixel = calArea(points);
	cout << "effective_pixel:" << effective_pixel << endl;
	factor = sqrt(effective_pixel / board_size_mm.width / board_size_mm.height) * 1.3;
	//Size dst_size(450, 250);
	cout << "factor:" << factor << endl;
	board_size.width = TO_PIXEL(board_size_mm.width, factor);
	board_size.height = TO_PIXEL(board_size_mm.height, factor);

#define OFFSET_X 0 // mm
#define OFFSET_Y 0 // mm
#define ADD_X 0 //300 //mm

	//plate_size /= factor;	//not used
	int offset_x = TO_PIXEL(OFFSET_X, factor);
	int offset_y = TO_PIXEL(OFFSET_Y, factor);
	int add_x = TO_PIXEL(ADD_X, factor);
	dst_points.push_back(Point2f{ (float)offset_x,(float)offset_y });
	dst_points.push_back(Point2f{ (float)board_size.width - 1 + offset_x, (float)offset_y });
	dst_points.push_back(Point2f{ (float)board_size.width - 1 + offset_x, (float)board_size.height - 1 + offset_y });
	dst_points.push_back(Point2f{ (float)offset_x, (float)board_size.height - 1 + offset_y });
	
	dewarpObj.calHomography(dst_points, points);
	//cout << "homography" << endl << homograpy << endl;

	Mat dewarped(board_size.height + offset_y, board_size.width + offset_x + add_x, CV_8UC3);
	cout << "dewarped width:" << board_size.width + offset_x << ", height:" << board_size.height + offset_y << endl;

	dewarpObj.makeImage(image, dewarped);
	cv::imshow("dewarped" + std::to_string(idx), dewarped);
	//imwrite("dewarped.jpg", dewarped);

	Mat gray_dewarped(dewarped.rows, dewarped.cols, CV_8UC1);
	cvtColor(dewarped, gray_dewarped, CV_BGR2GRAY);
	//cv::imshow("gray_dewarped" + std::to_string(idx), gray_dewarped);

	//hist
	//Mat hist;
	//equalizeHist(gray_dewarped, hist);
	//cv::imshow("hist" + std::to_string(idx), hist);

	//canny
	Mat canny;
	//Mat roi = edge_h(Range(std::max(band.start - 15, 0), std::min(band.end + 15, gray_dewarped2.rows)), Range(0, gray_dewarped2.cols));
	Canny(gray_dewarped, canny, 200, 160);	//normal :200, 160 

	cv::imshow(std::to_string(idx) + "canny", canny);
	//imwrite("canny.jpg", canny);

#define FONT_WIDTH 55
#define FONT_HEIGHT 90
#if 1
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(canny, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
	
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect((contours.size()));
	vector<Rect> boundRect2;
	for (int i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 1, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
	}


	Mat contour_mat = Mat::zeros(canny.size(), CV_8UC3);

	for (int i = 0; i < contours.size(); i++) {
		float ratio = (float)boundRect[i].height / boundRect[i].width;
		if (ratio <= 4.0 && ratio >= 1.0 && boundRect[i].area() > TO_PIXEL(FONT_WIDTH, factor) * TO_PIXEL(FONT_HEIGHT, factor) * 0.25  
			&& boundRect[i].area() < TO_PIXEL(FONT_HEIGHT, factor) * TO_PIXEL(FONT_HEIGHT, factor)) {
			//cout << "ratio:" << ratio << ", area:" << boundRect[i].area() << ", :" << boundRect[i].tl() <<"," << boundRect[i].br() <<endl;
			drawContours(contour_mat, contours, i, Scalar(255, 0, 0), 1, 8);
			//rectangle(contour_mat, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 1, 8, 0);
			boundRect2.push_back(boundRect[i]);
		}
		//rectangle(contour_mat, Point(100,100), Point(200,200), Scalar(0, 0, 255), 1, 8, 0);
	}

	vector<Rect> boundRect3; //포함되는 것은 제거
	sort(boundRect2.begin(), boundRect2.end(), [](Rect a, Rect b) -> bool {
			return a.area() > b.area();
		});
	for (auto box2 : boundRect2) {
		bool bInclude = false;
		for (auto box3 : boundRect3) {
			if (box3.tl().x <= box2.tl().x && box3.tl().y <= box2.tl().y && 
				box3.br().x >= box2.br().x && box3.br().y >= box2.br().y)
				bInclude = true;
		}
		if (!bInclude) {
			boundRect3.push_back(box2);
			float ratio = (float)box2.height / box2.width;
			//cout << "ratio:" << ratio << ", area:" << box2.area() << ", :" << box2.tl() << "," << box2.br() << endl;
			rectangle(contour_mat, box2.tl(), box2.br(), Scalar(0, 255, 0), 1, 8, 0);
		}
	}
#if 1
	//group
	vector<Group> groups;
	FindGroup findgroup(boundRect3);
	findgroup.process(groups, dewarped);

#ifdef GROUP_DEBUG
	int i = 0;
	for (auto group : groups) {
		cout << "[" << i++ << "] members: " << group.getMemberCount() << ", score:" << group.m_score << endl;
		rectangle(contour_mat, group.getGroupRect().tl(), group.getGroupRect().br(), Scalar(0, 0, 255), 1, 8, 0);
	}
	cv::imshow(std::to_string(idx) + "contour", contour_mat);
#endif


	for (auto group : groups) {
		//FloodFill
		group.sort();
		Rect leftRect = group.getLeftMember();
		Rect rightRect = group.getRightMember();
		//vector<PointEx> pointsInFrameEx, pointsInImageEx;
		bool(*bg_func)(Vec3b);	//bg 판별식
		vector<Point2i> pointsInFrame, pointsInImage;
		FloodFill::FF_Point ffpointsInImage[4];
		if (group.m_bgColor == Group::BG_WHITE) {                                             
			bg_func = isBG_WHITE_Color;
			findFloodFillTargetWhite(dewarped, )
		}
		else {
			cout << "Green is skip!!!!!!" << endl;
			break;
			bg_func = isBG_GREEN_Color;
			pointsPushRectGreen(dewarped, pointsInFrame, leftRect, is_GREEN_FontColor, isBG_GREEN_Color, position);
			pointsPushRectGreen(dewarped, pointsInFrame, rightRect, is_GREEN_FontColor, isBG_GREEN_Color, position);
		}
#if 1
		dewarp.calImagePoint(pointsInFrame, pointsInImage);

		FloodFill floodfill(image, bg_func, true);
		vector<Point2i> ffpoints;
		for (FloodFill::FF_Point p : ffpointsInImage) {
			floodfill.process(p, ffpoints);
		}
		

#endif		
		//for (auto center : pointsInFrame) {
		//	circle(dewarped, center, 5, Scalar(0, 0, 255), 3);
		//}
		cv::imwrite("result.jpg", image);
		cout << "Only best group check!!!!!!" << endl;
		break;



	}
	cv::imshow("dewarped_" + std::to_string(idx), dewarped);

#endif

#endif



#if 0

	Mat grid(canny.rows, canny.cols, CV_8UC3);

//#define EDGE_THRESHOLD_LESS_RATE 0.15
//#define EDGE_THRESHOLD_RATE 0.0001
	grid = 0;
	Size grid_cell(TO_PIXEL(FONT_WIDTH, factor) * 0.65, TO_PIXEL(FONT_HEIGHT, factor) * 0.65); //좀 작게 잡아서 겹치는 효과를 노린다.
	int grid_rows = grid.rows / grid_cell.height;
	int grid_cols = grid.cols / grid_cell.width;
#define SHOW_GRID_DEBUG
#ifdef SHOW_GRID_DEBUG
	Mat grid_debug = canny.clone();
	for (int i = 0; i < grid_rows; i++) {
		line(grid_debug, Point(0, i*grid_cell.height), Point(grid_debug.cols - 1, i*grid_cell.height), 255);
	}
	for (int i = 0; i < grid_cols; i++) {
		line(grid_debug, Point(i*grid_cell.width, 0), Point(i*grid_cell.width, grid_debug.rows - 1), 255);
	}
	cv::imshow(std::to_string(idx) + "grid_debug" , grid_debug);
#endif
	//int threshold_grid = grid_cell.height * grid_cell.width * EDGE_THRESHOLD_RATE;
	//int threshold_less_grid = grid_cell.height * grid_cell.width * EDGE_THRESHOLD_LESS_RATE;
	vector<Point> vec_tr;
	//vector<Point> vec_tlr;
	//vector<Point> vec_tlr2;

	Mat mat_count(grid_rows, grid_cols, CV_8UC1);
	mat_count = 0;
	int max_count = 0;
	for (int j = 0; j < grid_rows; j++) {
		for (int i = 0; i < grid_cols; i++) {
			Mat& cell = canny(Range(j * grid_cell.height, grid_cell.height *(j + 1)), Range(i * grid_cell.width, grid_cell.width *(i + 1)));
			//int count_width;
			//int split_count = 0;
			int count = 0;
			for (int _j = 0; _j < grid_cell.width; _j++) {
				int temp = 0;
				for (int _i = 0; _i < grid_cell.height; _i++) {
					if (cell.at<uchar>(_i, _j) > 160)
						temp++;
				}
				
				count += temp;
			}
			mat_count.at<uchar>(j, i) = count;
			if (count > max_count)
				max_count = count;
		}
	}
	//cout << "threshold_grid:" << threshold_grid << ", less:" << threshold_less_grid << endl;
	cout << "count:" << endl;
	for (int j = 0; j < grid_rows; j++) {
		for (int i = 0; i < grid_cols; i++) {
			cout << (int)mat_count.at<uchar>(j, i) << ", ";
		}
		cout << endl;
	}
	cout << "max_count:" << max_count << endl;
	Mat grid_bin(grid_rows, grid_cols, CV_8UC1);
	int *count_v = new int[grid_rows];
	int try_count = 0;
	int plate_start_col, plate_start_row, plate_len = 0;
	bool bFind = false;
	int threshold_grid;
	while (true) {
		grid_bin = 0;
		memset(count_v, 0x00, grid_rows * sizeof(int));
		vec_tr.clear();
		threshold_grid = max_count * (0.9 - 0.1 * try_count);
		for (int j = 0; j < grid_rows; j++) {
			for (int i = 0; i < grid_cols; i++) {
				if (mat_count.at<uchar>(j, i) > threshold_grid) {
					vec_tr.push_back(Point(i, j));
					grid_bin.at<uchar>(j, i)++;
					count_v[j]++;
				}
			}
		}
		int state = 0; // 0:disconnect 1:exist 2:1 disconnected 
		int connected_count = 0;
		int s;
		for (int j = grid_rows - 1; j >=0; j--) {	//아래부터 탐색.
			if (count_v[j] >= 4 / 0.65) {
				for (int i = 0; i < grid_cols; i++) {
					int val = grid_bin.at<uchar>(j, i);
					if (state == 0) {
						if (val) {
							state = 1;
							s = i;
							connected_count = 1;
						}
					}
					else if (state == 1) {
						if (val) {
							connected_count++;
						}
						else {
							state = 2;
						}
					}
					else if (state == 2) {
						if (val) {
							connected_count += 2;
							state = 1;
						}
						else {
							state = 0;
							connected_count = 0;
							int len = i - s - 1;
							if (len >= 4) {
								cout << "find length:" << len << endl << "rate:" << try_count << endl;
								if (len > plate_len) {
									plate_start_col = s;
									plate_start_row = j;
									plate_len = len;
								}
								bFind = true;
							}

						}
					}
				}
			}
			if (bFind)
				break;
		}
		if (bFind)
			break;
		try_count++;
	}
	cout << "find start:row:" << plate_start_row << ",col:" << plate_start_col << ", plate_len:"<< plate_len << endl;
#if 0
	for (auto p : vec_tr) {
		if (p.y > 1 && mat_count.at<uchar>(p.y - 1, p.x) > threshold_less_grid)
			vec_tlr.push_back(Point(p.x, p.y - 1));
		if (p.y < grid_rows - 1 && mat_count.at<uchar>(p.y + 1, p.x) > threshold_less_grid)
			vec_tlr.push_back(Point(p.x, p.y + 1));
		if (p.x > 1 && mat_count.at<uchar>(p.y, p.x - 1) > threshold_less_grid)
			vec_tlr.push_back(Point(p.x - 1, p.y));
		if (p.x < grid_cols - 1 && mat_count.at<uchar>(p.y, p.x + 1) > threshold_less_grid)
			vec_tlr.push_back(Point(p.x + 1, p.y));
		//grid(Range(j * mask.height, mask.height *(j + 1)), Range(i * mask.width, mask.width *(i + 1))) = 255;
	}
	for (auto p : vec_tlr) {
		if (p.y > 1 && mat_count.at<uchar>(p.y - 1, p.x) > threshold_less_grid)
			vec_tlr2.push_back(Point(p.x, p.y - 1));
		if (p.y < grid_rows - 1 && mat_count.at<uchar>(p.y + 1, p.x) > threshold_less_grid)
			vec_tlr2.push_back(Point(p.x, p.y + 1));
		if (p.x > 1 && mat_count.at<uchar>(p.y, p.x - 1) > threshold_less_grid)
			vec_tlr2.push_back(Point(p.x - 1, p.y));
		if (p.x < grid_cols - 1 && mat_count.at<uchar>(p.y, p.x + 1) > threshold_less_grid)
			vec_tlr2.push_back(Point(p.x + 1, p.y));
		//grid(Range(j * mask.height, mask.height *(j + 1)), Range(i * mask.width, mask.width *(i + 1))) = 255;
	}
#endif
//hough

	drawGrid(grid, grid_cell, vec_tr);
	//drawGrid(grid, mask, vec_tlr);
	//drawGrid(grid, mask, vec_tlr2);
	cv::imshow(std::to_string(idx) + "grid", grid);
#endif
#if 0
	vector<detectedLine> result_lines;
	if (!detect_line(canny, Rect(plate_start_col*grid_cell.width, plate_start_row*grid_cell.height, plate_len*grid_cell.width, grid_cell.height), grid_cell.height * 0.5, result_lines)) {
		printf("No line is detected!\n");
		return false;
	}
#if 1//def SHOW_DEBUG_LINE
	Mat debug_line;
	cvtColor(canny, debug_line, CV_GRAY2BGR);
	for (auto line : result_lines) {
		cout << "line:: start_x" << line.start_x << ", slope:" << line.slope << ", intercept_y:" << line.intercept_y << endl;
		cv::line(debug_line, Point(line.start_x, line.slope*line.start_x + line.intercept_y), Point(line.end_x, line.slope*line.end_x + line.intercept_y), Scalar(0, 255, 0), 1);
	}
	cv::imshow(std::to_string(idx) + "debug_line", debug_line);
	//imwrite("debug_line.jpg", debug_line);

#endif

	vector<Point> outline;
	outline.reserve(1000);
	Mat track(canny.rows, canny.cols, CV_8UC1);
	track = 0;
	find_outline(canny, Point((plate_start_col + plate_len * 0.5)* grid_cell.width, (plate_start_row + 0.5) * grid_cell.height), result_lines, outline, track);
	cout << "outline point size:" << outline.size() << endl;

	Mat debug_outline;
	cvtColor(canny, debug_outline, CV_GRAY2BGR);
	for (auto point : outline) {
		debug_outline.at<Vec3b>(point) = Vec3b(0, 255, 0);
	}
	cv::imshow(std::to_string(idx) + "debug_outline", debug_outline);

	Mat debug_outline2;
	cv::resize(debug_outline, debug_outline2, cv::Size(dewarped.cols *3, dewarped.rows *3), 0, 0, CV_INTER_NN);
	cv::imshow(std::to_string(idx) + "debug_outline2", debug_outline2);
	imwrite("debug_outline.jpg", debug_outline);
#endif
	return true;
}
struct direction {
	int count;
	Point val[5];
};
direction pri_direction[9] = {
	//{Point(-1, 0), Point(-1,1), Point(-1, -1), Point(0, 1), Point(0, -1), Point(1,0), Point(1, -1)},
	{ 5, {Point(0, -1), Point(-1,0), Point(-1, -1), Point(1, -1), Point(-1, 1) }},
	{ 3, {Point(0, -1), Point(1,-1), Point(-1, -1), Point(0, 0), Point(0, 0) }},
	{ 5, {Point(1, 0), Point(0,-1), Point(1, -1), Point(1, 1), Point(-1, -1) }},
	//{ Point(-1, 0), Point(-1,-1), Point(-1, 1), Point(0, -1), Point(0, 1), Point(1,-1), Point(1, 1) },
	{ 3, {Point(-1, 0), Point(-1,-1), Point(-1,1), Point(0, 0), Point(0, 0)} },
	{ 0, {Point(0,0), Point(0, 0), Point(0, 0), Point(0, 0), Point(0, 0)}},
	{ 3, {Point(1,0), Point(1, 1), Point(1, -1), Point(0, 0), Point(0, 0)} },
	{ 5, {Point(-1,0), Point(0, 1), Point(-1, 1), Point(1, 1), Point(-1, -1) }},
	//{ Point(0,1), Point(-1, 1), Point(1, 1), Point(-1, 0), Point(1, 0), Point(-1, -1), Point(1, -1) },
	{ 3 ,{Point(0, 1), Point(-1,1), Point(1, 1), Point(0, 0), Point(0, 0) }},
	//{ Point(0,1), Point(1, 0), Point(1, 1), Point(1, -1), Point(0, -1), Point(-1, 1), Point(-1, 0) },
	{ 5, {Point(0,1), Point(1, 0), Point(1, 1), Point(1, -1), Point(-1, 1) }},

};

uchar dir_mask[9] = { 9, 1, 3, 8, 0, 2, 12, 4, 6 };
#define DIR(_x,_y, x,y) ((y - _y + 1) * 3  + x - _x + 1)
//return 0:success 1:disconnect 2:reachEnd 
int find_path(Mat& img, Point start, Point end, int dir, vector<Point>& result_points, vector<Point>::iterator& itr, bool bStart, Mat& track, detectedLine& dl)
{
#define THRESHOLD 150
	if (!bStart && start == end) {
		cout << "1 meet start point!!" << endl;
		return 0;
	}
	int temp;
	cout << "1 (" << start.x << "," << start.y << ")" << endl;

	result_points.push_back(start);
	if(!bStart)
		track.at<uchar>(start) = 1;
	itr = result_points.end() - 1;

	int disconnet_count = 0;
	bool bFind = false;
	bool bReachEnd = false;
	direction* direction_info = &pri_direction[dir];
	vector<Point> target;
	for (int i = 0; i < direction_info->count; i++) {
		int _x, _y;
		_x = start.x + direction_info->val[i].x;
		_y = start.y + direction_info->val[i].y;
		if (_x < 0 || _x > img.cols - 1 || _y < 0 || _y > img.rows - 1) {
			bReachEnd = true;
			continue;
		}
		int _dir = DIR(start.x, start.y, _x, _y);
		if (track.at<uchar>(_y, _x) == 1)
			continue;
		if (img.at<uchar>(_y, _x) >= THRESHOLD ) {
			target.push_back(Point(_x, _y));
		}
	}
	//cout << "target count " << target.size() << endl;
	if (target.size() > 1) {
		bool bDisconnect = false;

		for (auto p : target) {
			vector<Point>::iterator _itr;
			int ret = find_path(img, p, end, DIR(start.x, start.y, p.x, p.y), result_points, _itr, false, track, dl);
			if (ret == 0)
				return 0;
			else if (ret == 1) {
				bDisconnect = true;
			}
			else if (ret == 2) {
				cout << "1 erase" << endl;
				//cout << "e start(" << _itr->x << ", " << _itr->y << ")" << endl;
				for (auto i = _itr; i != result_points.end(); i++)
					cout << "e (" << i->x << ", " << i->y << ")" << endl;
				result_points.erase(_itr, result_points.end());
			}
		}
		return (bDisconnect)? 1: 2;
	}
	else if (target.size() == 0) {
		if (bReachEnd) {
			cout << "1 ReachEnd " << endl;
			return 2;
		}
		cout << "1 disconnect " << endl;
#if 0
		return 1;
#else
		switch (dir) {
		case 0:
		case 2:
		case 3:
		case 6:
		case 8:
		case 5:
		case 7: {
			cout << "not found point" << endl;
			return 1;
		}
		case 1: //up
		{
			//disconnet_count++;
			if (start.y - 1 > 0 && img.at<uchar>(start.y - 2, start.x) >= THRESHOLD)
				target.push_back(Point(start.x, start.y - 1));
			else
				return 1;
			break;
		}


	}	//switch
#endif
	}

	//if (breachend) {
	//	cout << "reach end" << endl;
	//	return false;
	//}
	Point _p = start;
	Point p = target[0];
	bReachEnd = false;
	while(true){
		bool ret;
		cout << "2 (" << p.x << "," << p.y << ")" << endl;
		if (!bStart && p == end) {
			cout << "2 meet start point!!" << endl;
			return 0;
		}

		result_points.push_back(p);
		dir = DIR(_p.x, _p.y, p.x, p.y);

		track.at<uchar>(p) = 1;

		direction_info = &pri_direction[dir];
		target.clear();
		for (int i = 0; i < direction_info->count; i++) {
			int _x, _y;
			_x = p.x + direction_info->val[i].x;
			_y = p.y + direction_info->val[i].y;
			if (_x < 0 || _x > img.cols - 1 || _y < 0 || _y > img.rows - 1) {
				bReachEnd = true;
				continue;
			}
			int _dir = DIR(p.x, p.y, _x, _y);
			//cout << "dir_mask (" << _x << "," << _y << ") " << (int)track.at<uchar>(_y, _x) << " - " << (int)dir_mask[_dir] << endl;
			if (track.at<uchar>(_y, _x) == 1)
				continue;
			if (img.at<uchar>(_y, _x) >= THRESHOLD) {
				target.push_back(Point(_x, _y));
			}
		}
		//cout << "2 target count " << target.size() << endl;

		if (target.size() > 1) {
			bool bDisconnect = false;
			for (auto pp : target) {
				vector<Point>::iterator _itr = result_points.begin();
				int ret = find_path(img, pp, end, DIR(p.x, p.y, pp.x, pp.y), result_points, _itr, false, track, dl);
				if (ret == 0)
					return 0;
				else if (ret == 1) {
					bDisconnect = true;
				}
				else if (ret == 2) {
					cout << "2 erase" << endl;
					//cout << "ee start(" << _itr->x << ", " << _itr->y << ")" << endl;
					for (auto i = _itr; i != result_points.end(); i++)
						cout << "e (" << i->x << ", " << i->y << ")" << endl;
					result_points.erase(_itr, result_points.end());
				}
			}
			return (bDisconnect) ? 1 : 2;
		}
		else if (target.size() == 0) {
			if (bReachEnd) {
				cout << "1 ReachEnd " << endl;
				return 2;
			}
			cout << "2 disconnect count:" << endl;
#if 0
			return 1;
#else
			switch (dir) {
			case 0:
			case 2:
			case 3:
			case 6:
			case 8:
			case 5:
			case 7:{
					cout << "not found point" << endl;
					return 1;
				}
			case 1: //up
			{
				//disconnet_count++;
				if (p.y - 1 > 0 && img.at<uchar>(p.y - 2, p.x) >= THRESHOLD)
					target.push_back(Point(p.x, p.y - 1));
				else
					return 1;
				break;
			}
			}	//switch
#endif
		}
		else {
			disconnet_count = 0;
		}
		//cout << "(" << x << "," << y << ")" << endl;
		_p = p;
		p = target[0];
		//dir = (p.y - _p.y + 1) * 3 + p.x - _p.x + 1;
	}
}

bool find_outline(Mat& img, Point center, vector<detectedLine>& detected_lines, vector<Point>& result_points, Mat& track)
{
#define THRESHOLD 150
	//find point
	int min = 1000;
	int y;
	int x = center.x;
	int idx = -1;
	int i = 0;
	for (auto line : detected_lines) {
		int _y;
		_y = line.slope * x + line.intercept_y;
		float d = abs(_y - center.y);
		if (min > d) {
			min = d;
			y = _y;
			idx = i;
		}
		i++;
	}
	if (img.at<uchar>(y, x) < THRESHOLD) {
		if (img.at<uchar>(y - 1, x) > THRESHOLD) {
			y--;
		}
		else if (img.at<uchar>(y + 1, x) > THRESHOLD) {
			y++;
		}
		else
			return false;
	}
	cout << "find point (" << x << ", " << y << ")" << endl;

	Point start(x, y);
	//result_points.push_back(start);

	Point _p = start;

	for (int _x = detected_lines[idx].start_x - 5; _x < detected_lines[idx].end_x; _x++) {
		img.at<uchar>(detected_lines[idx].slope * x + detected_lines[idx].intercept_y, _x) = 255;
	}
	vector<Point>::iterator _itr;
	cout << "try left" << endl;
	int ret = find_path(img, start, start, 3, result_points, _itr, true, track, detected_lines[idx]);
	cout << "ret "<< ret << endl;
	if (ret != 0) {
		cout << "try right" << endl;
		find_path(img, start, start, 5, result_points, _itr, true, track, detected_lines[idx]);
	}
	return ret;

}



bool detect_line(Mat& img, Rect rect, int delta, vector<detectedLine>& result_lines)
{
	int i = 0;
	//Rect temp;
	bool bFind = false;
	for(int i = 0; i < 4; i++){
		int end_y = std::min(img.rows - 1, rect.y + rect.height + i* delta);
		rect.y = std::max(0, rect.y - i* delta);
		rect.height = end_y - rect.y + 1;
		cout << "**Detect Line Rect " << i << " [(" << rect.y << ", " << rect.height << ")]" << endl;
		//canny
		//Mat canny;
		Mat roi = img(rect);
		//Canny(roi, canny, 200, 200);
		//cv::imshow("roi" + to_string(i), roi);

		//hough
		Mat hough;
		double delta_rho = 1, delta_theta = CV_PI / 180;

		vector<Vec2f> vec_line;
		HoughLines(roi, vec_line, delta_rho, delta_theta, rect.width * 0.6);
		cout << "detect line count " << vec_line.size() << endl;
		for (auto line : vec_line) {
			float rho = line[0];
			float theta = line[1];
			if (theta > 1.57 - 0.2 && theta < 1.57 + 0.2) {
				detectedLine dl;
				cout << "line: rho:" << rho << ", theta:" << theta << "(" << theta * 180 / 3.141592 << ")" << endl;
				dl.slope = -1. / tan(theta);
				dl.intercept_y = rho / sin(theta) + rect.y - dl.slope*rect.x;
				dl.start_x = rect.x;
				dl.end_x = rect.x + rect.width - 1;
				result_lines.push_back(dl);
				bFind = true;
			}
		}
		if (bFind)
			break;
		//draw_houghLines(roi, hough, _vec_line, 3);

		//for (Vec2f line : _vec_line) {
		//	findLineSegment(roi, line, h_end / 3);
		//	cout << "-----------------------------------" << endl;
		//}
		//cv::imshow("hough" + to_string(i++), hough);
		//break;
	}
	return bFind;
}
const char* points_xml[] = {
	//"points_211_DL3.xml",
	"points_174_UL1.xml",
	//"points_174_UR1.xml",
	//"points_174_D.xml",
	//"points_174_DL1.xml",
	//"points_174_DR1.xml"
};

int main()
{
	cv::Mat image; // create an empty image
	image = cv::imread("capture_174_2.jpg");
	if (image.empty()) { // error handling
						 // no image has been created…
						 // possibly display an error message
						 // and quit the application
		cv::waitKey(0);
	}

	for (int i = 0; i < sizeof(points_xml) / sizeof(char*); i++) {
		findPlate(image, points_xml[i], i);
	}

#if 0
#define CHAR_COUNT 7

	int max = 0;
	Mat graph_v(canny.rows, canny.cols+ 1, CV_8UC3);
	graph_v = 0;
	vector<Range> vec_bands;
	bool bInBand = false;
	int start_r;
	int needed_band_pixel = std::max(((CHAR_COUNT + 2) * 2), int(CHAR_COUNT * FONT_WIDTH * factor * EDGE_THRESHOLD_RATE)); // canny.cols / 10;
	int needed_band_width = FONT_HEIGHT * factor;	
	cout << "needed band pixel:" << needed_band_pixel << ", width:" << needed_band_width << endl;
	for (int r = 0; r < canny.rows; r++) {
		int count = 0;
		for (int c = 0; c < canny.cols; c++) {
			if (canny.at<uchar>(r, c) > 0)
				count++;
		}
		if (count > max)
			max = count;
		if (count > needed_band_pixel) {
			cv::line(graph_v, Point(0, r), Point(count, r), Scalar(255, 255, 255), 1);
			if (!bInBand) {
				start_r = r;
				bInBand = true;
			}
		}
		else {
			if (bInBand) {
				bInBand = false;
				if(r - start_r > needed_band_width)
					vec_bands.push_back(Range(start_r, r - 1));
			}
		}
		//graph_v.at<uchar>(r, sum) = 255;
	}
	cout << "vertical projection's max count is " << max << endl;
	cv::imshow("graph_v", graph_v);
#ifdef SHOW_BAND
	Mat bands(canny.rows, canny.cols + 1, CV_8UC3);
	bands = 0;
	for (auto band : vec_bands) {
		for (int r = band.start; r < band.end; r++) {
			int count = 0;
			for (int c = 0; c < canny.cols; c++) {
				if (canny.at<uchar>(r, c) > 0)
					count++;
			}
			if (count > needed_band_pixel) {
				cv::line(bands, Point(0, r), Point(count, r), Scalar(255, 255, 255), 1);
			}

		}
	}
	cv::imshow("bands", bands);
#endif
	int i = 0;
	for (auto band : vec_bands) {
		int v_start = std::max(band.start - 15, 0);
		int v_end = std::min(band.end + 15, canny.rows - 1);
		int h_start = 0;
		int h_end = canny.cols;
		cout << "**band " << i << " [(" << h_start <<", " << v_start << ")-(" << h_end <<", " << v_end << ")]" <<  endl;
		//canny
		//Mat canny;
		Mat roi = canny(Range(v_start, v_end),Range(h_start, h_end));
		//Canny(roi, canny, 200, 200);
		//cv::imshow("roi" + to_string(i), roi);

		//hough
		Mat hough;
		double delta_rho = 1, delta_theta = CV_PI / 180;
#if 0
		CHoughLines hl(roi, delta_rho, delta_theta, 65);
		vector<CHoughLines::Line> vec_line = hl.cal();

		//houghLines(canny, lines, delta_rho, delta_theta, 60);
		std::vector<CHoughLines::Line> lines_;
		for (auto line : vec_line) {
			float rho = line.rho;
			float theta = line.theta;
			float count = line.points->size();
			if (theta > 1.57 - 0.2 && theta < 1.57 + 0.2) {
				cout << "line: rho:" << rho << ", theta:" << theta << "(" << theta*180/3.141592 << "), count:" << count << endl;
				lines_.push_back(CHoughLines::Line(rho, theta, line.points));
			}
		}

		draw_houghLines(roi, hough, lines_, 6);
#else
		vector<Vec2f> vec_line, _vec_line;
		HoughLines(roi, vec_line, delta_rho, delta_theta, h_end / 5);
		for (auto line : vec_line) {
			float rho = line[0];
			float theta = line[1];
			if (theta > 1.57 - 0.2 && theta < 1.57 + 0.2) {
				cout << "line: rho:" << rho << ", theta:" << theta << "(" << theta * 180 / 3.141592 << ")" << endl;
				_vec_line.push_back(line);
			}
		}
		draw_houghLines(roi, hough, _vec_line, 3);
#endif
		for (Vec2f line : _vec_line) {
			findLineSegment(roi, line, h_end / 3);
			cout << "-----------------------------------" << endl;
		}
		cv::imshow("hough" + to_string(i++), hough);
		//break;
	}
#endif	

	cv::waitKey(0);
    return 0;
}

