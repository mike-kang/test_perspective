// test_perspective.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "lens.h"
#include <algorithm>
//#include "HoughLines.h"

using namespace cv;
using namespace std;

#define M_PI 3.141592
#define TO_RADIAN(x) ((x)/180.0 * 3.141592)

#define SHOW_BAND
//#define SHOW_GRID_DEBUG
//extern void houghLines(Mat src, vector<Vec3f>& s_lines, double rho, double theta, int thresh);

Point2f center;
Point2f image_start;
double fl;
vector<Point2f> object_points;

void getPoints(const char* filename, vector<Point2f>& points, Size& board_size, Size& plate_size)
{
	FileStorage fs_P(filename, FileStorage::READ);
	CV_Assert(fs_P.isOpened());
	fs_P["points"] >> points;
	fs_P["board_size"] >> board_size;
	fs_P["plate_size"] >> plate_size;
	fs_P.release();
}

//센서상의 점들에서 왜곡을 제거한 p'를 구한다. 
vector<Point2f>& cal_points_(vector<Point2f>& src, vector<Point2f>&dst)
{
	float k;
	for (auto p : src) {
		Point2f p_0, p_;
		Point2f p0 = p - center;

		double length = norm(p0);
		double k = length / fl;
		float theta_;	//입사각
		findAngle(k, theta_);
		float length_ = fl * tan(theta_);
		p_0 = length_ / length * p0;
		p_ = p_0;
		dst.push_back(p_);
	}
	return dst;
}

Point2f& cal_points(Point2f& p_, Point2f& p)
{
	Point2f p0;
	Point2f p_0 = p_;
	float k;

	float length_ = norm(p_0);
	float theta_ = atan(length_ / fl);

	findk(theta_, k);
	float length = k * fl;
	p0 = length / length_ * p_0;
	p = p0 + center;

	return p;
}

//왜곡없는 점들에서 왜곡이 포함된 점들을 구한다.
vector<Point2f>& cal_points(vector<Point2f>& src, vector<Point2f>&dst)
{
	float k;
	for (auto p_ : src) {
		Point2f p0, p;
		Point2f p_0 = p_;

		float length_ = norm(p_0);
		float theta_ = atan(length_ / fl);

		findk(theta_, k);
		float length = k * fl;
		p0 = length / length_ * p_0;
		p = p0 + center;
		dst.push_back(p);

		return dst;
	}
}

void cal_points(Mat& src, Mat& dst)
{
	float k;

	for (int j = 0; j < dst.rows; j++) {
		for (int i = 0; i < dst.cols; i++) {
			Point2f temp;
			cal_points(Point2f(src.at<Vec2f>(j, i)), temp);
			dst.at<Vec2f>(j, i)[0] = temp.x;
			dst.at<Vec2f>(j, i)[1] = temp.y;
		}
	}
}
void calHomography(vector<Point2f>& P_points, vector<Point2f>& points, Mat& h)
{
	//cout << "points" << endl << points << endl;

	h = findHomography(P_points, points, NULL);

	vector<Point2f> dst;
	perspectiveTransform(P_points, dst, h);
	//cout << "dst" << endl << dst << endl;
	float sum = 0;
	for (int i = 0; i < points.size(); i++) {
		sum += norm(points[i] - dst[i]);
	}
	cout << "fl= " << fl << ", sum= " << sum << endl;
	//}
	//cout << dst << endl;
}

void makeImage(Mat& src_image, Mat& dst, Mat& h)
{
	int width = dst.cols;
	int height = dst.rows;

	Mat mat_dst_points(height, width, CV_32FC2);
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			mat_dst_points.at<Vec2f>(j, i)[0] = i;
			mat_dst_points.at<Vec2f>(j, i)[1] = j;
		}
	}
	Mat mat_points_(height, width, CV_32FC2);
	perspectiveTransform(mat_dst_points, mat_points_, h);
	Mat mat_p(height, width, CV_32FC2);
	cal_points(mat_points_, mat_p);
	int col, row;
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			dst.at<Vec3b>(j, i) = src_image.at<Vec3b>(Point2i(mat_p.at<Vec2f>(j,i)));

		}
	}
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
bool find_outline(Mat& img, Point center, vector<detectedLine>& detected_lines, vector<Point>& result_points);

bool test(Mat& image, const char* xml, int idx)
{
	center.x = 1937;
	center.y = 1488;
	float factor;;
	fl = 950;

	cout << "*************************test " << idx << endl;
	Mat homograpy;

	vector<Point2f> points;
	vector<Point2f> points_;
	vector<Point2f> dst_points;
	Size board_size, plate_size;

	getPoints(xml, points, board_size, plate_size);

	float effective_pixel = calArea(points);
	cout << "effective_pixel:" << effective_pixel << endl;
	factor = sqrt(effective_pixel / board_size.width / board_size.height);
	//Size dst_size(450, 250);
	cout << "factor:" << factor << endl;
	board_size.width = board_size.width * factor;
	board_size.height = board_size.height * factor;

#define OFFSET_X 0 // mm
#define OFFSET_Y 0 // mm
#define ADD_X 100 //mm

	//plate_size /= factor;	//not used
	int offset_x = OFFSET_X * factor;
	int offset_y = OFFSET_Y * factor;
	int add_x = ADD_X * factor;
	dst_points.push_back(Point2f{ (float)offset_x,(float)offset_y });
	dst_points.push_back(Point2f{ (float)board_size.width - 1 + offset_x, (float)offset_y });
	dst_points.push_back(Point2f{ (float)board_size.width - 1 + offset_x, (float)board_size.height - 1 + offset_y });
	dst_points.push_back(Point2f{ (float)offset_x, (float)board_size.height - 1 + offset_y });
	calHomography(dst_points, cal_points_(points, points_), homograpy);
	cout << "homography" << endl << homograpy << endl;

	Mat dewarped(board_size.height + offset_y, board_size.width + offset_x + add_x, CV_8UC3);
	cout << "dewarped width:" << board_size.width + offset_x << ", height:" << board_size.height + offset_y << endl;

	makeImage(image, dewarped, homograpy);
	//cv::imshow("dewarped" + std::to_string(idx), dewarped);
	imwrite("result.jpg", dewarped);

	Mat gray_dewarped(dewarped.rows, dewarped.cols, CV_8UC1);
	cvtColor(dewarped, gray_dewarped, CV_BGR2GRAY);

	//cv::imshow("gray_dewarped" + std::to_string(idx), gray_dewarped);
	//canny
	Mat canny;
	//Mat roi = edge_h(Range(std::max(band.start - 15, 0), std::min(band.end + 15, gray_dewarped2.rows)), Range(0, gray_dewarped2.cols));
	Canny(gray_dewarped, canny, 200, 100);

	//cv::imshow(std::to_string(idx) + "canny", canny);
	Mat grid(canny.rows, canny.cols, CV_8UC3);
#define FONT_WIDTH 40
#define FONT_HEIGHT 70
#define EDGE_THRESHOLD_LESS_RATE 0.15
#define EDGE_THRESHOLD_RATE 0.30
	grid = 0;
	Size grid_cell((FONT_WIDTH + 3) * factor, FONT_HEIGHT * factor);
	int grid_rows = grid.rows / grid_cell.height;
	int grid_cols = grid.cols / grid_cell.width;

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
	int threshold_grid = grid_cell.height * grid_cell.width * EDGE_THRESHOLD_RATE;
	int threshold_less_grid = grid_cell.height * grid_cell.width * EDGE_THRESHOLD_LESS_RATE;
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
			int split_count = 0;
			int count = 0;
			for (int _j = 0; _j < grid_cell.width; _j++) {
				int temp = 0;
				for (int _i = 0; _i < grid_cell.height; _i++) {
					if (cell.at<uchar>(_i, _j) > 100)
						temp++;
				}
				//count_width = temp;
				if (temp < 3)
					split_count++;
				count += temp;
			}
			//if (!split_count)
			//	count = 0;
			mat_count.at<uchar>(j, i) = count;
			if (count > max_count)
				max_count = count;
		}
	}
	cout << "threshold_grid:" << threshold_grid << ", less:" << threshold_less_grid << endl;
	cout << "max_count:" << max_count << endl;
	Mat grid_bin(grid_rows, grid_cols, CV_8UC1);
	int *count_v = new int[grid_rows];
	int _rate = 0;
	int plate_start_col, plate_start_row, plate_len = 0;
	bool bFind = false;
	while (true) {
		grid_bin = 0;
		memset(count_v, 0x00, grid_rows * sizeof(int));
		vec_tr.clear();
		threshold_grid = max_count * (0.9 - 0.1*_rate);
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
		for (int j = grid_rows - 1; j >=0; j--) {
			if (count_v[j] >= 4) {
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
								cout << "find length:" << len << endl;
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
		_rate++;
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
#endif

	vector<Point> outline;
	find_outline(canny, Point((plate_start_col + plate_len * 0.5)* grid_cell.width, (plate_start_row + 0.5) * grid_cell.height), result_lines, outline);
	cout << "outline point size:" << outline.size() << endl;

	Mat debug_outline;
	cvtColor(canny, debug_outline, CV_GRAY2BGR);
	for (auto point : outline) {
		debug_outline.at<Vec3b>(point) = Vec3b(0, 255, 0);
	}
	//cv::imshow(std::to_string(idx) + "debug_outline", debug_outline);
	Mat debug_outline2;
	cv::resize(debug_outline, debug_outline2, cv::Size(dewarped.cols *3, dewarped.rows *3), 0, 0, CV_INTER_NN);
	cv::imshow(std::to_string(idx) + "debug_outline2", debug_outline2);

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
//return 0: disconnect 1:
bool find_path(Mat& img, Point start, Point end, int dir, vector<Point>& result_points, vector<Point>::iterator& itr, bool bStart)
{
#define THRESHOLD 150
	if (!bStart && start == end) {
		return true;
	}
	//cout << "1 (" << start.x << "," << start.y << ")" << endl;
	result_points.push_back(start);
	auto start_itr = result_points.end() - 1;
	itr = start_itr;

	int disconnet_count = 0;
	bool bFind = false;
	bool bDisconnect = false;
	direction& _direction = pri_direction[dir];
	vector<Point> target;
	for (int i = 0; i < _direction.count; i++) {
		int _x, _y;
		_x = start.x + _direction.val[i].x;
		_y = start.y + _direction.val[i].y;
		if (_x < 0 || _x > img.cols - 1 || _y < 0 || _y > img.rows - 1) {
			//bReachEnd = true;
			continue;
		}
		if (img.at<uchar>(_y, _x) >= THRESHOLD) {
			target.push_back(Point(_x, _y));
		}
	}
	//cout << "target count " << target.size() << endl;
	if (target.size() > 1) {
		for (auto p : target) {
			vector<Point>::iterator _itr;
			bool ret = find_path(img, p, end, (p.y - start.y + 1) * 3 + p.x - start.x + 1, result_points, _itr, false);
			if (!ret) {
				//result_points.erase(_itr, result_points.end());
			}
			else
				return true;
		}
		return false;
	}
	else if (target.size() == 0) {
		bDisconnect = true;
		cout << "1 disconnect count:" << endl;
		return false;
	}

	//if (breachend) {
	//	cout << "reach end" << endl;
	//	return false;
	//}
	Point _p = start;
	Point p = target[0];
	while(true){
		bool ret;
		//cout << "2 (" << p.x << "," << p.y << ")" << endl;

		result_points.push_back(p);
		dir = (p.y - _p.y + 1) * 3 + p.x - _p.x + 1;
		_direction = pri_direction[dir];
		target.clear();
		for (int i = 0; i < _direction.count; i++) {
			int _x, _y;
			_x = p.x + _direction.val[i].x;
			_y = p.y + _direction.val[i].y;
			if (_x < 0 || _x > img.cols - 1 || _y < 0 || _y > img.rows - 1) {
				//bReachEnd = true;
				continue;
			}
			if (img.at<uchar>(_y, _x) >= THRESHOLD) {
				target.push_back(Point(_x, _y));
			}
		}
		//cout << "2 target count " << target.size() << endl;

		if (target.size() > 1) {
			for (auto pp : target) {
				vector<Point>::iterator _itr;
				bool ret = find_path(img, pp, end, (pp.y - p.y + 1) * 3 + pp.x - p.x + 1, result_points, _itr, false);
				if (!ret) {
					//result_points.erase(_itr, result_points.end());
				}
				else
					return true;
			}
			return false;
		}
		else if (target.size() == 0) {
			bDisconnect = true;
			cout << "2 disconnect count:" << endl;
			return false;
		}

		//cout << "(" << x << "," << y << ")" << endl;
		_p = p;
		p = target[0];
		//dir = (p.y - _p.y + 1) * 3 + p.x - _p.x + 1;



#if 0
		if (bDisconnect) {
			disconnet_count++;
			cout << "disconnect count:" << disconnet_count << endl;
			switch (dir) {
			case 0:
			case 2:
			case 6:
			case 8:
				cout << "not found point" << endl;
				return false;
			case 1: {
				result_points.push_back(Point(start.x, start.y - 1));
				vector<Point>::iterator _itr;
				ret = find_path(img, Point(start.x, start.y - 1), end, dir, result_points, _itr, false);
				if (!ret) {
					result_points.erase(_itr, result_points.end());
				}
				return ret;
			}
			case 3: {
				vector<Point>::iterator _itr;
				ret = find_path(img, Point(start.x - 1, start.y), end, dir, result_points, _itr, false);
				if (!ret) {
					result_points.erase(_itr, result_points.end());
				}
				return ret;
			}
			case 5: {
				vector<Point>::iterator _itr;
				ret = find_path(img, Point(start.x + 1, start.y), end, dir, result_points, _itr, false);
				if (!ret) {
					result_points.erase(_itr, result_points.end());
				}
				return ret;
			}
			case 7: {
				vector<Point>::iterator _itr;
				ret = find_path(img, Point(start.x, start.y + 1), end, dir, result_points, _itr, false);
				if (!ret) {
					result_points.erase(_itr, result_points.end());
				}
				return ret;
			}
			}	//switch
		}
#endif

	}



}

bool find_outline(Mat& img, Point center, vector<detectedLine>& detected_lines, vector<Point>& result_points)
{
#define THRESHOLD 150
	//find point
	int min = 1000;
	int y;
	int x = center.x;
	for (auto line : detected_lines) {
		int _y;
		_y = line.slope * x + line.intercept_y;
		float d = abs(_y - center.y);
		if (min > d) {
			min = d;
			y = _y;
		}
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
	int dir;
	if (start.y < center.y)
		dir = 5;
	else
		dir = 3;

	vector<Point>::iterator _itr;
	bool ret = find_path(img, start, start, dir, result_points, _itr, true);
	
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
	"points_174_UL1.xml",
	//"points_174_UR1.xml",
	//"points_174_D.xml",
	//"points_174_DL1.xml",
	//"points_174_DR1.xml"
};

int main()
{
	cv::Mat image; // create an empty image
	image = cv::imread("capture_174.jpg");
	if (image.empty()) { // error handling
						 // no image has been created…
						 // possibly display an error message
						 // and quit the application
		cv::waitKey(0);
	}

	//getPoints("points_174_UL1.xml", points, board_size, plate_size);
	//getPoints("points_174_UR1.xml", points, board_size, plate_size);
	//getPoints("points_174_D.xml", points, board_size, plate_size);
	//getPoints("points_174_DL1.xml", points, board_size, plate_size);
	//getPoints("points_174_DR1.xml", points, board_size, plate_size);
	for (int i = 0; i < sizeof(points_xml) / sizeof(char*); i++) {
		test(image, points_xml[i], i);
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

