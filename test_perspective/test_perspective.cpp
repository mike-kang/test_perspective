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


void pointsPushRectGreen(Mat img, vector<Point2i>& points, Rect rect, bool (*font_func)(Vec3b), bool(*bg_func)(Vec3b))
{
	Point2i point_delta;

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
void calSeedPoint(Point2i pt, Point2i direct, Point2i& seedPoint)
{
	float k;
	Vec2i c = direct - pt;
	k = min(abs(1.2 / c[0]), abs(1.2 / c[1]));
	seedPoint.x = pt.x + k * c[0];
	seedPoint.y = pt.y + k * c[1];
}

void findFloodFillTargetWhite(Mat img, Mat dewarped, Rect leftRect, Rect rightRect, Dewarp& dewarpObj, FloodFill::FF_Point* ffpointsInImage)
{
	#define V_THRESHOLD_OFFSET 20
	float s;
	int h;
	Point2i leftRectPoints[4] = { leftRect.tl(), leftRect.tl() + Point2i(leftRect.width, 0), leftRect.br(), leftRect.br() - Point2i(leftRect.width, 0) };
	Point2i RightRectPoints[4] = { rightRect.tl(), rightRect.tl() + Point2i(rightRect.width, 0), rightRect.br(), rightRect.br() - Point2i(rightRect.width, 0) };
	FloodFill::FF_Point* ffp = &ffpointsInImage[0];
	//lefttop
	dewarpObj.calImagePoint(leftRectPoints[3], ffp->pt[0]);
	dewarpObj.calImagePoint(leftRectPoints[0] + Point2i(1, 1), ffp->pt[1]);
	dewarpObj.calImagePoint(leftRectPoints[1], ffp->pt[2]);
	if (ffp->pt[0].x == ffp->pt[1].x) {
		if (ffp->pt[0].y > ffp->pt[1].y)
			ffp->pt[0].x--;
		else
			ffp->pt[0].x++;
	}
	if (ffp->pt[1].x == ffp->pt[2].x) {
		if (ffp->pt[1].y < ffp->pt[2].y)
			ffp->pt[2].x++;
		else
			ffp->pt[2].x--;
	}
	dewarpObj.calImagePoint(leftRectPoints[0] + Point2i(-5, -5), ffp->directPoint);
	calSeedPoint(ffp->pt[1], ffp->directPoint, ffp->seedPoint);
	//getColor(img, ffp->seedPoint, h, s, ffp->v_threshold);
	//ffp->v_threshold -= V_THRESHOLD_OFFSET;
	//ffp->print();
	//righttop
	ffp = &ffpointsInImage[1];
	dewarpObj.calImagePoint(RightRectPoints[0], ffp->pt[0]);
	dewarpObj.calImagePoint(RightRectPoints[1] + Point2i(-1, 1), ffp->pt[1]);
	dewarpObj.calImagePoint(RightRectPoints[2], ffp->pt[2]);
	if (ffp->pt[0].x == ffp->pt[1].x) {
		if (ffp->pt[0].y > ffp->pt[1].y)
			ffp->pt[0].x--;
		else
			ffp->pt[0].x++;
	}
	if (ffp->pt[1].x == ffp->pt[2].x) {
		if (ffp->pt[1].y < ffp->pt[2].y)
			ffp->pt[2].x++;
		else
			ffp->pt[2].x--;
	}
	dewarpObj.calImagePoint(RightRectPoints[1] + Point2i(5, -5), ffp->directPoint);
	calSeedPoint(ffp->pt[1], ffp->directPoint, ffp->seedPoint);
	//getColor(img, ffp->seedPoint, h, s, ffp->v_threshold);
	//ffp->v_threshold -= V_THRESHOLD_OFFSET;
	//ffp->print();

	//rightbottom
	ffp = &ffpointsInImage[2];
	dewarpObj.calImagePoint(RightRectPoints[1], ffp->pt[0]);
	dewarpObj.calImagePoint(RightRectPoints[2] + Point2i(-1, -1), ffp->pt[1]);
	dewarpObj.calImagePoint(RightRectPoints[3], ffp->pt[2]);
	if (ffp->pt[0].x == ffp->pt[1].x) {
		if (ffp->pt[0].y > ffp->pt[1].y)
			ffp->pt[0].x--;
		else
			ffp->pt[0].x++;
	}
	if (ffp->pt[1].x == ffp->pt[2].x) {
		if (ffp->pt[1].y < ffp->pt[2].y)
			ffp->pt[2].x++;
		else
			ffp->pt[2].x--;
	}
	dewarpObj.calImagePoint(RightRectPoints[2] + Point2i(5, 5), ffp->directPoint);
	calSeedPoint(ffp->pt[1], ffp->directPoint, ffp->seedPoint);
	getColor(img, ffp->seedPoint, h, s, ffp->v_threshold);
	//ffp->v_threshold -= V_THRESHOLD_OFFSET;
	//ffp->print();

	//leftbottom
	ffp = &ffpointsInImage[3];
	dewarpObj.calImagePoint(leftRectPoints[2], ffp->pt[0]);
	dewarpObj.calImagePoint(leftRectPoints[3] + Point2i(1, -1), ffp->pt[1]);
	dewarpObj.calImagePoint(leftRectPoints[0], ffp->pt[2]);
	if (ffp->pt[0].x == ffp->pt[1].x) {
		if (ffp->pt[0].y > ffp->pt[1].y)
			ffp->pt[0].x--;
		else
			ffp->pt[0].x++;
	}
	if (ffp->pt[1].x == ffp->pt[2].x) {
		if (ffp->pt[1].y < ffp->pt[2].y)
			ffp->pt[2].x++;
		else
			ffp->pt[2].x--;
	}
	dewarpObj.calImagePoint(leftRectPoints[3] + Point2i(-5, 5), ffp->directPoint);
	calSeedPoint(ffp->pt[1], ffp->directPoint, ffp->seedPoint);
	//getColor(img, ffp->seedPoint, h, s, ffp->v_threshold);
	//ffp->v_threshold -= V_THRESHOLD_OFFSET;
	//ffp->print();

}

bool dewarpPlate(Mat img, vector<Point2i>& warp_plate_points, Mat& plate, Point2i center)
{
	Size ori_plate_size(496, 100);
	float factor = 28. / 83;
	Size plate_size(496 * factor, 104 * factor);
	Dewarp dewarpObj(Dewarp::CameraParam(950, center));

	vector<Point2f> plate_points;

	plate_points.push_back(Point2f{ 0,0 });
	plate_points.push_back(Point2f{ (float)plate_size.width - 1, 0 });
	plate_points.push_back(Point2f{ (float)plate_size.width - 1, (float)plate_size.height - 1});
	plate_points.push_back(Point2f{ (float)0, (float)plate_size.height - 1});

	vector<Point2f> warp_plate_points_f;
	for (auto p : warp_plate_points) {
		warp_plate_points_f.push_back(p);
	}

	dewarpObj.calHomography(plate_points, warp_plate_points_f);
	//cout << "homography" << endl << homograpy << endl;

	plate.create(plate_size, CV_8UC3);

	dewarpObj.makeImage(img, plate);
#if 1
#define TOP 4
	Rect rects[] = {
		Rect(Point2i(11, TOP), Size(19, 28)),
		Rect(Point2i(11 + 19, TOP), Size(19, 28)),
		Rect(Point2i(11 + 19 * 2, TOP), Size(32, 28)),
		Rect(Point2i(11 + 19 * 2 + 32, TOP), Size(19, 28)),
		Rect(Point2i(11 + 19 * 3 + 32, TOP), Size(19, 28)),
		Rect(Point2i(11 + 19 * 4 + 32, TOP), Size(19, 28)),
		Rect(Point2i(11 + 19 * 5 + 32, TOP), Size(19, 28))
	};
	for (auto rect : rects) {
		rectangle(plate, rect.tl(), rect.br(), Scalar(0, 255, 0), 1, 8, 0);
	}
#endif
	return true;
}

bool findPlate(Mat& image, Point2i center, vector<Point2f> warp_points, Size board_size_mm, Mat& plate, int idx)
{
	float factor;
	Dewarp dewarpObj(Dewarp::CameraParam(950, center));
	cout << endl;
	cout << "******************************************************************* " << endl;
	cout << "*****                    findPlate " << idx << "                         ****** " << endl;
	cout << "******************************************************************* " << endl;

	vector<Point2f> dst_points;
	//Size board_size_mm/*(mm,mm)*/, plate_size;
	Size board_size;

	float effective_pixel = calArea(warp_points);
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

	dewarpObj.calHomography(dst_points, warp_points);
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
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(canny, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

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

	//포함되는 것은 제거하자.
	vector<Rect> boundRect3; 
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

	Mat test_img(image.size(), CV_8UC3);
	for (auto group : groups) {
		//FloodFill
		group.sort();
		Rect leftRect = group.getLeftMember();
		Rect rightRect = group.getRightMember();
		//vector<PointEx> pointsInFrameEx, pointsInImageEx;
		bool(*bg_func)(Vec3b, uchar);	//bg 판별식
		vector<Point2i> pointsInFrame, pointsInImage;
		FloodFill::FF_Point ffpointsInImage[4];
		if (group.m_bgColor == Group::BG_WHITE) {
			bg_func = isBG_WHITE_ColorEx;
			findFloodFillTargetWhite(image, dewarped, leftRect, rightRect, dewarpObj, ffpointsInImage);	//foloodfill할 4점에 대한 정보를 얻는다.
		}
		else {
			cout << "Green is skip!!!!!!" << endl;
			return false;
			//bg_func = isBG_GREEN_Color;
			//pointsPushRectGreen(dewarped, pointsInFrame, leftRect, is_GREEN_FontColor, isBG_GREEN_Color, position);
			//pointsPushRectGreen(dewarped, pointsInFrame, rightRect, is_GREEN_FontColor, isBG_GREEN_Color, position);
		}

		//FloodFill
		FloodFill floodfill(image, bg_func, true);
		vector<Point2i> ffpointsResult[4];
		for (int i = 0; i < 4; i++) {
			cout << "<<<<<<<<<<<<<<<<<floodfill.process " << i << endl;
			int error = floodfill.process(ffpointsInImage[i], &ffpointsResult[i], 120, 40);
			//cout << "v_diff 40 point count :" << ffpointsResult[i].size() << endl;
			if (error) {
				if (error == 3) {
					ffpointsResult[i].clear();
					cout << "flood fill error 3, retry v_diff=20" << endl;
					error = floodfill.process(ffpointsInImage[i], &ffpointsResult[i], 120, 20);
				}
				else {
					cout << "flood fill error: " << error << endl;
					return false;
				}
			}
			//cout << "v_diff 20 point count :" << ffpointsResult[i].size() << endl;

			if (error) {
				cout << "flood fill error[" <<  i << "] :" << error << endl;
			}

			for (auto p : ffpointsResult[i]) {
				test_img.at<cv::Vec3b>(p) = cv::Vec3b(0, 0, 255);
			}
		}
		vector<Point2i> warp_plate_points(4);
		Point2i _center[4] = { Point2i(0,0), Point2i(0,0),Point2i(0,0),Point2i(0,0) };
		for (int i = 0; i < 4; i++) {
			for (Point2i p : ffpointsResult[i]) {
				_center[i] += p;
			}
			int count = ffpointsResult[i].size();
			_center[i] /= count;
		}

		for (int i = 0; i < 4; i++) {
			Point2i __center = _center[(i + 2) % 4];
#if 0	//dewarp해서 모서리점을 찾기.
			Point2i p = dst_points[i];
			vector<Point2i> dewarp_points;
			
			dewarpObj.calDewarpPoints(ffpointsResult[i], dewarp_points);

			Point2i dewarp_min = *min_element(dewarp_points.begin(), dewarp_points.end(), [p](Point2i a, Point2i b) -> bool {
				return ((a.x - p.x) *(a.x - p.x )+ (a.y - p.y)*(a.y - p.y)) < ((b.x - p.x)*(b.x - p.x)+ (b.y - p.y)*(b.y - p.y));
			});
			cout << "min: " << dewarp_min << endl;
			circle(dewarped, dewarp_min, 1, Scalar(255, 0, 0), 2);
			for (auto po : dewarp_points) {
				dewarped.at<Vec3b>(po) = Vec3b(0, 0, 255);
			}

#else	//warp된 상태에서 모서리 찾기.
#if 1	//board 모서리와 가까운 점을 찾기.
			Point2i p = warp_points[i];
			warp_plate_points[i] = *min_element(ffpointsResult[i].begin(), ffpointsResult[i].end(), [p](Point2i a, Point2i b) -> bool {
				return ((a.x - p.x) *(a.x - p.x) + (a.y - p.y)*(a.y - p.y)) < ((b.x - p.x)*(b.x - p.x) + (b.y - p.y)*(b.y - p.y));
			});
			for (auto po : ffpointsResult[i]) {
				image.at<Vec3b>(po) = Vec3b(0, 0, 255);
			}
			circle(image, warp_plate_points[i], 1, Scalar(255, 0, 0), 1);
			circle(image, p, 1, Scalar(0, 0, 255), 2);
#else	//중심에서 먼점을 찾기.
			warp_plate_points[i] = *max_element(ffpointsResult[i].begin(), ffpointsResult[i].end(), [__center](Point2i a, Point2i b) -> bool {
				return ((a.x - __center.x) *(a.x - __center.x) + (a.y - __center.y)*(a.y - __center.y)) < ((b.x - __center.x)*(b.x - __center.x) + (b.y - __center.y)*(b.y - __center.y));
			});

			circle(image, warp_plate_points[i], 1, Scalar(255, 0, 0), 1);

#endif
#endif
		}
		
		//cv::imshow("dewarped_" + std::to_string(idx), dewarped);

#if 0 //debug
		for (auto ffp : ffpointsInImage) {
			circle(image, ffp.seedPoint, 1, Scalar(255, 0, 0), 1);
			circle(image, ffp.directPoint, 1, Scalar(0, 0, 255), 1);
			circle(image, ffp.pt[0], 1, Scalar(0, 255, 0), 1);
			circle(image, ffp.pt[1], 1, Scalar(0, 255, 0), 1);
			circle(image, ffp.pt[2], 1, Scalar(0, 255, 0), 1);
		}
#endif
		dewarpPlate(image, warp_plate_points, plate, center);

		//Mat plate2;
		//resize(plate, plate2, Size(plate.cols * 2, plate.rows * 2));

		cout << "Only best group check!!!!!!" << endl;
		break;



	}	//for groups
	cv::imwrite("test_img.jpg", test_img);
	return true;

}

void getConfig(const char* filename, Point2i& center, vector<Point2f>& points, Size& board_size)
{
	FileStorage fs_P(filename, FileStorage::READ);
	CV_Assert(fs_P.isOpened());
	fs_P["center"] >> center;
	fs_P["points"] >> points;
	fs_P["board_size"] >> board_size;

	fs_P.release();
}

int main()
{
	cv::Mat image; // create an empty image
	
	image = cv::imread("capture_174_2.jpg");
	//image = cv::imread("192.168.10.41.jpg");
	if (image.empty()) { // error handling
						 // no image has been created…
						 // possibly display an error message
						 // and quit the application
		cv::waitKey(0);
	}

	vector<Point2f> points;
	Point2i center;
	Size boardSize;
	getConfig("config_174.xml", center, points, boardSize);
	//getConfig("points_41.xml", center, points, boardSize);
	int count = points.size() / 4;
	Mat plate;
	for (int i = 0; i < count; i++) {
		vector<Point2f> warped_frame_points(4);
		std::copy(points.begin() + i * 4, points.begin() + i * 4 + 4, warped_frame_points.begin());
		if (findPlate(image, center, warped_frame_points, boardSize, plate, i)) {
			cout << "findPlate Success!!!!" << endl;
			cv::imshow("plate_" + std::to_string(i), plate);

		}
		else {
			cout << "findPlate Fail!!!!" << endl;
		}
	}

	cv::imwrite("result.jpg", image);


	cv::waitKey(0);
    return 0;
}

