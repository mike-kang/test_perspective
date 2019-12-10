// testDewarp.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include "Dewarp.h"

using namespace cv;
using namespace std;

int main()
{
	cv::Mat image; // create an empty image
	image = cv::imread("capture_174_1.jpg");
	if (image.empty()) { // error handling
						 // no image has been created…
						 // possibly display an error message
						 // and quit the application
		cv::waitKey(0);
	}

	//Size board_size(340, 175);
	Size board_size(136, 70);

	vector<Point2f> img_points;
	vector<Point2f> frame_points;

	img_points.push_back(Point2f{1911, 2490});
	img_points.push_back(Point2f{1816 , 2487});
	img_points.push_back(Point2f{1820 , 2458});
	img_points.push_back(Point2f{1911 , 2462});

	frame_points.push_back(Point2f{ 0,0 });
	frame_points.push_back(Point2f{ (float)board_size.width - 1, (float)0 });
	frame_points.push_back(Point2f{ (float)board_size.width - 1, (float)board_size.height - 1});
	frame_points.push_back(Point2f{ (float)0, (float)board_size.height - 1 });

	Point2f center;
	center.x = 1950;
	center.y = 1511;
	Dewarp dewarp(Dewarp::CameraParam(950, center));
	dewarp.calHomography(frame_points, img_points);
	Mat dewarped(board_size.height, board_size.width, CV_8UC3);
	dewarp.makeImage(image, dewarped);
	cv::imshow("dewarped", dewarped);
	imwrite("dewarped.jpg", dewarped);
	cv::waitKey(0);

    return 0;
}

