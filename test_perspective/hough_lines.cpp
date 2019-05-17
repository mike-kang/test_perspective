#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

vector<Point> vec_point;

void hough_coord(Mat image, Mat& acc_mat, double rho, double theta)
{
	int  acc_h = int((image.rows + image.cols) * 2 / rho);
	int  acc_w = int(CV_PI / theta);

	acc_mat = Mat(acc_h, acc_w, CV_32S, Scalar(0));

	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++)
		{
			Point pt(j, i);
			if (image.at<uchar>(pt) > 0) {

				for (int t = 0; t < acc_w; t++)
				{
					double radian = t * theta;
					double r = pt.x * cos(radian) + pt.y * sin(radian);
					r = cvRound(r / rho + acc_mat.rows / 2.0);
					acc_mat.at<int>( (int)r, t)++;
				}

			}
		}
	}
}
void acc_mask(Mat acc_mat, Mat& acc_dst, Size size, int thresh)
{
	acc_dst = Mat(acc_mat.size(), CV_32S, Scalar(0));
	Point  h_m = size / 2;	// 마스크 크기 절반
#if 1
	/*
	Size sum_mask_size = Size(5, 5);
	Point  h_sum_m = sum_mask_size / 2;	// 마스크 크기 절반
	Mat sum_mat = Mat(acc_mat.size(), CV_32S, Scalar(0));
	for (int r = h_sum_m.y; r < acc_mat.rows - h_sum_m.y; r++) {
		for (int t = h_sum_m.x; t < acc_mat.cols - h_sum_m.x; t++)
		{
			Point left_top = Point(t, r) - h_sum_m;
			
			int sum = 0;
			for (int u = 0; u < sum_mask_size.height; u++) {
				for (int v = 0; v < sum_mask_size.width; v++)
				{
					Point p = left_top + Point(v, u);
					sum += acc_mat.at<int>(p);
				}
			}
			sum_mat.at<int>(Point(t,r)) = sum;


		}
	}
	*/
	for (int r = h_m.y; r < acc_mat.rows - h_m.y; r++) {
		for (int t = h_m.x; t < acc_mat.cols - h_m.x; t++)
		{
			Point left_top = Point(t, r) - h_m;
			int c_value = acc_mat.at<int>(left_top);
			if (c_value >= thresh)
			{
				double maxVal = 0;
				for (int u = 0; u < size.height; u++) {
					for (int v = 0; v < size.width; v++)
					{
						Point start = left_top + Point(v, u);
						if (start != left_top && acc_mat.at<int>(start) > maxVal)
							maxVal = acc_mat.at<int>(start);
					}
				}

				Rect rect(left_top, size);
				if (c_value >= maxVal)
				{
					acc_dst.at<int>(left_top) = c_value;
					acc_mat(rect).setTo(0);
				}
			}
		}
	}
#else
	multimap<int, Point> acc_rank;
	for (int r = h_m.y; r < acc_mat.rows - h_m.y; r++) {
		for (int t = h_m.x; t < acc_mat.cols - h_m.x; t++)
		{
			Point center = Point(t, r) - h_m;
			//int c_value = acc_mat.at<int>(center);	// 중심화소
			//if (c_value >= thresh)
			{
				int sum = 0;
				for (int u = 0; u < size.height; u++) {
					for (int v = 0; v < size.width; v++)
					{
						Point p = center + Point(v, u);
						sum += acc_mat.at<int>(p);
					}
				}
				acc_rank.insert(pair<int, Point>(sum, Point(t, r)));
				if (acc_rank.size() > 5)
					acc_rank.erase(acc_rank.begin());
			}
		}
	}




	cout << "acc_rank count : " << acc_rank.size() << endl;
#endif
}


void thres_lines(Mat acc_dst, Mat& lines, double _rho, double theta, int thresh)
{
	for (int r = 0; r < acc_dst.rows; r++) {
		for (int t = 0; t < acc_dst.cols; t++)
		{
			float value = (float)acc_dst.at<int>(r, t);			// 누적값
			if (value >= thresh)								// 직선 길이 임계값
			{
				float rho = (float)((r - acc_dst.rows / 2) * _rho);		// 수직거리
				float radian = (float)(t * theta);						// 각도

				Matx13f line(rho, radian, value); 				// 단일 직선
				lines.push_back((Mat)line);
			}
		}
	}
}
#ifdef _LINE
void thres_lines(Mat acc_dst, Mat& lines, double _rho, double theta, int thresh)
{
	for (int r = 0; r < acc_dst.rows; r++) {
		for (int t = 0; t < acc_dst.cols; t++)
		{
			float value = (float)acc_dst.at<int>(r, t);			// 누적값
			if (value >= thresh)								// 직선 길이 임계값
			{
				float rho = (float)((r - acc_dst.rows / 2) * _rho);		// 수직거리
				float radian = (float)(t * theta);						// 각도

				Matx13f line(rho, radian, value); 				// 단일 직선
				lines.push_back((Mat)line);
			}
		}
	}
}
#endif
void sort_lines(Mat lines, vector<Vec3f>& s_lines )
{
	Mat acc = lines.col(2), idx;
	sortIdx(acc, idx, SORT_EVERY_COLUMN + SORT_DESCENDING);
	
	for (int i = 0; i < idx.rows; i++)
	{
		int id = idx.at<int>(i);
		float rho    = lines.at<float>(id, 0);		// 수직거리
		float radian = lines.at<float>(id, 1);
		float count = lines.at<float>(id, 2);
		s_lines.push_back( Vec3f(rho,radian, count));
	}
}

#ifdef _LINE
void sort_lines(Mat lines, vector<Line>& s_lines)
{
	Mat acc = lines.col(2), idx;
	sortIdx(acc, idx, SORT_EVERY_COLUMN + SORT_DESCENDING);

	for (int i = 0; i < idx.rows; i++)
	{
		int id = idx.at<int>(i);
		float rho = lines.at<float>(id, 0);		// 수직거리
		float radian = lines.at<float>(id, 1);
		float count = lines.at<float>(id, 2);
		//s_lines.push_back(Vec3f(rho, radian, count));
	}
}

struct Line {
	float rho;
	float theta;
	vector<Point> points;
};
#endif
void houghLines(Mat src, vector<Vec3f>& s_lines, double rho, double theta, int thresh)
{
	Mat  acc_mat, acc_dst, lines;
	hough_coord(src, acc_mat, rho, theta);
	acc_mask(acc_mat, acc_dst, Size(3, 7), thresh);

	thres_lines(acc_dst, lines, rho, theta, thresh);
	sort_lines(lines, s_lines);
}
#ifdef _LINE

void houghLines(Mat src, vector<Line>& s_lines, double rho, double theta, int thresh)
{
	Mat  acc_mat, acc_dst, lines;
	hough_coord(src, acc_mat, rho, theta);
	acc_mask(acc_mat, acc_dst, Size(3, 7), thresh);

	thres_lines(acc_dst, lines, rho, theta, thresh);
	sort_lines(lines, s_lines);
}
#endif
/*

void draw_houghLines(Mat image, Mat& dst, vector<Vec2f> lines, int nline)
{
	if (image.channels() == 3)	image.copyTo(dst);
	else 						cvtColor(image, dst, COLOR_GRAY2BGR);

	for (int i = 0; i < min((int)lines.size(), nline); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		double a = cos(theta), b = sin(theta);

		Point2d delta(1000 * -b, 1000 * a);
		Point2d pt(a*rho, b*rho);
		line(dst, pt + delta, pt - delta, Scalar(0, 255, 0), 1, LINE_AA);
	}
}

int main()
{

	Mat image = imread("../image/hough_test.jpg", 0);
	CV_Assert(image.data);
	
	Mat canny, dst1, dst2;
	GaussianBlur(image, canny, Size(5, 5), 2, 2);
	Canny(canny, canny, 100, 150, 3);

	double rho = 1, theta = CV_PI / 180;

	vector<Vec2f> lines1, lines2;
	houghLines(canny, lines1, rho, theta, 50);
	HoughLines(canny, lines2, rho, theta, 50);
	draw_houghLines(canny, dst1, lines1, 10);
	draw_houghLines(canny, dst2, lines2, 10);

	imshow("source", image);
	imshow("canny", canny);
	imshow("detected lines", dst1);
	imshow("detected lines_OpenCV", dst2);
	waitKey();
}


*/