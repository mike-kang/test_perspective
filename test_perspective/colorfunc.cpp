#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

bool isBG_GREEN_Color(Vec3b color)
{
	uchar b = color[0];
	uchar g = color[1];
	uchar r = color[2];

	float h, delta, max;
	if (b > g) {
		if (b > r) {
			max = b;
			if (g > r) {
				delta = b - r;
			}
			else {
				delta = b - g;
			}
			h = 60 * ((r - g) / delta + 4);
		}
		else {
			//r > b > g
			max = r;
			delta = b - g;
			h = 60 * ((int)((g - b) / delta) % 6);
		}
	}
	else {
		if (g > r) {
			//g
			max = g;
			if (b > r) {	//g > b> r
				delta = g - r;
			}
			else {			//g > r > b
				delta = g - b;
			}
			h = 60 * ((b - r) / delta + 2);
		}
		else {
			max = r;
			//r > g > b
			delta = r - b;
			h = 60 * ((int)((g - b) / delta) % 6);
		}


	}
	float s = (max != 0) ? delta / max : 0;
	float v = max;
	if (h < 0)
		h += 360;
	//hsv[0] = h / 2;
	//hsv[1] = s * 100;
	//hsv[2] = v * 100;
	//if (debug)
	//	cout << h << " " << s << " " << v << endl;
	if ((s > 0.22 && h >= 70 && h <= 180)
		|| (s < 0.22 && v < 100)) {
		return true;
	}
	return false;
}
bool is_GREEN_FontColor(Vec3b color)
{
	uchar b = color[0];
	uchar g = color[1];
	uchar r = color[2];

	float h, delta, max;
	if (b > g) {
		if (b > r) {
			max = b;
			if (g > r) {
				delta = b - r;
			}
			else {
				delta = b - g;
			}
			h = 60 * ((r - g) / delta + 4);
		}
		else {
			//r > b > g
			max = r;
			delta = b - g;
			h = 60 * ((int)((g - b) / delta) % 6);
		}
	}
	else {
		if (g > r) {
			//g
			max = g;
			if (b > r) {	//g > b> r
				delta = g - r;
			}
			else {			//g > r > b
				delta = g - b;
			}
			h = 60 * ((b - r) / delta + 2);
		}
		else {
			max = r;
			//r > g > b
			delta = r - b;
			h = 60 * ((int)((g - b) / delta) % 6);
		}


	}
	if (h < 0)
		h += 360;
	float s = (max != 0) ? delta / max : 0;
	float v = max;
	cout << h << " " << s << " " << v << endl;
	if (s < 0.22 && v > 150) {
		return true;
	}
	return false;
}

bool is_GREEN_Color(Vec3b color)
{
	uchar b = color[0];
	uchar g = color[1];
	uchar r = color[2];

	float h, delta, max;
	if (b > g) {
		if (b > r) {
			max = b;
			if (g > r) {
				delta = b - r;
			}
			else {
				delta = b - g;
			}
			h = 60 * ((r - g) / delta + 4);
		}
		else {
			//r > b > g
			max = r;
			delta = b - g;
			h = 60 * ((int)((g - b) / delta) % 6);
		}
	}
	else {
		if (g > r) {
			//g
			max = g;
			if (b > r) {	//g > b> r
				delta = g - r;
			}
			else {			//g > r > b
				delta = g - b;
			}
			h = 60 * ((b - r) / delta + 2);
		}
		else {
			max = r;
			//r > g > b
			delta = r - b;
			h = 60 * ((int)((g - b) / delta) % 6);
		}


	}
	float s = (max != 0) ? delta / max : 0;
	float v = max;
	if (h < 0)
		h += 360;
	//hsv[0] = h / 2;
	//hsv[1] = s * 100;
	//hsv[2] = v * 100;
	//if (debug)
	//	cout << h << " " << s << " " << v << endl;
	if (s > 0.22 && h >= 70 && h <= 180) {
		return true;
	}
	return false;
}
bool isBG_WHITE_Color(Vec3b color)
{
	uchar b = color[0];
	uchar g = color[1];
	uchar r = color[2];


	uchar max = std::max(b, g);
	max = std::max(max, r);
	uchar min = std::min(b, g);
	min = std::min(min, r);
	float delta = max - min;
	float s = (max != 0) ? delta / max : 0;
	float v = max;
	if (s < 0.196 && v >= 150) {
		return true;
	}
	return false;
}