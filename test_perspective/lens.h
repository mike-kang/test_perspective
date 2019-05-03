#define M_PI 3.141592
#include "lens_transmission_angles.h"
#include <math.h>
//float arr_k[] = { 0,0.014835185,0.02966982,0.044503328,0.059335172,0.074164774,0.088991598,0.103815088,0.118634685,0.133449855,0.148260045,0.163064721,0.177863338,0.192655374,0.207440298,0.222217595,0.236986742,0.251747242,0.26649858,0.281240267,0.29597182,0.310692757,0.325402603,0.340100902,0.3547872,0.369461055,0.384122039,0.398769717,0.413403696,0.428023568,0.442628946,0.457219456,0.471794746,0.486354454,0.500898263,0.515425838,0.529936889,0.544431122,0.55890826,0.573368052,0.587810259,0.602234664,0.61664105,0.631029246,0.645399078,0.659750394,0.67408308,0.688397026,0.702692139,0.716968356,0.731225644,0.745463975,0.75968335,0.773883802,0.78806538,0.80222815,0.816372222,0.830497718,0.84460478,0.858693592,0.872764358,0.886817302,0.900852681,0.914870782,0.92887191,0.94285641,0.956824644,0.970777019,0.984713948,0.998635894,1.012543326,1.02643676,1.040316738,1.054183826,1.068038613,1.081881717,1.095713799,1.109535515,1.123347581,1.137150701,1.150945637,1.164733149,1.178514032,1.192289074,1.206059111,1.219824977,1.233587508,1.24734756,1.261106003,1.274863678,1.288621455,1.302380182,1.316140683,1.329903785,1.343670275,1.357440916,1.371216434,1.384997501,1.398784734,1.412578706,1.42637989,1.440645405,	1.454909221,1.469173037,1.483436853,1.497700669,1.511964485 };
//float arr_angles[] = { 0,0.014835299,0.029670597,0.044505896,0.059341195,0.074176493,0.089011792,0.10384709,0.118682389,0.133517688,0.148352986,0.163188285,0.178023584,0.192858882,0.207694181,0.22252948,0.237364778,0.252200077,0.267035376,0.281870674,0.296705973,0.311541271,0.32637657,0.341211869,0.356047167,0.370882466,0.385717765,0.400553063,0.415388362,0.430223661,0.445058959,0.459894258,0.474729557,0.489564855,0.504400154,0.519235452,0.534070751,0.54890605,0.563741348,0.578576647,0.593411946,0.608247244,0.623082543,0.637917842,0.65275314,0.667588439,0.682423738,0.697259036,0.712094335,0.726929633,0.741764932,0.756600231,0.771435529,0.786270828,0.801106127,0.815941425,0.830776724,0.845612023,0.860447321,0.87528262,0.890117919,0.904953217,0.919788516,0.934623814,0.949459113,0.964294412,0.97912971,0.993965009,1.008800308,1.023635606,1.038470905,1.053306204,1.068141502,1.082976801,1.0978121,1.112647398,1.127482697,1.142317995,1.157153294,1.171988593,1.186823891,1.20165919,1.216494489,1.231329787,1.246165086,1.261000385,1.275835683,1.290670982,1.30550628,1.320341579,1.335176878,1.350012176,1.364847475,1.379682774,1.394518072,1.409353371,1.42418867,1.439023968,1.453859267,1.468694566,1.483529864, 1.498365163, 1.513200461,	1.52803576,	1.542871059, 1.557706357, 1.572541656 };
//const int arr_count = 101 + 6;
//const int fp = 671;
#define FOCAL_LENGTH 1.47232
#define ROOT_2 sqrt(2)
#define DEG_TO_RADIAN(x) ((x) * M_PI / 180) 
#define RADIAN_TO_DEG(x) ((x) * 180 / M_PI) 
//return k (height/focal length)
static bool findk(float angle/*radian*/, float& k)
{
	float deg_angle = RADIAN_TO_DEG(angle);
	if (deg_angle < LENS_ANGLES_START_OFFSET) {
		k = angle;
		return true;
	}

	int i = (deg_angle - LENS_ANGLES_START_OFFSET)/ SAMPLE_T;
	if (i < ANGLES_COUNT - 1) {
		float delta = (deg_angle - LENS_ANGLES_START_OFFSET - SAMPLE_T * i)*(arr_k[i + 1] - arr_k[i]) / SAMPLE_T;
		k = arr_k[i] + delta;
		return true;
	}
	k = arr_k[ANGLES_COUNT - 1];
	return false;
}

//return incidence angle (�Ի簢)
static bool findAngle(float k, float& angle)
{
	bool bFound = false;
	float transmission_angle = atan(k);
	float deg_angle;
	if (transmission_angle == 0.) {
		angle = 0;
		bFound = true;
	}
	else {
		for (int i = 1; i < ANGLES_COUNT; i++) {
			if (transmission_angle < transmission_angles[i]) {
				float delta = (transmission_angle - transmission_angles[i - 1])* SAMPLE_T / (transmission_angles[i] - transmission_angles[i - 1]);
				deg_angle = LENS_ANGLES_START_OFFSET + SAMPLE_T * (i - 1) + delta;
				angle = DEG_TO_RADIAN(deg_angle);
				bFound = true;
				break;
			}
			else if (transmission_angle == transmission_angles[i]) {
				deg_angle = LENS_ANGLES_START_OFFSET + SAMPLE_T * i;
				angle = DEG_TO_RADIAN(deg_angle);
				bFound = true;
				break;
			}
		}
	}

	return bFound;
}

//extention function
static int get_view_height(int width, float theta/*radian*/, int r_i, int r_o, float sensor_pixel_size)
{
	float radius = width / 2.0 / sin(theta / 2.);
	float small_radius = width / 2.0 / tan(theta / 2.);
	float f = FOCAL_LENGTH * 1000 / sensor_pixel_size;
	float i_a, o_a;;
	if (!findAngle(r_i / f, i_a))
		return -1;
	if (!findAngle(r_o / f, o_a))
		return -1;
	float h_i = small_radius / tan(i_a);
	float h_o = radius / tan(o_a);
	return h_i - h_o;
}

static bool get_view_rect(int count, int width, float* angles/*radian*/, int r_i, int r_o, float sensor_pixel_size, int* widths, int* heights)
{
	float max_angle = 0;
	for (int i = 0; i < count; i++) {
		if (max_angle < angles[i]) {
			max_angle = angles[i];
		}
	}
	float radius = width / 2.0 / sin(max_angle / 2.);
	float f = FOCAL_LENGTH * 1000 / sensor_pixel_size;
	float i_a, o_a;;
	if (!findAngle(r_i / f, i_a))
		return false;
	if (!findAngle(r_o / f, o_a))
		return false;
	float h_o = radius / tan(o_a);

	for (int i = 0; i < count; i++) {
		float small_radius = radius * cos(angles[i] / 2.);
		float h_i = small_radius / tan(i_a);
		heights[i] = h_i - h_o;
		widths[i] = radius * sin(angles[i] / 2.) * 2;
	}
	return true;
}

static float get_focal_length_pixel(float sensor_pixel_size)
{
	return FOCAL_LENGTH * 1000 / sensor_pixel_size;
}

static bool verify_radius(int r_i, int r_o, float sensor_pixel_size)
{
	float f = FOCAL_LENGTH * 1000 / sensor_pixel_size;
	float a;;

	if (!findAngle(r_i / f, a))
		return false;

	return findAngle(r_o / f, a);
}



