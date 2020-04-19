#include "central.h"

//  Standard Library
#include <stdlib.h>			// standard library
#include <math.h>

// ChibiOS & others
#include "ch.h"					// main include files
#include <chprintf.h>			// mini printf-like functionality

// this project files
#include "move.h"
#include "sensors.h"
#include "constantes.h"

// local defines
#define SEARCH_SPEED	MOTOR_SPEED_LIMIT / 2

// semaphores
static BSEMAPHORE_DECL(central_semaphore_image_request, TRUE);

static int16_t compute_distance(int16_t ball_seen_half_angle)
{
	// = BALL_DIAMETER/2/sin((EPUCK2DEG(MIN_HALF_ANGLE_BALL)+i*EPUCK2DEG(ANGLE_TO_DIST_ANGLE_RES))*M_PI/180)
	static const int16_t precalculated_values[N_PRECALCULATED_ANGLE_TO_DIST_VALUES] =
	{3494,	3457,	3421,	3386,	3351,	3317,	3284,	3252,	3220,	3189,
	3158,	3128,	3099,	3070,	3041,	3013,	2986,	2959,	2933,	2907,
	2881,	2856,	2832,	2807,	2784,	2760,	2737,	2715,	2692,	2671,
	2649,	2628,	2607,	2587,	2566,	2547,	2527,	2508,	2489,	2470,
	2452,	2433,	2416,	2398,	2381,	2364,	2347,	2330,	2314,	2297,
	2282,	2266,	2250,	2235,	2220,	2205,	2190,	2176,	2162,	2148,
	2134,	2120,	2106,	2093,	2080,	2067,	2054,	2041,	2028,	2016,
	2004,	1992,	1980,	1968,	1956,	1945,	1933,	1922,	1911,	1900,
	1889,	1878,	1867,	1857,	1846,	1836,	1826,	1816,	1806,	1796,
	1786,	1777,	1767,	1758,	1748,	1739,	1730,	1721,	1712,	1703,
	1694,	1686,	1677,	1669,	1660,	1652,	1644,	1636,	1628,	1620,
	1612,	1604,	1596,	1588,	1581,	1573,	1566,	1558,	1551,	1544,
	1536,	1529,	1522,	1515,	1508,	1502,	1495,	1488,	1481,	1475,
	1468,	1462,	1455,	1449,	1442,	1436,	1430,	1424,	1418,	1412,
	1406,	1400,	1394,	1388,	1382,	1376,	1371,	1365,	1359,	1354,
	1348,	1343,	1337,	1332,	1327,	1321,	1316,	1311,	1306,	1300,
	1295,	1290,	1285,	1280,	1275,	1270,	1266,	1261,	1256,	1251,
	1246,	1242,	1237,	1232,	1228,	1223,	1219,	1214,	1210,	1206,
	1201,	1197,	1192,	1188,	1184,	1180,	1176,	1171,	1167,	1163,
	1159,	1155,	1151,	1147,	1143,	1139,	1135,	1131,	1127,	1124,
	1120,	1116,	1112,	1109,	1105,	1101,	1098,	1094,	1090,	1087,
	1083,	1080,	1076,	1073,	1069,	1066,	1062,	1059,	1056,	1052,
	1049,	1046,	1042,	1039,	1036,	1033,	1029,	1026,	1023,	1020,
	1017,	1014,	1011,	1007,	1004,	1001,	998,	995,	992,	989,
	987,	984,	981,	978,	975,	972,	969,	966,	964,	961,
	958,	955,	953,	950,	947,	944,	942,	939,	936,	934,
	931,	929,	926,	923,	921,	918,	916,	913,	911,	908,
	906,	903,	901,	898,	896,	894,	891,	889,	887,	884,
	882,	879,	877,	875,	873,	870,	868,	866,	864,	861,
	859,	857,	855,	852,	850,	848,	846,	844,	842,	840,
	837,	835,	833,	831,	829,	827,	825,	823,	821,	819,
	817,	815,	813,	811,	809,	807,	805,	803,	801,	799,
	797,	796,	794,	792,	790,	788,	786,	784,	782,	781,
	779,	777,	775,	773,	772,	770,	768,	766,	765,	763,
	761,	759,	758,	756,	754,	753,	751,	749,	747,	746,
	744,	742,	741,	739,	738,	736,	734,	733,	731,	730,
	728,	726,	725,	723,	722,	720,	719,	717,	716,	714,
	712,	711,	709,	708,	706,	705,	704,	702,	701,	699,
	698,	696,	695,	693,	692,	690,	689,	688,	686,	685,
	683,	682,	681,	679,	678,	677,	675,	674,	673,	671,
	670,	669,	667,	666,	665,	663,	662,	661,	659,	658,
	657,	655,	654,	653,	652,	650,	649,	648,	647,	645,
	644,	643,	642,	641,	639,	638,	637,	636,	634,	633,
	632,	631,	630,	629,	627,	626,	625,	624,	623,	622,
	620,	619,	618,	617,	616,	615,	614,	613,	611,	610,
	609,	608,	607,	606,	605,	604,	603,	602,	601,	600,
	598,	597,	596,	595,	594,	593,	592,	591,	590,	589,
	588,	587,	586,	585,	584,	583,	582,	581,	580,	579,
	578,	577,	576,	575,	574,	573,	572,	571,	570,	569,
	568,	567,	566,	566,	565,	564,	563,	562,	561,	560,
	559,	558,	557,	556,	555,	554,	554,	553,	552,	551,
	550,	549,	548,	547,	546,	546,	545,	544,	543,	542,
	541,	540,	540,	539,	538,	537,	536,	535,	534,	534,
	533,	532,	531,	530,	529,	529,	528,	527,	526,	525,
	525,	524,	523,	522,	521,	521,	520,	519,	518,	517,
	517,	516,	515,	514,	514,	513,	512,	511,	510,	510,
	509,	508,	507,	507,	506,	505,	504,	504,	503,	502,
	501,	501,	500,	499,	499,	498,	497,	496,	496,	495,
	494,	494,	493,	492,	491,	491,	490,	489,	489,	488,
	487,	487,	486,	485,	485,	484,	483,	482,	482,	481,
	480,	480,	479,	478,	478,	477,	476,	476,	475,	474,
	474,	473,	473,	472,	471,	471,	470,	469,	469,	468,
	467,	467,	466};

	//fit to array
	ball_seen_half_angle = abs(ball_seen_half_angle);
	ball_seen_half_angle = ball_seen_half_angle > MAX_HALF_ANGLE_BALL ? MAX_HALF_ANGLE_BALL : ball_seen_half_angle;
	ball_seen_half_angle = ball_seen_half_angle < MIN_HALF_ANGLE_BALL ? MIN_HALF_ANGLE_BALL : ball_seen_half_angle;

	ball_seen_half_angle -= MIN_HALF_ANGLE_BALL;
	ball_seen_half_angle /= ANGLE_TO_DIST_ANGLE_RES;

	return	precalculated_values[ball_seen_half_angle];
}

void central_control_loop(void)
{
	int16_t ball_angle = 0;
	int16_t ball_seen_half_angle = 0;
	int16_t ball_distance = 0;
	bool ball_found = false;

	while(1)
	{
		chThdSleepMilliseconds(5000);

		ball_found = false;
		sensors_set_ball_to_be_search();
		move_change_state(ROTATION);
		while(!ball_found)
		{
			chBSemSignal(&central_semaphore_image_request);
			chBSemWait(sensors_get_semaphore_authorization_move());

			ball_found = sensors_is_ball_found(&ball_angle, &ball_seen_half_angle);
			if(ball_found)
				break;

			if(sensors_search_clockwise())
				move_rotate(-EPUCK_SEARCH_ROTATION_ANGLE, SEARCH_SPEED);
			else
				move_rotate(EPUCK_SEARCH_ROTATION_ANGLE, SEARCH_SPEED);

			// wait for the end of the turn plus some inertia stability (e-puck is shaky)
			// meanwhile, image process can occur
			//chThdSleepMilliseconds(250);
			chThdSleepMilliseconds(1100);
		}

		move_rotate(ball_angle, SEARCH_SPEED);
		move_change_state(TRANSLATION);
		chThdSleepMilliseconds(1000);

        chprintf((BaseSequentialStream *)&SD3, "ball distance from robot %f mm\n", EPUCK2MM((float)compute_distance(ball_seen_half_angle)));
		ball_distance = compute_distance(ball_seen_half_angle);

		//fetch the ball
		move_straight(ball_distance-ROTATION_RADIUS, SEARCH_SPEED);
		move_round_about(ROTATION_RADIUS, SEARCH_SPEED);
		move_straight(ball_distance+ROTATION_RADIUS, SEARCH_SPEED);
		move_change_state(STATIC);
	}
}

void * central_get_semaphore_authorization_acquire(void)
{
	return &central_semaphore_image_request;
}
