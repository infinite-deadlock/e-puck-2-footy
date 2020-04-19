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
	{3503,	3462,	3421,	3382,	3344,	3306,	3269,	3234,	3198,	3164,
	3130,	3097,	3065,	3034,	3003,	2972,	2943,	2914,	2885,	2857,
	2830,	2803,	2776,	2750,	2725,	2700,	2675,	2651,	2628,	2604,
	2582,	2559,	2537,	2515,	2494,	2473,	2453,	2432,	2412,	2393,
	2374,	2355,	2336,	2318,	2299,	2282,	2264,	2247,	2230,	2213,
	2197,	2180,	2164,	2149,	2133,	2118,	2103,	2088,	2073,	2059,
	2044,	2030,	2016,	2003,	1989,	1976,	1963,	1950,	1937,	1924,
	1912,	1899,	1887,	1875,	1863,	1852,	1840,	1829,	1817,	1806,
	1795,	1784,	1774,	1763,	1753,	1742,	1732,	1722,	1712,	1702,
	1692,	1683,	1673,	1664,	1654,	1645,	1636,	1627,	1618,	1609,
	1601,	1592,	1583,	1575,	1567,	1558,	1550,	1542,	1534,	1526,
	1518,	1510,	1503,	1495,	1488,	1480,	1473,	1466,	1458,	1451,
	1444,	1437,	1430,	1423,	1416,	1410,	1403,	1396,	1390,	1383,
	1377,	1370,	1364,	1358,	1352,	1345,	1339,	1333,	1327,	1321,
	1316,	1310,	1304,	1298,	1293,	1287,	1281,	1276,	1270,	1265,
	1259,	1254,	1249,	1244,	1238,	1233,	1228,	1223,	1218,	1213,
	1208,	1203,	1198,	1193,	1189,	1184,	1179,	1175,	1170,	1165,
	1161,	1156,	1152,	1147,	1143,	1138,	1134,	1130,	1125,	1121,
	1117,	1113,	1109,	1104,	1100,	1096,	1092,	1088,	1084,	1080,
	1076,	1073,	1069,	1065,	1061,	1057,	1053,	1050,	1046,	1042,
	1039,	1035,	1032,	1028,	1024,	1021,	1017,	1014,	1010,	1007,
	1004,	1000,	997,	994,	990,	987,	984,	980,	977,	974,
	971,	968,	965,	961,	958,	955,	952,	949,	946,	943,
	940,	937,	934,	931,	928,	925,	923,	920,	917,	914,
	911,	909,	906,	903,	900,	898,	895,	892,	890,	887,
	884,	882,	879,	876,	874,	871,	869,	866,	864,	861,
	859,	856,	854,	851,	849,	847,	844,	842,	839,	837,
	835,	832,	830,	828,	825,	823,	821,	819,	816,	814,
	812,	810,	808,	805,	803,	801,	799,	797,	795,	793,
	790,	788,	786,	784,	782,	780,	778,	776,	774,	772,
	770,	768,	766,	764,	762,	760,	758,	756,	755,	753,
	751,	749,	747,	745,	743,	741,	740,	738,	736,	734,
	732,	731,	729,	727,	725,	724,	722,	720,	718,	717,
	715,	713,	712,	710,	708,	707,	705,	703,	702,	700,
	698,	697,	695,	693,	692,	690,	689,	687,	686,	684,
	682,	681,	679,	678,	676,	675,	673,	672,	670,	669,
	667,	666,	664,	663,	661,	660,	659,	657,	656,	654,
	653,	651,	650,	649,	647,	646,	644,	643,	642,	640,
	639,	638,	636,	635,	634,	632,	631,	630,	628,	627,
	626,	624,	623,	622,	621,	619,	618,	617,	616,	614,
	613,	612,	611,	609,	608,	607,	606,	604,	603,	602,
	601,	600,	598,	597,	596,	595,	594,	593,	591,	590,
	589,	588,	587,	586,	585,	583,	582,	581,	580,	579,
	578,	577,	576,	575,	574,	572,	571,	570,	569,	568,
	567,	566,	565,	564,	563,	562,	561,	560,	559,	558,
	557,	556,	555,	554,	553,	552,	551,	550,	549,	548,
	547,	546,	545,	544,	543,	542,	541,	540,	539,	538,
	537,	536,	535,	534,	533,	532,	531,	530,	530,	529,
	528,	527,	526,	525,	524,	523,	522,	521,	521,	520,
	519,	518,	517,	516,	515,	514,	514,	513,	512,	511,
	510,	509,	508,	508,	507,	506,	505,	504,	503,	503,
	502,	501,	500,	499,	498,	498,	497,	496,	495,	494,
	494,	493,	492,	491,	491,	490,	489,	488,	487,	487,
	486,	485,	484,	484,	483,	482,	481,	481,	480,	479,
	478,	478,	477,	476,	475,	475,	474,	473,	472,	472,
	471,	470,	470,	469,	468,	467,	467,	466,	465,	465,
	464,	463};

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
			if(sensors_search_clockwise())
				move_rotate(-EPUCK_SEARCH_ROTATION_ANGLE, SEARCH_SPEED);
			else
				move_rotate(EPUCK_SEARCH_ROTATION_ANGLE, SEARCH_SPEED);

			// wait for the end of the turn plus some inertia stability (e-puck is shaky)
			// meanwhile, image process can occur
			//chThdSleepMilliseconds(250);
			chThdSleepMilliseconds(1100);

			chBSemSignal(&central_semaphore_image_request);
			chBSemWait(sensors_get_semaphore_authorization_move());

			ball_found = sensors_is_ball_found(&ball_angle, &ball_seen_half_angle);
			if(ball_found)
				break;

		}

		move_rotate(ball_angle, SEARCH_SPEED);
		move_change_state(TRANSLATION);
		chThdSleepMilliseconds(1000);

        chprintf((BaseSequentialStream *)&SD3, "ball distance from robot %d mm\n", compute_distance(ball_seen_half_angle));
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
