#include "central.h"

//  Standard Library
#include <stdlib.h>			// standard library

// this project files
#include "constantes.h"
#include "debug.h"
#include "move.h"
#include "sensors.h"

// local defines
#define DEFAULT_SPEED	(MOTOR_SPEED_LIMIT / 4)
#define CHARGE_SPEED	(MOTOR_SPEED_LIMIT / 3)

// local functions prototypes
static int16_t compute_distance(int16_t ball_seen_half_angle);

// functions
void central_control_loop(void)
{
	int16_t ball_angle = 0;
	int16_t ball_seen_half_angle = 0;
	int16_t ball_distance;
	bool ball_found;

	while(1)
	{
		chThdSleepMilliseconds(5000);

		ball_found = false;
		sensors_set_ball_to_be_search();
		move_change_state(ROTATION);

		do
		{
			// wait for the end of the turn plus some inertia stability (e-puck is shaky)
<<<<<<< HEAD
			chThdSleepMilliseconds(250);
=======
			 chThdSleepMilliseconds(250);
>>>>>>> final1

			sensors_capture_and_search();
			ball_found = sensors_is_ball_found(&ball_angle, &ball_seen_half_angle);

			if(!ball_found)
			{
				if(sensors_search_clockwise())
					move_rotate(-EPUCK_SEARCH_ROTATION_ANGLE, DEFAULT_SPEED);
				else
					move_rotate(EPUCK_SEARCH_ROTATION_ANGLE, DEFAULT_SPEED);
			}
		}while(!ball_found);

		move_rotate(ball_angle, DEFAULT_SPEED);

		move_change_state(TRANSLATION);
		chThdSleepMilliseconds(1000);

		ball_distance = compute_distance(ball_seen_half_angle);
        chprintf((BaseSequentialStream *)&SD3, "ball distance from robot %d epuck units\n", ball_distance);

		// retrieve the ball
		move_straight(ball_distance-ROTATION_RADIUS, DEFAULT_SPEED);
		move_round_about(ROTATION_RADIUS, DEFAULT_SPEED);
		move_straight(ball_distance+ROTATION_RADIUS, CHARGE_SPEED);
		move_change_state(STATIC);
	}
}

// local functions

static int16_t compute_distance(int16_t ball_seen_half_angle)
{
	// = BALL_DIAMETER/2/sin((EPUCK2DEG(MIN_HALF_ANGLE_BALL)+i*EPUCK2DEG(ANGLE_TO_DIST_ANGLE_RES))*M_PI/180)
	static const int16_t precalculated_values[N_PRECALCULATED_ANGLE_TO_DIST_VALUES] = {
			3509,	3468,	3428,	3388,	3350,	3313,	3277,	3241,	3206,	3171,
			3138,	3105,	3073,	3041,	3010,	2980,	2950,	2920,	2893,	2864,
			2837,	2809,	2783,	2757,	2732,	2707,	2682,	2658,	2634,	2611,
			2588,	2565,	2543,	2521,	2500,	2479,	2458,	2437,	2417,	2398,
			2378,	2360,	2341,	2322,	2304,	2286,	2268,	2251,	2234,	2217,
			2201,	2184,	2168,	2152,	2137,	2121,	2106,	2091,	2076,	2062,
			2047,	2033,	2019,	2005,	1992,	1978,	1965,	1952,	1939,	1926,
			1914,	1901,	1889,	1877,	1865,	1853,	1842,	1830,	1819,	1808,
			1796,	1786,	1774,	1764,	1753,	1743,	1733,	1722,	1713,	1702,
			1692,	1683,	1673,	1664,	1654,	1646,	1636,	1627,	1618,	1609,//99
			1600,	1591,	1583,	1574,	1566,	1558,	1550,	1541,	1533,	1525,
			1517,	1510,	1502,	1494,	1487,	1480,	1472,	1464,	1457,	1450,
			1443,	1436,	1428,	1422,	1415,	1409,	1401,	1395,	1389,	1381,
			1375,	1369,	1362,	1356,	1350,	1343,	1338,	1332,	1325,	1320,
			1314,	1308,	1302,	1296,	1291,	1285,	1279,	1273,	1268,	1263,
			1257,	1252,	1247,	1242,	1236,	1231,	1226,	1221,	1216,	1210,
			1206,	1200,	1196,	1191,	1187,	1181,	1177,	1172,	1167,	1163,
			1158,	1154,	1149,	1144,	1140,	1136,	1131,	1127,	1123,	1119,
			1114,	1110,	1106,	1102,	1097,	1094,	1089,	1085,	1082,	1077,
			1073,	1070,	1066,	1062,	1058,	1054,	1050,	1047,	1043,	1039,//199
			1035,	1032,	1029,	1025,	1022,	1018,	1014,	1010,	1008,	1004,
			1000,	997,	994,	991,	987,	983,	981,	977,	974,	970,
			968,	965,	961,	958,	956,	952,	949,	946,	942,	940,
			937,	934,	931,	928,	925,	922,	919,	916,	914,	911,
			908,	905,	902,	899,	897,	894,	891,	889,	886,	884,
			881,	878,	876,	873,	870,	868,	866,	863,	860,	858,
			855,	853,	851,	848,	845,	843,	840,	839,	836,	834,
			831,	829,	826,	824,	822,	820,	818,	815,	813,	810,
			808,	807,	804,	802,	800,	797,	795,	793,	791,	789,
			787,	785,	783,	781,	778,	776,	775,	773,	771,	769,//299
			767,	764,	762,	760,	758,	757,	755,	753,	751,	749,
			747,	745,	743,	741,	740,	738,	736,	734,	732,	731,
			729,	727,	725,	723,	722,	720,	718,	717,	715,	713,
			711,	709,	708,	706,	705,	703,	702,	700,	698,	696,
			695,	693,	691,	690,	688,	687,	686,	684,	682,	681,
			679,	677,	676,	674,	672,	671,	669,	668,	667,	666,
			664,	662,	661,	659,	658,	656,	655,	653,	652,	650,
			649,	648,	647,	645,	644,	642,	641,	640,	638,	637,
			635,	634,	632,	631,	630,	629,	628,	626,	625,	624,
			622,	621,	620,	618,	617,	616,	614,	613,	612,	610,//399
			609,	608,	607,	606,	605,	604,	602,	601,	600,	599,
			597,	596,	595,	594,	592,	591,	590,	589,	587,	587,
			586,	585,	584,	582,	581,	580,	579,	578,	577,	575,
			574,	573,	572,	571,	570,	569,	567,	566,	565,	565,
			564,	563,	562,	561,	560,	559,	557,	556,	555,	554,
			553,	552,	551,	550,	549,	548,	547,	546,	545,	544,
			543,	543,	542,	541,	540,	539,	538,	537,	536,	535,
			534,	533,	532,	531,	530,	529,	528,	527,	526,	525,
			524,	523,	522,	521,	520,	520,	519,	518,	517,	517,
			516,	515,	514,	513,	512,	511,	510,	509,	508,	508,//499
			507,	506,	505,	504,	503,	502,	501,	501,	500,	499,
			498,	497,	496,	495,	495,	495,	494,	493,	492,	491,
			491,	490,	489,	488,	487,	486,	486,	485,	484,	483,
			482,	482,	481,	480,	479,	479,	478,	477,	476,	475,
			475,	474,	473,	472,	472,	471,	470,	470,	470,	469,
			468,	467,	467,	466,	465,	464,	464,	463,	462,	461,
			461,	460,	459,	459,	458,	457,	456,	456,	455,	454,//569
			454,	453,	452};//572

	// fit to array
	ball_seen_half_angle = abs(ball_seen_half_angle);
	ball_seen_half_angle = ball_seen_half_angle > MAX_HALF_ANGLE_BALL ? MAX_HALF_ANGLE_BALL : ball_seen_half_angle;
	ball_seen_half_angle = ball_seen_half_angle < MIN_HALF_ANGLE_BALL ? MIN_HALF_ANGLE_BALL : ball_seen_half_angle;

	ball_seen_half_angle -= MIN_HALF_ANGLE_BALL;
	ball_seen_half_angle /= ANGLE_TO_DIST_ANGLE_RES;

	return	precalculated_values[ball_seen_half_angle];
}
