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
		// do not start right away for reasons of comfort
		chThdSleepMilliseconds(5000);

		// prepare to search a ball
		ball_found = false;
		sensors_set_ball_to_be_search();
		move_change_state(ROTATION);

		do
		{
			// wait for the end of the turn plus some inertia stability (e-puck is shaky)
			chThdSleepMilliseconds(250);

			// try to find a ball
			sensors_capture_and_search();
			ball_found = sensors_is_ball_found(&ball_angle, &ball_seen_half_angle);

			// keep trying to find a ball if not yet found
			if(!ball_found)
			{
				if(sensors_search_clockwise())
					move_rotate(-EPUCK_SEARCH_ROTATION_ANGLE, DEFAULT_SPEED);
				else
					move_rotate(EPUCK_SEARCH_ROTATION_ANGLE, DEFAULT_SPEED);
			}
		}while(!ball_found);

		// aim for the ball
		move_rotate(ball_angle, DEFAULT_SPEED);

		// get ready to go
		move_change_state(TRANSLATION);
		chThdSleepMilliseconds(1000);

		ball_distance = compute_distance(ball_seen_half_angle);
        chprintf((BaseSequentialStream *)&SD3, "ball distance from robot %d epuck units\n", ball_distance);

		// retrieve the ball
		move_straight(ball_distance - ROTATION_RADIUS, DEFAULT_SPEED);	// get to the ball
		move_round_about(ROTATION_RADIUS, DEFAULT_SPEED);				// get around the ball
		move_straight(ball_distance + ROTATION_RADIUS, CHARGE_SPEED);	// hit it !
		move_change_state(STATIC);		// make ready to restart
	}
}

// local functions
static int16_t compute_distance(int16_t ball_seen_half_angle)
{
	// = BALL_DIAMETER/2/sin((EPUCK2DEG(MIN_HALF_ANGLE_BALL)+i*EPUCK2DEG(ANGLE_TO_DIST_ANGLE_RES))*M_PI/180)
	static const int16_t precalculated_values[N_PRECALCULATED_ANGLE_TO_DIST_VALUES] = {
			3499,	3396,	3303,	3219,	3144,	3076,	3014,	2958,	2907,	2860,
			2817,	2778,	2742,	2709,	2679,	2651,	2625,	2601,	2578,	2557,
			2537,	2518,	2501,	2484,	2469,	2454,	2440,	2426,	2413,	2401,
			2389,	2377,	2366,	2356,	2345,	2335,	2325,	2316,	2306,	2297,
			2288,	2279,	2271,	2262,	2254,	2245,	2237,	2229,	2221,	2213,
			2206,	2198,	2190,	2182,	2175,	2167,	2160,	2152,	2145,	2137,
			2130,	2123,	2116,	2109,	2101,	2094,	2087,	2080,	2073,	2066,
			2059,	2052,	2045,	2038,	2031,	2025,	2018,	2011,	2004,	1997,
			1991,	1984,	1977,	1971,	1964,	1957,	1951,	1944,	1938,	1932,
			1925,	1918,	1912,	1906,	1899,	1893,	1887,	1880,	1874,	1868,
			1862,	1856,	1849,	1843,	1837,	1831,	1825,	1819,	1813,	1807,
			1801,	1795,	1789,	1784,	1778,	1772,	1766,	1761,	1755,	1749,
			1744,	1738,	1732,	1727,	1721,	1716,	1710,	1704,	1699,	1694,
			1688,	1683,	1678,	1673,	1667,	1662,	1657,	1652,	1646,	1641,
			1636,	1631,	1626,	1621,	1616,	1611,	1606,	1601,	1596,	1591,
			1587,	1581,	1576,	1572,	1567,	1562,	1557,	1553,	1548,	1544,
			1539,	1534,	1530,	1525,	1520,	1516,	1512,	1507,	1502,	1498,
			1494,	1489,	1485,	1481,	1477,	1472,	1468,	1464,	1460,	1455,
			1451,	1447,	1443,	1438,	1434,	1431,	1427,	1422,	1418,	1414,
			1410,	1406,	1403,	1399,	1395,	1391,	1387,	1383,	1379,	1375,
			1372,	1368,	1364,	1361,	1357,	1353,	1349,	1346,	1342,	1339,
			1335,	1331,	1328,	1324,	1321,	1318,	1313,	1310,	1307,	1303,
			1300,	1296,	1293,	1290,	1287,	1283,	1280,	1277,	1273,	1270,
			1267,	1263,	1260,	1257,	1254,	1251,	1248,	1244,	1241,	1238,
			1235,	1232,	1229,	1226,	1223,	1220,	1217,	1213,	1211,	1208,
			1204,	1202,	1199,	1196,	1193,	1191,	1187,	1184,	1181,	1179,
			1176,	1173,	1170,	1167,	1164,	1162,	1159,	1156,	1153,	1151,
			1148,	1146,	1143,	1140,	1138,	1135,	1132,	1129,	1127,	1125,
			1122,	1120,	1117,	1114,	1112,	1109,	1106,	1104,	1101,	1099,
			1096,	1094,	1091,	1089,	1087,	1085,	1082,	1080,	1078,	1075,
			1073,	1070,	1068,	1066,	1063,	1061,	1059,	1056,	1054,	1052,
			1050,	1047,	1045,	1043,	1041,	1039,	1035,	1033,	1031,	1029,
			1027,	1025,	1023,	1020,	1018,	1016,	1014,	1012,	1010,	1008,
			1006,	1004,	1002,	1000,	997,	995,	993,	991,	989,	987,
			985,	984,	982,	980,	978,	975,	973,	971,	969,	968,
			966,	964,	962,	960,	959,	956,	954,	952,	951,	949,
			947,	945,	944,	941,	939,	938,	936,	934,	933,	931,
			929,	927,	925,	923,	922,	920,	918,	917,	914,	913,
			911,	910,	908,	906,	905,	902,	901,	899,	898,	896,
			895,	893,	891,	889,	888,	886,	885,	883,	882,	880,
			878,	877,	875,	874,	872,	870,	869,	867,	866,	864,
			863,	862,	859,	858,	857,	855,	854,	853,	850,	849,
			848,	846,	845,	844,	841,	840,	839,	838,	836,	834,
			833,	832,	830,	829,	828,	827,	824,	823,	822,	821,
			819,	817,	816,	815,	814,	812,	810,	809,	808,	807,
			806,	804,	802,	801,	800,	799,	798,	797,	794,	793,
			792,	791,	790,	789,	787,	786,	785,	784,	782,	780,
			779,	778,	777,	776,	775,	773,	772,	771,	770,	769,
			768,	766,	765,	764,	763,	762,	761,	759,	758,	757,
			756,	755,	753,	752,	751,	750,	749,	748,	746,	745,
			744,	743,	742,	740,	739,	738,	737,	736,	734,	733,
			732,	732,	731,	730,	729,	727,	726,	725,	724,	723,
			721,	721,	720,	719,	718,	717,	715,	714,	713,	713,
			712,	711,	709,	708,	707,	706,	706,	704,	703,	702,
			701,	700,	700,	698,	697,	696,	695,	694,	694,	692,
			691,	690,	689,	689,	687,	686,	685,	685,	684,	683,
			681,	680,	680,	679,	678,	677,	677,	675,	674,	673,
			673,	672,	670,	669,	669,	668,	667,	665,	665,	664,
			663,	662,	662,	661,	659,	659,	658,	657,	656,	656,
			654,	653,	653,	652,	651};

	// fit to array
	ball_seen_half_angle = abs(ball_seen_half_angle);
	ball_seen_half_angle = ball_seen_half_angle > MAX_HALF_ANGLE_BALL ? MAX_HALF_ANGLE_BALL : ball_seen_half_angle;
	ball_seen_half_angle = ball_seen_half_angle < MIN_HALF_ANGLE_BALL ? MIN_HALF_ANGLE_BALL : ball_seen_half_angle;

	ball_seen_half_angle -= MIN_HALF_ANGLE_BALL;
	ball_seen_half_angle /= ANGLE_TO_DIST_ANGLE_RES;

	return	precalculated_values[ball_seen_half_angle];
}
