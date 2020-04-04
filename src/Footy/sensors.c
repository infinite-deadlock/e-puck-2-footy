#include "sensors.h"

// ChibiOS & others
#include "ch.h"						// main include files
#include "hal.h"					// Hardware Abstraction Layer subsystem header
#include <usbcfg.h>					// USB services
#include <chprintf.h>				// mini printf-like functionality

// EPFL-MICRO 315 library
#include <camera/dcmi_camera.h>		// e-puck-2 Digital CaMera Interface
#include <camera/po8030.h>			// e-puck-2 main frontal camera

// this project files
#include "debug.h"


// local defines
#define IMAGE_BUFFER_SIZE	PO8030_MAX_WIDTH		// size of the image acquired, in RGB565 pixel
#define IMAGE_LENS_CENTER_Y	PO8030_MAX_HEIGHT/2		// vertical position of the center
#define NOT_FOUND			IMAGE_BUFFER_SIZE		// pixel not on the image

///WI: Les tolérances sont peut-être trop élevées, il faudrait affiner en testant ce que voit la caméra
///De plus couper la balance auto des blancs si fluctue trop avec l'exposition ? Idéalement il faudrait une balle de couleur, par exemple rouge
#define BALL_R_MIN			0b0110		//ball RGB color with tolerances
#define BALL_R_MAX			0b1100
#define BALL_G_MIN			0b001100
#define BALL_G_MAX			0b110010
#define BALL_B_MIN			0b0110
#define BALL_B_MAX			0b1100

typedef enum {
	RED = 0,
	GREEN,
	BLUE,
	N_COLORS
} pixel;

//Static functions
static uint16_t getRelativeAngle(uint16_t pixelIndex)
{
	//calculated atan values formatted to angles unit
	static uint16_t relative_angles[IMAGE_BUFFER_SIZE/2] =
		{7,		13,		20,		27,		34,		40,		47,		54,		61,		67,
		74,		81,		88,		94,		101,	108,	115,	121,	128,	135,
		142,	148,	155,	162,	169,	175,	182,	189,	196,	202,
		209,	216,	223,	229,	236,	243,	250,	256,	263,	270,
		276,	283,	290,	297,	303,	310,	317,	324,	330,	337,
		344,	350,	357,	364,	371,	377,	384,	391,	397,	404,
		411,	418,	424,	431,	438,	444,	451,	458,	464,	471,
		478,	485,	491,	498,	505,	511,	518,	525,	531,	538,
		545,	551,	558,	565,	571,	578,	585,	591,	598,	605,
		611,	618,	625,	631,	638,	645,	651,	658,	665,	671,
		678,	684,	691,	698,	704,	711,	718,	724,	731,	737,
		744,	751,	757,	764,	770,	777,	784,	790,	797,	803,
		810,	817,	823,	830,	836,	843,	850,	856,	863,	869,
		876,	882,	889,	895,	902,	909,	915,	922,	928,	935,
		941,	948,	954,	961,	967,	974,	980,	987,	993,	1000,
		1006,	1013,	1019,	1026,	1032,	1039,	1045,	1052,	1058,	1065,
		1071,	1078,	1084,	1091,	1097,	1104,	1110,	1116,	1123,	1129,
		1136,	1142,	1149,	1155,	1161,	1168,	1174,	1181,	1187,	1194,
		1200,	1206,	1213,	1219,	1225,	1232,	1238,	1245,	1251,	1257,
		1264,	1270,	1276,	1283,	1289,	1295,	1302,	1308,	1314,	1321,
		1327,	1333,	1340,	1346,	1352,	1359,	1365,	1371,	1378,	1384,
		1390,	1396,	1403,	1409,	1415,	1421,	1428,	1434,	1440,	1446,
		1453,	1459,	1465,	1471,	1478,	1484,	1490,	1496,	1502,	1509,
		1515,	1521,	1527,	1533,	1540,	1546,	1552,	1558,	1564,	1570,
		1577,	1583,	1589,	1595,	1601,	1607,	1613,	1620,	1626,	1632,
		1638,	1644,	1650,	1656,	1662,	1668,	1674,	1681,	1687,	1693,
		1699,	1705,	1711,	1717,	1723,	1729,	1735,	1741,	1747,	1753,
		1759,	1765,	1771,	1777,	1783,	1789,	1795,	1801,	1807,	1813,
		1819,	1825,	1831,	1837,	1843,	1849,	1855,	1861,	1866,	1872,
		1878,	1884,	1890,	1896,	1902,	1908,	1914,	1920,	1925,	1931,
		1937,	1943,	1949,	1955,	1961,	1966,	1972,	1978,	1984,	1990,
		1995,	2001,	2007,	2013,	2019,	2024,	2030,	2036,	2042,	2048};
	uint16_t relAngle = 0;

	if(pixelIndex < IMAGE_BUFFER_SIZE/2)
	{
		//left direction -> positive angle (counterclockwise)
		pixelIndex = IMAGE_BUFFER_SIZE/2 - pixelIndex-1;
		relAngle = relative_angles[pixelIndex];
	}
	else
	{
		//negative angle but cyclic
		pixelIndex -= IMAGE_BUFFER_SIZE/2;
		relAngle = ANGLE_MAX-relative_angles[pixelIndex];//negative angle due to modulo
	}

	return relAngle;
}

//Other functions

void sensors_start(void)
{
	// image
	/// @PI: on prend les lignes 10 et 11 comme vu dans le cours ? Ou on tente quelque choses de mieux ? (Contrôler, Datasheet & cours)
	/// @WI: La taille de la capture est de 640x480, donc en prendant 480/2=240 pour le centre on devrait avoir ce qui est en face du robot, sans déformation

    // acquire a line from the camera, pixel 0 in line 10 to pixel IMAGE_BUFFER_SIZE in line 11
    // caution: two lines asked for internal reasons
	po8030_advanced_config(FORMAT_RGB565, 0, IMAGE_LENS_CENTER_Y, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
}

void capture_front_view(void)
{
	// start capturing the image
	dcmi_capture_start();
	// wait until completed
	wait_image_ready();
}

struct Analysis_result get_analysis_result(void)
{
	uint8_t * img_raw_RGB565_pixels = dcmi_get_last_image_ptr();
	uint8_t pixel[N_COLORS];

	uint16_t left_edge = NOT_FOUND;
	uint16_t right_edge = NOT_FOUND;
	struct Analysis_result result = { INVALID_ANGLE, INVALID_ANGLE, false, false, false };

	static bool send_computer = true;
    //send_computer = !send_computer;
    //if(send_computer)
    	//debug_send_uint8_array_to_computer(image, IMAGE_BUFFER_SIZE);
	debug_send_for_printlinke_uint16(img_raw_RGB565_pixels, IMAGE_BUFFER_SIZE);
    //chSysLock();
    //debug_am_i_responding();
    //chSysUnlock();

    // image analysis
    for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i += 1)
    {
    	//2 bytes, MSB RRRRRGGG GGGBBBBB LSB
    	pixel[RED] = (uint8_t)img_raw_RGB565_pixels[i*2]&0xF8; // red pixels
    	pixel[GREEN] = (((uint8_t)img_raw_RGB565_pixels[i*2]&0x7)<<3); // green pixels MSB part
    	pixel[GREEN] |= (((uint8_t)img_raw_RGB565_pixels[i*2+1]&0xe0)>>5); // green pixels LSB part
    	pixel[BLUE] = (uint8_t)img_raw_RGB565_pixels[i*2+1]&0x1f; // blue pixels

    	if(pixel[RED] >= BALL_R_MIN && pixel[RED] <= BALL_R_MAX
    			&& pixel[GREEN] >= BALL_G_MIN && pixel[GREEN] <= BALL_G_MAX
    			&& pixel[BLUE] >= BALL_B_MIN && pixel[RED] <= BALL_B_MAX)
    	{
    		//if first ball pixel, rise
    		if(left_edge==NOT_FOUND)
    		{
    			left_edge = i;
    			right_edge = IMAGE_BUFFER_SIZE-1;//must fall, latest is at the end of buffer
    		}
    	}
    	else if(right_edge == IMAGE_BUFFER_SIZE-1)//left edge already found -> fall
    	{
    		right_edge=i;
    	}
    }

    //result translation for detection
    result.found = left_edge != NOT_FOUND;
    if(result.found)
    {
    	result.leftCropped = left_edge == 0;
    	result.rightCropped = right_edge == IMAGE_BUFFER_SIZE-1;

    	result.leftAngle = getRelativeAngle(left_edge);
    	result.rightAngle = getRelativeAngle(right_edge);
    }

    return result;
}
