#include "detection.h"
#include "sensors.h"
#include "move.h"
#include "constantes.h"

#include <motors.h>

#define SEARCH_STEPS_ANGLE			2730//30° each step, must be << 45°

#define MAX_BALL_ANGLE				5000//max = 2*arcsin(BALL_DIAMETER/2/(BALL_DIAMETER/2+ROBOT_DIAMETER/2))~=50° -> 5000 -> ~55° for tolerance
#define N_DIVISION_HALF_ANGLE		100//number of values in lookup table for half the angle -> precision 25 -> ~16.5'
#define ANGLE_PRECISION				MAX_BALL_ANGLE/2/N_DIVISION_HALF_ANGLE

static uint16_t calculate_ball_distance(uint16_t viewAngle)
{
	static uint16_t half_angle_division_to_dist[N_DIVISION_HALF_ANGLE] =
	{61531,	30766,	20511,	15384,	12307,	10257,	8792,	7693,	6839,	6155,
	5596,	5130,	4736,	4398,	4106,	3849,	3623,	3423,	3243,	3081,
	2935,	2802,	2681,	2569,	2467,	2373,	2285,	2204,	2129,	2058,
	1992,	1930,	1872,	1818,	1766,	1718,	1672,	1628,	1587,	1548,
	1510,	1475,	1441,	1409,	1378,	1349,	1320,	1293,	1267,	1242,
	1219,	1196,	1174,	1152,	1132,	1112,	1093,	1075,	1057,	1040,
	1023,	1007,	992,	977,	962,	948,	934,	921,	908,	896,
	884,	872,	860,	849,	838,	828,	818,	808,	798,	788,
	779,	770,	761,	753,	744,	736,	728,	720,	713,	705,
	698,	691,	684,	677,	671,	664,	658,	652,	645,	640};


	if(viewAngle > MAX_BALL_ANGLE)//if too close, return in contact
	{
		return (WHEEL_DISTANCE/2+BALL_DIAMETER/2)/LINEAR_RES*1000;
	}

	viewAngle /= 2;
	if(viewAngle < ANGLE_PRECISION)
	{
		return FIELD_RADIUS;//too far for precision, return end of field
	}

	viewAngle /= ANGLE_PRECISION;

	return half_angle_division_to_dist[viewAngle-1];
}

void findBall(void)
{
	int16_t rotationSpeed = MOTOR_SPEED_LIMIT;
	struct Analysis_result last_analysis;
	uint16_t robotAngle = getAngle();
	uint16_t leftEdgeAngle = INVALID_ANGLE;
	uint16_t rightEdgeAngle = INVALID_ANGLE;

	uint16_t ballDirection = INVALID_ANGLE;
	uint16_t ballDist = FIELD_RADIUS;


	//loop until ball found
	while(leftEdgeAngle == INVALID_ANGLE || rightEdgeAngle == INVALID_ANGLE)
	{
		capture_front_view();
		last_analysis = get_analysis_result();

		if(last_analysis.found)
		{
			if(!last_analysis.rightCropped)
			{
				//right edge is found
				rightEdgeAngle = (robotAngle + last_analysis.rightAngle)%ANGLE_MAX;

				//Right edge found, go to left edge
				rotationSpeed *= rotationSpeed >= 0 ? 1 : -1;
			}

			if(!last_analysis.leftCropped)
			{
				//left edge is found
				leftEdgeAngle = (robotAngle + last_analysis.leftAngle)%ANGLE_MAX;

				//Left edge found, go to right edge
				rotationSpeed *= rotationSpeed < 0 ? 1 : -1;
			}
		}

		if(leftEdgeAngle == INVALID_ANGLE || rightEdgeAngle == INVALID_ANGLE)
		{
			//next position
			robotAngle = rotate(SEARCH_STEPS_ANGLE, rotationSpeed, true);
		}
	}

	ballDirection = (leftEdgeAngle+rightEdgeAngle)/2;
	ballDist = calculate_ball_distance((leftEdgeAngle+ANGLE_MAX-rightEdgeAngle)%ANGLE_MAX);

	///TEST VISE LA BALLE ET AVANCE JUSQU'À ELLE
	int16_t delta = (ballDirection+ANGLE_MAX-robotAngle)%ANGLE_MAX - ANGLE_MAX/2;
	rotationSpeed = delta < 0 ? MOTOR_SPEED_LIMIT : -MOTOR_SPEED_LIMIT;
	delta = delta < 0 ? -delta : delta;
	rotate((uint16_t)delta, rotationSpeed, false);//TEST DU MOUVEMENT SANS ATTENTE; MISE DANS UN BUFFER ET THREAD EN PARALLEL
	translate(ballDist, MOTOR_SPEED_LIMIT, false);//FONCE JUSQU'À LA POSITION APPROXIMEE DE LA BALLE
}
