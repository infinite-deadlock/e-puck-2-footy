#include "move.h"

// this project files
#include "constantes.h"
#include "debug.h"
#include "sensors.h"

#define BOOST_FACTOR			2

#define OBSTACLE_DETECT_DELAY	150	// in ms

// static global variables
static Move_state s_move_state;
static bool s_boost_speed;
static bool s_clockwise;

// local functions prototypes
static void check_dynamic_triggers(bool force_update);
static void make_move(int16_t speed_left, int16_t speed_right, uint32_t duration);

// semaphores
static BSEMAPHORE_DECL(move_semaphore_interrupt, TRUE);

// mutexes
static MUTEX_DECL(move_mutex_free_to_move);

// threaded functions

static THD_WORKING_AREA(wa_check_dynamic, 256);
static THD_FUNCTION(check_dynamic, arg)
{
	// this function is used in a thread
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    bool stopped = false;

    s_move_state = STATIC;
    s_boost_speed = false;

    while(1)
    {
    	// manual control
    	check_dynamic_triggers(false);	// update only if there is a change

    	// obstacle
    	if(!stopped && !sensors_can_move())	// stops moving
    	{
    		// interrupts current move if any, else not effects
    		chBSemSignal(&move_semaphore_interrupt);
    		left_motor_set_speed(0);
    		right_motor_set_speed(0);
    		chMtxLock(&move_mutex_free_to_move);	// prevents any move

    		stopped = true;
    	}
    	if(stopped && sensors_can_move())
    	{
    		chMtxUnlock(&move_mutex_free_to_move);	// resume moves

    		stopped = false;
    	}
		chThdSleepMilliseconds(OBSTACLE_DETECT_DELAY);
    }
}

// function definitions
void move_init_threads(void)
{
	chThdCreateStatic(wa_check_dynamic, sizeof(wa_check_dynamic), NORMALPRIO, check_dynamic, NULL);
}

void move_change_state(Move_state new_state)
{
	s_move_state = new_state;
	check_dynamic_triggers(true); //force to update dynamic state
}

void move_straight(int16_t distance, int16_t speed)
{
	// fit speed to motors limitations
	speed = speed >  MOTOR_SPEED_LIMIT ?  MOTOR_SPEED_LIMIT : speed;
	speed = speed < -MOTOR_SPEED_LIMIT ? -MOTOR_SPEED_LIMIT : speed;

	if(distance < 0.f)
		speed *= -1;

	make_move(speed, speed, (uint32_t)abs((int32_t)distance * 1000 / speed));
}

void move_rotate(int16_t angle, int16_t speed)
{
	// fits speed to motors limitations
	speed = speed >  MOTOR_SPEED_LIMIT ?  MOTOR_SPEED_LIMIT : speed;
	speed = speed < -MOTOR_SPEED_LIMIT ? -MOTOR_SPEED_LIMIT : speed;

	if(angle < 0)
		speed *= -1;

	s_clockwise = speed < 0;
	make_move(-speed, speed, (uint32_t)abs(1000*(int32_t)angle/(EPUCK_ANGULAR_RES/ANGULAR_UNIT)/speed));
}

void move_round_about(int16_t radius, int16_t speed)
{
	static int16_t speed_fast_wheel = 0;	// rotation clockwise so left wheel turns faster in a circle
	static int16_t speed_slow_wheel = 0;
	static uint32_t duration = 0;

	// fits speed to motors limitations
	speed_fast_wheel = (int16_t)((int32_t)speed*(radius+MM2EPUCK(WHEEL_DISTANCE))/(radius+MM2EPUCK(WHEEL_DISTANCE)/2));
	speed_fast_wheel = speed_fast_wheel >  MOTOR_SPEED_LIMIT ?  MOTOR_SPEED_LIMIT : speed_fast_wheel;
	speed_fast_wheel = speed_fast_wheel < -MOTOR_SPEED_LIMIT ? -MOTOR_SPEED_LIMIT : speed_fast_wheel;

	speed_slow_wheel = (int16_t)((int32_t)speed_fast_wheel*(radius)/(radius+MM2EPUCK(WHEEL_DISTANCE)));

	duration = (uint32_t)1000*DEG2EPUCK(180)/(EPUCK_ANGULAR_RES/ANGULAR_UNIT)*2/abs(speed_fast_wheel - speed_slow_wheel);	// half circle -> robot must rotate of 180deg around his center

	//chprintf((BaseSequentialStream *)&SD3, "speed_fast_wheel, speed_slow_wheel, speed: %d %d %d\n", speed_fast_wheel, speed_slow_wheel, speed);

	move_rotate(DEG2EPUCK(90), speed);							// rotate to be tangent
	make_move(speed_fast_wheel, speed_slow_wheel, duration);	// half circle
	move_rotate(DEG2EPUCK(-90), speed);							// face center
}

// local functions

static void check_dynamic_triggers(bool force_update)
{
    static struct IR_triggers previous_triggers = {0};
    static struct IR_triggers current_triggers;

	current_triggers = sensors_get_IR_triggers();
	switch(s_move_state)
	{
		case	TRANSLATION:
			if(force_update || current_triggers.back_triggered != previous_triggers.back_triggered)
			{
				s_boost_speed = current_triggers.back_triggered;
				chBSemSignal(&move_semaphore_interrupt);
			}
			break;
		case	ROTATION:
			if(force_update || ( s_clockwise && current_triggers.left_triggered  != previous_triggers.left_triggered)
							|| (!s_clockwise && current_triggers.right_triggered != previous_triggers.right_triggered))
			{
				// triggered in same direction as rotation
				s_boost_speed = (s_clockwise && current_triggers.left_triggered) || (!s_clockwise && current_triggers.right_triggered);
				chBSemSignal(&move_semaphore_interrupt);
			}
			else if((s_clockwise && current_triggers.right_triggered) || (!s_clockwise && current_triggers.left_triggered))
			{
				// triggered in opposite direction of rotation
				sensors_invert_rotation();
			}
			break;
		case	STATIC:
			s_boost_speed = false;
			break;
		default:
			break;
	}
	previous_triggers = current_triggers;
}

static void make_move(int16_t speed_left, int16_t speed_right, uint32_t duration)
{
	systime_t time_start;
	uint32_t time_moved = 0;
	int16_t max_speed;
	int16_t boost_factor;

	// move
	do
	{
		duration -= time_moved;

		chMtxLock(&move_mutex_free_to_move); // wait until not blocked
		if(s_boost_speed)
		{
			// calculate boost_factor
			max_speed = abs(abs(speed_left) > abs(speed_right) ? speed_left : speed_right);
			boost_factor = max_speed * BOOST_FACTOR * 10;	// factor 10 to avoid decimals
			boost_factor = max_speed > MOTOR_SPEED_LIMIT * 10 ? MOTOR_SPEED_LIMIT * 10 : boost_factor;
			boost_factor /= max_speed;
		}
		else
			boost_factor = 10;

		left_motor_set_speed(speed_left*boost_factor/10);
		right_motor_set_speed(speed_right*boost_factor/10);
	    time_start = chVTGetSystemTime();

		chBSemWaitTimeout(&move_semaphore_interrupt, MS2ST(duration * 10 / boost_factor));	// exit if dynamic control or obstacle interruption - time reduced by boost
		time_moved = (ST2MS(chVTGetSystemTime() - time_start)+1) * boost_factor / 10;		// +1 to avoid truncation errors (negligeable). Boosted time counts for more
		chMtxUnlock(&move_mutex_free_to_move);
	}while(duration > time_moved);	// while move not finished

	//end move
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}
