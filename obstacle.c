#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include <obstacle.h>
#include <math.h>

/*messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);*/

#define STEP_ADD_TOUR	5
#define	DIST_STOP		10
#define IR_NUMBER		7 // de 0 a 7 donc 8 sensor
#define TAN(x)			(float)tan(x)
#define SIN(x)			(float)sin(x)
#define ACOS(x)			(float)acos(x)
#define S_LEFT			1
#define	S_RIGHT 		2
#define EQUAL			3


void sensor_init()
{
	//TOF
	VL53L0X_start();

	//IR init
	/*messagebus_init( & bus, & bus_lock, & bus_condvar);
	proximity_start();
	calibrate_ir();*/
}

static THD_WORKING_AREA(waDetectObjet, 1024);
static THD_FUNCTION(DetectObjet, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while (1)
	{
		scan_obstacle();
		//chThdSleepMilliseconds(10);
	}

}
/*
static THD_WORKING_AREA(waDetectObjet_IR, 256);
static THD_FUNCTION(DetectObjet_IR, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while (1)
	{
		detect_obstacle();
	}

}
void object_detect_start()
{
	chThdCreateStatic(waDetectObjet_IR, sizeof(waDetectObjet_IR), NORMALPRIO+1, DetectObjet_IR, NULL);
}
*/
void object_length_start()
{
	chThdCreateStatic(waDetectObjet, sizeof(waDetectObjet), NORMALPRIO, DetectObjet, NULL);
}

void detect_obstacle()
{
	int distance[IR_NUMBER] = {100};
	for(int i = 0; i<IR_NUMBER;i++)
	{
		distance[i]=get_calibrated_prox(i);
		if(distance[i]<=DIST_STOP)
		{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
	}

}

//mesure et décide de quel côtés de l'obstacle aller avec le TOF

void scan_obstacle()
{
	uint16_t distance_mm = 80;
	uint16_t ob_leng_mm_l = 0;
	uint16_t ob_leng_mm_r = 0;
	uint8_t direction =0;

	distance_mm = VL53L0X_get_dist_mm();
	if(distance_mm <= 70)
	{

		ob_leng_mm_r = obstacle_length_right();
		ob_leng_mm_l = obstacle_length_left();
		direction = direction_choose(ob_leng_mm_l, ob_leng_mm_r);
		motor_command_obs_dodge(direction, ob_leng_mm_l, ob_leng_mm_r);

	}
}


uint16_t obstacle_length_right()
{
	float ob_leng_TOF_mm = 0.0;
	float ob_leng_mm = 0.0;

	uint16_t return_value[2] = {0}; //left 0 right 1


	uint8_t stop = 0;
	for(uint16_t i = 1; i<90; i++)
	{
		right_motor_set_pos(0);
		left_motor_set_pos(0);
		left_motor_set_speed(0);
		right_motor_set_speed(0);


		while(!stop)
		{
			if((abs(right_motor_get_pos()) > 4) && (abs(left_motor_get_pos()) >4))
			{
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				return_value[0] += left_motor_get_pos();
				return_value[1] += right_motor_get_pos();
				stop=1;
			}
			else
			{
				left_motor_set_speed(326);
				right_motor_set_speed(-326);
			}

		}

		stop =0;
		ob_leng_TOF_mm = ((float)sin(i*3.141592/180))*((float)(VL53L0X_get_dist_mm()));
		ob_leng_mm = ((float)tan(i*3.141592/180))*95.0;

		if(ob_leng_TOF_mm > ob_leng_mm)
		{
			break;
		}

	}

	right_motor_set_pos(return_value[1]);
	left_motor_set_pos(return_value[0]);

	while(!stop)
	{
				if((right_motor_get_pos() > 0) && (left_motor_get_pos() < 0))
				{
					left_motor_set_speed(0);
					right_motor_set_speed(0);
					stop=1;
				}
				else
					{
					left_motor_set_speed(-326);
					right_motor_set_speed(326);
					}

	}
	return	ob_leng_mm;
}



uint16_t obstacle_length_left()
{
	float ob_leng_TOF_mm = 0.0;
	float ob_leng_mm = 0.0;
	uint16_t return_value[2] = {0}; //left 0 right 1
	uint8_t stop = 0;
	for(uint16_t i = 1; i<90; i++)
	{

		right_motor_set_pos(0);
		left_motor_set_pos(0);
		left_motor_set_speed(0);
		right_motor_set_speed(0);


		while(!stop)
		{
			if((abs(right_motor_get_pos()) > 4) && (abs(left_motor_get_pos()) > 4))
			{
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				return_value[0] += left_motor_get_pos();
				return_value[1] += right_motor_get_pos();

				stop=1;
			}
			else
			{
				left_motor_set_speed(-326);
				right_motor_set_speed(326);
			}

		}
		stop =0;

		ob_leng_TOF_mm = ((float)sin(i*3.141592/180))*((float)(VL53L0X_get_dist_mm()));
		ob_leng_mm = ((float)tan(i*3.141592/180))*95.0;

		if(ob_leng_TOF_mm > ob_leng_mm)
		{
			break;
		}

	}

	right_motor_set_pos(return_value[1]);
	left_motor_set_pos(return_value[0]);

	while(!stop)
	{
				if((right_motor_get_pos() < 0) && (left_motor_get_pos() > 0))
				{
					left_motor_set_speed(0);
					right_motor_set_speed(0);
					stop=1;
				}
				else
					{
					left_motor_set_speed(326);
					right_motor_set_speed(-326);
					}

	}
	return	ob_leng_mm;
}


void motor_turn_obs(uint16_t pos_right, uint16_t pos_left)
{
	uint8_t stop = 0;
	right_motor_set_pos(0);
	left_motor_set_pos(0);
	left_motor_set_speed(0);
	right_motor_set_speed(0);

	while(!stop)
	{
		if((abs(right_motor_get_pos()) > pos_right) && (abs(left_motor_get_pos()) > pos_left))
		{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			stop=1;
		}
		else
		{
			left_motor_set_speed(500);
			right_motor_set_speed(500);
		}
	}
}

uint8_t direction_choose(uint16_t ob_leng_mm_l, uint16_t ob_leng_mm_r)
{
	uint16_t way_2_go = 0;
	if(ob_leng_mm_l > ob_leng_mm_r)
	{
		way_2_go = S_RIGHT;
	}
	else if(ob_leng_mm_l < ob_leng_mm_r)
	{
		way_2_go = S_LEFT;
	}
	else
	{
		way_2_go = EQUAL;
	}
	return way_2_go;
}


void motor_command_obs_dodge(uint8_t direction, uint16_t ob_leng_mm_l, uint16_t ob_leng_mm_r)
{
	right_motor_set_pos(0);
		left_motor_set_pos(0);
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		uint8_t stop= 0;

	if(direction == S_RIGHT)
	{
		while(!stop)
		{
			if((abs(right_motor_get_pos()) > 320) && (abs(left_motor_get_pos()) > 320))
			{
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				stop=1;
			}
			else
			{
				left_motor_set_speed(400);
				right_motor_set_speed(-400);
			}
		}
		stop =0;
		motor_turn_obs(8*ob_leng_mm_r+40, 8*ob_leng_mm_l+40);
	}
	else if(direction == S_LEFT)
		{
			while(!stop)
			{
				if((abs(right_motor_get_pos()) > 320) && (abs(left_motor_get_pos()) > 320))
				{
					left_motor_set_speed(0);
					right_motor_set_speed(0);
					stop=1;
				}
				else
				{
					left_motor_set_speed(-400);
					right_motor_set_speed(400);
				}
			}
			stop =0;
			motor_turn_obs(8*ob_leng_mm_r+40, 8*ob_leng_mm_l+40);
		}
	else
	{
		while(!stop)
		{
			if((abs(right_motor_get_pos()) > 40) && (abs(left_motor_get_pos()) > 40))
			{
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				stop=1;
			}
			else
			{
				left_motor_set_speed(-400);
				right_motor_set_speed(-400);
			}
		}
		stop =0;
	}

}
