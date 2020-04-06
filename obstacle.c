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
#define TAN(x)	(float)tan(x)
#define SIN(x)	(float)sin(x)
#define ACOS(x)	(float)acos(x)
#define S_LEFT	1
#define	S_RIGHT 2
#define EQUAL	3

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
	//uint16_t ob_leng_mm_r = 0;
	distance_mm = VL53L0X_get_dist_mm();
	if(distance_mm <= 70 )
	{

		//left_motor_set_speed(-600);
		//right_motor_set_speed(-600);

		ob_leng_mm_l = obstacle_length_left();
		//ob_leng_mm_r = obstacle_length_rightt();
		//direction_choose(ob_leng_mm_l, ob_leng_mm_r);

	}
}

//en travaux
uint16_t obstacle_length_left()
{
	uint16_t ob_leng_TOF_mm = 0;
	uint16_t ob_leng_mm = 0;
	uint16_t left_return =0;
	uint16_t right_return =0;

	int stop = 0;
	for(uint16_t i = 1; i<90; i++)
	{

		//motor_angle(0);
		//sensé faire bouger de 1 degrés a chaque appel...

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
				left_return += left_motor_get_pos();
				right_return += right_motor_get_pos();
				stop=1;
			}
			else
				{
				left_motor_set_speed(326);
				right_motor_set_speed(-326);
				}

		}
		stop =0;
		/*ob_leng_TOF_mm = (uint16_t) (sin(i)*((float)(VL53L0X_get_dist_mm())));
		ob_leng_mm = (uint16_t) (tan(i)*70);

		if(ob_leng_mm != ob_leng_TOF_mm)
		{
			break;
		}*/

	}
	right_motor_set_pos(right_return);
	left_motor_set_pos(left_return);
			////left_motor_set_speed(0);
			//right_motor_set_speed(0);
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


//similaire a o.._l.._left mais tourne dans le sens inverse
uint16_t obstacle_length_right()
{
	uint16_t ob_leng_TOF_mm = 0;
	uint16_t ob_leng_mm = 0;
	for(int i = 1; i<90; i++)
	{

		//motor_angle(0);


		ob_leng_TOF_mm = (uint16_t) (sin(i)*((float)(VL53L0X_get_dist_mm())));
		ob_leng_mm = (uint16_t) (tan(i)*DIST_STOP);

		if(ob_leng_mm != ob_leng_TOF_mm)
		{
			motor_angle(-i);
			break;
		}

	}
	return	ob_leng_mm;
}

uint16_t direction_choose(uint16_t ob_leng_mm_l, uint16_t ob_leng_mm_r)
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

