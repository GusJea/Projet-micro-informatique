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
#include "msgbus/messagebus.h"
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

#define STEP_ADD_TOUR	5
#define	DIST_STOP		70.0
#define IR_NUMBER		8 // de 0 a 7 donc 8 sensor
#define TAN(x)			(float)tan(x)
#define SIN(x)			(float)sin(x)
#define ACOS(x)			(float)acos(x)
#define S_LEFT			1
#define	S_RIGHT 		2
#define EQUAL			3
#define TO_CENTER		4
#define CENTER			5
#define IDLE			6
static uint8_t	dodge_obs = IDLE;
void sensor_init()
{
	//TOF
	VL53L0X_start();
	//IR init
	messagebus_init( & bus, & bus_lock, & bus_condvar);
	proximity_start();
	calibrate_ir();
}

static THD_WORKING_AREA(waDetectObjet, 1024);
static THD_FUNCTION(DetectObjet, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while (1)
	{
		scan_obstacle();
		chThdSleepMilliseconds(10);
	}

}

void object_length_start()
{
	chThdCreateStatic(waDetectObjet, sizeof(waDetectObjet), NORMALPRIO+1, DetectObjet, NULL);
}

static THD_WORKING_AREA(waDetectObjet_IR, 256);
static THD_FUNCTION(DetectObjet_IR, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while (1)
	{
		if(dodge_obs == TO_CENTER)
		{
			detect_obstacle();
		}
		chThdSleepMilliseconds(1);
	}

}
void object_detect_start()
{
	chThdCreateStatic(waDetectObjet_IR, sizeof(waDetectObjet_IR), NORMALPRIO+1, DetectObjet_IR, NULL);
}

void ir_values(int* valeurs)
{
	for(int i = 0; i<IR_NUMBER; i++)
	{
		valeurs[i]=get_calibrated_prox(i);
	}
}

void detect_obstacle()
{
		int distance[IR_NUMBER] = {0};
		ir_values(distance);


		//aligné
		if((distance[0] >= 2274) && (distance[7] >= 2274))
		{
			dodge_obs = CENTER;
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}

		//trop a gauche
		else if(distance[0] >= 2275 || distance[1] >= 2275 /*|| distance[2] >= 2275*/)
		{
			motor_turn(-326, 326, 3,3);

		}

		//trop a droite
		else if(distance[7] >= 2275 || distance[6] >= 2275 /*|| distance[5] >= 2275*/)
		{
			motor_turn(326, -326, 3, 3);

		}
		else
			motor_turn(326, 326, 3, 3);
}


//mesure et décide de quel côtés de l'obstacle aller avec le TOF

void scan_obstacle()
{

	uint16_t distance_mm = 0;
	uint8_t direction = 0;

	//.._side[0] = angle, .._side[1] = tof final, .._side[2] = tof ini, .._side[3] = longueure gauche
	uint16_t left_side[4] ={0};
	uint16_t right_side[4] ={0};

	distance_mm = VL53L0X_get_dist_mm();
	if(distance_mm <= 70)
	{
		if(dodge_obs == IDLE)
		{
			dodge_obs = TO_CENTER;
		}
		else if(dodge_obs == CENTER)
		{
			while(distance_mm < 70)
			{
				motor_turn(-326, -326, 4, 4);
				distance_mm = VL53L0X_get_dist_mm();
			}
			left_side[2]=distance_mm;
			right_side[2]=distance_mm;
			obstacle_length_left(left_side);
			obstacle_length_right(right_side);

			direction = direction_choose(left_side, right_side);
			motor_command_obs_dodge(direction, left_side, right_side);
			dodge_obs = IDLE;
		}
	}
}


void obstacle_length_right(uint16_t* right_side)
{
	uint16_t return_value[2] = {0}; //left 0 right 1
	uint8_t stop = 0;
	double ob_leng_TOF_mm = 0;
	double ob_leng_mm = 0;
	uint16_t dist_obs = 0;
	for(uint16_t i = 1; i<90; i++)
	{

		motor_turn(-326, 326, 4,4);
		return_value[0] += left_motor_get_pos();
		return_value[1] += right_motor_get_pos();

		ob_leng_TOF_mm = sin(i*3.141592/180)*((double)(VL53L0X_get_dist_mm()));
		ob_leng_mm = tan(i*3.141592/180)*((double)(DIST_STOP+35.0));

		if(ob_leng_TOF_mm > ob_leng_mm)
		{
			right_side[3] = dist_obs;
			break;
		}
		dist_obs = (uint16_t)(ob_leng_mm);

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

}


void obstacle_length_left(uint16_t* left_side)
{
	uint16_t return_value[2] = {0}; //left 0 right 1
	uint8_t stop = 0;
	double ob_leng_TOF_mm = 0;
	double ob_leng_mm = 0;
	uint16_t dist_obs = 0;
	for(uint16_t i = 1; i<90; i++)
	{

		motor_turn(326, -326, 4,4);
		return_value[0] += left_motor_get_pos();
		return_value[1] += right_motor_get_pos();

		ob_leng_TOF_mm = sin(i*3.141592/180)*((double)(VL53L0X_get_dist_mm()));
		ob_leng_mm = tan(i*3.141592/180)*(double)(DIST_STOP+35.0);

		if(ob_leng_TOF_mm > ob_leng_mm)
		{
			left_side[3] = dist_obs;
			break;
		}
		dist_obs = (uint16_t)(ob_leng_mm);
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
}


/*

uint8_t obstacle_center(uint16_t d_0)
{
	uint16_t min = 0;
	uint16_t test = 0;

	uint8_t direction = 0;
	uint8_t stop =0;
	min = d_0;
	motor_turn(326,-326, 30, 30);

	test = VL53L0X_get_dist_mm();
	//VL53L0X_stop();
	if(test<d_0)
	{
		//VL53L0X_start();
		direction=S_LEFT;
		while(!stop)
		{
			test = VL53L0X_get_dist_mm();
			if(test<=min)
			{
				min=test;
				motor_turn(326, -326, 4, 4);
			}
			else
			{
				stop = 1;
			}
		}
		motor_turn(-326, 326, 4, 4);
		return direction;
	}


	else
	{
		//VL53L0X_start();
		motor_turn(-326,326, 60, 60);
		test = VL53L0X_get_dist_mm();
		direction=S_RIGHT;
		uint8_t compteur =0;
		min=d_0;
			while(!stop)
			{
				test = VL53L0X_get_dist_mm();
				if(test<=min)
				{
					min=test;
					motor_turn(-326, 326, 4, 4);
					compteur++;
				}
				else
				{
					if(compteur==0)
					{
						motor_turn(326,-326, 30, 30);
						stop = 1;
					}

					else if(compteur!=0)
					{
						motor_turn(326, -326, 4, 4);
						stop = 1;
					}
				}
			}

			return direction;
	}

}
*/
uint8_t direction_choose(uint16_t* left_side, uint16_t* right_side)
{
	uint16_t way_2_go = 0;

	if(left_side[3] > right_side[3])
	{
		way_2_go = S_RIGHT;
	}
	else if(left_side[3] < right_side[3])
	{
		way_2_go = S_LEFT;
	}
	else
	{
		way_2_go = EQUAL;
	}
	return way_2_go;
}

void motor_turn(int speed_r, int speed_l, int32_t pos_right, int32_t pos_left)
{
	uint8_t stop= 0;
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
					left_motor_set_speed(speed_l);
					right_motor_set_speed(speed_r);
				}
			}
}

void motor_command_obs_dodge(uint8_t direction, uint16_t* left_side, uint16_t* right_side)
{
	if(direction == S_RIGHT)
	{
		motor_turn(-400, 400, 320,320);
		motor_turn(500, 500,(int32_t)(7.9*(right_side[3]+35)), (int32_t)(7.9*(right_side[3]+35)));
		motor_turn(400, -400, 320,320);
		motor_turn(500, 500, (int32_t)(7.9*(35+DIST_STOP)), (int32_t)(7.9*(35+DIST_STOP)));
	}
	else if(direction == S_LEFT)
		{
			motor_turn(400, -400, 320,320);
			motor_turn(500, 500, (int32_t)(7.9*(left_side[3]+35)), (int32_t)(7.9*(left_side[3]+35)));
			motor_turn(-400, 400, 320,320);
			motor_turn(500, 500, (int32_t)(7.9*(35+DIST_STOP)), (int32_t)(7.9*(35+DIST_STOP)));
		}
	else
	{
		motor_turn(-400, -400, 40, 40);
	}

}
