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
#define TAN(x)			tan(x)
#define SIN(x)			sin(x)
#define ACOS(x)			acos(x)
#define DEG2RAD			M_PI/180
#define S_LEFT			1
#define	S_RIGHT 		2
#define ESCAPE			3
#define TO_CENTER		4
#define CENTER			5
#define IDLE			6
#define R_EPUCK			37.5
static uint8_t	dodge_obs = IDLE;
static uint8_t	go_along = 0;
static uint16_t	length = 0;

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
			center_IR();
		}
		else if(dodge_obs == IDLE)
		{
			detect_IR();
		}
		else if(go_along)
		{
			obj_ir_dodge();
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

void detect_IR()
{
	int distance[IR_NUMBER] = {0};
	ir_values(distance);


	/*
	if(distance[7] >= 300 || distance[6] >= 300)
	{
		if(distance[5]<2500)
		{
			motor_turn(-400, 400, 4,4);
		}
	}
	else if(distance[0] >= 300 || distance[1] >= 300)
	{
		if(distance[2]<1000)
		{
			motor_turn(-400, 400, 4,4);
		}
	}
	*/
}

void center_IR()
{
	int distance[IR_NUMBER] = {0};
	ir_values(distance);

	//aligné
	if((distance[0] >= 3000) && (distance[7] >= 3000))
	{
		dodge_obs = CENTER;
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	//trop a gauche
	else if(distance[0] >= 3000 || distance[1] >= 1500)
	{
		motor_turn(-326, 326, 3,3);

	}
	//trop a droite
	else if(distance[7] >= 3000 || distance[6] >= 1500)
	{
		motor_turn(326, -326, 3, 3);
	}
	else
		motor_turn(326, 326, 3, 3);
}

void scan_obstacle()
{
	uint16_t distance_mm = 0;
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
			direction_choose(obstacle_length_left(),obstacle_length_right());
		}
	}
}

uint16_t obstacle_length_right(void)
{
	uint16_t return_value[2] = {0}; //left 0 right 1
	double ob_leng_TOF_mm = 0;
	double ob_leng_mm = 0;
	for(uint8_t i = 1; i<90; i++)
	{

		motor_turn(-326, 326, 4,4);
		return_value[0] += left_motor_get_pos();
		return_value[1] += right_motor_get_pos();

		ob_leng_TOF_mm = sin(i*DEG2RAD)*((double)(VL53L0X_get_dist_mm()));
		ob_leng_mm = tan(i*DEG2RAD)*((double)(DIST_STOP+R_EPUCK));

		if(ob_leng_TOF_mm > ob_leng_mm)
		{
			break;
		}
	}

	right_motor_set_pos(return_value[1]);
	left_motor_set_pos(return_value[0]);

	while(1)
	{
				if((right_motor_get_pos() > 0) && (left_motor_get_pos() < 0))
				{
					left_motor_set_speed(0);
					right_motor_set_speed(0);
					break;
				}
				else
				{
					left_motor_set_speed(-326);
					right_motor_set_speed(326);
				}
	}
	return (uint16_t)(ob_leng_mm);
}

uint16_t obstacle_length_left(void)
{
	uint16_t return_value[2] = {0}; //left 0 right 1
	double ob_leng_TOF_mm = 0;
	double ob_leng_mm = 0;
	for(uint8_t i = 1; i<90; i++)
	{

		motor_turn(326, -326, 4,4);
		return_value[0] += left_motor_get_pos();
		return_value[1] += right_motor_get_pos();

		ob_leng_TOF_mm = sin(i*DEG2RAD)*((double)(VL53L0X_get_dist_mm()));
		ob_leng_mm = tan(i*DEG2RAD)*(double)(DIST_STOP+R_EPUCK);

		if(ob_leng_TOF_mm > ob_leng_mm)
		{
			break;
		}
	}

	right_motor_set_pos(return_value[1]);
	left_motor_set_pos(return_value[0]);

	while(1)
	{
		if((right_motor_get_pos() < 0) && (left_motor_get_pos() > 0))
		{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			break;
		}
		else
		{
			left_motor_set_speed(326);
			right_motor_set_speed(-326);
		}
	}
	return (uint16_t)(ob_leng_mm);
}

void direction_choose(uint16_t left_side, uint16_t right_side)
{

	if(left_side > right_side)
	{
		dodge_obs = S_RIGHT;
		motor_turn(400, 400, 400,400);
		motor_turn(-400, 400, 320,320);
		length = right_side;
		if(VL53L0X_get_dist_mm()>=(length+2*R_EPUCK))
		{
			go_along = 1;
		}
		else
		{
			length = left_side;
			motor_turn(400, -400, 640,640);
			if(VL53L0X_get_dist_mm()>=(length+2*R_EPUCK))
			{
				dodge_obs = S_LEFT;
				go_along = 1;
			}
			else
			{
				motor_turn(-400, 400, 640,640);
				escape_dead_end();
			}
		}
	}
	else if(left_side <= right_side)
	{
		dodge_obs = S_LEFT;
		motor_turn(400, 400, 400,400);
		motor_turn(400, -400, 320,320);
		length = left_side;
		if(VL53L0X_get_dist_mm()>=(length+2*R_EPUCK))
		{
			go_along = 1;
		}
		else
		{
			length=right_side;
			motor_turn(-400, 400, 640,640);
			if(VL53L0X_get_dist_mm()>=(length+2*R_EPUCK))
			{
				dodge_obs = S_RIGHT;
				go_along = 1;
			}
			else
			{
				motor_turn(400, -400, 640,640);
				escape_dead_end();
			}
		}
	}
}

void escape_dead_end(void)
{
	//vl..get_dist, avancer de la valeur, tourner du coté ou il y a pas l'obstracle de fond,
	//longer jusqu'au bord tourner, puis longer de nouveau
}

void motor_turn(int speed_r, int speed_l, int32_t pos_right, int32_t pos_left)
{
	right_motor_set_pos(0);
	left_motor_set_pos(0);
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	while(1)
	{
		if((abs(right_motor_get_pos()) > pos_right) && (abs(left_motor_get_pos()) > pos_left))
		{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			break;
		}
		else
		{
			left_motor_set_speed(speed_l);
			right_motor_set_speed(speed_r);
		}
	}
}

void obj_ir_dodge()
{
	int distance[IR_NUMBER] = {0};
	ir_values(distance);

	/*switch(dodge_obs)
	{
		case*/
	if(dodge_obs== S_RIGHT)
		{
			if(distance[5] >= 1500 )
			{
				motor_turn(400, 400, 4,4);
			}
			else if(distance[6] < 100)
			{
				go_along = S_RIGHT;
				dodge_obs = ESCAPE;
			}
			//break;
		}
	else if(dodge_obs== S_LEFT)
		{

			if(distance[2] < 1500)
			{
				motor_turn(400, 400, 4,4);
			}
			else if(distance[1] < 100)
			{
				dodge_obs = ESCAPE;
				go_along = S_LEFT;
			}
			//break;
		}
	else if(dodge_obs== ESCAPE)
		{

				motor_turn(400, 400, 380, 380);
				if(go_along == S_LEFT)
				{
					motor_turn(-326, 326, 320,320);
				}
				else if(go_along == S_RIGHT)
				{
					motor_turn(326, -326, 320,320);
				}

				dodge_obs = IDLE;
				go_along =0;
				//break;

		}

	//}
}






