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
#include <utilities.h>
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


#define	DIST_STOP			70.0
#define IR_NUMBER			8 // de 0 a 7 donc 8 sensor
#define TAN(x)				tan(x)
#define SIN(x)				sin(x)
#define ACOS(x)				acos(x)
#define DEG2RAD				M_PI/180

#define S_LEFT				1
#define	S_RIGHT 			2
#define ESCAPE				3
#define TO_CENTER			4
#define CENTER				5
#define IDLE				6

#define STEP_ADD_TOUR		5

#define ONE_DEG				4
#define QUAT_TURN			320
#define HALF_TURN			640
#define DODGE_SPEED			400
#define MIN_SPEED			326
#define CLOSE_OBS			400
#define R_EPUCK				37.5

#define IR_CLOSE			3000
#define IR_MID_DIST			1500
#define	IR_FAR				100

static uint8_t				dodge_obs = IDLE;
static uint8_t				go_along = 0;
static uint16_t				length = 0;

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
/*
 * @brief get the IR sensor values
 *
 * @param	valeurs value mesured by the sensor
 */
void ir_values(int* valeurs)
{
	for(int i = 0; i<IR_NUMBER; i++)
	{
		valeurs[i]=get_calibrated_prox(i);
	}
}


/*
 * @brief 	store the IR sensor values in distance table and then compare them
 *
 */
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
/*
 *
 */
void center_IR()
{
	int distance[IR_NUMBER] = {0};
	ir_values(distance);

	//aligné
	if((distance[0] >= IR_CLOSE) && (distance[7] >= IR_CLOSE))
	{
		dodge_obs = CENTER;
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	//trop a gauche
	else if(distance[0] >= IR_CLOSE || distance[1] >= IR_MID_DIST)
	{
		motor_turn(-MIN_SPEED, MIN_SPEED, ONE_DEG,ONE_DEG);

	}
	//trop a droite
	else if(distance[7] >= IR_CLOSE || distance[6] >= IR_MID_DIST)
	{
		motor_turn(MIN_SPEED, -MIN_SPEED, ONE_DEG, ONE_DEG);
	}
	else
		motor_turn(MIN_SPEED, MIN_SPEED, ONE_DEG, ONE_DEG);
}

void scan_obstacle()
{
	uint16_t distance_mm = 0;
	distance_mm = VL53L0X_get_dist_mm();
	if(distance_mm <= DIST_STOP)
	{
		if(dodge_obs == IDLE)
		{
			dodge_obs = TO_CENTER;
		}
		else if(dodge_obs == CENTER && !go_along)
		{
			while(distance_mm < DIST_STOP)
			{
				motor_turn(-MIN_SPEED, -MIN_SPEED, ONE_DEG, ONE_DEG);
				distance_mm = VL53L0X_get_dist_mm();
			}
			direction_choose(obstacle_length_left(),obstacle_length_right());
		}
		else if(go_along)
		{
			dodge_obs = TO_CENTER;
			go_along = 0;
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

		motor_turn(-MIN_SPEED, MIN_SPEED, ONE_DEG,ONE_DEG);
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
					left_motor_set_speed(-MIN_SPEED);
					right_motor_set_speed(MIN_SPEED);
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

		motor_turn(MIN_SPEED, -MIN_SPEED, ONE_DEG,ONE_DEG);
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
			left_motor_set_speed(MIN_SPEED);
			right_motor_set_speed(-MIN_SPEED);
		}
	}
	return (uint16_t)(ob_leng_mm);
}

void direction_choose(uint16_t left_side, uint16_t right_side)
{

	if(left_side > right_side)
	{
		dodge_obs = S_RIGHT;
		motor_turn(DODGE_SPEED, DODGE_SPEED, CLOSE_OBS,CLOSE_OBS);
		motor_turn(-DODGE_SPEED, DODGE_SPEED, QUAT_TURN,QUAT_TURN);
		length = right_side;
		if(VL53L0X_get_dist_mm()>=(length+2*R_EPUCK))
		{
			go_along = 1;
		}
		else
		{
			length = left_side;
			motor_turn(DODGE_SPEED, -DODGE_SPEED, HALF_TURN,HALF_TURN);
			if(VL53L0X_get_dist_mm()>=(length+2*R_EPUCK))
			{
				dodge_obs = S_LEFT;
				go_along = 1;
			}
			else
			{
				motor_turn(-DODGE_SPEED, DODGE_SPEED, HALF_TURN,HALF_TURN);
				escape_dead_end();
			}
		}
	}
	else if(left_side <= right_side)
	{
		dodge_obs = S_LEFT;
		motor_turn(DODGE_SPEED, DODGE_SPEED, CLOSE_OBS,CLOSE_OBS);
		motor_turn(DODGE_SPEED, -DODGE_SPEED, QUAT_TURN,QUAT_TURN);
		length = left_side;
		if(VL53L0X_get_dist_mm()>=(length+2*R_EPUCK))
		{
			go_along = 1;
		}
		else
		{
			length=right_side;
			motor_turn(-DODGE_SPEED, DODGE_SPEED, HALF_TURN,HALF_TURN);
			if(VL53L0X_get_dist_mm()>=(length+2*R_EPUCK))
			{
				dodge_obs = S_RIGHT;
				go_along = 1;
			}
			else
			{
				motor_turn(DODGE_SPEED, -DODGE_SPEED, HALF_TURN,HALF_TURN);
				escape_dead_end();
			}
		}
	}
}

void escape_dead_end(void)
{
	//vl..get_dist check, avancer de la valeur check, tourner du coté ou il y a pas l'obstracle de fond check,
	//longer jusqu'au bord check, puis longer de nouveau
	uint16_t distance_mur = VL53L0X_get_dist_mm()-5;
	uint16_t step_2_go = mm2steps(distance_mur);
	motor_turn(DODGE_SPEED, DODGE_SPEED, step_2_go, step_2_go);
	if(dodge_obs == S_RIGHT)
	{
		motor_turn(-DODGE_SPEED, DODGE_SPEED, QUAT_TURN,QUAT_TURN);
		go_along = 1;
	}
	else if(dodge_obs == S_LEFT)
	{
		motor_turn(DODGE_SPEED, -DODGE_SPEED, QUAT_TURN,QUAT_TURN);
		go_along = 1;
	}
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
			if(distance[5] >= IR_MID_DIST )
			{
				motor_turn(DODGE_SPEED, DODGE_SPEED, ONE_DEG,ONE_DEG);
			}
			else if(distance[6] < IR_FAR)
			{
				go_along = S_RIGHT;
				dodge_obs = ESCAPE;
			}
			//break;
		}
	else if(dodge_obs== S_LEFT)
		{

			if(distance[2] < IR_MID_DIST)
			{
				motor_turn(DODGE_SPEED, DODGE_SPEED, ONE_DEG,ONE_DEG);
			}
			else if(distance[1] < IR_FAR)
			{
				dodge_obs = ESCAPE;
				go_along = S_LEFT;
			}
			//break;
		}
	else if(dodge_obs== ESCAPE)
		{

				motor_turn(DODGE_SPEED, DODGE_SPEED, 380, 380);
				if(go_along == S_LEFT)
				{
					motor_turn(-MIN_SPEED, MIN_SPEED, QUAT_TURN,QUAT_TURN);
				}
				else if(go_along == S_RIGHT)
				{
					motor_turn(MIN_SPEED, -MIN_SPEED, QUAT_TURN,QUAT_TURN);
				}

				dodge_obs = IDLE;
				go_along =0;
				//break;

		}

	//}
}






