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
#include <chprintf.h>
#include <audio_processing.h>
#include "msgbus/messagebus.h"
#include "arm_math.h"
#include "arm_common_tables.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

#define	DIST_STOP_LONG		300
#define	DIST_STOP_SHORT		150
#define R_EPUCK				37.5
#define PERIMETER			130
#define NB_STEP				1000
#define NBR_MOT				2
#define MOT_L				0
#define MOT_R				1
#define DEG2RAD				M_PI/180
#define	DEG_MAX				90

#define ON					1
#define	OFF					0
#define ESCAPE				3
#define TO_CENTER			4
#define CENTER				5
#define IDLE				6

#define ONE_DEG				4
#define QUAT_TURN			320
#define HALF_TURN			640
#define STEP_CORRECTION		210

#define DODGE_SPEED			400
#define MIN_SPEED			326
#define	ESCAPE_SPEED		290

#define CLOSE_OBS			400
#define POS_DODGE			380

#define TOF_CORRECTION		5

#define IR_NUMBER			8
#define IR_CLOSE_L			3400
#define IR_CLOSE_R			3500
#define IR_MID_DIST			400
#define	IR_FAR				70
#define IR_COUNTER			25
#define IR_CORRECTION		10

#define	RIGHT_FRONT_IR_SENS	0
#define	RIGHT_45_IR_SENS	1
#define	RIGHT_IR_SENS		2
#define	RIGHT_BACK_IR_SENS	3

#define	LEFT_BACK_IR_SENS	4
#define	LEFT_IR_SENS		5
#define	LEFT_45_IR_SENS		6
#define	LEFT_FRONT_IR_SENS	7

static uint8_t				dodge_obs = IDLE;
static uint8_t				go_along = 0;
static uint16_t				length = 0;
static uint16_t				old_ir = IR_MID_DIST;
static int8_t				counter_ir = 0;
static int8_t 				obstacle_ir = 0;

/*
 * @brief				initialize the IR sensor and the TOF sensor
 */
void sensor_init()
{
	//TOF
	VL53L0X_start();

	//IR init
	messagebus_init( & bus, & bus_lock, & bus_condvar);
	proximity_start();
	calibrate_ir();
}


/*
 * @brief				thread handling the TOF detection
 */
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

/*
 * @brief				thread handling the IR sensor
 * 						The counter_ir is there to slow down the frequency of ir
 * 						measurement because the ir sensor frequency is lower than thread call
 */
static THD_WORKING_AREA(waDetectObjet_IR, 256);
static THD_FUNCTION(DetectObjet_IR, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while (1)
	{
		counter_ir++;
		if(dodge_obs == TO_CENTER && counter_ir >= IR_COUNTER-IR_CORRECTION)
		{
			center_IR();
			counter_ir = OFF;
		}
		else if(dodge_obs == IDLE && (counter_ir >= IR_COUNTER || obstacle_ir))
		{
			detect_IR();
			counter_ir = OFF;
		}
		else if(go_along)
		{
			obj_ir_dodge();
			counter_ir = OFF;
		}
		chThdSleepMilliseconds(1);
	}
}

void object_detect_start()
{
	chThdCreateStatic(waDetectObjet_IR, sizeof(waDetectObjet_IR), NORMALPRIO+1, DetectObjet_IR, NULL);
}

/*
 * @brief 				get the IR sensor values
 *
 * @param	valeurs 	value mesured by the sensor
 */
void ir_values(int* valeurs)
{
	for(int i = 0; i < IR_NUMBER; i++)
	{
		valeurs[i]=get_calibrated_prox(i);
	}
}

/*
 * @brief 				store the IR sensor values in distance table and then compare them
 *
 */
void detect_IR()
{
	if(get_dir_sound() != DIR_STOP)
	{
		int distance[IR_NUMBER] = {OFF};
		ir_values(distance);

		if((distance[LEFT_FRONT_IR_SENS] >= IR_MID_DIST || distance[LEFT_45_IR_SENS] >= IR_MID_DIST || distance[LEFT_IR_SENS] >= IR_MID_DIST))
		{
			dodge_obs = DIR_RIGHT;
			detect_left_obs(distance);
			if(dodge_obs == ESCAPE)
			{
				motor_turn(DODGE_SPEED, DODGE_SPEED, mm2steps(R_EPUCK), mm2steps(R_EPUCK));
				old_ir = IR_MID_DIST;
				go_along = OFF;
				obstacle_ir = OFF;
			}
			dodge_obs = IDLE;
		}
		else if((distance[RIGHT_FRONT_IR_SENS] >= IR_MID_DIST || distance[RIGHT_45_IR_SENS] >= IR_MID_DIST || distance[RIGHT_IR_SENS] >= IR_MID_DIST - 10*IR_CORRECTION))
		{
			dodge_obs = DIR_LEFT;
			detect_right_obs(distance);
			if(dodge_obs == ESCAPE)
			{
				motor_turn(DODGE_SPEED, DODGE_SPEED, POS_DODGE, POS_DODGE);
				old_ir = IR_MID_DIST;
				go_along = OFF;
				obstacle_ir = OFF;
			}
			dodge_obs = IDLE;
		}
	}
}

/*
 * @brief 				center the epuck with IR sensor in order to be perpendicular to the object to avoid
 */
void center_IR()
{
	if(get_dir_sound()!=DIR_STOP)
	{
		int distance[IR_NUMBER] = {OFF};
		ir_values(distance);

		//the epuck is align
		if((distance[RIGHT_FRONT_IR_SENS] >= IR_CLOSE_L) && (distance[LEFT_FRONT_IR_SENS] >= IR_CLOSE_R))
		{
			dodge_obs = CENTER;
			left_motor_set_speed(V_NULL);
			right_motor_set_speed(V_NULL);
		}

		//epuck is too much on the left side
		else if((distance[RIGHT_FRONT_IR_SENS] >= IR_CLOSE_R || distance[RIGHT_45_IR_SENS] >= IR_MID_DIST))
		{
			motor_turn(-MIN_SPEED, MIN_SPEED, ONE_DEG,ONE_DEG);
		}

		//epuck is too much on the right side
		else if((distance[LEFT_FRONT_IR_SENS] >= IR_CLOSE_L || distance[LEFT_45_IR_SENS] >= IR_MID_DIST))
		{
			motor_turn(MIN_SPEED, -MIN_SPEED, ONE_DEG, ONE_DEG);
		}

		//if the epuck is on the corner, need to use detect_ir and no more center_ir
		else if(VL53L0X_get_dist_mm() >= (DIST_STOP_SHORT/3) && (distance[LEFT_FRONT_IR_SENS] >= IR_MID_DIST || distance [RIGHT_FRONT_IR_SENS] >= IR_MID_DIST))
		{
			dodge_obs = IDLE;
		}

		//go forward while the object is far
		else
			motor_turn(MIN_SPEED, MIN_SPEED, ONE_DEG, ONE_DEG);
	}
}

/*
 * @brief 				main function of the object scan and mesurment withg the TOF sensor.
 */
void scan_obstacle()
{
	if(get_dir_sound() != DIR_STOP)
	{
		uint16_t distance_mm = VL53L0X_get_dist_mm();
		int IR_values[IR_NUMBER] = {OFF};
		ir_values(IR_values);

		if(distance_mm <= DIST_STOP_SHORT && distance_mm)
		{
			obstacle_ir = OFF;

			//first detection
			if(dodge_obs == IDLE)
			{
				dodge_obs = TO_CENTER;
			}
			//already center, then proceed to measure it
			else if(dodge_obs == CENTER && !go_along)
			{
				while(distance_mm < DIST_STOP_SHORT)
				{
					ir_values(IR_values);

					if(IR_values[LEFT_FRONT_IR_SENS] >= IR_MID_DIST && IR_values[LEFT_BACK_IR_SENS] >= IR_MID_DIST)
					{
						break;
					}
					motor_turn(-MIN_SPEED, -MIN_SPEED, ONE_DEG, ONE_DEG);
					distance_mm = VL53L0X_get_dist_mm();
				}

				length=distance_mm;
				direction_choose(obstacle_length_left(),obstacle_length_right());
			}
			//already going along the object
			else if(go_along)
			{
				dodge_obs = TO_CENTER;
				go_along = OFF;
			}
		}
		else if(distance_mm > DIST_STOP_LONG && !go_along)
		{
			old_ir = IR_MID_DIST;
			dodge_obs = IDLE;
		}
		//dodge a little bit if the epuck is far from the obstacle
		else if(distance_mm <= DIST_STOP_LONG && distance_mm)
		{
			if(get_dir_sound() == DIR_LEFT || get_dir_sound() == DIR_FORWARD)
			{
				motor_turn(V_SLOW, -V_SLOW, 2*ONE_DEG, 2*ONE_DEG);
			}
			else if(get_dir_sound() == DIR_RIGHT)
			{
				motor_turn(-V_SLOW, V_SLOW, 2*ONE_DEG, 2*ONE_DEG);
			}
			// atst motor_turn(V_SLOW, V_SLOW, mm2steps(50), mm2steps(50));
			motor_turn(V_SLOW, V_SLOW, POS_DODGE, POS_DODGE);
		}
	}
}

/*
 * @brief				measure the right length of the object to avoid
 *
 * @param	return the right length
 */
uint16_t obstacle_length_right(void)
{
	uint16_t return_value[NBR_MOT] = {OFF};
	float32_t ob_leng_TOF_mm = OFF;
	float32_t ob_leng_mm = OFF;
	for(uint8_t i = 1; i<DEG_MAX; i++)
	{

		motor_turn(-MIN_SPEED, MIN_SPEED, ONE_DEG,ONE_DEG);
		return_value[MOT_L] += left_motor_get_pos();
		return_value[MOT_R] += right_motor_get_pos();

		ob_leng_TOF_mm = arm_sin_f32(i*DEG2RAD)*((float32_t)(VL53L0X_get_dist_mm()));
		float32_t	tan_i = arm_sin_f32(i*DEG2RAD)/arm_cos_f32(i*DEG2RAD);
		ob_leng_mm = tan_i*((float32_t)(length)+R_EPUCK);

		if(ob_leng_TOF_mm > ob_leng_mm)
		{
			break;
		}
	}

	right_motor_set_pos(return_value[MOT_R]);
	left_motor_set_pos(return_value[MOT_L]);

	while(1)
	{
				if((right_motor_get_pos() > OFF) && (left_motor_get_pos() < OFF))
				{
					left_motor_set_speed(V_NULL);
					right_motor_set_speed(V_NULL);
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

/*
 * @brief				measure the left length of the object to avoid
 *
 */

uint16_t obstacle_length_left(void)
{
	uint16_t return_value[NBR_MOT] = {OFF};
	float32_t ob_leng_TOF_mm = OFF;
	float32_t ob_leng_mm = OFF;
	for(uint8_t i = 1; i<DEG_MAX; i++)
	{

		motor_turn(MIN_SPEED, -MIN_SPEED, ONE_DEG,ONE_DEG);
		return_value[MOT_L] += left_motor_get_pos();
		return_value[MOT_R] += right_motor_get_pos();

		ob_leng_TOF_mm = arm_sin_f32(i*DEG2RAD)*((float32_t)(VL53L0X_get_dist_mm()));
		float32_t	tan_i = arm_sin_f32(i*DEG2RAD)/arm_cos_f32(i*DEG2RAD);
		ob_leng_mm = tan_i*((float32_t)(length)+R_EPUCK);

		if(ob_leng_TOF_mm > ob_leng_mm)
		{
			break;
		}
	}

	right_motor_set_pos(return_value[MOT_R]);
	left_motor_set_pos(return_value[MOT_L]);

	while(1)
	{
		// atst ptit doute pour le -2*on deg car c'est 8 et non 7 comme avant
		if((right_motor_get_pos() < -2*ONE_DEG) && (left_motor_get_pos() > 2*ONE_DEG))
		{
			left_motor_set_speed(V_NULL);
			right_motor_set_speed(V_NULL);
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

/*
 * @brief				choose which direction to go and start to avoid the object.
 * 						Also make sure the epuck can avoid the obstacle without beeing blocked by another object on the right/left
 *
 * @pram left_side 		length of the left side of the object
 * @pram right_side 	length of the right side of the object
 */
void direction_choose(uint16_t left_side, uint16_t right_side)
{
	motor_turn(DODGE_SPEED, DODGE_SPEED, mm2steps(length)-STEP_CORRECTION, mm2steps(length)-STEP_CORRECTION);

	if(left_side > right_side)
	{
		dodge_obs = DIR_RIGHT;
		motor_turn(-DODGE_SPEED, DODGE_SPEED, QUAT_TURN,QUAT_TURN);
		if(VL53L0X_get_dist_mm()>=(right_side+2*R_EPUCK))
		{
			go_along = DIR_RIGHT;
		}
		else
		{
			motor_turn(DODGE_SPEED, -DODGE_SPEED, HALF_TURN,HALF_TURN);
			if(VL53L0X_get_dist_mm()>=(left_side+2*R_EPUCK))
			{
				dodge_obs = DIR_LEFT;
				go_along = DIR_LEFT;
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
		dodge_obs = DIR_LEFT;
		// atst motor_turn(DODGE_SPEED, -DODGE_SPEED, QUAT_TURN-7,QUAT_TURN-7);
		motor_turn(DODGE_SPEED, -DODGE_SPEED, QUAT_TURN-2*ONE_DEG,QUAT_TURN-2*ONE_DEG);
		if(VL53L0X_get_dist_mm()>=(left_side+2*R_EPUCK))
		{
			go_along = DIR_LEFT;
		}
		else
		{
			motor_turn(-DODGE_SPEED, DODGE_SPEED, HALF_TURN,HALF_TURN);
			if(VL53L0X_get_dist_mm()>=(right_side+2*R_EPUCK))
			{
				dodge_obs = DIR_RIGHT;
				go_along = DIR_RIGHT;
			}
			else
			{
				motor_turn(DODGE_SPEED, -DODGE_SPEED, HALF_TURN,HALF_TURN);
				escape_dead_end();
			}
		}
	}
}

/*
 * @brief				"save" if the epuck is stuk in a dead end by avoiding it
 */
void escape_dead_end(void)
{
	uint16_t distance_mur = VL53L0X_get_dist_mm()-TOF_CORRECTION;
	uint16_t step_2_go = mm2steps(distance_mur);

	motor_turn(DODGE_SPEED, DODGE_SPEED, step_2_go, step_2_go);
	if(dodge_obs == DIR_RIGHT)
	{
		motor_turn(-DODGE_SPEED, DODGE_SPEED, QUAT_TURN,QUAT_TURN);
		go_along = DIR_RIGHT;
	}
	else if(dodge_obs == DIR_LEFT)
	{
		motor_turn(DODGE_SPEED, -DODGE_SPEED, QUAT_TURN,QUAT_TURN);
		go_along = DIR_LEFT;
	}
}

/*
 *	@brief	 			move the eupck at a fixed speed to a cetrain position
 *
 *	@param speed_r		set the right motor speed
 *	@param speed_l		set the left motor speed
 *	@param pos_right	set the right position to be reached by the motor in step
 *	@param pos_left		set the left position to be reached by the motor in step
 *
 */
void motor_turn(int speed_r, int speed_l, int32_t pos_right, int32_t pos_left)
{
	right_motor_set_pos(OFF);
	left_motor_set_pos(OFF);
	left_motor_set_speed(V_NULL);
	right_motor_set_speed(V_NULL);
	while(1)
	{
		if((abs(right_motor_get_pos()) > pos_right) && (abs(left_motor_get_pos()) > pos_left))
		{
			left_motor_set_speed(V_NULL);
			right_motor_set_speed(V_NULL);
			break;
		}
		else
		{
			left_motor_set_speed(speed_l);
			right_motor_set_speed(speed_r);
		}
	}
}

uint16_t mm2steps(uint16_t millimeters)
{
	return (millimeters*NB_STEP/PERIMETER);
}

/*
 * @brief				Go along the object to avoid using the IR sensor
 */
void obj_ir_dodge()
{
	int distance[IR_NUMBER] = {OFF};
	ir_values(distance);

	go_along_after_dodge(distance);

	if(dodge_obs == ESCAPE)
	{
		motor_turn(DODGE_SPEED, DODGE_SPEED, ESCAPE_SPEED, ESCAPE_SPEED);
		if(go_along == DIR_LEFT)
		{
			motor_turn(-MIN_SPEED, MIN_SPEED, QUAT_TURN,QUAT_TURN);
		}
		else if(go_along == DIR_RIGHT)
		{
			motor_turn(MIN_SPEED, -MIN_SPEED, QUAT_TURN,QUAT_TURN);
		}
		motor_turn(DODGE_SPEED, DODGE_SPEED, POS_DODGE, POS_DODGE);
		dodge_obs = IDLE;
		go_along = OFF;
		old_ir = IR_MID_DIST;
	}
}
/*
 * @brief				go along a wall and corrects the deviation if there is any
 *
 * @parameters distance	table with the ir sensor values stored in it
 */
void go_along_after_dodge(int* distance)
{
	if(dodge_obs == DIR_RIGHT)
	{
		if(distance[LEFT_IR_SENS] >= IR_MID_DIST)
		{
			if(distance[LEFT_IR_SENS] > old_ir)
			{
				motor_turn(-DODGE_SPEED, DODGE_SPEED, 4*ONE_DEG, 4*ONE_DEG);
			}
			else if(distance[LEFT_IR_SENS] < old_ir)
			{
				motor_turn(DODGE_SPEED, -DODGE_SPEED, 4*ONE_DEG, 4*ONE_DEG);
			}
			motor_turn(DODGE_SPEED, DODGE_SPEED, ONE_DEG,ONE_DEG);
			old_ir = distance[LEFT_IR_SENS];
		}
		else if(distance[LEFT_45_IR_SENS] < IR_FAR)
		{
			go_along = DIR_RIGHT;
			dodge_obs = ESCAPE;
			obstacle_ir = OFF;
		}
	}
	else if(dodge_obs == DIR_LEFT)
	{
		if(distance[RIGHT_IR_SENS] >= IR_MID_DIST-10*POS_DODGE)
		{
			if(distance[RIGHT_IR_SENS] > old_ir)
			{
				motor_turn(DODGE_SPEED, -DODGE_SPEED, 5*ONE_DEG, 5*ONE_DEG);
			}
			else if(distance[RIGHT_IR_SENS] < old_ir)
			{
				motor_turn(-DODGE_SPEED, DODGE_SPEED, 5*ONE_DEG, 5*ONE_DEG);
			}
			motor_turn(DODGE_SPEED, DODGE_SPEED, ONE_DEG,ONE_DEG);
			old_ir = distance[RIGHT_IR_SENS];
		}
		else if(distance[RIGHT_45_IR_SENS] < IR_FAR)
		{
			go_along = DIR_LEFT;
			dodge_obs = ESCAPE;
			obstacle_ir = OFF;
		}
	}
}

/*
 * @brief				detect object on the left of the epuck and avoid if there is any
 *
 * @parameters distance	table with the ir sensor values stored in it
 */
void detect_left_obs(int* distance)
{

	if(distance[LEFT_IR_SENS] >= IR_MID_DIST)
	{
		obstacle_ir = ON;
		go_along_after_dodge(distance);
	}
	else
	{
		motor_turn(-DODGE_SPEED, DODGE_SPEED, 45*ONE_DEG, 45*ONE_DEG);
		dodge_obs = ESCAPE;
		obstacle_ir = OFF;
	}
}

/*
 * @brief				detect object on the right of the epuck and avoid if there is any
 *
 * @parameters distance	table with the ir sensor values stored in it
 */
void detect_right_obs(int* distance)
{

	if(distance[RIGHT_IR_SENS] >= IR_MID_DIST)
	{
		obstacle_ir = ON;
		go_along_after_dodge(distance);
	}
	else
	{
		motor_turn(DODGE_SPEED, -DODGE_SPEED, 45*ONE_DEG, 45*ONE_DEG);
		dodge_obs = ESCAPE;
		obstacle_ir = OFF;
	}
}
