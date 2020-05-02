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

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


#define	DIST_STOP_LONG		500
#define	DIST_STOP_SHORT		150
#define IR_NUMBER			8 // de 0 a 7 donc 8 sensor
#define TAN(x)				tan(x)
#define SIN(x)				sin(x)
#define ACOS(x)				acos(x)
#define DEG2RAD				M_PI/180

#define S_LEFT				1
#define	S_RIGHT 			2
#define ESCAPE				3
#define ESCAPE_2			7
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
#define R_EPCUK_ROUND		40

#define IR_CLOSE			3100
#define IR_CLOSE_L			3400
#define IR_CLOSE_R			3500
#define IR_MID_DIST			400
#define	IR_FAR				70

#define PERIMETER			130
#define NB_STEP				1000
#define CORRECTION			210

static uint8_t				dodge_obs = IDLE;
static uint8_t				go_along = 0;
static uint16_t				length = 0;
static uint16_t				old_ir = IR_MID_DIST;
static int8_t				counter_ir = 0;

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
	//int test[8] = {0};
	while (1)
	{
		/*ir_values(test);
		chprintf((BaseSequentialStream *) &SD3, "----------------------\n\r");

		chprintf((BaseSequentialStream *) &SD3, "ir 0: %d\n\r", test[0]);
		chprintf((BaseSequentialStream *) &SD3, "ir 7: %d\n\r", test[7]);*/

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
	for(int i = 0; i < IR_NUMBER; i++)
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
	counter_ir++;

	if((distance[7] >= IR_MID_DIST || distance[6] >= IR_MID_DIST || distance[5] >= IR_MID_DIST) && counter_ir == 25)
	{
		dodge_obs = S_RIGHT;
		detect_left_obs(distance);
		if(dodge_obs == ESCAPE)
		{
			motor_turn(DODGE_SPEED, DODGE_SPEED, mm2steps(R_EPUCK), mm2steps(R_EPUCK));
			old_ir = IR_MID_DIST;
			go_along = 0;
		}
		dodge_obs = IDLE;
		counter_ir = 0;
	}
	else if((distance[0] >= IR_MID_DIST || distance[1] >= IR_MID_DIST || distance[2] >= IR_MID_DIST) && counter_ir == 25)
	{
		dodge_obs = S_LEFT;
		detect_right_obs(distance);
		if(dodge_obs == ESCAPE)
		{
			motor_turn(DODGE_SPEED, DODGE_SPEED, 380, 380);
			old_ir = IR_MID_DIST;
			go_along = 0;
		}
		dodge_obs = IDLE;
		counter_ir = 0;
	}
}

/*
 * @brief center the epuck with IR sensor in order to be perpendicular to the object to avoid
 */
void center_IR()
{
	int distance[IR_NUMBER] = {0};
	ir_values(distance);

	//align�
	if((distance[0] >= IR_CLOSE_L) && (distance[7] >= IR_CLOSE_R))
	{
		dodge_obs = CENTER;
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	//dans un bord d'objet qui est a la droite du robot
	/*else if(distance[0] >= IR_CLOSE && distance[7] < IR_FAR)
	{
		motor_turn(-DODGE_SPEED	, DODGE_SPEED, QUAT_TURN, QUAT_TURN);
		motor_turn(DODGE_SPEED	, DODGE_SPEED, mm2steps(R_EPCUK_ROUND), mm2steps(R_EPCUK_ROUND));
		go_along= 0;
		dodge_obs=IDLE;
	}
	//dans un bord d'objet qui est a la gauche du robot
	else if(distance[7] >= IR_CLOSE && distance[0] < IR_FAR)
	{
		motor_turn(DODGE_SPEED	, -DODGE_SPEED, QUAT_TURN, QUAT_TURN);
		motor_turn(DODGE_SPEED	, DODGE_SPEED, mm2steps(R_EPCUK_ROUND), mm2steps(R_EPCUK_ROUND));
		go_along= 0;
		dodge_obs=IDLE;
	}*/
	//trop a gauche
	else if(distance[0] >= IR_CLOSE_R || distance[1] >= IR_MID_DIST)
	{
		motor_turn(-MIN_SPEED, MIN_SPEED, ONE_DEG,ONE_DEG);
	}
	//trop a droite
	else if(distance[7] >= IR_CLOSE_L || distance[6] >= IR_MID_DIST)
	{
		motor_turn(MIN_SPEED, -MIN_SPEED, ONE_DEG, ONE_DEG);
	}
	else
		motor_turn(MIN_SPEED, MIN_SPEED, ONE_DEG, ONE_DEG);
}

/*
 * @brief main function of the object scan and mesurment withg the TOF sensor.
 */
void scan_obstacle()
{
	if(get_dir_sound() != DIR_STOP)
	{
		uint16_t distance_mm = VL53L0X_get_dist_mm();
		int IR_values[IR_NUMBER] = {0};
		ir_values(IR_values);
		//chprintf((BaseSequentialStream *) &SD3, "distance: %d\n\r", distance_mm);

		if(distance_mm <= DIST_STOP_SHORT && distance_mm)
		{
			if(dodge_obs == IDLE)
			{
				dodge_obs = TO_CENTER;
			}
			else if(dodge_obs == CENTER && !go_along)
			{
				while(distance_mm < DIST_STOP_SHORT)
				{
					ir_values(IR_values);
					if(IR_values[3] >= IR_MID_DIST && IR_values[4] >= IR_MID_DIST)
					{
						break;
					}
					motor_turn(-MIN_SPEED, -MIN_SPEED, ONE_DEG, ONE_DEG);
					distance_mm = VL53L0X_get_dist_mm();
				}
				length=distance_mm;
				direction_choose(obstacle_length_left(),obstacle_length_right());
			}
			else if(go_along)
			{
				dodge_obs = TO_CENTER;
				go_along = 0;
			}
		}
		else if(distance_mm > DIST_STOP_LONG && !go_along)
		{
			dodge_obs = IDLE;
		}
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
			motor_turn(V_SLOW, V_SLOW, mm2steps(50), mm2steps(50));
		}
	}
}

/*
 * @brief	measure the right length of the object to avoid
 *
 * @param	return the right length
 */
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
		ob_leng_mm = tan(i*DEG2RAD)*((double)(length)+R_EPUCK);

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

/*
 * @brief	measure the left length of the object to avoid
 *
 * @param	return the left length
 */

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
		ob_leng_mm = tan(i*DEG2RAD)*((double)(length)+R_EPUCK);

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

/*
 * @brief	choose which direction to go and start to avoid the object.
 * 						Also make sure the epuck can avoid the obstacle without beeing blocked by another object on the right/left
 *
 * @pram left_side 		length of the left side of the object
 * @pram right_side 	length of the right side of the object
 */
void direction_choose(uint16_t left_side, uint16_t right_side)
{
	motor_turn(DODGE_SPEED, DODGE_SPEED, mm2steps(length)-CORRECTION, mm2steps(length)-CORRECTION);

	if(left_side > right_side)
	{
		dodge_obs = S_RIGHT;
		motor_turn(-DODGE_SPEED, DODGE_SPEED, QUAT_TURN,QUAT_TURN);
		if(VL53L0X_get_dist_mm()>=(right_side+2*R_EPUCK))
		{
			go_along = 1;
		}
		else
		{
			motor_turn(DODGE_SPEED, -DODGE_SPEED, HALF_TURN,HALF_TURN);
			if(VL53L0X_get_dist_mm()>=(left_side+2*R_EPUCK))
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
		motor_turn(DODGE_SPEED, -DODGE_SPEED, QUAT_TURN,QUAT_TURN);
		if(VL53L0X_get_dist_mm()>=(left_side+2*R_EPUCK))
		{
			go_along = 1;
		}
		else
		{
			motor_turn(-DODGE_SPEED, DODGE_SPEED, HALF_TURN,HALF_TURN);
			if(VL53L0X_get_dist_mm()>=(right_side+2*R_EPUCK))
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

/*
 * @brief	"save" if the epuck is stuk in a dead end by avoiding it
 */
void escape_dead_end(void)
{
	//vl..get_dist check, avancer de la valeur check, tourner du cot� ou il y a pas l'obstracle de fond check,
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

/*
 *	@brief	 move the eupck at a fixed speed to a cetrain position
 *
 *	@param speed_r		set the right motor speed
 *	@param speed_l		set the left motor speed
 *	@param pos_right	set the right position to be reached by the motor in step
 *	@param pos_left		set the left position to be reached by the motor in step
 *
 */
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

uint16_t mm2steps(uint16_t millimeters)
{
	return (millimeters*NB_STEP/PERIMETER);
}

/*
 * @brief	Go along the object to avoid using the IR sensor
 */
void obj_ir_dodge()
{
	int distance[IR_NUMBER] = {0};
	ir_values(distance);

	go_along_after_dodge(distance);

	if(dodge_obs== ESCAPE)
	{
		motor_turn(DODGE_SPEED, DODGE_SPEED, 290, 290);
		if(go_along == S_LEFT)
		{
			motor_turn(-MIN_SPEED, MIN_SPEED, QUAT_TURN,QUAT_TURN);
		}
		else if(go_along == S_RIGHT)
		{
			motor_turn(MIN_SPEED, -MIN_SPEED, QUAT_TURN,QUAT_TURN);
		}
		motor_turn(DODGE_SPEED, DODGE_SPEED, 380, 380);
		dodge_obs = IDLE;
		go_along =0;
		old_ir = IR_MID_DIST;
	}
}

void go_along_after_dodge(int* distance)
{
	if(dodge_obs == S_RIGHT)
	{
		if(distance[5] >= IR_MID_DIST)
		{
			if(distance[5] > old_ir)
			{
				motor_turn(-DODGE_SPEED, DODGE_SPEED, 4*ONE_DEG, 4*ONE_DEG);
			}
			else if(distance[5] < old_ir)
			{
				motor_turn(DODGE_SPEED, -DODGE_SPEED, 4*ONE_DEG, 4*ONE_DEG);
			}
			motor_turn(DODGE_SPEED, DODGE_SPEED, ONE_DEG,ONE_DEG);
			old_ir = distance[5];
		}
		else if(distance[6] < IR_FAR)
		{
			go_along = S_RIGHT;
			dodge_obs = ESCAPE;
		}
		//break;
	}
	else if(dodge_obs == S_LEFT)
	{
		if(distance[2] >= IR_MID_DIST)
		{
			if(distance[2] > old_ir)
			{
				motor_turn(DODGE_SPEED, -DODGE_SPEED, 4*ONE_DEG, 4*ONE_DEG);
			}
				else if(distance[2] < old_ir)
					{
						motor_turn(-DODGE_SPEED, DODGE_SPEED, 4*ONE_DEG, 4*ONE_DEG);
					}
					motor_turn(DODGE_SPEED, DODGE_SPEED, ONE_DEG,ONE_DEG);
					old_ir = distance[2];
				}
				else if(distance[1] < IR_FAR)
				{
					go_along = S_LEFT;
					dodge_obs = ESCAPE;
				}
				//break;
			}
}

void detect_left_obs(int* distance)
{
	//longé assez
	if(distance[5] >= IR_MID_DIST)
	{
		go_along_after_dodge(distance);
	}
	else
	{
		chprintf((BaseSequentialStream *) &SD3, "distance 5: %d\n\r", distance[5]);
		chprintf((BaseSequentialStream *) &SD3, "distance 6: %d\n\r", distance[6]);
		chprintf((BaseSequentialStream *) &SD3, "distance 7: %d\n\r", distance[7]);
		chprintf((BaseSequentialStream *) &SD3, "------------------------ %d\n\r");


		motor_turn(-DODGE_SPEED, DODGE_SPEED, 45*ONE_DEG, 45*ONE_DEG);
		dodge_obs = ESCAPE;
	}
}

void detect_right_obs(int* distance)
{
	//longé assez
	if(distance[2] >= IR_MID_DIST)
	{
		go_along_after_dodge(distance);
	}
	else
	{
		motor_turn(DODGE_SPEED, -DODGE_SPEED, 45*ONE_DEG, 45*ONE_DEG);
		dodge_obs = ESCAPE;
	}
}

