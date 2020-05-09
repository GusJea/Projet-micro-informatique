#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <audio_processing.h>

#define KP				4
#define KI				0.05
#define INTENSITY_MAX	1000000.0
#define MAX_SUM_ERROR	44000.0
#define MAX_ERROR		50000.0
#define ERROR_THRESHOLD	50000.0

#define V_SLOW			600
#define V_NULL			0

/* pi_state determine if the pi_regulator is used or not. It depends of the frequency.
 * -> 0: PI is not used
 * -> 1: PI is used
 */
static  int8_t pi_state = STATE_PI;
static  int8_t pi_dir = DIR_STOP;
static  int8_t pi_dir_phase = DIR_STOP;

//simple PI regulator implementation
int16_t pi_regulator(float intensity, float goal){

	float error = 0;
	float speed = 0;
	static float sum_error = 0;
	error = goal - intensity;

	chprintf((BaseSequentialStream *) &SD3, "intensity : %.3f\n\r", intensity);
	chprintf((BaseSequentialStream *) &SD3, "error : %.3f\n\r", error);

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the mics are a bit noisy
	if(fabs(error) < ERROR_THRESHOLD)
	{
		return V_NULL;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR)
	{
		sum_error = MAX_SUM_ERROR;
	}
	else if(sum_error < -MAX_SUM_ERROR)
	{
		sum_error = -MAX_SUM_ERROR;
	}
	speed = KP * error + KI * sum_error;
    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed = V_NULL;
    //int16_t speed_correction = 0;

    while(1)
    {
        time = chVTGetSystemTime();
       	//computes the speed to give to the motors
       	//the intensity is modified by the sound processing thread
        if(pi_state == STATE_PI)
        {
        	speed = pi_regulator(get_intensity(), INTENSITY_MAX);
        }
        else
        {
        	speed = V_SLOW;
        }
       	//computes a correction factor to let the robot rotate to be in front of the line
       	//speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

        	//if the line is nearly in front of the camera, don't rotate
        	//if(abs(speed_correction) < ROTATION_THRESHOLD){
        		//speed_correction = 0;
        	//}

        //applies the speed and the correction for the rotation
        switch(pi_dir)
        {
        	case DIR_LEFT:
               	right_motor_set_speed(speed);
               	left_motor_set_speed(-speed);
        		break;
        	case DIR_RIGHT:
               	right_motor_set_speed(-speed);
               	left_motor_set_speed(speed);
        		break;
        	case DIR_FORWARD:
        		if(pi_dir_phase == DIR_FORWARD)
        		{
                   	right_motor_set_speed(speed);
                   	left_motor_set_speed(speed);
        		}
        		else if(pi_dir_phase == DIR_LEFT)
               	{
        			right_motor_set_speed(speed+300);
                   	left_motor_set_speed(speed-300);
               	}else if(pi_dir_phase == DIR_RIGHT)
               	{
               		right_motor_set_speed(speed-300);
                   	left_motor_set_speed(speed+300);
               	}
        		break;
        	case DIR_BACKWARD:
               	right_motor_set_speed(-speed);
               	left_motor_set_speed(-speed);
        		break;
        	case DIR_STOP:
        		right_motor_set_speed(V_NULL);
            	left_motor_set_speed(V_NULL);
        		break;
        }
       	//10kHz
       	chThdSleepUntilWindowed(time, time + MS2ST(0.1));
    }
}

void pi_regulator_start(void)
{
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

/*
 * Set the state to know if the pi regulator is active or not and the direction
 * state:
 * -> 0: no PI
 * -> 1: PI
 * direction:
 * -> 0: left
 * -> 1: right
 * -> 2: forward
 * -> 3: backward
 */
void set_state(int8_t state, int8_t direction)
{
	pi_state=state;
	pi_dir=direction;
}

/*
 * Set the correction of the direction due to the phase between the right and the left mics
 */
void set_phase(int8_t direction_phase)
{
	pi_dir_phase = direction_phase;
}
