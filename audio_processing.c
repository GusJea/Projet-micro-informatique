#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdio.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <pi_regulator.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE); // @suppress("Field cannot be resolved")

//static int8_t old_phase = 0;

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	30000
#define MAX_VALUE_LIMIT		1000000
#define INIT_VALUE			0

#define FREQ_MIN		45	//Start scanning
#define FREQ_FAST		50	//779Hz
#define FREQ_MAX		55	//End scanning

#define FREQ_FAST_L		FREQ_FAST-1
#define FREQ_FAST_H		FREQ_FAST+1

#define FRONT_BACK		1
#define LEFT_RIGHT		2

static 	int8_t old_phase = 0;
static  int8_t dir_sound = DIR_STOP;
static float coef_right = V_NULL;
static float coef_left = V_NULL;

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* dataR, float* dataL, float* dataF, float* dataB)
{
	float max_norm_r = MIN_VALUE_THRESHOLD;
	float max_norm_l = MIN_VALUE_THRESHOLD;
	float max_norm_f = MIN_VALUE_THRESHOLD;
	float max_norm_b = MIN_VALUE_THRESHOLD;

	int8_t max_index_l = INIT_VALUE;
	int8_t max_index_r = INIT_VALUE;
	int8_t max_index_f = INIT_VALUE;
	int8_t max_index_b = INIT_VALUE;

	int8_t direction = DIR_STOP;

	//Search for the maximum sound intensity between the 4 mics to determine the direction and frequency peak
	for(int i=FREQ_MIN; i <=FREQ_MAX; i++)
	{
		//Maximum intensity and frequency peak of the right and left mics
		if(dataL[i]>=max_norm_l)
		{
			max_norm_l=dataL[i];
			max_index_l=i;
		}
		if(dataR[i]>=max_norm_r)
		{
			max_norm_r=dataR[i];
			max_index_r=i;
		}
		if(dataF[i]>=max_norm_f)
		{
			max_norm_f=dataF[i];
			max_index_f=i;
		}
		if(dataB[i]>=max_norm_b)
		{
			max_norm_b=dataB[i];
			max_index_b=i;
		}
	}

	float phase_lr = 0;
	float phase_fb = 0;

	///////////////////////////////////////////////////////////////////////
		//Get phase between front-back and left-right
		if(max_index_r == max_index_l && max_index_r >= FREQ_FAST_L && max_index_r <= FREQ_FAST_H)
		{
			phase_lr = phase(max_index_l, LEFT_RIGHT);
		}
		if(max_index_f == max_index_b && max_index_f >= FREQ_FAST_L && max_index_f <= FREQ_FAST_H)
		{
			phase_fb = phase(max_index_f, FRONT_BACK);
		}

		//chprintf((BaseSequentialStream *) &SD3, "phase fb: %.3f\n\r", phase_fb);


		//Chose the direction
		if(phase_fb > 0 && fabs(phase_fb) <= 2.0)
		{
			direction = DIR_FORWARD;
		}
		else if(phase_fb < 0 && fabs(phase_fb) <= 2.0)
		{
			direction = DIR_BACKWARD;
		}

		phase_lr = round(phase_lr*100);

		//Command the motors
		if(direction == DIR_BACKWARD)
		{
			if(phase_lr > 5 && fabs(phase_lr) <= 200.0)
			{
				coef_right = SPEED_POS;
				coef_left = SPEED_NEG;
				//chprintf((BaseSequentialStream *) &SD3, "BACK LEFT\n\r");
				old_phase = 1;
			}
			else if(phase_lr < -5 && fabs(phase_lr) <= 200.0)
			{
				coef_right = SPEED_NEG;
				coef_left = SPEED_POS;
				//chprintf((BaseSequentialStream *) &SD3, "BACK RIGHT\n\r");
				old_phase = -1;
			}
			dir_sound = DIR_BACKWARD;
		}
		else if(direction == DIR_FORWARD)
		{
			if(fabs(phase_lr) <= 45 && fabs(phase_lr) <= 200)
			{
				if(phase_lr > 15 && old_phase == 0)
				{
					coef_right = SPEED_F_POS;
					coef_left = SPEED_F_NEG;
					old_phase = 1;
					dir_sound = DIR_LEFT;
				}
				else if(phase_lr < -15 && old_phase == 0)
				{
					coef_right = SPEED_F_NEG;
					coef_left = SPEED_F_POS;
					old_phase = -1;
					dir_sound = DIR_RIGHT;
				}
				else
				{
					coef_right = SPEED_POS;
					coef_left = SPEED_POS;
					//chprintf((BaseSequentialStream *) &SD3, "FRONT\n\r");
					old_phase = 0;
					dir_sound = DIR_FORWARD;
				}
			}
			else if(phase_lr > 45 && fabs(phase_lr) <= 200)
			{
				coef_right = SPEED_POS;
				coef_left = SPEED_NEG;
				dir_sound = DIR_LEFT;
				//chprintf((BaseSequentialStream *) &SD3, "LEFT\n\r");
			}
			else if(phase_lr < -45 && fabs(phase_lr) <= 200)
			{
				coef_right = SPEED_NEG;
				coef_left = SPEED_POS;
				dir_sound = DIR_RIGHT;
				//chprintf((BaseSequentialStream *) &SD3, "RIGHT\n\r");
			}
		}
		///////////////////////////////////////////////////////////////
	if(max_norm_f >= MAX_VALUE_LIMIT || max_norm_f <= MIN_VALUE_THRESHOLD)
	{
		coef_right = V_NULL;
		coef_left = V_NULL;
		dir_sound = DIR_STOP;
	}
	motor_command(coef_right, coef_left);
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*/

	static uint16_t nb_samples = 0;
	//static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		/*if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}*/
		nb_samples = 0;
		//mustSend++;

		sound_remote(micRight_output, micLeft_output, micFront_output, micBack_output);
	}
}

void wait_send_to_computer(void)
{
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name)
{
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}

/*
 * 	Simple function to get the phase
 */
float phase(int8_t index, int8_t state)
{
	float phase = 0;
	if(state == LEFT_RIGHT)
	{
		float phase_right = 0;
		float phase_left = 0;
		phase_right = (float)atan(micRight_cmplx_input[2*index+1]/micRight_cmplx_input[2*index]);
		phase_left = (float)atan(micLeft_cmplx_input[2*index+1]/micLeft_cmplx_input[2*index]);
		phase = (phase_left-phase_right);
	}
	else if(state == FRONT_BACK)
	{
		float phase_front = 0;
		float phase_back = 0;
		phase_front = (float)atan(micFront_cmplx_input[2*index+1]/micFront_cmplx_input[2*index]);
		phase_back = (float)atan(micBack_cmplx_input[2*index+1]/micBack_cmplx_input[2*index]);
		phase = (phase_front-phase_back);
	}
	return phase;
}

/*
 * 	Function to set the speed to the motors
 */
void motor_command(float coef_r, float coef_l)
{
	right_motor_set_speed(coef_r*V_SLOW);
	left_motor_set_speed(coef_l*V_SLOW);
}

int8_t get_dir_sound()
{
	return dir_sound;
}
