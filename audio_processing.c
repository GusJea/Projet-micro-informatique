#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <pi_regulator.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

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
#define INIT_VALUE			-1

#define FREQ_SLOW		30	//467Hz
#define FREQ_FAST		50	//779Hz

static float global_norm = 0;

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* dataR, float* dataL, float* dataB, float* dataF)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	int8_t direction = INIT_VALUE;
	int16_t max_norm_index = INIT_VALUE;

	//Search for the maximum sound intensity between the 2 frequencies we want to look at
	//for(int i=0; i<2; i++)
	//{
		if(dataL[FREQ_SLOW/*+i*20*/] > max_norm)
		{
			max_norm = dataL[FREQ_SLOW/*+i*20*/];
			max_norm_index = FREQ_SLOW/*+i*20*/;
			direction = DIR_LEFT;
		}
		if(dataR[FREQ_SLOW/*+i*20*/] > max_norm)
		{
			max_norm = dataR[FREQ_SLOW/*+i*20*/];
			max_norm_index = FREQ_SLOW/*+i*20*/;
			direction = DIR_RIGHT;
		}
		if(dataF[FREQ_SLOW/*+i*20*/] > max_norm)
		{
			max_norm = dataF[FREQ_SLOW/*+i*20*/];
			max_norm_index = FREQ_SLOW/*+i*20*/;
			direction = DIR_FORWARD;
		}
		if(dataB[FREQ_SLOW/*+i*20*/] > max_norm)
		{
			max_norm = dataB[FREQ_SLOW/*+i*20*/];
			max_norm_index = FREQ_SLOW/*+i*20*/;
			direction = DIR_BACKWARD;
		}
		/////////////////////////////////////////////////////////////////////////////////////
		//BRUTE FORCE
		if(dataL[FREQ_FAST] > max_norm)
		{
			max_norm = dataL[FREQ_FAST];
			max_norm_index = FREQ_FAST;
			direction = DIR_LEFT;
		}
		if(dataR[FREQ_FAST] > max_norm)
		{
			max_norm = dataR[FREQ_FAST];
			max_norm_index = (FREQ_FAST);
			direction = DIR_RIGHT;
		}
		if(dataF[FREQ_FAST] > max_norm)
		{
			max_norm = dataF[FREQ_FAST];
			max_norm_index = FREQ_FAST;
			direction = DIR_FORWARD;
		}
		if(dataB[FREQ_FAST] > max_norm)
		{
			max_norm = dataB[FREQ_FAST];
			max_norm_index = FREQ_FAST;
			direction = DIR_BACKWARD;
		}
		//FIN BRUTE FORCE
		/////////////////////////////////////////////////////////////////////////////////////
		if(max_norm <= MIN_VALUE_THRESHOLD)
		{
			max_norm_index = FREQ_SLOW;
			direction = DIR_STOP;
		}
	//}

	set_intensity(max_norm);
	motor_command(direction, max_norm_index);
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

		sound_remote(micRight_output, micLeft_output, micBack_output, micFront_output);
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
*	Send the new speed of each motor depending of the direction and the frequency
*	of the highest intensity of the sound detected.
*
*	params :
*	int direction			Tells the direction of the robot:
*							-> 0:	go left
*							-> 1:	go right
*							-> 2:	go forward
*							-> 3:	go backward
*							-> 4:	stop the motors
*	uint16_t max_norm_index	Tells the frequency of the sound captured so the speed can depend of it.
*/
void motor_command(int8_t direction, int16_t max_norm_index)
{
	//check the direction
	switch(direction)
	{
		case DIR_LEFT:
			//check if the PI is used or not
			if(max_norm_index == FREQ_SLOW)
			{
				set_state(STATE_NPI, DIR_LEFT);
			}
			else if(max_norm_index == FREQ_FAST)
			{
				set_state(STATE_PI, DIR_LEFT);
			}
			break;
		case DIR_RIGHT:
			//check if the PI is used or not
			if(max_norm_index == FREQ_SLOW)
			{
				set_state(STATE_NPI, DIR_RIGHT);
			}
			else if(max_norm_index == FREQ_FAST)
			{
				set_state(STATE_PI, DIR_RIGHT);
			}
			break;
		case DIR_FORWARD:
			//check if the PI is used or not
			if(max_norm_index == FREQ_SLOW)
			{
				set_state(STATE_NPI, DIR_FORWARD);
			}
			else if(max_norm_index == FREQ_FAST)
			{
				set_state(STATE_PI, DIR_FORWARD);
			}
			break;
		case DIR_BACKWARD:
			//check if the PI is used or not
			if(max_norm_index == FREQ_SLOW)
			{
				set_state(STATE_NPI, DIR_BACKWARD);
			}
			else if(max_norm_index == FREQ_FAST)
			{
				set_state(STATE_PI, DIR_BACKWARD);
			}
			break;
		case DIR_STOP:
			//check if the PI is used or not
			if(max_norm_index == FREQ_SLOW)
			{
				set_state(STATE_NPI, DIR_STOP);
			}
			else if(max_norm_index == FREQ_FAST)
			{
				set_state(STATE_PI, DIR_STOP);
			}
			break;
	}
}

/*
*	Simple function to set the maximum intensity
*/
void set_intensity(float norm)
{
	global_norm = norm;
}

/*
*	Simple function to get the maximum intensity
*/
float get_intensity()
{
	return global_norm;
}
