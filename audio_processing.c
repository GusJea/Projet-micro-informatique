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

static int8_t old_phase = 0;

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

#define FREQ_MIN		25	//Start scanning
#define FREQ_SLOW		30	//467Hz
#define FREQ_FAST		50	//779Hz
#define FREQ_MAX		55	//End scanning

#define FREQ_SLOW_L		FREQ_SLOW-1
#define FREQ_SLOW_H		FREQ_SLOW+1
#define FREQ_FAST_L		FREQ_FAST-1
#define FREQ_FAST_H		FREQ_FAST+1

#define V_SLOW 			600
#define V_NULL			0

static float ap_norm = MIN_VALUE_THRESHOLD;

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* dataR, float* dataL, float* dataB, float* dataF)
{
	float max_norm_r = MIN_VALUE_THRESHOLD;
	float max_norm_l = MIN_VALUE_THRESHOLD;
	float max_norm = MIN_VALUE_THRESHOLD;

	int8_t max_index_l = INIT_VALUE;
	int8_t max_index_r = INIT_VALUE;
	int8_t max_index = INIT_VALUE;

	int8_t direction = INIT_VALUE;

	//Search for the maximum sound intensity between the 2 frequencies we want to look at
	for(int i=FREQ_MIN; i <=FREQ_MAX; i++ )
	{
		//Maximum intensity and frequency peak of the right and left mics
		if(dataL[i]>=max_norm_l)
		{
			max_norm_l=dataL[i];
			max_index_l = i;
		}
		if(dataR[i]>=max_norm_r)
		{
			max_norm_r=dataL[i];
			max_index_r = i;
		}

		//Maximum intensity and frequency peak of the 4 mics
		if(dataL[i]>=max_norm)
		{
			direction = DIR_LEFT;
			max_index = i;
			max_norm = dataL[i];
		}
		if(dataR[i]>=max_norm)
		{
			direction = DIR_RIGHT;
			max_index = i;
			max_norm = dataR[i];
		}
		if(dataF[i]>=max_norm)
		{
			direction = DIR_FORWARD;
			max_index = i;
			max_norm = dataF[i];
		}
		if(dataB[i]>=max_norm)
		{
			direction = DIR_BACKWARD;
			max_index = i;
			max_norm = dataL[i];
		}
		if(max_norm == MIN_VALUE_THRESHOLD || max_norm >= 1000000)
		{
			direction = DIR_STOP;
		}

	}
	//Motor command depending on the direction of the maximum intensity
	if(max_index >= FREQ_FAST_L && max_index <= FREQ_FAST_H)
	{
		switch(direction)
		{
			case DIR_LEFT:
				set_state(STATE_PI, DIR_LEFT);
				break;
			case DIR_RIGHT:
				set_state(STATE_PI, DIR_RIGHT);
				break;
			case DIR_FORWARD:
				if(max_index_r == max_index_l && max_index_r >= FREQ_FAST_L && max_index_r <= FREQ_FAST_H)
				{
					float dephasage = phase(max_index_r);
					if(fabs(dephasage) <= 0.5)
					{
						dephasage = round(dephasage*10);
						//chprintf((BaseSequentialStream *) &SD3, "round dphase : %.3f\n\r", dephasage);
						if(dephasage > 2 && old_phase == 0)
						{
							set_state(STATE_PI, DIR_FORWARD);
							set_phase(DIR_LEFT);
							old_phase = 1;
						}
						else if(dephasage < -2 && old_phase == 0)
						{
							set_state(STATE_PI, DIR_FORWARD);
							set_phase(DIR_RIGHT);
							old_phase = -1;
						}
						else if(dephasage <= 0 && old_phase == 1)
						{
							set_state(STATE_PI, DIR_FORWARD);
							set_phase(DIR_FORWARD);
							old_phase = 0;
						}
						else if(dephasage >= 0 && old_phase == -1)
						{
							set_state(STATE_PI, DIR_FORWARD);
							set_phase(DIR_FORWARD);
							old_phase = 0;
						}
					}
				}

				break;
			case DIR_BACKWARD:
				if(max_norm_r >= max_norm_l)
				{
					set_state(STATE_PI, DIR_RIGHT);
				}
				else
				{
					set_state(STATE_PI, DIR_LEFT);
				}
				break;
			case DIR_STOP:
				set_state(STATE_PI, DIR_STOP);
				break;
		}
	}
	else
	{
		set_state(STATE_PI, DIR_STOP);
	}
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
 * 	Simple function to give the norm
 */
float get_intensity()
{
	return ap_norm;
}

/*
 * 	Simple function to get the phase
 */
float phase(int8_t index)
{
	float phase_right = 0;
	float phase_left = 0;
	phase_left = (float)atan(micLeft_cmplx_input[2*index+1]/micLeft_cmplx_input[2*index]);
	phase_right =  (float)atan(micRight_cmplx_input[2*index+1]/micRight_cmplx_input[2*index]);
	return (phase_left-phase_right);
}

