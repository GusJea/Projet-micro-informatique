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

#define MIN_VALUE_THRESHOLD	10000

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_SLOW		30	//467HZ
#define FREQ_FAST		50	//779Hz
#define MAX_FREQ		60	//we don't analyze after this index to not use resources for nothing

#define FREQ_SLOW_L		(FREQ_SLOW-1)
#define FREQ_SLOW_H		(FREQ_SLOW+1)
#define FREQ_FAST_L		(FREQ_FAST-1)
#define FREQ_FAST_H		(FREQ_FAST+1)

#define V_NULL			0
#define V_SLOW			600

static float global_norm = 0;

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* dataR, float* dataL, float* dataB, float* dataF){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t direction = -1;
	int16_t old_norm_index = -1;
	int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(dataL[i] > max_norm)
		{
			max_norm = dataL[i];
			max_norm_index = i;
			direction = DIR_LEFT;
		}
		if(dataR[i] > max_norm)
		{
			max_norm = dataR[i];
			max_norm_index = i;
			direction = DIR_RIGHT;
		}
		if(dataF[i] > max_norm)
		{
			max_norm = dataF[i];
			max_norm_index = i;
			direction = DIR_FORWARD;
		}
		if(dataB[i] > max_norm)
		{
			max_norm = dataB[i];
			max_norm_index = i;
			direction = DIR_BACKWARD;
		}
	}
	if(old_norm_index == -1 || old_norm_index == max_norm_index)
	{
		old_norm_index = max_norm_index;
		set_intensity(max_norm);
		motor_command(direction, max_norm_index);
	}
	else
	{
		set_intensity(max_norm);
		motor_command(DIR_STOP, max_norm_index);
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
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

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
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		sound_remote(micRight_output, micLeft_output, micBack_output, micFront_output);
	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
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
void motor_command(int16_t direction, int16_t max_norm_index)
{
	//go forward
	if(direction == DIR_FORWARD)
	{
		if(max_norm_index >= FREQ_SLOW_L && max_norm_index <= FREQ_SLOW_H)
		{
			set_state(STATE_NPI, DIR_FORWARD);
			left_motor_set_speed(V_SLOW);
			right_motor_set_speed(V_SLOW);
		}
		else if(max_norm_index >= FREQ_FAST_L && max_norm_index <= FREQ_FAST_H)
		{
			//left_motor_set_speed(2000);
			//right_motor_set_speed(2000);
			set_state(STATE_PI, DIR_FORWARD);
		}
		else
		{
			set_state(STATE_NPI, DIR_FORWARD);
			left_motor_set_speed(V_NULL);
			right_motor_set_speed(V_NULL);
		}

	}
	//turn left
	else if(direction == DIR_LEFT)
	{
		if(max_norm_index >= FREQ_SLOW_L && max_norm_index <= FREQ_SLOW_H)
		{
			set_state(STATE_NPI, DIR_LEFT);
			left_motor_set_speed(-V_SLOW);
			right_motor_set_speed(V_SLOW);
		}
		else if(max_norm_index >= FREQ_FAST_L && max_norm_index <= FREQ_FAST_H)
		{
			//left_motor_set_speed(-2000);
			//right_motor_set_speed(2000);
			set_state(STATE_PI, DIR_LEFT);
		}
		else
		{
			set_state(STATE_NPI, DIR_LEFT);
			left_motor_set_speed(V_NULL);
			right_motor_set_speed(V_NULL);
		}
	}
	//turn right
	else if(direction == DIR_RIGHT)
	{
		if(max_norm_index >= FREQ_SLOW_L && max_norm_index <= FREQ_SLOW_H)
		{
			set_state(STATE_NPI, DIR_RIGHT);
			left_motor_set_speed(V_SLOW);
			right_motor_set_speed(-V_SLOW);
		}
		else if(max_norm_index >= FREQ_FAST_L && max_norm_index <= FREQ_FAST_H)
		{
			//left_motor_set_speed(2000);
			//right_motor_set_speed(-2000);
			set_state(STATE_PI, DIR_RIGHT);
		}
		else
		{
			set_state(STATE_NPI, DIR_RIGHT);
			left_motor_set_speed(V_NULL);
			right_motor_set_speed(V_NULL);
		}
	}
	//go backward
	else if(direction == DIR_BACKWARD)
	{
		if(max_norm_index >= FREQ_SLOW_L && max_norm_index <= FREQ_SLOW_H)
		{
			set_state(STATE_NPI, DIR_BACKWARD);
			left_motor_set_speed(-V_SLOW);
			right_motor_set_speed(-V_SLOW);
		}
		else if(max_norm_index >= FREQ_FAST_L && max_norm_index <= FREQ_FAST_H)
		{
			//left_motor_set_speed(-2000);
			//right_motor_set_speed(-2000);
			set_state(STATE_PI, DIR_BACKWARD);
		}
		else
		{
			set_state(STATE_NPI, DIR_BACKWARD);
			left_motor_set_speed(V_NULL);
			right_motor_set_speed(V_NULL);
		}
	}
	else
	{
		set_state(STATE_NPI, DIR_BACKWARD);
		left_motor_set_speed(V_NULL);
		right_motor_set_speed(V_NULL);
	}
}

/*
*	Simple function to set the maximum intensity
*/
void set_intensity(float max_norm)
{
	global_norm = max_norm;
}

/*
*	Simple function to get the maximum intensity
*/
float get_intensity()
{
	return global_norm;
}
