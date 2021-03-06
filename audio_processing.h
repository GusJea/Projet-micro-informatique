#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#define FFT_SIZE 	1024
#define DIR_LEFT		1
#define DIR_RIGHT		2
#define DIR_FORWARD		3
#define DIR_BACKWARD	4
#define DIR_STOP		5

#define SPEED_POS		1
#define SPEED_NEG		-1
#define SPEED_F_POS		1.5
#define SPEED_F_NEG		0.5

#define V_SLOW 			600
#define V_NULL			0

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	Put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

/*
 * 	Simple function to give the norm
 */
float get_intensity(void);

/*
 * 	Simple function to get the phase
 */
float phase(int8_t index, int8_t state);

/*
 * 	Function to set the speed to the motors
 */
void motor_command(float coef_r, float coef_l);

/*
 * 	Function to get the direction the sound comes from
 */
int8_t get_dir_sound(void);

#endif /* AUDIO_PROCESSING_H */
