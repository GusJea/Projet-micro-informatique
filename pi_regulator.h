#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define	STATE_PI	1
#define STATE_NPI	0

//start the PI regulator thread
void pi_regulator_start(void);
//Change the state and the direction
void set_state(int8_t state, int8_t direction);
//Change the direction of the phase
void set_phase(int8_t direction_phase);

#endif /* PI_REGULATOR_H */
