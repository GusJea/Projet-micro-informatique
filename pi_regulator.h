#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define STATE_NPI 0
#define STATE_PI  1

//start the PI regulator thread
void pi_regulator_start(void);
//Change the state and the direction
void set_state(int state, int direction);

#endif /* PI_REGULATOR_H */
