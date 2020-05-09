
#ifndef OBSTACLE_H_
#define OBSTACLE_H_

//init the sensor
void sensor_init(void);

//start the TOF thread
void object_length_start(void);

//start the IR thread
void object_detect_start(void);

//center the front of the epuck perpendicularly to the wall
void center_IR(void);

//detect obstacle not in front of the robot, so using the IR rather than the TOF
void detect_IR(void);

//main TOF obstacle avoiding function
void scan_obstacle(void);

//put the IR sensor values in a table
void ir_values(int* valeurs);

//measure the left and right obstacle length
uint16_t obstacle_length_left(void);
uint16_t obstacle_length_right(void);

//choose with direction is the smallest and turn the epuck to start the avoiding
void direction_choose(uint16_t left_side, uint16_t right_side);

//avoid obstacle using the IR sensor
void obj_ir_dodge(void);

//escape a dead end it there is one
void escape_dead_end(void);

//move the left/right motor at speed spped_l/speed_r to the right/left wheel position pos_right/pos_left
void motor_turn(int speed_r, int speed_l, int32_t pos_right, int32_t pos_left);

//convert the number of mm to step
uint16_t mm2steps(uint16_t millimeters);

//turn the robot if there is any obstacle in front of the IR sensor on the right and on the left
void detect_left_obs(int* distance);
void detect_right_obs(int* distance);

//go along a wall until the end of it
void go_along_after_dodge(int* distance);

#endif /* OBSTACLE_H_ */


