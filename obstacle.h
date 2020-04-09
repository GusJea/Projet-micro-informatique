
#ifndef OBSTACLE_H_
#define OBSTACLE_H_

void object_length_start(void);

void object_detect_start(void);

void detect_obstacle(void);

void scan_obstacle(void);
void sensor_init(void);

//mesure la longueure de l'obstacle a gauche et droite
uint16_t obstacle_length_left(uint16_t d_0);
uint16_t obstacle_length_right(uint16_t d_0);


//d�fini dans quelle direction aller
uint8_t direction_choose(uint16_t ob_leng_mm_l, uint16_t ob_leng_mm_r);


//commande une tour de 90 degr�s dans la direction a suivre puis evite l'obstacle
void motor_command_obs_dodge(uint8_t direction, uint16_t ob_leng_mm_l, uint16_t ob_leng_mm_r);


uint8_t obstacle_center(uint16_t d_0);

//void motor_turn_obs(uint16_t pos_right, uint16_t pos_left);

void motor_turn(int speed_r, int speed_l, int32_t pos_right, int32_t pos_left);

#endif /* OBSTACLE_H_ */