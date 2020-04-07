
#ifndef OBSTACLE_H_
#define OBSTACLE_H_

void object_length_start(void);

void object_detect_start(void);

void detect_obstacle(void);

void scan_obstacle(void);
void sensor_init(void);

//mesure la longueure de l'obstacle a gauche et droite
uint16_t obstacle_length_left(void);
uint16_t obstacle_length_right(void);


//défini dans quelle direction aller
uint8_t direction_choose(uint16_t ob_leng_mm_l, uint16_t ob_leng_mm_r);


//commande une tour de 90 degrés dans la direction a suivre puis evite l'obstacle
void motor_command_obs_dodge(uint8_t direction, uint16_t ob_leng_mm_l, uint16_t ob_leng_mm_r);


void motor_turn_obs(uint16_t pos_right, uint16_t pos_left);

#endif /* OBSTACLE_H_ */
