
#ifndef OBSTACLE_H_
#define OBSTACLE_H_

//init les sensors
void sensor_init(void);

//lance le thread TOF
void object_length_start(void);

//lance le thread IR
void object_detect_start(void);

//mets le robot a angle droit face a l'obstacle avec les capteurs IRs
void center_IR(void);

//detecte des obstacle pas en face
void detect_IR(void);

//fonction principale qui g�re le scan de l'obstacle + choix de direction
void scan_obstacle(void);

//r�cup�re les valeurs des capteurs IRs
void ir_values(int* valeurs);

//mesure la longueure de l'obstacle a gauche et droite
uint16_t obstacle_length_left(void);
uint16_t obstacle_length_right(void);

//d�fini dans quelle direction aller
//void direction_choose(uint16_t* left_side, uint16_t* right_side);
void direction_choose(uint16_t left_side, uint16_t right_side);

//�vite l'obstacle avec IRs
void obj_ir_dodge(void);

//sort d'un cul de sac
void escape_dead_end(void);

//bouge le robot a la vitesse speed_r/l jusqu'� pos_left/right aka le nbr de step
void motor_turn(int speed_r, int speed_l, int32_t pos_right, int32_t pos_left);

uint16_t mm2steps(uint16_t millimeters);

#endif /* OBSTACLE_H_ */
