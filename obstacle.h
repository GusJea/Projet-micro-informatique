
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

//fonction principale qui gère le scan de l'obstacle + choix de direction
void scan_obstacle(void);

//récupère les valeurs des capteurs IRs
void ir_values(int* valeurs);

//mesure la longueure de l'obstacle a gauche et droite
/*
void obstacle_length_left(uint16_t* left_side);
void obstacle_length_right(uint16_t* right_side);
*/
uint16_t obstacle_length_left(void);
uint16_t obstacle_length_right(void);

//défini dans quelle direction aller
//void direction_choose(uint16_t* left_side, uint16_t* right_side);
void direction_choose(uint16_t left_side, uint16_t right_side);

//évite l'obstacle avec IRs
void obj_ir_dodge(void);



//bouge le robot a la vitesse speed_r/l jusqu'à pos_left/right aka le nbr de step
void motor_turn(int speed_r, int speed_l, int32_t pos_right, int32_t pos_left);

#endif /* OBSTACLE_H_ */

/* osef
 *
uint8_t obstacle_center(uint16_t d_0);
 *
void motor_turn_obs(uint16_t pos_right, uint16_t pos_left);
 *
commande une tour de 90 degrés dans la direction a suivre puis evite l'obstacle baleq
void motor_command_obs_dodge(uint16_t* left_side, uint16_t* right_side);
 *
 */
