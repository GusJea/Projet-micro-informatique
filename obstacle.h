
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
uint16_t direction_choose(uint16_t ob_leng_mm_l, uint16_t ob_leng_mm_r);

#endif /* OBSTACLE_H_ */
