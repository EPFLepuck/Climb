/*
 * check_collision.h
 *
 *  Created on: 5 mai 2022
 *      Author: Corentin Jossi
 */

#ifndef CHECK_COLLISION_H_
#define CHECK_COLLISION_H_

// Function to create the threat that check if there is a wall or not (high prio)
void check_collision_start(void);

// Getter of wall_detection
uint8_t get_wall_detection(void);

#endif /* CHECK_COLLISION_H_ */
