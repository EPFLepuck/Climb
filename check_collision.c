/*
 * check_collision.c
 *
 *  Created on: 5 mai 2022
 *      Author: Corentin Jossi
 */

#include <sensors/proximity.h>
#include <ch.h>
#include <motors.h>
#include "check_collision.h"

// thread that detect a wall via infra-red captor
static THD_WORKING_AREA(waCheckCollision, 64); //Value of the stack has to be defined later on.
static THD_FUNCTION(CheckCollision, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint16_t proxi = 0;

    while(1) {

    	//proxi = get_calibrated_prox();

    	//if(proxi >= 100){
    	//	right_motor_set_speed(0);
    	//	left_motor_set_speed(0);
    	//
    	//}

    	// Check each 500ms if there is a wall or not
    	chThdSleepMilliseconds(1000);
    }
}

// Function to create the thread CheckCollision
void check_collision_start(void){
	chThdCreateStatic(waCheckCollision, sizeof(waCheckCollision), NORMALPRIO+1, CheckCollision, NULL);
}
