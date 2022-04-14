/*
 * compute_case.c
 *
 *  Created on: 14 avr. 2022
 *      Author: Corentin Jossi
 */

#include "sensors/imu.h"
#include "compute_case.h"

#define		XAXIS	0
#define		YAXIS	1

static float acc_x = 0;
static uint8_t 	acc_case = 0;

// Thread that tells us in which case (0, I, II, III) (spqr) we are
static THD_WORKING_AREA(waSelectCase, 256); //Value of the stack has to be defined later on.
static THD_FUNCTION(SelectCase, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    float acc_y = 0;

    while(1){
    	// get x and y component accelerometer
    	acc_x = get_acceleration(XAXIS);
    	acc_y = get_acceleration(YAXIS);

    	// Compute case
    	if( acc_x >= 0 ) {
    		if( acc_y >= 0 ) {
    			acc_case = 0;
    		}else {
    			acc_case = 1;
    		}
    	}else {
    		if( acc_y >= 0 ) {
    			acc_case = 2;
    		}else {
    			acc_case = 3;
    		}
    	}
    }
}

// Function to create the thread SelectCase
void select_case_start(void){
	chThdCreateStatic(waSelectCase, sizeof(waSelectCase), NORMALPRIO, SelectCase, NULL);
}

float get_acc_x(void){
	return acc_x;
}

uint8_t get_acc_case(void){
	return acc_case;
}
