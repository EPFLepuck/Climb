/*
 * compute_case.c
 *
 *  Created on: 14 avr. 2022
 *      Author: Corentin Jossi
 */
#include <ch.h>
#include <hal.h>
#include <sensors/imu.h>
#include "compute_case.h"

#define		XAXIS		0
#define		YAXIS		1
#define		ZAXIS		2
#define		START		1.0f
#define		Y_THERSHOLD	0.7f
#define		GRAVITY		9.80665f

static float acc_x = 0;
static uint8_t 	acc_case = 0;

// Thread that tells us in which case (0, I, II, III, IV) (spqr) we are
static THD_WORKING_AREA(waSelectCase, 128); //Value of the stack has to be defined later on.
static THD_FUNCTION(SelectCase, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    float acc_y = 0;
    float acc_z = 0;

    while(1){
    	// get x and y component accelerometer
    	//Correct the offset of the z axis
    	acc_x = get_acceleration(XAXIS);
    	acc_y = get_acceleration(YAXIS);
    	acc_z = get_acceleration(ZAXIS) + GRAVITY;

    	// Compute case
    	if( (acc_case == 0) & ((acc_z > START) | (acc_z < -START)) ){
    		// Set the start of the program

    		if( acc_x >= 0 ) {
				if( acc_y >= Y_THERSHOLD ) {
					acc_case = 1;
				}else if( acc_y <= -Y_THERSHOLD ) {
					acc_case = 2;
				}
			}else {
				if( acc_y >= Y_THERSHOLD ) {
					acc_case = 3;
				}else if(acc_y <= -Y_THERSHOLD){
					acc_case = 4;
				}
			}
    	}else if(acc_case != 0){
    		if( acc_x >= 0 ) {
				if( acc_y >= Y_THERSHOLD ) {
					acc_case = 1;
				}else if( acc_y <= -Y_THERSHOLD ) {
					acc_case = 2;
				}
			}else {
				if( acc_y >= Y_THERSHOLD ) {
					acc_case = 3;
				}else if(acc_y <= -Y_THERSHOLD){
					acc_case = 4;
				}
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
