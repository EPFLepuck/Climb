/*
 * check_collision.c
 *
 *  Created on: 5 mai 2022
 *      Author: Corentin Jossi
 */

#include <sensors/proximity.h>
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <motors.h>
#include <leds.h>
#include "check_collision.h"

#define DETECTION_THRESHOLD	100
/*
 * wall_detection = 0 -> nothing changes
 * wall_detection = 1 -> wall detected in front
 * wall_detection = 2 -> wall detected in the back
 */
static uint8_t wall_detection = 0;

// thread that detect a wall via infra-red captor
static THD_WORKING_AREA(waCheckCollision, 256); //Value of the stack has to be defined later on.
static THD_FUNCTION(CheckCollision, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int proxi_values[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    while(1) {
    	// Stocks the values in array
    	for(uint8_t i = 0 ; i < 8 ; ++i){
    		proxi_values[i] = get_calibrated_prox(i);
    	}


    	uint8_t check_sensors = 0;
    	for(uint8_t i = 0 ; i < 8 ; ++i){
    		// Checks if no walls
    		if(proxi_values[i] < DETECTION_THRESHOLD){
    			if(check_sensors == 7){
    				wall_detection = 0;
    				clear_leds();
    				check_sensors = 0;
    			}else{
        			check_sensors += 1;
    			}
    		// Walls detection
    		}else if( (i == 0) | (i == 1) | (i == 6) | (i == 7) ){
    			wall_detection = 1;
    			//Toggle LED
    			clear_leds();
    			set_led(LED1, 1);
    			set_rgb_led(LED2, 255, 255, 255);
    			set_rgb_led(LED8, 255, 255, 255);
    		}else if( (i == 3) | (i == 4) ){
    			wall_detection = 2;

				// Toggle LED
				clear_leds();
				set_led(LED5, 1);
				set_rgb_led(LED4, 255, 255, 255);
				set_rgb_led(LED6, 255, 255, 255);
    		}else{
				wall_detection = 0;
				clear_leds();
			}
    	}

    	// Check each 50ms if there is a wall or not
    	chThdSleepMilliseconds(50);
    }
}

// Function to create the thread CheckCollision
void check_collision_start(void){
	chThdCreateStatic(waCheckCollision, sizeof(waCheckCollision), NORMALPRIO+1, CheckCollision, NULL);
}

uint8_t get_wall_detection(void){
	return wall_detection;
}
