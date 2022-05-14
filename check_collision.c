/*
 * check_collision.c
 *
 *  Created on: 5 mai 2022
 *      Author: Corentin Jossi
 */
// Standard includes
#include <sensors/proximity.h>
#include <ch.h>
#include <hal.h>
#include <motors.h>
#include <leds.h>
#include <selector.h>
#include <audio/audio_thread.h>

// Project files includes
#include "check_collision.h"
#include "compute_case.h"

#define DETECTION_THRESHOLD	100
/*
 * wall_detection = 0 -> nothing changes
 * wall_detection = 1 -> wall detected in front
 * wall_detection = 2 -> wall detected in the back or (front and back)
 */
static uint8_t wall_detection = 0;
static uint8_t check_back_wall = 0;

// Declaration of functions
void clear_led_but_back(void);

// thread that detect a wall via infra-red captor
static THD_WORKING_AREA(waCheckCollision, 256);
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
    			// If all sensors detect nothing, there is no wall
    			if(check_sensors == 7){
    				wall_detection = 0;
    				/*
    				 *  We do not want to change the state of the back LED (LED5)
    				 *  That is why we do not use clear_leds()
    				 */
    				clear_led_but_back();

    				// Stop sound
    				dac_stop();

    				check_sensors = 0;
    			}else{
        			check_sensors += 1;
    			}

    		// Walls detection
    		}else if( (i == 0) | (i == 1) | (i == 6) | (i == 7) ){
    			// If back wall not detected
    			if( (proxi_values[3] < DETECTION_THRESHOLD) &
    				(proxi_values[4] < DETECTION_THRESHOLD) ){

					// Set front LEDs and clear back LEDs
    				wall_detection = 1;

    				// Clear back LEDs
					set_rgb_led(LED4, 0, 0, 0);
					set_rgb_led(LED6, 0, 0, 0);

					// Set front LEDs
					set_led(LED1, 1);
					set_rgb_led(LED2, 255, 255, 255);
					set_rgb_led(LED8, 255, 255, 255);

					// Front wall sound
					if( (get_selector() == 0) | (get_selector() == 15) ){
						dac_stop();
					}else{
						dac_play(600);
					}

				// Front and back detected
    			}else{
    				wall_detection = 2;
    				check_back_wall = 1;

    				// Set acc_case to 0
    				reset_acc_case();

    				// Set all LEDs to red
    				set_led(LED1, 1);
    				set_led(LED5, 1);
					set_rgb_led(LED2, 255, 0, 0);
					set_rgb_led(LED4, 255, 0, 0);
					set_rgb_led(LED6, 255, 0, 0);
					set_rgb_led(LED8, 255, 0, 0);

					// Stop sound
					if( (get_selector() == 0) | (get_selector() == 15) ){
						dac_stop();
					}else{
						dac_play(900);
					}
    			}

			}else if( (i == 3) | (i == 4) ){
				// Only back detected
				if( (proxi_values[0] < DETECTION_THRESHOLD) & (proxi_values[1] < DETECTION_THRESHOLD) &
					(proxi_values[6] < DETECTION_THRESHOLD) & (proxi_values[7] < DETECTION_THRESHOLD) ){

					// Set back LEDs and clear front LEDs
					wall_detection = 2;
					check_back_wall = 1;

					set_led(LED1, 0);
					set_rgb_led(LED2, 0, 0, 0);
					set_rgb_led(LED8, 0, 0, 0);
					// Set front LEDs
					set_led(LED5, 1);
					set_rgb_led(LED4, 255, 255, 255);
					set_rgb_led(LED6, 255, 255, 255);

					// Back wall sound
					if( (get_selector() == 0) | (get_selector() == 15) ){
						dac_stop();
					}else{
						dac_play(300);
					}
				}
			// Only side detection
    		}else if( (proxi_values[0] < DETECTION_THRESHOLD) & (proxi_values[1] < DETECTION_THRESHOLD) &
    				  (proxi_values[3] < DETECTION_THRESHOLD) & (proxi_values[4] < DETECTION_THRESHOLD) &
					  (proxi_values[6] < DETECTION_THRESHOLD) & (proxi_values[7] < DETECTION_THRESHOLD) ){

					wall_detection = 0;

					clear_led_but_back();

					// Stop sound
					dac_stop();
    		}
    	}
    	// Check each 50ms if there is a wall or not
    	chThdSleepMilliseconds(50);
    }
}

// Function to create the thread CheckCollision
void check_collision_start(void){
	chThdCreateStatic(waCheckCollision, sizeof(waCheckCollision), NORMALPRIO, CheckCollision, NULL);
}

uint8_t get_wall_detection(void){
	return wall_detection;
}

uint8_t get_check_back_wall(void){
	return check_back_wall;
}

void clear_check_back_wall(void){
	check_back_wall = 0;
}

// Clear all LEDs except the back LED
void clear_led_but_back(void){
	set_led(LED1, 0);
	set_rgb_led(LED2, 0, 0, 0);
	set_rgb_led(LED4, 0, 0, 0);
	set_rgb_led(LED6, 0, 0, 0);
	set_rgb_led(LED8, 0, 0, 0);
}
