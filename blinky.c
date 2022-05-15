/*
 * blinky.c
 *
 *  Created on: 12 mai 2022
 *      Author: oliver
 */
// Standard includes
#include <ch.h>
#include <hal.h>
#include <leds.h>

// Project files includes
#include "blinky.h"
#include "check_collision.h"
#include "compute_case.h"

static THD_WORKING_AREA(waBlinky, 128);
static THD_FUNCTION(Blinky, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1){
	// Blink the back LED when the robot moves backward
		if(get_wall_detection() != 2){
			if( ((get_acc_case() == 2) | (get_acc_case() == 4)) ){

				set_led(LED5, 1);
				chThdSleepMilliseconds(300);
				set_led(LED5, 0);
				chThdSleepMilliseconds(200);
			}else{
				set_led(LED5, 0);
			}
		}
	    // 10Hz
	    chThdSleepMilliseconds(100);
	}
}

void blinky_start(void){
	chThdCreateStatic(waBlinky, sizeof(waBlinky), NORMALPRIO, Blinky, NULL);
}
