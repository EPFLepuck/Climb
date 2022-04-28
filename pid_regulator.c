/*
 * pid_regulator.c
 *
 *  Created on: 14 avr. 2022
 *      Author: Corentin Jossi
 */
#include "ch.h"
#include "hal.h"
#include <math.h>

#include "compute_case.h"
#include "pid_regulator.h"
#include <motors.h>
#include <selector.h>


#define	KP				650.0f
#define	KI				2.0f
#define MAX_SUM_ERROR 	(MOTOR_SPEED_LIMIT/KI)
#define	ERROR_THRESHOLD	0.1f
#define SPEED			600
#define GOAL			0

// The PID regulator thread
static THD_WORKING_AREA(waPIDRegulator, 256); //Value of the stack has to be defined later on.
static THD_FUNCTION(PIDRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    //uint8_t  up_down = 0;
    float error = 0;
    float correction_speed = 0;
	float sum_error = 0;

    while(1){
		time = chVTGetSystemTime();

		//----------PID regulator----------

		error = get_acc_x() - GOAL;

		if(fabs(error) <= ERROR_THRESHOLD){
			error = 0;
			sum_error = 0;
		}

		sum_error += error;

		//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
		if(sum_error > MAX_SUM_ERROR){
			sum_error = MAX_SUM_ERROR;
		}else if(sum_error < -MAX_SUM_ERROR){
			sum_error = -MAX_SUM_ERROR;
		}

		correction_speed = KP * error + KI * sum_error;

		// Selector is just for the robot stops all movement
		// Speed to the motor by cases for rotation
		if( ((get_acc_case() == 0) | (get_acc_case() == 2)) & (get_selector() != 0) ) {
			// Case 0 and II
			// Front on top
			//up_down = 0;
			right_motor_set_speed((int16_t)(-correction_speed));
			left_motor_set_speed((int16_t)(correction_speed));
		}else if( get_selector() != 0 ) {
			// Case I and III
			// Back on top
			//up_down = 1;
			right_motor_set_speed((int16_t)(correction_speed));
			left_motor_set_speed((int16_t)(-correction_speed));
		}else{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
		}


		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}


// Function to create the thread PIDRegulator
void pid_regulator_start(void){
	chThdCreateStatic(waPIDRegulator, sizeof(waPIDRegulator), NORMALPRIO, PIDRegulator, NULL);
}
