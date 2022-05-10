/*
 * pid_regulator.c
 *
 *  Created on: 14 avr. 2022
 *      Author: Corentin Jossi
 */
#include <ch.h>
#include <hal.h>
#include <math.h>
#include <motors.h>
#include <selector.h>
#include <chprintf.h>
#include "compute_case.h"
#include "pid_regulator.h"


#define	KP				500.0f
#define	KD				2.0f
#define	KI				3.5f
#define MAX_SUM_ERROR 	(MOTOR_SPEED_LIMIT/KI)
#define	ERROR_THRESHOLD	0.2f
#define MIN_ERROR		2.0f
#define GOAL			0

// The PID regulator thread
static THD_WORKING_AREA(waPIDRegulator, 256); //Value of the stack has to be defined later on.
static THD_FUNCTION(PIDRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    //uint8_t  up_down = 0;
    float error = 0;
    float derivative = 0;
    float last_error = 0;
    float correction_speed = 0;
    float straight_speed = 0;
	float sum_error = 0;

    while(1){
		time = chVTGetSystemTime();

		//----------PID regulator----------

		error = get_acc_x() - GOAL;
		derivative = error - last_error;

		// Set threshold
		if(fabs(error) <= ERROR_THRESHOLD){
			error = 0;
			sum_error = 0;
			last_error = 0;
		}

		sum_error += error;

		//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
		if(sum_error > MAX_SUM_ERROR){
			sum_error = MAX_SUM_ERROR;
		}else if(sum_error < -MAX_SUM_ERROR){
			sum_error = -MAX_SUM_ERROR;
		}

		correction_speed = KP*error + KD*derivative +  KI*sum_error;


		//we set a minimum for the error for the X axis to know when to start rolling forwards and backwards
		if((error < MIN_ERROR) | (error > -MIN_ERROR)) {
			// 40 percent of max speed
			straight_speed = MOTOR_SPEED_LIMIT*0.5;
		}else{
			straight_speed = 0;
		}

		last_error = error;

		//--------------Motors and other things----------------------

		// Speed to the motor by cases
		if( (get_acc_case() == 0) | (get_selector() == 0) ){
			right_motor_set_speed(0);
			left_motor_set_speed(0);
		}else if( (get_acc_case() == 1) | (get_acc_case() == 3) ) {
			// Case I and III
			// Front on top
			right_motor_set_speed((int16_t)(straight_speed-correction_speed));
			left_motor_set_speed((int16_t)(straight_speed+correction_speed));
		}else{
			// Case II and IV
			// Back on top
			right_motor_set_speed((int16_t)(-straight_speed+correction_speed));
			left_motor_set_speed((int16_t)(-straight_speed-correction_speed));
		}

		//25Hz
		chThdSleepUntilWindowed(time, time + MS2ST(40));
	}
}


// Function to create the thread PIDRegulator
void pid_regulator_start(void){
	chThdCreateStatic(waPIDRegulator, sizeof(waPIDRegulator), NORMALPRIO, PIDRegulator, NULL);
}
