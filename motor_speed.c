/*
 * motor_speed.c
 *
 *  Created on: 14 avr. 2022
 *      Author: Corentin Jossi
 */
#include "motor_speed.h"

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <motors.h>
#include <selector.h>
#include "compute_case.h"
#include "check_collision.h"


#define	KP				500.0f
#define	KD				2.0f
#define	KI				3.5f
#define MAX_SUM_ERROR 	(MOTOR_SPEED_LIMIT/KI)
#define	ERROR_THRESHOLD	0.2f
#define MIN_ERROR		2.0f
#define GOAL			0

// Declaration of function
void pid_regulator(	float *error, float *derivative, float *last_error, float *correction_speed,
					float *straight_speed, float *sum_error);
void stop_motor(void);

// The PID regulator thread
static THD_WORKING_AREA(waMotorSpeed, 256); //Value of the stack has to be defined later on.
static THD_FUNCTION(MotorSpeed, arg) {

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

		pid_regulator( &error, &derivative, &last_error, &correction_speed,
					   &straight_speed, &sum_error);

		//--------------Motors and other things----------------------

		// Check if the motor speed is greater than the limit
		// straight_speed + correction_speed = MOTOR_SPEED_LIMIT
		// straight_speed - correction_speed stays the same
		if(straight_speed+correction_speed > MOTOR_SPEED_LIMIT){
			correction_speed = (MOTOR_SPEED_LIMIT-straight_speed+correction_speed)/2;
			straight_speed = MOTOR_SPEED_LIMIT-correction_speed;
		}

		// Speed to the motor by cases
		if( (get_acc_case() == 0) | (get_selector() == 0) ){
			stop_motor();
		}else if( (get_acc_case() == 1) | (get_acc_case() == 3) ) {
			// Case I and III
			// Front on top
			if(get_wall_detection() == 1){
				stop_motor();
			}else{
				right_motor_set_speed((int16_t)(straight_speed-correction_speed));
				left_motor_set_speed((int16_t)(straight_speed+correction_speed));
			}
		}else{
			// Case II and IV
			// Back on top
			if(get_wall_detection() == 2){
				stop_motor();
			}else{
				right_motor_set_speed((int16_t)(-(straight_speed-correction_speed)));
				left_motor_set_speed((int16_t)(-(straight_speed+correction_speed)));
			}
		}

		//25Hz
		chThdSleepUntilWindowed(time, time + MS2ST(40));
	}
}


// Function to create the thread MotorSpeed
void motor_speed_start(void){
	chThdCreateStatic(waMotorSpeed, sizeof(waMotorSpeed), NORMALPRIO, MotorSpeed, NULL);
}

// Function of the PID regulator
void pid_regulator(	float *error, float *derivative, float *last_error, float *correction_speed,
					float *straight_speed, float *sum_error){

	*error = get_acc_x() - GOAL;
	*derivative = *error - *last_error;

	// Set threshold
	if(fabs(*error) <= ERROR_THRESHOLD){
		*error = 0;
		*sum_error = 0;
		*last_error = 0;
	}

	*sum_error += *error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(*sum_error > MAX_SUM_ERROR){
		*sum_error = MAX_SUM_ERROR;
	}else if(*sum_error < -MAX_SUM_ERROR){
		*sum_error = -MAX_SUM_ERROR;
	}

	*correction_speed = KP * (*error) + KD * (*derivative) +  KI * (*sum_error);


	//we set a minimum for the error for the X axis to know when to start rolling forwards and backwards
	if((*error < MIN_ERROR) | (*error > -MIN_ERROR)) {
		*straight_speed = MOTOR_SPEED_LIMIT*0.7;
	}else{
		*straight_speed = 0;
	}

	*last_error = *error;
}

void stop_motor(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}
