/*
 * compute_case.h
 *
 *  Created on: 14 avr. 2022
 *      Author: Corentin Jossi
 */

#ifndef COMPUTE_CASE_H_
#define COMPUTE_CASE_H_

// Function to create the thread SelectCase
void select_case_start(void);

// Get the case
uint8_t get_acc_case(void);

// Get acceleration of the X axis
float get_acc_x(void);

// Reset to 0 acc_case
void reset_acc_case(void);

#endif /* COMPUTE_CASE_H_ */
