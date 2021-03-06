/*
 * main.c
 *
 *  Created on: 14 avr. 2022
 *      Author: Corentin Jossi
 */
// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <motors.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <spi_comm.h>
#include <msgbus/messagebus.h>
#include <i2c_bus.h>
#include <audio/audio_thread.h>


// Project files includes
#include "main.h"
#include "compute_case.h"
#include "check_collision.h"
#include "motor_speed.h"
#include "blinky.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*
 *  Use of the selector :
 *  Selector 0 		: No sound and no motor
 *  Selector 1 		: Sound and no motor
 *  Selector 15 	: No sound and motor
 *  Other Selector 	: Sound and motor
 */

static void serial_start(void)
{
    static SerialConfig ser_cfg = {
        115200,
        0,
        0,
        0,
    };

    sdStart(&SD3, &ser_cfg); // UART3. Connected to the second com port of the programmer
}

int main(void)
{
	// System init
    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Wait calibration
    chThdSleepMilliseconds(1000);

    // Starts the imu
    imu_start();
    // Inits the motors
    motors_init();
    // Starts the serial communication
    serial_start();
    // Starts the IR proximity sensor
    proximity_start();
    // Starts the SPI for the RGB LED
    spi_comm_start();
    // Audio start
    dac_start();


    messagebus_find_topic_blocking(&bus, "/imu");

    // Calibration
    calibrate_acc();
    calibrate_ir();

    // Start threads
    check_collision_start();
    select_case_start();
    motor_speed_start();
    blinky_start();

    // Front LED indicates that the calibration is completed
    for(uint8_t i = 0 ; i < 3; ++i){
    	set_front_led(1);
    	chThdSleepMilliseconds(100);
    	set_front_led(0);
    	chThdSleepMilliseconds(100);
    }
    // Clear the LEDs to be sure that they are correctly initialised
    clear_leds();

    /* Infinite loop. */
    while (1) {
    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}

