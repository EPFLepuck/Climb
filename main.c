// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <arm_math.h>
#include "sensors/imu.h"

// project files includes
#include "compute_case.h"
#include "pid_regulator.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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
    //mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Start the imu
    imu_start();
    //inits the motors
    motors_init();
    //starts the serial communication
    serial_start();
    //starts the USB communication
    //usb_start();

   messagebus_find_topic_blocking(&bus, "/imu");

    // Wait callibration
    chThdSleepMilliseconds(2000);

    // Start thread
    select_case_start();
    pid_regulator_start();

    /* Infinite loop. */
    while (1) {
    	//test
    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}

