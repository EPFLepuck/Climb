// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <arm_math.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>

// project files includes
#include "compute_case.h"
#include "pid_regulator.h"
#include "check_collision.h"

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
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);




    // Start the imu
    imu_start();
    // Inits the motors
    motors_init();
    // Starts the serial communication
    serial_start();
    // Starts the IR proximity sensor
    proximity_start();
    //starts the USB communication
    //usb_start();



   messagebus_find_topic_blocking(&bus, "/imu");
   // Callibration
   calibrate_acc();
   calibrate_ir();

    // Wait callibration
    chThdSleepMilliseconds(2000);

    // Start thread
    check_collision_start();
    select_case_start();
    pid_regulator_start();

    /* Infinite loop. */
    while (1) {
    	palTogglePad(GPIOD, GPIOD_LED_FRONT);
    	chprintf((BaseSequentialStream *)&SD3, "AccX = %f \n",get_acc_x());
    	chprintf((BaseSequentialStream *)&SD3, "Case = %d \n",get_acc_case());
    	//chprintf((BaseSequentialStream *)&SD3, "IR2 = %d \n",get_calibrated_prox(1));
    	//chprintf((BaseSequentialStream *)&SD3, "IR1 = %d \n",get_calibrated_prox(0));
    	//chprintf((BaseSequentialStream *)&SD3, "IR7 = %d \n",get_calibrated_prox(6));
    	//chprintf((BaseSequentialStream *)&SD3, "IR8 = %d \n",get_calibrated_prox(7));

    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}

