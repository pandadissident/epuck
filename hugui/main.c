#include <ch.h>
#include <hal.h>
#include <memory_protection.h>

#include <main.h>

#include "leds.h"
#include "spi_comm.h"
#include "motors.h"
#include "sensors/imu.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"
#include "selector.h"
#include "calibration.h"
#include "pid_regulator.h"
#include "mass_computation.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root;

// @brief
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

// @brief
void startingAnimation(void) {

	chThdSleepMilliseconds(100);

}

// @brief
void readyAnimation(void) {

	set_body_led(ON);
	chThdSleepMilliseconds(100);
	set_body_led(OFF);
	chThdSleepMilliseconds(50);
	set_body_led(ON);
	chThdSleepMilliseconds(100);
	set_body_led(OFF);
	chThdSleepMilliseconds(50);
	set_body_led(ON);
	chThdSleepMilliseconds(100);
	set_body_led(OFF);
	chThdSleepMilliseconds(50);
	set_body_led(ON);

}

// @brief
int main(void) {

	/*********************/
	/** local variables **/
	/*********************/

	static bool newTask = FALSE;
	static bool sysInitialised = FALSE;
	static int8_t fsm_state = 0;
	//static uint8_t eqPosEstimate = 0;
	//static uint8_t eqPos = 0;
	//static uint8_t mass = 0;


	/**********************/
	/** Startup sequence **/
	/**********************/

	if (!sysInitialised) {

	    //system init
		halInit();
	    chSysInit();
	    mpu_init();

	    // Inits the Inter Process Communication bus
		messagebus_init(&bus, &bus_lock, &bus_condvar);
		parameter_namespace_declare(&parameter_root, NULL, NULL);

	    // Init the peripherals.

		//battery_level_start();
		//VL53L0X_start();
		//spi_comm_start();
		serial_start();
		proximity_start();
		imu_start();
		motors_init();

	    sysInitialised = TRUE;

	    startingAnimation();

	} else {
		clear_leds();
		set_body_led(OFF);
		set_front_led(OFF);
		right_motor_set_speed(STOP);
		left_motor_set_speed(STOP);
	}

	/********************/
	/** State selector **/
	/********************/

    switch (fsm_state = get_selector()) {

    	case CALIB_PHASE_1 :
    		calibrate_imu_prox();
    		break;

    	case CALIB_PHASE_2 :
    		calibrate_tof(); //!!!!!!!!!!! HUGO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    		break;

    	case MEASUREMENT :
    		measure_mass();
    		break;

    	case MOTOR_TEST :
			straight_line();
			break;

    	default:
    		chThdSleepMilliseconds(500);
    		break;

    }


	/*************/
	/** Threads **/
	/*************/

    //thread_t *name_p = chThdCreateStatic(name_wa, sizeof(name_wa), NORMALPRIO, name, NULL);
    //chThdTerminate(name_p);

	/********************/
	/** Infinite loop. **/
	/********************/

	while (1) {

		// idle behavior : waiting for new task
		newTask = !(fsm_state == get_selector()); // + aboveCritAngle();
        if (newTask) {
        	set_front_led(OFF);
        	newTask = false;
        	main();
        }
        set_front_led(TOGGLE);
        chThdSleepMilliseconds(500);
    }
}


#define STACK_CHK_GUARD 0xe2dee396  //check if correct and remove magic number
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

// @brief memory protection
void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
