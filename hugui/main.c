//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <math.h>
#include <ch.h>
#include <hal.h>

#include <main.h>

#include "leds.h"
#include "selector.h"
#include "calibration.h"
#include "sensors/imu.h"
#include "pid_regulator.h"
#include "memory_protection.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root;

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

int main(void) {

	/*********************/
	/** local variables **/
	/*********************/

	static bool newTask = FALSE;
	static bool sysInitialised = FALSE;
	//static uint8_t eqPosEstimate = 0;
	//static uint8_t eqPos = 0;
	//static uint8_t mass = 0;


	/**********************/
	/** Startup sequence **/
	/**********************/

	if(!sysInitialised){

	    //system init
		halInit();
	    chSysInit();
	    mpu_init();

	    // Inits the Inter Process Communication bus
		//messagebus_init(&bus, &bus_lock, &bus_condvar);
		//parameter_namespace_declare(&parameter_root, NULL, NULL);

	    // Init the peripherals.
		//motors_init();
		//proximity_start();
		//battery_level_start();
		imu_start();
		serial_start();

		//start timers
			//timer11_start();
			//timer12_start();

	    sysInitialised = TRUE;

	}

	/*****************/
	/** Clear LEDS **/
	/*****************/

	clear_leds();
	set_body_led(OFF);
	set_front_led(OFF);

	/*****************/
	/** Calibration **/
	/*****************/

    switch (get_selector()) {

    	case IMU_CALIBRATION :
    		calibrate_imu();
    		break;

    	case TOF_CALIBRATION :
    		//calibrate_tof();
    		break;

    	case MEASUREMENT :
    	default:
    		break;

    }


	/*******************/
	/** Main Sequence **/
	/*******************/

    //launch ir thread for lateral positioning

    //driveTo(ORIGIN); if not already

    // Waiting 5 seconds to place weight onto scale
    //chThdSleepMilliseconds(5000);

    //eqPosEstimate = estimateTipOverPosition(); //commande en vitesse
    	// -> start thread to check angular velocity
    	// -> driveTo(END)
    	// -> while (1)
    	// -> return estimated x value

    //eqPos = preciseTipOverPosition(eqPosEstimate); //commande en position
    	// -> start PID thread
    	// -> maintainPosition(); for t seconds
    	// -> return accurate x value

    //mass = computeMass(eqPos);

    //sendMass(mass);


	/********************/
	/** Infinite loop. **/
	/********************/

	while (1) {

		// wait for new measurement task
		newTask = get_selector(); // + aboveCritAngle();
        if (newTask) {
        	newTask = false;
        	main();
        }
        set_front_led(TOGGLE);
        //chThdSleepMilliseconds(500);
    }
}


#define STACK_CHK_GUARD 0xe2dee396  //check if correct and remove magic number
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
