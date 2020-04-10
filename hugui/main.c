#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ch.h>
#include <hal.h>

#include <main.h>

#include "selector.h"
#include "calibration.h"
#include "pid_regulator.h"
#include "memory_protection.h"

#define MEASUREMENT 0
#define IMU_CALIBRATION 1
#define TOF_CALIBRATION 2

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*
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
*/

/*
static void timer11_start(void){
    //General Purpose Timer configuration
    //timer 11 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt11cfg = {
        1000000,        // 1MHz timer clock in order to measure uS.
        NULL,           // Timer callback.
        0,
        0
    };

    gptStart(&GPTD11, &gpt11cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD11, 0xFFFF);
}
 */

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

		//start timers
			//timer11_start();
			//timer12_start();
		//start the serial communication
			//serial_start();
		//start communications protocols
			//usb_start();
			//i2c_start();
		//start the sensors
			//imu_start(); //gyro central
			//tof sensor
			//ir sensors
		//initialise the motors
			//motors_init();

		//initialise the Inter Process Communication bus.
			//messagebus_init(&bus, &bus_lock, &bus_condvar);

	    sysInitialised = TRUE;

	}


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
    chThdSleepMilliseconds(5000);

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
		// newTask = aboveCritAngle() + get_selector();
        if (newTask) {
        	//kill led thread
        	main();
        }
        chThdSleepMilliseconds(500);
    }
}


#define STACK_CHK_GUARD 0xe2dee396  //check if correct and remove magic number
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
