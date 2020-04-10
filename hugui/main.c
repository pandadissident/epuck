#include <ch.h>
#include <hal.h>
#include <memory_protection.h>

#include <main.h>

#include "leds.h"
#include "selector.h"
#include "sensors/imu.h"
#include "calibration.h"
#include "pid_regulator.h"

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

// idle behavior
/*
static THD_WORKING_AREA(idle_wa, 128);
static THD_FUNCTION(idle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(!chThdShouldTerminateX()){
    	set_led(LED5, TOGGLE);
    	chThdSleepMilliseconds(500);
    }

    chThdExit((msg_t)"");
}
*/

// wait for new task
/*
static THD_WORKING_AREA(waitForTask_wa, 128);
static THD_FUNCTION(waitForTask, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    static bool newTask = FALSE;

    while(!chThdShouldTerminateX()){
    	newTask = get_selector(); // + aboveCritAngle();
        if (newTask) {
        	newTask = FALSE;
        	chThdTerminate(waitForTask_p)
        }
    }

    chThdExit((msg_t)"");
    main();
}
*/


void measure_mass(void) {

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

	return;

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

	if (!sysInitialised) {

	    //system init
		halInit();
	    chSysInit();
	    mpu_init();

	    // Inits the Inter Process Communication bus
		messagebus_init(&bus, &bus_lock, &bus_condvar);
		parameter_namespace_declare(&parameter_root, NULL, NULL);

	    // Init the peripherals.
		//motors_init();
		//proximity_start();
		//battery_level_start();
		serial_start();
		imu_start();


		//start timers
			//timer11_start();
			//timer12_start();

	    sysInitialised = TRUE;

	} else {

		// kill threads

		//chThdTerminate(waitForTask_p);
		//chThdTerminate(idle_p);

	}


	/********************/
	/** State selector **/
	/********************/

    switch (get_selector()) {

    	case IMU_CALIBRATION :
    		calibrate_imu();
    		break;

    	case TOF_CALIBRATION :
    		//calibrate_tof();
    		break;

    	case MEASUREMENT :
    		//measure_mass();
    		break;

    	default:
    		// nothing
    		break;

    }

	/*************/
	/** Threads **/
	/*************/

    //thread_t *waitForTask_p = chThdCreateStatic(waitForTask_wa, sizeof(waitForTask_wa), NORMALPRIO, waitForTask, NULL);
    //thread_t *idle_p = chThdCreateStatic(idle_wa, sizeof(idle_wa), NORMALPRIO, idle, NULL);

	/********************/
	/** Infinite loop. **/
	/********************/

	while (1) {

		// wait for new measurement task
		newTask = get_selector(); // + aboveCritAngle();
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

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
