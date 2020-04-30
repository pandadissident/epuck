#include <ch.h>
#include <hal.h>
#include <memory_protection.h>

#include <main.h>

#include "button.h"
#include "calibration.h"
#include "leds.h"
#include "mass_computation.h"
#include "motors.h"
#include "pid_regulator.h"
#include "selector.h"
#include "sensors/imu.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"
#include "spi_comm.h"


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


// @brief
static THD_WORKING_AREA(fsm_wa, 2048);
static THD_FUNCTION(fsm, arg) {

    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    while (1) {
    	if (button_is_pressed()) {
       	    clear_leds();
			set_body_led(OFF);
			set_front_led(OFF);
    	    switch (get_selector()) {
    	    	case CALIB_PHASE_1 :
    	    		calibrate_imu_prox();
    	    		break;
    	    	case CALIB_PHASE_2 :
    	    		calibrate_tof();
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
    	    clear_leds();
			set_body_led(OFF);
			set_front_led(OFF);
			right_motor_set_speed(STOP);
			left_motor_set_speed(STOP);
    	}
    	set_front_led(TOGGLE);
    	chThdSleepMilliseconds(250);
    }
}


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

	readyAnimation();

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

	chThdSleepMilliseconds(500);

}

// @brief
int main(void) {

	//system init
	halInit();
	chSysInit();
	mpu_init();

	// Inits the Inter Process Communication bus
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	// Init leds
	clear_leds();
	set_body_led(OFF);
	set_front_led(OFF);
	chThdSleepMilliseconds(100);

	// Init the peripherals
//	usb_start();
//	dcmi_start();
//	po8030_start();
	motors_init();
	proximity_start();
//	battery_level_start();
//	dac_start();
//	exti_start();
	imu_start();
//	ir_remote_start();
	spi_comm_start();
	//VL53L0X_start(); // <-----------
	serial_start();

	startingAnimation();

	// Launch main thread
    chThdCreateStatic(fsm_wa, sizeof(fsm_wa), NORMALPRIO, fsm, NULL);

    set_body_led(OFF);

    static float dummy_a, dummy_g, dummy_i, dummy_t = 0;

    //infinite loop
	while (1) {
        chThdSleepMilliseconds(1000);
        dummy_a = get_acceleration(X_AXIS);
		dummy_g = get_gyro_rate(X_AXIS);
		dummy_i = get_prox(7);
		//dummy_t = VL53L0X_get_dist_mm();
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

// @brief memory protection
void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
