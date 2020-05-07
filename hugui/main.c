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
#include "serial.h"
#include "spi_comm.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static thread_t *button_p;
static thread_t *fsm_p;

// @brief
static THD_WORKING_AREA(button_wa, 128);
static THD_FUNCTION(button, arg) {

    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    button_p = chThdGetSelfX();

    // debounce

	while (!chThdShouldTerminateX()) {

		if(button_is_pressed()){
			CH_IRQ_PROLOGUE(); // retirer
			chSysLockFromISR(); //chSysLock() ?
			chEvtSignalI(fsm_p, (eventmask_t)1);
			chSysUnlockFromISR();
			CH_IRQ_EPILOGUE();
			chThdSleepMilliseconds(1000);
		} else {
			CH_IRQ_PROLOGUE();
			chSysLockFromISR();
			chEvtSignalI(fsm_p, (eventmask_t)0);
			chSysUnlockFromISR();
			CH_IRQ_EPILOGUE();
		}
		chThdSleepMilliseconds(100);

	}
}

// @brief
static THD_WORKING_AREA(fsm_wa, 128);
static THD_FUNCTION(fsm, arg) {

    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    fsm_p = chThdGetSelfX();

    while (!chThdShouldTerminateX()) {

    	// waits for a message from button
    	chEvtWaitAny((eventmask_t)1);

    	// stop all actions
    	pid_regulator_stop();
    	find_equilibrium_stop();
    	stop_social_distancing();

    	// reset state
		clear_leds();
		set_body_led(OFF);
		set_front_led(OFF);
		right_motor_set_speed(STOP);
		left_motor_set_speed(STOP);

		switch (get_selector()) {
			case MEASUREMENT :
				measure_mass();
				break;
			case CALIBRATION :
				calibrate_imuNprox();
				break;
			case TOF_TUNING :
				tune_tof();
				break;
			case MOTOR_TEST :
				set_body_led(ON);
				start_social_distancing();
				break;
			default:
				set_body_led(ON);
				chThdSleepMilliseconds(500);
				break;
    	}

		//deletes message
		CH_IRQ_PROLOGUE();
		chSysLockFromISR();
		chEvtSignalI(fsm_p, (eventmask_t)0);
		chSysUnlockFromISR();
		CH_IRQ_EPILOGUE();
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

	toggle_rgb_led(LED2, RED_LED, 100);
	toggle_rgb_led(LED4, RED_LED, 100);
	toggle_rgb_led(LED6, RED_LED, 100);
	toggle_rgb_led(LED8, RED_LED, 100);

//	//circle on
//	set_led(LED1, ON);
//	chThdSleepMilliseconds(50);
//	set_rgb_led(LED2, 0, 0, 100);
//	chThdSleepMilliseconds(50);
//	set_led(LED3, ON);
//	chThdSleepMilliseconds(50);
//	set_rgb_led(LED4, 0, 0, 100);
//	chThdSleepMilliseconds(50);
//	set_led(LED5, ON);
//	chThdSleepMilliseconds(50);
//	set_rgb_led(LED6, 0, 0, 100);
//	chThdSleepMilliseconds(50);
//	set_led(LED7, ON);
//	chThdSleepMilliseconds(50);
//	set_rgb_led(LED8, 0, 0, 100);
//	//circle off
//	chThdSleepMilliseconds(200);
//	set_led(LED1, OFF);
//	chThdSleepMilliseconds(50);
//	set_rgb_led(LED2, 0, 0, 0);
//	chThdSleepMilliseconds(50);
//	set_led(LED3, OFF);
//	chThdSleepMilliseconds(50);
//	set_rgb_led(LED4, 0, 0, 0);
//	chThdSleepMilliseconds(50);
//	set_led(LED5, OFF);
//	chThdSleepMilliseconds(50);
//	set_rgb_led(LED6, 0, 0, 0);
//	chThdSleepMilliseconds(50);
//	set_led(LED7, OFF);
//	chThdSleepMilliseconds(50);
//	set_rgb_led(LED8, 0, 0, 0);
//
//	chThdSleepMilliseconds(500);
//
//	readyAnimation();
}


// @brief
void readyAnimation(void) {

	clear_leds();

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
	//VL53L0X_start();
	serial_start();

	startingAnimation();

	// Launch main thread
//  chThdCreateStatic(fsm_wa, sizeof(fsm_wa), NORMALPRIO, fsm, NULL);
//	chThdCreateStatic(button_wa, sizeof(button_wa), NORMALPRIO, button, NULL);

    static float dummy_a, dummy_g, dummy_i, dummy_t = 0;

    //infinite loop does nothing
	while (1) {
        chThdSleepMilliseconds(1000);
        dummy_a = get_acceleration(X_AXIS);
		dummy_g = get_gyro_rate(X_AXIS);
		dummy_i = get_prox(7);
		dummy_t = VL53L0X_get_dist_mm();
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

// @brief memory protection
void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
