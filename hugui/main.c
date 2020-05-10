#include <ch.h>
#include <chprintf.h>
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

// threads
static thread_t *button_p;
static thread_t *fsm_p;

// @brief thread to watch if button is pressed
static THD_WORKING_AREA(button_wa, 128);
static THD_FUNCTION(button, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    button_p = chThdGetSelfX();

	while (!chThdShouldTerminateX()) {

		if(button_is_pressed()){
			chEvtSignal(fsm_p, (eventmask_t)1);
			// cancel long presses
			chThdSleepMilliseconds(1000);
		} else {
			chEvtSignal(fsm_p, (eventmask_t)0);
		}
		chThdSleepMilliseconds(100);
	}
}

// @brief main FSM machine
static THD_WORKING_AREA(fsm_wa, 128);
static THD_FUNCTION(fsm, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    fsm_p = chThdGetSelfX();

    while (!chThdShouldTerminateX()) {

    	// waits for a message from button thread
    	chEvtWaitAny((eventmask_t)1);

    	// stop all actions
    	stop_pid_regulator();
    	stop_assess_stability();
		right_motor_set_speed(STOP);
		left_motor_set_speed(STOP);

    	// reset led state
		clear_leds();
		set_body_led(OFF);
		set_front_led(OFF);

		// simple fsm
		switch (get_selector()) {
			case MEASUREMENT :
				chprintf((BaseSequentialStream *)&SD3, "Mesure de la masse...\n");
				measure_mass();
				chprintf((BaseSequentialStream *)&SD3, "Termine\n");
				break;
			case SENSORS_CALIBRATION :
				chprintf((BaseSequentialStream *)&SD3, "Premiere etape de calibration...\n");
				calibrate_imu_prox();
				chprintf((BaseSequentialStream *)&SD3, "Calibration terminee\n");
				break;
			case ORIGIN_CALIBRATION :
				chprintf((BaseSequentialStream *)&SD3, "Seconde etape de calibration...\n");
				calibrate_tof();
				chprintf((BaseSequentialStream *)&SD3, "Calibration terminee\n");
				break;
			case BLUETOOTH_TEST :
				set_body_led(ON);
				chprintf((BaseSequentialStream *)&SD3, "Envoie de la masse...\n");
				startingAnimation();
				send_mass();
				chprintf((BaseSequentialStream *)&SD3, "Termine\n");
				break;
			default:
				set_body_led(ON);
				chprintf((BaseSequentialStream *)&SD3, " - STOP -\n");
				chThdSleepMilliseconds(500);
				break;
    	}
		chprintf((BaseSequentialStream *)&SD3, "\nEN ATTENTE D'INSTRUCTIONS\n");
		readyAnimation();
    }
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.

	return;
}

void startingAnimation(void)
{
	//circle on
	set_led(LED1, ON);
	chThdSleepMilliseconds(50);
	set_rgb_led(LED2, 0, 0, 100);
	chThdSleepMilliseconds(50);
	set_led(LED3, ON);
	chThdSleepMilliseconds(50);
	set_rgb_led(LED4, 0, 0, 100);
	chThdSleepMilliseconds(50);
	set_led(LED5, ON);
	chThdSleepMilliseconds(50);
	set_rgb_led(LED6, 0, 0, 100);
	chThdSleepMilliseconds(50);
	set_led(LED7, ON);
	chThdSleepMilliseconds(50);
	set_rgb_led(LED8, 0, 0, 100);

	//circle off
	chThdSleepMilliseconds(200);
	set_led(LED1, OFF);
	chThdSleepMilliseconds(50);
	set_rgb_led(LED2, 0, 0, 0);
	chThdSleepMilliseconds(50);
	set_led(LED3, OFF);
	chThdSleepMilliseconds(50);
	set_rgb_led(LED4, 0, 0, 0);
	chThdSleepMilliseconds(50);
	set_led(LED5, OFF);
	chThdSleepMilliseconds(50);
	set_rgb_led(LED6, 0, 0, 0);
	chThdSleepMilliseconds(50);
	set_led(LED7, OFF);
	chThdSleepMilliseconds(50);
	set_rgb_led(LED8, 0, 0, 0);

	chThdSleepMilliseconds(500);
	readyAnimation();
	return;
}

void readyAnimation(void)
{
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

	return;
}

int main(void)
{
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
	motors_init();
	proximity_start();
	imu_start();
	spi_comm_start();
	//VL53L0X_start();
	serial_start();

	startingAnimation();

	// Launch main thread
	chThdCreateStatic(fsm_wa, sizeof(fsm_wa), NORMALPRIO, fsm, NULL);
	chThdCreateStatic(button_wa, sizeof(button_wa), NORMALPRIO, button, NULL);

	chprintf((BaseSequentialStream *)&SD3, "EN ATTENTE D'INSTRUCTIONS\n");

    //infinite loop that does nothing
	while (1) {
        chThdSleepMilliseconds(1000);
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

// @brief memory protection
void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
