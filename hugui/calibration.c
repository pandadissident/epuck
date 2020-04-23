#include <ch.h>
#include <hal.h>
#include <math.h>
#include <main.h>

#include "calibration.h"

#include "sensors/mpu9250.h"
#include "sensors/imu.h"
#include "leds.h"
#include "selector.h"

static int16_t originPos = 0;

// @brief makes led blink
static THD_WORKING_AREA(blinkLed_wa, 128);
static THD_FUNCTION(blinkLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(!chThdShouldTerminateX()){
    	set_led(LED5, TOGGLE);
    	chThdSleepMilliseconds(500);
    }

    set_led(LED5, ON);
}

// @brief calibrates imu
void calibrate_imu(void) {

	uint16_t i = 0;
	int16_t altSum[SAMPLES][NB_AXIS] = {0}; //acceleration circular buffer of last 3 sec
	float tot = 0; //takes wide range of values
	bool unstable = TRUE;

	systime_t time;

	// blink led
	thread_t *blinkLed_p = chThdCreateStatic(blinkLed_wa, sizeof(blinkLed_wa), NORMALPRIO, blinkLed, NULL);

	// wait for EPuck to be stable
	while (unstable) {

		get_acc_all(altSum[i % SAMPLES]);
		time = chVTGetSystemTime();

		if (i % 2) {
			tot = 0;
			for (uint8_t j = 0 ; j < SAMPLES ; j += 2) {
				tot += fabs(altSum[j][X_AXIS] - altSum[j+1][X_AXIS]) + fabs(altSum[j][Y_AXIS] - altSum[j+1][Y_AXIS]) + fabs(altSum[j][Z_AXIS] - altSum[j+1][Z_AXIS]);
			}
			if ((i > 3*SAMPLES) && (tot < (float)THRESHOLD)) {
				unstable = FALSE;
			}
		}

		i++;

		// prevent overflow and reset

		if (i >= MAX_ITERATIONS) {
			i = 0;
			for (uint8_t j = 0 ; j < SAMPLES ; j++) {
				for (uint8_t k = 0 ; k < NB_AXIS ; k++) {
					altSum[j][k] = 0;
				}
			}
		}

		//reduced the sample rate
		chThdSleepUntilWindowed(time, time + MS2ST(150));

	}

	// kill blink led and sets it on
	chThdTerminate(blinkLed_p);

	// calibration with led animation
	calibrate_acc();

	set_rgb_led(LED6, 10, 0, 0);
	set_rgb_led(LED4, 10, 0, 0);

	calibrate_gyro();

	set_led(LED7, ON);
	set_led(LED3, ON);

	chThdSleepMilliseconds(400);

	set_rgb_led(LED8, 10, 0, 0);
	set_rgb_led(LED2, 10, 0, 0);

	chThdSleepMilliseconds(400);

	set_led(LED1, ON);

	chThdSleepMilliseconds(400);

	clear_leds();

	readyAnimation();

	//get_acceleration(X_AXIS);
	//get_gyro_rate(X_AXIS);

	while (get_selector() == 1) {
		chThdSleepMilliseconds(500);
	}

	// go back to main
	main();
}

// @brief calibrates imu
void calibrate_tof(void) {

	// visualiser le mode
	set_led(LED7, ON);
	set_led(LED3, ON);
	chThdSleepMilliseconds(1000);

	/**********/
	/** HUGO **/
	/**********/

	// set local variable originPos
	//originPos = ??

	readyAnimation();

	while (get_selector() == 2) {
		chThdSleepMilliseconds(500);
	}

	// go back to main
	main();
}

// @brief calibrates imu
int get_originPos(void) {
	return originPos;
}
