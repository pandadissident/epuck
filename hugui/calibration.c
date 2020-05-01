#include <ch.h>
#include <hal.h>
#include <math.h>
#include <main.h>

#include "calibration.h"

#include "leds.h"
#include "selector.h"
#include "sensors/mpu9250.h"
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"


static float originPos = 0;
static thread_t *waitingLed_p;

// @brief makes led blink
static THD_WORKING_AREA(waitingLed_wa, 128);
static THD_FUNCTION(waitingLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    waitingLed_p = chThdGetSelfX();

    while(!chThdShouldTerminateX()){
    	set_led(LED5, TOGGLE);
    	chThdSleepMilliseconds(500);
    }

    set_led(LED5, OFF);
}

// @brief
void wait_for_stability(void) {

	uint16_t i = 0;
	int16_t altSum[SAMPLES][NB_AXIS] = {0}; //acceleration circular buffer of last 3 sec
	float tot = 0; //takes wide range of values
	bool unstable = TRUE;

	systime_t time;

	// blink led
	chThdCreateStatic(waitingLed_wa, sizeof(waitingLed_wa), NORMALPRIO, waitingLed, NULL);

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

	// kill blink led
	chThdTerminate(waitingLed_p);
}

// @brief calibrates imu and ir sensors aka proximity sensors
void calibrate_imuNprox(void) {

	wait_for_stability();

	set_led(LED5, ON);

	// calibrate gyro acc and ir with led animation
	calibrate_acc();
	chThdSleepMilliseconds(100);

	set_rgb_led(LED6, 10, 0, 0);
	set_rgb_led(LED4, 10, 0, 0);

	calibrate_gyro();
	chThdSleepMilliseconds(100);

	set_rgb_led(LED8, 10, 0, 0);
	set_rgb_led(LED2, 10, 0, 0);

	calibrate_ir();
	chThdSleepMilliseconds(100);

	set_led(LED1, ON);

	chThdSleepMilliseconds(400);
	clear_leds();
	readyAnimation();

	return;
}

// @brief calibrates imu
void tune_tof(void) {

	// visualiser le mode
	set_led(LED7, ON);
	set_led(LED3, ON);

	wait_for_stability();

	//drive_uphill();

	//start find_balance thread;

	//wait_for_stability();

	chThdSleepMilliseconds(1000);

	originPos = VL53L0X_get_dist_mm();

	readyAnimation();

	return;
}

// @brief calibrates imu
float get_originPos(void) {
	return originPos;
}
