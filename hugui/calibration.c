#include <ch.h>
#include <hal.h>
#include <math.h>
#include <main.h>

#include "calibration.h"

#include "leds.h"
#include "pid_regulator.h"
#include "selector.h"
#include "sensors/mpu9250.h"
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"

// threads
static thread_t *waitingLed_p = NULL;

// static variables
static uint16_t originPosition = 0;

// @brief makes led blink
static THD_WORKING_AREA(waitingLed_wa, 128);
static THD_FUNCTION(waitingLed, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    waitingLed_p = chThdGetSelfX();

    while(!chThdShouldTerminateX()) {
    	set_led(LED5, TOGGLE);
    	chThdSleepMilliseconds(500);
    }

    set_led(LED5, OFF);
}

void wait_for_stability(void)
{
	uint16_t i = 0;
	int16_t altSum[SAMPLES][NB_AXIS] = {0}; //acceleration circular buffer of samples
	float tot = 0; //takes wide range of values
	bool unstable = TRUE;

	// blink led
	chThdCreateStatic(waitingLed_wa, sizeof(waitingLed_wa), NORMALPRIO, waitingLed, NULL);

	// wait for EPuck to be stable
	while (unstable) {

		get_acc_all(altSum[i % SAMPLES]);

		if (i % 2) {
			tot = 0;
			for (uint8_t j = 0 ; j < SAMPLES ; j += 2) {
				tot += fabs(altSum[j][X_AXIS] - altSum[j+1][X_AXIS]) + fabs(altSum[j][Y_AXIS] - altSum[j+1][Y_AXIS]) + fabs(altSum[j][Z_AXIS] - altSum[j+1][Z_AXIS]);
			}
			if ((i > 3*SAMPLES) && (tot < STABILITY_THRESHOLD)) {
				unstable = FALSE;
			}
		}

		i++;

		// prevent overflow and resets
		if (i >= MAX_ITERATIONS) {
			i = 0;
			for (uint8_t j = 0 ; j < SAMPLES ; j++) {
				for (uint8_t k = 0 ; k < NB_AXIS ; k++) {
					altSum[j][k] = 0;
				}
			}
		}

		chThdSleepMilliseconds(100);
	}
	// kill blink led
	chThdTerminate(waitingLed_p);
	chThdWait(waitingLed_p);
	waitingLed_p = NULL;
}

void calibrate_imu_prox(void)
{
	wait_for_stability();

    set_led(LED5, ON);

	calibrate_ir();

	set_rgb_led(LED6, 0, 100, 100);
	set_rgb_led(LED4, 0, 100, 100);

	calibrate_gyro();

	set_rgb_led(LED8, 100, 100, 100);
	set_rgb_led(LED2, 100, 100, 100);

	calibrate_acc();

	set_led(LED1, ON);

	chThdSleepMilliseconds(300);

	return;
}

void calibrate_tof(void)
{
	reset_equilibrium();
	wait_for_stability();

	drive_uphill();

	start_pid_regulator();
	start_assess_stability();

	while (!equilibrium_found()) {
		chThdSleepMilliseconds(500);
	}

	// is only used to rotate the EPuck
	mesure_position();

	originPosition = tof_distance();

	chThdSleepMilliseconds(200);

	return;
}

uint16_t tof_distance(void)
{
	uint16_t distance = 0;
	uint8_t i = 0;

	set_front_led(ON);
	for (i = 0; i<SAMPLES; i++) {
		distance += VL53L0X_get_dist_mm()/SAMPLES;
		chThdSleepMilliseconds(50);
	}
	set_front_led(OFF);

	return distance;
}

uint16_t get_pos_zero(void) {
	return originPosition;
}
