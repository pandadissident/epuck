#include <ch.h>
#include <hal.h>
#include <math.h>
#include <main.h>

#include "calibration.h"

#include "sensors/imu.h"
#include "leds.h"
#include "selector.h"

// @brief makes led blink
static THD_WORKING_AREA(blinkLed_wa, 128);
static THD_FUNCTION(blinkLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(!chThdShouldTerminateX()){
    	set_led(LED5, TOGGLE);
    	chThdSleepMilliseconds(500);
    }

    chThdExit((msg_t)"");
}

// @brief calibrates imu
void calibrate_imu(void) {

	uint8_t i = 0;
	int16_t altSum[6][3] = {0}; //acceleration circular buffer of last 3 sec
	int16_t tot = 0;
	bool unstable = TRUE;

	systime_t time;

	// blink led
	thread_t *blinkLed_p = chThdCreateStatic(blinkLed_wa, sizeof(blinkLed_wa), NORMALPRIO, blinkLed, NULL);


	// wait for EPuck to be stable
	while (unstable) {

		get_acc_all(altSum[i % 6]);
		time = chVTGetSystemTime();

		if (i % 2) {

			tot = 0;

			// alternate sum of all acceleration values
			for (uint8_t j = 0 ; j < 6 ; j += 2) {
				tot += altSum[j][0] - altSum[j+1][0] + altSum[j][1] - altSum[j+1][1] + altSum[j][2] - altSum[j+1][2];
			}

			if ((fabs(tot) < THRESHOLD) && (i > 12)) {
				unstable = FALSE;
			}
		}

		i++;

		//reduced the sample rate
		chThdSleepUntilWindowed(time, time + MS2ST(250));

	}

	// kill blink led
	chThdTerminate(blinkLed_p);

	// calibration with led animation
	set_led(LED5, ON);
	calibrate_acc(); //pk ça éteint la led ?
	calibrate_gyro();
	set_led(LED5, ON);
	chThdSleepMilliseconds(500);

	// why not work ? waii ?
	set_rgb_led(LED6, 10, 0, 0);
	set_rgb_led(LED4, 10, 0, 0);
	chThdSleepMilliseconds(750);

	set_led(LED7, ON);
	set_led(LED3, ON);
	chThdSleepMilliseconds(500);
	//set_rgb_led(LED8, 10, 0, 0);
	//set_rgb_led(LED2, 10, 0, 0);
	//chThdSleepMilliseconds(750);
	set_led(LED1, ON);
	chThdSleepMilliseconds(500);
	clear_leds();

	// blink body led until selector is switched
	while (get_selector() == 1) {
		set_body_led(TOGGLE);
		chThdSleepMilliseconds(500);
	}

	set_body_led(OFF);
	main();
}
