#include "calibration.h"

#include "sensors/imu.h"
#include "leds.h"
#include "selector.h"
#include "main.h"
#include <math.h>


void calibrate_imu(void) {

	uint8_t i = 0;
	int16_t altSum[6][3] = {0}; //circular buffer of the last 6 acceleration values
	int16_t temp[3] = {0};
	uint16_t tot = 0;
	bool unstable = TRUE;

	systime_t time;

	// led animation
	set_led(LED5, ON);
	chThdSleepMilliseconds(500);


	// wait for EPuck to be stable
	while ((unstable) + (i<6)) {

		get_acc_all(temp);
		time = chVTGetSystemTime();

		if (i % 2 == 1) {
			altSum[i % 6][0] -= temp[0];
			altSum[i % 6][1] -= temp[1];
			altSum[i % 6][2] -= temp[2];

			tot = 0;

			for (uint8_t j = 0 ; j < 6 ; j++) {
				tot += fabs(altSum[j][0]) + fabs(altSum[j][1]) + fabs(altSum[j][2]);
			}

			if (tot < THRESHOLD) {
				unstable = FALSE;
			}
		} else {
			altSum[i % 6][0] += temp[0];
			altSum[i % 6][1] += temp[1];
			altSum[i % 6][2] += temp[2];
		}
		i++;

		//reduced the sample rate
		chThdSleepUntilWindowed(time, time + MS2ST(200));

	}

	// led animation
	//set_rgb_led(LED6, 10, 0, 0);
	//set_rgb_led(LED4, 10, 0, 0);
	//chThdSleepMilliseconds(750);
	set_led(LED7, ON);
	set_led(LED3, ON);
	chThdSleepMilliseconds(750);
	//set_rgb_led(LED8, 10, 0, 0);
	//set_rgb_led(LED2, 10, 0, 0);
	//chThdSleepMilliseconds(750);
	set_led(LED1, ON);

	// calibration
	calibrate_acc();
	calibrate_gyro();
	clear_leds();

	// blink body led until selector is switched
	while (get_selector() == 1) {
		set_body_led(TOGGLE);
		chThdSleepMilliseconds(500);
	}

	main();
}
