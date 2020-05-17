#include <ch.h>
#include <chprintf.h>
#include <hal.h>
#include <main.h>
#include <math.h>

#include "mass_computation.h"

#include "calibration.h"
#include "leds.h"
#include "pid_regulator.h"
#include "selector.h"
#include "sensors\VL53L0X\VL53L0X.h"
#include "serial.h"

void measure_mass(void)
{
	reset_equilibrium();
	wait_for_stability();

	drive_uphill();

	start_pid_regulator();
	start_assess_stability();

    while (!equilibrium_found()) {
    	set_front_led(TOGGLE);
    	chThdSleepMilliseconds(500);
    }

    set_front_led(OFF);

	mesure_position();

	send_mass();

	return;
}


void send_mass(void)
{
	float distance = 0;
	float massBig = 0;
	float massMedium = 0;
	float massSmall = 0;

	distance = get_pos_zero() - get_distance();

	//empiric factor correction because epuck tends to overestimate values when farther away
	distance *= CORRECTION;

	massBig = M_EPUCK*distance/L_BASCULE*8;
	massMedium = M_EPUCK*distance/L_BASCULE*4;
	massSmall = M_EPUCK*distance/L_BASCULE*8/3;

	chprintf((BaseSequentialStream *)&SD3, "\nRESULTATS :\n");
	chThdSleepMilliseconds(5);
	chprintf((BaseSequentialStream *)&SD3, "Petite masse");
	chThdSleepMilliseconds(5);
	chprintf((BaseSequentialStream *)&SD3, "  = %.2f\n", massSmall);
	chThdSleepMilliseconds(5);
	chprintf((BaseSequentialStream *)&SD3, "Masse moyenne");
	chThdSleepMilliseconds(5);
	chprintf((BaseSequentialStream *)&SD3, " = %.2f\n", massMedium);
	chThdSleepMilliseconds(5);
	chprintf((BaseSequentialStream *)&SD3, "Grosse masse");
	chThdSleepMilliseconds(5);
	chprintf((BaseSequentialStream *)&SD3, "  = %.2f\n", massBig);
	chThdSleepMilliseconds(5);
	chprintf((BaseSequentialStream *)&SD3, "distance");
	chThdSleepMilliseconds(5);
	chprintf((BaseSequentialStream *)&SD3, "  = %.2f\n", distance);

	return;
}
