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
	float originPos, eqPos = 0;
	float massBig, massMedium, massSmall = 0;

	originPos = get_pos_zero();
	eqPos = get_distance();

	massBig = M_EPUCK*(originPos-eqPos)/(L_BASCULE/4);
	massMedium = M_EPUCK*(originPos-eqPos)/(L_BASCULE/2);
	massSmall = M_EPUCK*(originPos-eqPos)/(3*L_BASCULE/4);

	chprintf((BaseSequentialStream *)&SD3, "RESULTATS :\n");
	chThdSleepMilliseconds(100);
	chprintf((BaseSequentialStream *)&SD3, "Petite masse  = %.2f\n", massSmall);
	chThdSleepMilliseconds(100);
	chprintf((BaseSequentialStream *)&SD3, "Masse moyenne = %.2f\n", massMedium);
	chThdSleepMilliseconds(100);
	chprintf((BaseSequentialStream *)&SD3, "Grosse masse  = %.2f\n", massBig);

	return;
}
