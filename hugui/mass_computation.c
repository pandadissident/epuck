#include <ch.h>
#include <hal.h>
#include <math.h>
#include <main.h>

#include "mass_computation.h"

#include "calibration.h"
#include "chprintf.h"
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

//    while (!get_equilibrium()) {
//    	//chprintf((BaseSequientialStream*&SD3,"Attente d'équilibre"));
//    	chThdSleepMilliseconds(500);
//    }

	//send_mass();

	readyAnimation();
	return;

}

void send_mass(void)
{
	float mass_h, mass_m, mass_l = 0;
	static float originPos, eqPos = 0;

	mass_h = M_EPUCK*(originPos-eqPos)/(L_BASCULE/4);
	mass_m = M_EPUCK*(originPos-eqPos)/(L_BASCULE/2);
	mass_l = M_EPUCK*(originPos-eqPos)/(3*L_BASCULE/4);

	//chprintf((BaseSequentialStream *)&SD3,"Petite masse = %f",mass_l));
	//chprintf((BaseSequentialStream *)&SD3,"Moyenne masse = %f",mass_m));
	//chprintf((BaseSequentialStream *)&SD3,"Grosse masse = %f",mass_h));

	return;

}
