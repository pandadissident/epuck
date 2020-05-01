#include <ch.h>
#include <hal.h>
#include <math.h>
#include <main.h>

#include "mass_computation.h"
#include "chprintf.h"
#include "calibration.h"
#include "selector.h"
#include "sensors\VL53L0X\VL53L0X.h"
#include "leds.h"


static float masses[3];

void computeMass(float eqPos, float originPos) {

	float mass_h = 0;
	float mass_m = 0;
	float mass_l = 0;

	mass_h = Me*(originPos-eqPos)/(L/4);
	mass_m = Me*(originPos-eqPos)/(L/2);
	mass_l = Me*(originPos-eqPos)/(3*L/4);

	masses[0] = mass_h;
	masses[1] = mass_m;
	masses[2] = mass_l;
}

void measure_mass(void) {

	float posEstimate, eqPos, originPos = 0;
	bool equilibre = FALSE;

	// visualiser le mode
	set_led(LED5, ON);
	set_led(LED1, ON);
	chThdSleepMilliseconds(1000);

	originPos = get_originPos();

	pid_regulator_start();
	equilibre = get_eq;
	if(equilibre){

		eqPos = VL53L0X_get_dist_mm();
		computeMass(eqPos,originPos);

		chprintf((BaseSequientialStream*&SD3,"Petite masse = %f",masses[2]));
		chprintf((BaseSequientialStream*&SD3,"Moyenne masse = %f",masses[1]));
		chprintf((BaseSequientialStream*&SD3,"Grosse masse = %f",masses[0]));

		readyAnimation();

	}

	else {
		chprintf((BaseSequientialStream*&SD3,"Attente d'équilibre"));
	}

    //launch ir thread for lateral positioning

    //driveTo(ORIGIN); if not already

    // Waiting 5 seconds to place weight onto scale
    //chThdSleepMilliseconds(5000);


    //sendMass(mass); //par bluetooth

}
