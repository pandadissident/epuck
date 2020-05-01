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

static float masses[3];
static thread_t *equilibrium_p;

// @brief
static THD_WORKING_AREA(equilibrium_wa, 128);
static THD_FUNCTION(equilibrium, arg) {

	float eqPos, originPos = 0;

    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    equilibrium_p = chThdGetSelfX();

    //originPos = get_originPos();

    while (!get_eq()) {
    	//chprintf((BaseSequientialStream*&SD3,"Attente d'équilibre"));
    	chThdSleepMilliseconds(500);
    }

	eqPos = VL53L0X_get_dist_mm();
	compute_mass(eqPos,originPos);
    readyAnimation();

}

void compute_mass(float eqPos, float originPos) {

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

	// visualiser le mode
	set_led(LED5, ON);
	set_led(LED1, ON);

	wait_for_stability();

	// find balanced position and lateral positioning
	pid_regulator_start();
	find_equilibrium_start();

	//send mass via bluetooth
	send_mass();

}

void send_mass(void) {

	//chprintf((BaseSequientialStream*&SD3,"Petite masse = %f",masses[2]));
	//chprintf((BaseSequientialStream*&SD3,"Moyenne masse = %f",masses[1]));
	//chprintf((BaseSequientialStream*&SD3,"Grosse masse = %f",masses[0]));

}

void find_equilibrium_start(void) {
	chThdCreateStatic(equilibrium_wa, sizeof(equilibrium_wa), NORMALPRIO, equilibrium, NULL);
}

void find_equilibrium_stop(void) {
	chThdTerminate(equilibrium_p);
}
