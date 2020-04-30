#include <ch.h>
#include <hal.h>
#include <math.h>
#include <main.h>

#include "mass_computation.h"

#include "calibration.h"
#include "selector.h"
#include "sensors\VL53L0X\VL53L0X.h"
#include "leds.h"


int computeMass(int eqPos) {

	int8_t mass = 0;

	//blabla

	return mass;
}

void measure_mass(void) {

	int8_t originPos, eqPos = 0;
	bool equilibre = FALSE;

	// visualiser le mode
	set_led(LED5, ON);
	set_led(LED1, ON);
	chThdSleepMilliseconds(1000);

	originPos = get_originPos();

	pid_regulator_start();

	if(equilibre){
		//check add tof
	}

    //launch ir thread for lateral positioning

    //driveTo(ORIGIN); if not already

    // Waiting 5 seconds to place weight onto scale
    //chThdSleepMilliseconds(5000);

    //eqPosEstimate = estimateTipOverPosition(); //commande en vitesse
    	// -> start thread to check angular velocity
    	// -> driveTo(END)
    	// -> while (1)
    	// -> return estimated x value

    //eqPos = preciseTipOverPosition(eqPosEstimate); //commande en position
    	// -> start PID thread
    	// -> maintainPosition(); for t seconds
    	// -> return accurate x value

    //mass = computeMass(eqPos);

    //sendMass(mass); //par bluetooth

	readyAnimation();

	while (get_selector() == 0) {
		chThdSleepMilliseconds(500);
	}

	// go back to main
	main();
}
