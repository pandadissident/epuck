#include <ch.h>
#include <hal.h>
#include <math.h>
#include <main.h>

#include "mass_computation.h"


void measure_mass(void) {

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

    //sendMass(mass);

	return;

}

