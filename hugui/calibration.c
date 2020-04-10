#include "calibration.h"

/*

//Effet barre de chargement

static THD_WORKING_AREA(waitForMeasureAnimation, 128);
static THD_FUNCTION(waitForMeasureAnimation, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    static uint8_t angle = 0;

	/*
    * Quadrant:
    *
    *       BACK
    *       ####
    *    #    0   #
    *  #            #
    * #-PI/2 TOP PI/2#
    * #      VIEW    #
    *  #            #
    *    # -PI|PI #
    *       ####
    *       FRONT
    */

	/*

    while(1){

        time = chVTGetSystemTime();

		//rotates the angle by 45 degrees (simpler to compare with PI and PI/2 than with 5*PI/4)
		angle += M_PI/4;

		//if the angle is greater than PI, then it has shifted on the -PI side of the quadrant
		//so we correct it
		if(angle > M_PI){
			angle = -2 * M_PI + angle;
		}

		if(angle >= 0 && angle < M_PI/2){
			led5 = 1;
		} else if(angle >= M_PI/2 && angle < M_PI){
			led7 = 1;
		} else if(angle >= -M_PI && angle < -M_PI/2){
			led1 = 1;
		} else if(angle >= -M_PI/2 && angle < 0){
			led3 = 1;
		}

		//we invert the values because a led is turned on if the signal is low
		palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
		palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
		palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
		palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);

        chThdSleepUntilWindowed(time, time + MS2ST(100));
    }

}
*/
