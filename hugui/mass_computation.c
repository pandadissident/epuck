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

// @brief
void measure_mass(void)
{
	wait_for_stability();

//	drive_uphill();

	start_pid_regulator();

//	start_assess_stability();

//    while (!get_equilibrium()) {
//    	chThdSleepMilliseconds(500);
//    }
//
//	send_mass();

	right_motor_set_speed(STOP);
	left_motor_set_speed(STOP);

	readyAnimation();
	return;
}

// @brief
void send_mass(void)
{
	float massBig, massMedium, massSmall = 0;
	static float originPos, eqPos = 0;

	originPos = get_pos_zero();
	eqPos = get_distance();

	massBig = M_EPUCK*(originPos-eqPos)/(L_BASCULE/4);
	massMedium = M_EPUCK*(originPos-eqPos)/(L_BASCULE/2);
	massSmall = M_EPUCK*(originPos-eqPos)/(3*L_BASCULE/4);

	chprintf((BaseSequentialStream *)&SD3, "RESULTATS :/n");
	chprintf((BaseSequentialStream *)&SD3, "Petite masse  = %f/n", massSmall);
	chprintf((BaseSequentialStream *)&SD3, "Masse moyenne = %f/n", massMedium);
	chprintf((BaseSequentialStream *)&SD3, "Grosse masse  = %f/n", massBig);

	return;
}
