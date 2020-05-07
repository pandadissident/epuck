#include <ch.h>
#include <hal.h>
#include <math.h>
#include <main.h>
#include <chprintf.h>

#include "pid_regulator.h"

#include "calibration.h"
#include "motors.h"
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"

static bool equilibre = FALSE;

static thread_t *pidRegulator_p = NULL;
static thread_t *social_distancing_p = NULL;


// @brief
static THD_WORKING_AREA(social_distancing_wa, 128);
static THD_FUNCTION(social_distancing, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    float error, distance = 0;

    social_distancing_p = chThdGetSelfX();
    distance = get_pos_zero();

    while (!chThdShouldTerminateX()) {

    	error = VL53L0X_get_dist_mm() - distance;

    	if ( fabs(error) > 10) {
        	right_motor_set_speed(5*error);
        	left_motor_set_speed(5*error);
    	} else {
        	right_motor_set_speed(0);
        	left_motor_set_speed(0);
    	}
    	chThdSleepMilliseconds(250);
    }
}


// @brief
static THD_WORKING_AREA(pidRegulator_wa, 1024);
static THD_FUNCTION(pidRegulator, arg) {

	chRegSetThreadName(__FUNCTION__);
    (void)arg;

    pidRegulator_p = chThdGetSelfX();

    systime_t time = 0;

    float speed = 0;
    float rotation = 0;
    float angle = 0;

    while(!chThdShouldTerminateX()){

        //computes the angle
		angle = angle_estimation();
		chprintf((BaseSequentialStream *)&SD3, "Angle : %.3f\n", angle);
		//computes speed to which epuck is going to travel
        speed = pd_speed(angle);
        chprintf((BaseSequentialStream *)&SD3, "Speed : %.3f\n", speed);
        //computes a correction factor to let the robot rotate to stay in the path
        rotation = 0;//pi_yaw_correction(speed);
        chprintf((BaseSequentialStream *)&SD3, "Rotation speed : %.3f\n", rotation);
    	chprintf((BaseSequentialStream *)&SD3, "\n");

        //applies the speed from the PID regulator and the correction for the rotation
		right_motor_set_speed(speed - rotation);
		left_motor_set_speed(speed + rotation);

		//assess_stability();

        //sample at 100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}


void drive_uphill(void)
{

	float angle, gyro = 0;

	angle = angle_estimation();

	if ( angle > 0 ) {
		right_motor_set_speed(250);
		left_motor_set_speed(250);
	} else {
		right_motor_set_speed(-250);
		left_motor_set_speed(-250);
	}

	chThdSleepMilliseconds(100);

	while (gyro < TIPPING_THRESHOLD) {
		chThdSleepMilliseconds(50);
		gyro = fabs(get_gyro_rate(X_AXIS));
	}

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	return;
}

// @brief numerical PI regulator implementation
float pi_yaw_correction(float speed) {

	static float error[3] = {0};
	static float output[3] = {0};

	// update variables
	error[2] = error[1];
	error[1] = error[0];
	output[2] = output[1];
	output[1] = output[0];

	//error = distance - goal;
	error[0] =  0.25*(get_prox(4) - get_prox(3)); 	// back
	error[0] += 1.00*(get_prox(5) - get_prox(2));	// sides
	error[0] += 2*(get_prox(6) - get_prox(1));	// front
	error[0] += 0.75*(get_prox(7) - get_prox(0));	// front

	//chprintf((BaseSequentialStream *)&SD3, "yaw error : %.3f\n", error[0]);

	//disables the PID regulator if the error is too small
	if(fabs(error[0]) < 5*YAW_ERROR_THRESHOLD){
		return 0;
	}

	// corrects error sign given the direction where the epuck is heading
	if (speed < 0) {
		error[0] = - error[0];
    }

	output[0] = - KU1*output[1] - KU2*output[2] + KE0*error[0] + KE1*error[1] + KE2*error[2];

	// anti windup
	if(output[0] > ROTATION_THRESHOLD){
		output[0] = ROTATION_THRESHOLD;
	} else if (output[0] < -ROTATION_THRESHOLD) {
		output[0] = -ROTATION_THRESHOLD;
	}

    return output[0];
}


// @brief simple PI regulator implementation
float pd_speed(float angle) {

	static float error, der_error, speed = 0;

	// proportional error
	error = angle;

	//disables the PID regulator if the error is too small
	if(fabs(error) < ANGLE_ERROR_THRESHOLD){
		return 0;
	}

	// derivative error with lowpass filter
	der_error = 0.9*der_error + 0.1*get_gyro_rate(X_AXIS);

	chprintf((BaseSequentialStream *)&SD3, " - angle errror : %.3f\n", error);
	chprintf((BaseSequentialStream *)&SD3, " - angle derivative : %.3f\n", der_error);

	speed = KP_SPEED*error + KD_SPEED*der_error;

    return speed;
}

float complementary_lowpass(float input1, float input2) {

	float cutoff = 0.90;
	float gyro_pond = 0.99;
	static float output = 0;
	static bool initialiased = FALSE;

	if(!initialiased) {
		output = (input1+input2)/2;
		initialiased = TRUE;
	}

	output = (1-cutoff)*(gyro_pond*input1 + (1-gyro_pond)*input2) + cutoff*output;

	return output;
}

float angle_estimation(void) {

    static float dt = 0.01;
    static float angle, acc_angle, gyro_angle, temp = 0;
    static int16_t acc_values[NB_AXIS] = {0};

    // estimate angle through acceleration
    get_acc_all(acc_values);

	temp = atan2(acc_values[Y_AXIS], acc_values[Z_AXIS])*180/M_PI;
	if (temp > 0) {
		acc_angle = temp - 180;
	} else {
		acc_angle = temp + 180;
	}

	// estimate angle through gyro
	gyro_angle = angle + get_gyro_rate(X_AXIS)*dt;

	// complementary filter and invert sign
    angle = complementary_lowpass(gyro_angle, acc_angle);

    return - angle;
}

void start_pid_regulator(void) {
	if (pidRegulator_p == NULL) {
		chThdCreateStatic(pidRegulator_wa, sizeof(pidRegulator_wa), NORMALPRIO, pidRegulator, NULL);
	}
	return;
}

void stop_pid_regulator(void){
	if (pidRegulator_p != NULL) {
		chThdTerminate(pidRegulator_p);
		chThdWait(pidRegulator_p);
		pidRegulator_p = NULL;
	}
	return;
}

void start_social_distancing(void){
	if (social_distancing_p == NULL) {
		chThdCreateStatic(social_distancing_wa, sizeof(social_distancing_wa), NORMALPRIO, social_distancing, NULL);
	}
	return;
}


void stop_social_distancing(void){
	if (social_distancing_p != NULL) {
		chThdTerminate(social_distancing_p);
		chThdWait(social_distancing_p);
		social_distancing_p = NULL;
	}
	return;
}

bool get_equilibrium(void){
	return equilibre;
}
