#include <ch.h>
#include <hal.h>
#include <math.h>
#include <main.h>

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
static THD_WORKING_AREA(social_distancing_wa, 256);
static THD_FUNCTION(social_distancing, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    float error, distance = 0;

    social_distancing_p = chThdGetSelfX();
    distance = get_originPos();

    while (!chThdShouldTerminateX()) {

    	error = VL53L0X_get_dist_mm() - distance;

    	if ( abs(error) > 10) {
        	right_motor_set_speed(5*error);
        	left_motor_set_speed(5*error);
    	} else {
        	right_motor_set_speed(0);
        	left_motor_set_speed(0);
    	}
    	chThdSleepMilliseconds(250);
    }
}


// @brief simple PI regulator implementation
float pi_yaw_correction(void) {

	static float error[3] = {0};
	static float output[3] = {0};

	// update variables
	error[2] = error[1];
	error[1] = error[0];
	output[2] = output[1];
	output[1] = output[0];

	//error = distance - goal;
	error[0] = get_prox(4) + get_prox(5) + get_prox(6) + get_prox(7) - (get_prox(0) + get_prox(1) + get_prox(2) +  get_prox(3));

	//disables the PID regulator if the error is too small
	if(fabs(error[0]) < 5*YAW_ERROR_THRESHOLD){
		return 0;
	}

//	// corrects error sign given the direction where the epuck is heading
//	if (speed < 0) {
//		error[0] = - error[0];
//    }

	output[0] = - rku1*output[1] - rku2*output[2] + rke0*error[0] + rke1*error[1] + rke2*error[2];

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

	static float error[2] = {0};
	static float der_error, speed = 0;
	static float dt = 0.01;

	// update variables
	error[1] = error[0];

	// proportional error
	error[0] = angle;

	//disables the PID regulator if the error is too small
	if(fabs(error[0]) < ANGLE_ERROR_THRESHOLD){
		return 0;
	}

	// derivative error with lowpass filter
	der_error = 0.33*(error[0] - error[1])/dt + 0.67*der_error;

	// non linear for lower speed near zero
	//speed = pow(KP_SPEED*error[0] + KI_SPEED*int_error + KD_SPEED*der_error,3);
	speed = KP_SPEED*error[0] + KD_SPEED*der_error;

    return speed;
}

float complementary_lowpass(float input1, float input2) {

	float a = 0.90;
	float b = 0.99;
	static float output;

	output = (1-a)*(b*input1 + (1-b)*input2) + a*output;

	return output;
}

float angle_estimation(void) {

    static float dt = 0.01;
    static float angle, acc_angle, gyro_angle, temp, acc = 0;
    static float acc_values[NB_AXIS] = {0};

    // estimate angle through acceleration
	acc_values[X_AXIS] = get_acceleration(X_AXIS);
	acc_values[Y_AXIS] = get_acceleration(Y_AXIS);
	acc_values[Z_AXIS] = get_acceleration(Z_AXIS);

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

// @brief
static THD_WORKING_AREA(pidRegulator_wa, 2048);
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
		//computes speed to which epuck is going to travel
        speed = pd_speed(angle);
        //computes a correction factor to let the robot rotate to stay in the path


        //applies the speed from the PID regulator and the correction for the rotation
		right_motor_set_speed(speed - rotation);
		left_motor_set_speed(speed + rotation);

        //sample at 100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pid_regulator_start(void){
	if (pidRegulator_p == NULL) {
		chThdCreateStatic(pidRegulator_wa, sizeof(pidRegulator_wa), NORMALPRIO, pidRegulator, NULL);
	}
}

void pid_regulator_stop(void){
	if (pidRegulator_p != NULL) {
		chThdTerminate(pidRegulator_p);
		chThdWait(pidRegulator_p);
		pidRegulator_p = NULL;
	}
}

void start_social_distancing(void){
	if (social_distancing_p == NULL) {
		chThdCreateStatic(social_distancing_wa, sizeof(social_distancing_wa), NORMALPRIO, social_distancing, NULL);
	}
}

void stop_social_distancing(void){
	if (social_distancing_p != NULL) {
		chThdTerminate(social_distancing_p);
		chThdWait(social_distancing_p);
		social_distancing_p = NULL;
	}
}

bool get_eq(void){
	return equilibre;
}
