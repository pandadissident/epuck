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

static bool initialized = FALSE;
static bool equilibre = FALSE;

static thread_t *pidRegulator_p;
static thread_t *drive_uphill_p;
static thread_t *social_distancing_p;

// @brief
static THD_WORKING_AREA(drive_uphill_wa, 256);
static THD_FUNCTION(drive_uphill, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    float acc = 0;

    drive_uphill_p = chThdGetSelfX();

    while (!chThdShouldTerminateX()) {
    	acc = get_acceleration(Y_AXIS);
    	if ( acc > 0.1) {
        	right_motor_set_speed(50*acc*acc);
        	left_motor_set_speed(50*acc*acc);
    	} else if ( acc < -0.1) {
        	right_motor_set_speed(-50*acc*acc);
        	left_motor_set_speed(-50*acc*acc);
    	} else {
        	right_motor_set_speed(0);
        	left_motor_set_speed(0);
    	}
    	chThdSleepMilliseconds(250);
    }
}

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

// @brief simple PID regulator implementation
int16_t pid_regulator_align(void){

	float error_align = 0;
	float speed = 0;

	static float sum_error_align = 0;

	//error = distance - goal;
	error_align = get_prox(4) + get_prox(5) + get_prox(6) + get_prox(7) - (get_prox(0) + get_prox(1) + get_prox(2) +  get_prox(3));
    if (get_acceleration(Y_AXIS) < 0) {
    	error_align = - error_align;
    }
	//disables the PID regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error_align) < 5*ERROR_THRESHOLD_ALIGN){
		return 0;
	}

	sum_error_align += error_align;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error_align > MAX_SUM_ERROR_ALIGN){
		sum_error_align = MAX_SUM_ERROR_ALIGN;
	}else if(sum_error_align < -MAX_SUM_ERROR_ALIGN){
		sum_error_align = -MAX_SUM_ERROR_ALIGN;
	}

	speed = KP_ALIGN * error_align + KI_ALIGN * sum_error_align;

    return (int16_t)speed;
}

// @brief
int16_t pid_regulator_angle(float angle){

		float error_angle = 0;
		float speed = 0;

		static float sum_error_angle = 0;

		//error = distance - goal;
		error_angle = angle;
		//disables the PID regulator if the error is to small
		//this avoids to always move as we cannot exactly be where we want and
		//the camera is a bit noisy
		if(fabs(error_angle) < ERROR_THRESHOLD_ANGLE){
			equilibre = TRUE;
			return 0;
		}

		sum_error_angle += error_angle;

		//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
		if(sum_error_angle > MAX_SUM_ERROR_ANGLE){
			sum_error_angle = MAX_SUM_ERROR_ANGLE;
		}else if(sum_error_angle < -MAX_SUM_ERROR_ANGLE){
			sum_error_angle = -MAX_SUM_ERROR_ANGLE;
		}

		speed = KP_ANGLE * error_angle + KI_ANGLE * sum_error_angle;

	    return (int16_t)speed;
	}

// @brief
static THD_WORKING_AREA(pidRegulator_wa, 1024);
static THD_FUNCTION(pidRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    pidRegulator_p = chThdGetSelfX();

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;
    float angle_mes = 0;
    float dt = 0.01;//time window between two measurement

    while(!chThdShouldTerminateX()){
        time = chVTGetSystemTime();
        float acc_values[NB_AXIS] = {0,0,0};
        float gyro_pitch, r = 0;

        // calcul de l'angle initial
        if(!initialized){
       	acc_values[X_AXIS] = get_acceleration(X_AXIS);
       	acc_values[Y_AXIS] = get_acceleration(Y_AXIS);
       	acc_values[Z_AXIS] = get_acceleration(Z_AXIS);
       	r = sqrt(acc_values[X_AXIS]*acc_values[X_AXIS] + acc_values[Y_AXIS]*acc_values[Y_AXIS] + acc_values[Z_AXIS]*acc_values[Z_AXIS]);
       	angle_mes = 180 - acos(acc_values[Z_AXIS]/r)*180/3.1415;
       	initialized = TRUE;
       }
//
        //mesure de l'angle

        gyro_pitch = get_gyro_rate(X_AXIS);

        angle_mes += gyro_pitch*dt;

        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
		speed = pid_regulator_angle(angle_mes);
        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = pid_regulator_align();

        //if the epuck goes straight, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }

        //applies the speed from the PID regulator and the correction for the rotation
		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pid_regulator_start(void){
	chThdCreateStatic(pidRegulator_wa, sizeof(pidRegulator_wa), NORMALPRIO, pidRegulator, NULL);
}

void start_social_distancing(void){
	chThdCreateStatic(social_distancing_wa, sizeof(social_distancing_wa), NORMALPRIO, social_distancing, NULL);
}

void stop_social_distancing(void){
	chThdTerminate(social_distancing_p);
}


void pid_regulator_stop(void){
	chThdTerminate(pidRegulator_p);
}

void drive_uphill_start(void){
	chThdCreateStatic(drive_uphill_wa, sizeof(drive_uphill_wa), NORMALPRIO, drive_uphill, NULL);
}

void drive_uphill_stop(void){
	chThdTerminate(drive_uphill_p);
}

bool get_eq(void){
	return equilibre;
}
