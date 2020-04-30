#include "pid_regulator.h"

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <main.h>

#include "motors.h"
#include "sensors\imu.h"
#include "sensors\proximity.h"

static bool initialized = FALSE;
static bool equilibre = FALSE;

void straight_line(void) {

	float speed = 10;

	right_motor_set_speed(speed);
	left_motor_set_speed(speed);

}

//simple PID regulator implementation
int16_t pid_regulator_align(int dist_lat_r, int dist_lat_l){

	float error_align = 0;
	float speed = 0;

	static float sum_error_align = 0;

	//error = distance - goal;
	error_align = dist_lat_r - dist_lat_l;
	//disables the PID regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error_align) < ERROR_THRESHOLD_ALIGN){
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

		equilibre = FALSE;

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

static THD_WORKING_AREA(waPidRegulator, 256);
static THD_FUNCTION(PidRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;
    float angle_init = 0;
    float angle_mes = 0;
    float dt = 0.01;//time window between two measurement



    while(1){
        time = chVTGetSystemTime();
        int acc_values[NB_AXIS] = {0,0,0};
        float gyro_pitch = 0;
        float acc_angle = 0;
        float gyro_angle = 0;

        // calcul de l'angle initial
        if(!initialized){
        	acc_values[X_AXIS] = get_acceleration(X_AXIS);
        	acc_values[Y_AXIS] = get_acceleration(Y_AXIS);
        	acc_values[Z_AXIS] = get_acceleration(Z_AXIS);
        	angle_init = asin((float)acc_values[Z_AXIS]/sqrt(pow(acc_values[X_AXIS],2) + pow(acc_values[Y_AXIS],2) + pow(acc_values[Z_AXIS],2)));
        	gyro_angle = angle_init;
        	initialized = TRUE;
        }
        
        //mesure de l'angle
        acc_values[X_AXIS] = get_acceleration(X_AXIS);
        acc_values[Y_AXIS] = get_acceleration(Y_AXIS);
        acc_values[Z_AXIS] = get_acceleration(Z_AXIS);
        gyro_pitch = get_gyro_rate(X_AXIS);

        acc_angle = asin((float)acc_values[Z_AXIS]/sqrt(pow(acc_values[X_AXIS],2) + pow(acc_values[Y_AXIS],2) + pow(acc_values[Z_AXIS],2)));

        gyro_angle -= gyro_pitch*dt;

        //compenser la dérive du gyro
        angle_mes = 0.996 * gyro_angle + 0.004 * acc_angle;
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        speed = pid_regulator_angle(angle_mes);
        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = pid_regulator_align(get_prox(2),get_prox(5));

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
	chThdCreateStatic(waPidRegulator, sizeof(waPidRegulator), NORMALPRIO, PidRegulator, NULL);
}
bool get_eq(void){
	return equilibre;
}
