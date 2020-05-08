#include <ch.h>
#include <hal.h>
#include <main.h>
#include <math.h>

#include <chprintf.h>

#include "pid_regulator.h"

#include "calibration.h"
#include "motors.h"
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"

// threads
static thread_t *pidRegulator_p = NULL;
static thread_t *assessStability_p = NULL;

// static variables
static bool equilibre = FALSE;
static float angle = 0;
static float distance = 0;

#define sSAMPLES	5
#define sTHRESHOLD	0.001

// @brief
static THD_WORKING_AREA(assessStability_wa, 1024);
static THD_FUNCTION(assessStability, arg)
{
	chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int i = 0;
    float sum = 0;
    float angle_buffer[sSAMPLES] = {0};

    assessStability_p = chThdGetSelfX();

    while (!chThdShouldTerminateX()) {

    	angle_buffer[i % sSAMPLES] = angle;

        sum += 1/sSAMPLES*angle_buffer[i % sSAMPLES];
        sum -= 1/sSAMPLES*angle_buffer[(i+1) % sSAMPLES];

        if ( (sum < sTHRESHOLD) & (i > sSAMPLES)) {
        	stop_pid_regulator();
        	distance = VL53L0X_get_dist_mm();
        	equilibre = TRUE;
        	chThdExit((msg_t)"");
        }

    	i++;
    	chprintf((BaseSequentialStream *)&SD3, "STABILITY TOTAL -- : %.3f\n", sum);
        chThdSleepMilliseconds(100);
    }
}

// @brief
static THD_WORKING_AREA(pidRegulator_wa, 1024);
static THD_FUNCTION(pidRegulator, arg)
{
	chRegSetThreadName(__FUNCTION__);
    (void)arg;

    pidRegulator_p = chThdGetSelfX();

    systime_t time = 0;

    float speed = 0;
    float rotation = 0;

    while(!chThdShouldTerminateX()) {

        //computes the angle
		angle = angle_estimation();
		//computes speed to which epuck is going to travel
        speed = pd_speed(angle);
        //computes a correction factor to let the robot rotate to stay in the path
        rotation = ROTATION_FACTOR*pi_yaw_correction(speed);

        //applies the speed from the PID regulator and the correction for the rotation
		right_motor_set_speed(speed - rotation);
		left_motor_set_speed(speed + rotation);

        //sample at 100Hz
		time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(1000*TS));
    }
}

// @brief
void drive_uphill(void)
{
	float speed, rotation, angle, gyro = 0;
    systime_t time, start = 0;

	angle = angle_estimation();

	if ( angle > 0 ) {
		speed = 250;
	} else {
		speed = -250;
	}

	// stabilise EPuck for 2
	start = chVTGetSystemTime();
	while (time < start + S2ST(2)) {
		rotation = pi_yaw_correction(speed);
		right_motor_set_speed(speed - rotation);
		left_motor_set_speed(speed + rotation);

		//sample at 100Hz
		time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(1000*TS));
	}

	while (gyro < PITCH_THRESHOLD) {
		gyro = fabs(get_gyro_rate(X_AXIS));
		rotation = pi_yaw_correction(speed);
		right_motor_set_speed(speed - rotation);
		left_motor_set_speed(speed + rotation);

		//sample at 100Hz
		time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(1000*TS));
	}

	right_motor_set_speed(STOP);
	left_motor_set_speed(STOP);

	return;
}

// @brief numerical PI regulator implementation
float pi_yaw_correction(float speed)
{
	static float error[3] = {0};
	static float output[3] = {0};

	// update variables
	error[2] = error[1];
	error[1] = error[0];
	output[2] = output[1];
	output[1] = output[0];

	//error = distance - goal;
	error[0] =  (get_prox(4) - get_prox(3)); 	// back
	error[0] += (get_prox(5) - get_prox(2));	// sides
	error[0] += (get_prox(6) - get_prox(1));	// front
	error[0] += (get_prox(7) - get_prox(0));	// front

	output[0] = - KU1*output[1] - KU2*output[2] + KE0*error[0] + KE1*error[1] + KE2*error[2];

	// anti windup
	if(output[0] > MOTOR_SPEED_LIMIT){
		output[0] = MOTOR_SPEED_LIMIT;
	} else if (output[0] < -MOTOR_SPEED_LIMIT) {
		output[0] = -MOTOR_SPEED_LIMIT;
	}

	// corrects error sign given the direction where the epuck is heading
	if (speed > 0) {
		return output[0];
    } else {
    	return -output[0];
    }
}

// @brief simple PI regulator implementation
float pd_speed(float angle)
{
	static float error, der_error, speed = 0;

	// proportional error
	error = angle;

	//disables the PID regulator if the error is too small
	if(fabs(error) < PITCH_ERROR_THRESHOLD){
		error = 0;
	}

	// derivative error with lowpass filter
	der_error = 0.9*der_error - 0.1*get_gyro_rate(X_AXIS);

	//disables the PID regulator if the error is too small
	if(fabs(der_error) < 0.01){
		der_error = 0;
	}

	speed = KP_SPEED*error + KD_SPEED*der_error;

    return speed;
}

// @brief
float complementary_lowpass(float input1, float input2)
{
	float cutoff = 0.8;
	float gyro_pond = 0.85;
	static float output = 0;
	static bool initialiased = FALSE;

	if(!initialiased) {
		output = (input1+input2)/2;
		initialiased = TRUE;
	}

	output = (1-cutoff)*(gyro_pond*input1 + (1-gyro_pond)*input2) + cutoff*output;

	return output;
}

// @brief
float angle_estimation(void)
{
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
	gyro_angle = angle + get_gyro_rate(X_AXIS)*TS;

	// complementary filter
    angle = complementary_lowpass(gyro_angle, acc_angle);

    return -angle;
}

// @brief
void start_pid_regulator(void)
{
	if (pidRegulator_p == NULL) {
		chThdCreateStatic(pidRegulator_wa, sizeof(pidRegulator_wa), NORMALPRIO, pidRegulator, NULL);
	}
	return;
}

// @brief
void stop_pid_regulator(void)
{
	if (pidRegulator_p != NULL) {
		chThdTerminate(pidRegulator_p);
		chThdWait(pidRegulator_p);
		pidRegulator_p = NULL;
	}
	return;
}

// @brief
void start_assess_stability(void)
{
	if (assessStability_p == NULL) {
		chThdCreateStatic(assessStability_wa, sizeof(assessStability_wa), NORMALPRIO, assessStability, NULL);
	}
	return;
}

// @brief
void stop_assess_stability(void)
{
	if (pidRegulator_p != NULL) {
		chThdTerminate(assessStability_p);
		chThdWait(assessStability_p);
		assessStability_p = NULL;
	}
	return;
}

// @brief
bool get_equilibrium(void)
{
	return equilibre;
}

// @brief
float get_distance(void)
{
	return distance;
}
