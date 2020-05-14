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
static float angle = 0;
static uint16_t distance = 0;
static bool equilibre = FALSE;

// @brief finds out if EPuck i on a stable position
static THD_WORKING_AREA(assessStability_wa, 1024);
static THD_FUNCTION(assessStability, arg)
{
	chRegSetThreadName(__FUNCTION__);
    (void)arg;

    float angle_buffer[STAB_SAMPLES] = {0}; //circular buffer
    uint8_t i = 0;
    systime_t time = 0;
    systime_t start = 0;

    assessStability_p = chThdGetSelfX();
	start = chVTGetSystemTime();

    while (!chThdShouldTerminateX()) {

    	// update buffer with lowpass angle filtering
    	angle_buffer[i % STAB_SAMPLES] = STAB_CUTOFF*angle_buffer[(i-1) % STAB_SAMPLES] + (1-STAB_CUTOFF)*angle;
        time = chVTGetSystemTime();

    	// waiting for 3 seconds before assessing stability
        equilibre = recursive_stability_check(angle_buffer, i);
        if ((time > start + S2ST(3)) & equilibre) {
        	stop_pid_regulator();
        	right_motor_set_speed(STOP);
        	left_motor_set_speed(STOP);
        	equilibre = TRUE;
        	chThdExit((msg_t)"");
        }
        chprintf((BaseSequentialStream *)&SD3, "Angle = %.2f\n", angle);
        chThdSleepMilliseconds(100);
        i++;
    }
}

// @brief has control over EPuck mouvements and has for goal to find stability point
static THD_WORKING_AREA(pidRegulator_wa, 1024);
static THD_FUNCTION(pidRegulator, arg)
{
	chRegSetThreadName(__FUNCTION__);
    (void)arg;

    pidRegulator_p = chThdGetSelfX();

    float speed = 0;
    float rotationSpeed = 0;
//    systime_t time = 0;

    while(!chThdShouldTerminateX()) {

        //computes the angle
		angle = angle_estimation();
		//computes speed to which epuck is going to travel
        speed = pd_speed(angle);
        //computes a correction factor to let the robot rotate to stay in the path
        rotationSpeed = ROTATION_FACTOR*pid_yaw_correction(speed);

        //applies the speed from the PID regulator and the correction for the rotation
		right_motor_set_speed((int16_t)(speed - ROTATION_FACTOR*rotationSpeed));
		left_motor_set_speed((int16_t)(speed + ROTATION_FACTOR*rotationSpeed));

		//sample at 100Hz
//		time = chVTGetSystemTime();
//      chThdSleepUntilWindowed(time, time + S2ST(TS)); // does not work !?
//		alternative (bad) solution, since one loop takes around 1ms to 2ms
		chThdSleepMilliseconds(8);
    }
}

bool recursive_stability_check(float *angle, uint8_t pos)
{
	static uint8_t i = 0;

	if (i > STAB_SAMPLES) {
		i = 0;
		return TRUE;
	} else if (fabs(angle[pos % STAB_SAMPLES]) < STAB_THRESHOLD) {
		i++;
		return recursive_stability_check(angle, pos+1);
	} else {
		i = 0;
		return FALSE;
	}
}

void drive_uphill(void)
{
	float pitchRate = 0;
	float rollRate = 0;
	float yawRate = 0;
	float speed = 0;
	float rotation = 0;
//    systime_t time = 0;

	angle = angle_estimation(); //

	if ( angle > 0 ) {
		speed = INITIAL_SPEED;
	} else {
		speed = -INITIAL_SPEED;
	}

	right_motor_set_speed((int16_t)(speed));
	left_motor_set_speed((int16_t)(speed));
	chThdSleepMilliseconds(200);

	while ((pitchRate < PITCH_RATE_THRESHOLD) | (rollRate > PITCH_RATE_THRESHOLD) | (yawRate > PITCH_RATE_THRESHOLD)) {

		// lowpass filter for pitch roll and yaw
		pitchRate = UPHILL_CUTOFF*pitchRate + (1-UPHILL_CUTOFF)*fabs(get_gyro_rate(X_AXIS));
		rollRate  = UPHILL_CUTOFF*rollRate  + (1-UPHILL_CUTOFF)*fabs(get_gyro_rate(Y_AXIS));
		yawRate   = UPHILL_CUTOFF*yawRate   + (1-UPHILL_CUTOFF)*fabs(get_gyro_rate(Z_AXIS));

		// correcting path
		rotation = pid_yaw_correction(speed);
		right_motor_set_speed((int16_t)(speed - ROTATION_FACTOR*rotation));
		left_motor_set_speed((int16_t)(speed + ROTATION_FACTOR*rotation));

		//sample at 100Hz
//		time = chVTGetSystemTime();
//      chThdSleepUntilWindowed(time, time + S2ST(TS)); // does not work !?
//		alternative (bad) solution, since one loop takes around 2ms
		chThdSleepMilliseconds(7);
	}

	right_motor_set_speed(STOP);
	left_motor_set_speed(STOP);

	return;
}

float pid_yaw_correction(float speed)
{
	static float error[3] = {0};
	static float output[3] = {0};

	// update variables
	error[2] = error[1];
	error[1] = error[0];
	output[2] = output[1];
	output[1] = output[0];

	//error = distance - goal;
	error[0] = (get_prox(5) - get_prox(2));	// sides 45 deg

	// adds front or back sensors input depending on direction
	if (speed > 0) {
		//error[0] += (get_prox(7) - get_prox(0));		// front 10deg
		error[0] += (get_prox(6) - get_prox(1));	// front 20deg
    } else {
    	error[0] += (get_prox(4) - get_prox(3)); 		// back 165deg
    }

	output[0] = - KU1*output[1] - KU2*output[2] + KE0*error[0] + KE1*error[1] + KE2*error[2];

	// anti windup & anti DAC saturation
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

float pd_speed(float angle)
{
	static float error = 0;
	float der_error = 0;
	float speed = 0;

	// proportional error
	error = angle;

	// derivative error with lowpass filter
	der_error = PD_SPEED_CUTOFF*der_error - (1-PD_SPEED_CUTOFF)*get_gyro_rate(X_AXIS);

	speed = KP_SPEED*error + KD_SPEED*der_error;

    return speed;
}

float complementary_lowpass(float input1, float input2)
{
	static float output = 0;
	static bool initialiased = FALSE;

	if(!initialiased) {
		initialiased = TRUE;
		return output = (input1+input2)/2;
	}

	output = (1-LOWPASS_CUTOFF)*(LOWPASS_PONDERATION*input1 + (1-LOWPASS_PONDERATION)*input2) + LOWPASS_CUTOFF*output;

	return output;
}

float angle_estimation(void)
{
    static float angle = 0;
    static float acc_angle = 0;
    static float gyro_angle = 0;
    static int16_t acc_values[NB_AXIS] = {0};

    // estimate angle through acceleration direction
    get_acc_all(acc_values);
    acc_angle = atan2(acc_values[Y_AXIS], acc_values[Z_AXIS])*180/M_PI;
	if (acc_angle > 0) {
		acc_angle -= 180;
	} else {
		acc_angle += 180;
	}

	// estimate angle through gyro integration
	gyro_angle = angle + get_gyro_rate(X_AXIS)*TS;

	// complementary filter to obtain usable angle estimation
    angle = complementary_lowpass(gyro_angle, acc_angle);

    return -angle;
}

void mesure_position(void)
{
	float error = 0;

	error =  get_prox(6) + get_prox(3) - get_prox(4) - get_prox(1);

	while (fabs(error) > 2*IR_THRESHOLD) {
		//lowpass filter
		error = MESURE_POSITION_CUTOFF*error + (1-MESURE_POSITION_CUTOFF)*(get_prox(6) + get_prox(3) - get_prox(4) - get_prox(1));
		right_motor_set_speed(-error);
		left_motor_set_speed(+error);
        chThdSleepMilliseconds(50);
	}

	right_motor_set_speed(STOP);
	left_motor_set_speed(STOP);

	distance = VL53L0X_get_dist_mm();

	return;
}

void start_pid_regulator(void)
{
	if (pidRegulator_p == NULL) {
		chThdCreateStatic(pidRegulator_wa, sizeof(pidRegulator_wa), NORMALPRIO, pidRegulator, NULL);
	}
	return;
}

void stop_pid_regulator(void)
{
	if (pidRegulator_p != NULL) {
		chThdTerminate(pidRegulator_p);
		chThdWait(pidRegulator_p);
		pidRegulator_p = NULL;
	}
	return;
}

void start_assess_stability(void)
{
	if (assessStability_p == NULL) {
		chThdCreateStatic(assessStability_wa, sizeof(assessStability_wa), NORMALPRIO, assessStability, NULL);
	}
	return;
}

void stop_assess_stability(void)
{
	if (pidRegulator_p != NULL) {
		chThdTerminate(assessStability_p);
		chThdWait(assessStability_p);
		assessStability_p = NULL;
	}
	return;
}

bool equilibrium_found(void)
{
	return equilibre;
}

uint16_t get_distance(void)
{
	return distance;
}
