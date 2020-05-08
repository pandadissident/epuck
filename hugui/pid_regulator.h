#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

// constants for the different parts of the project
#define KP_YAW		2
#define KI_YAW		1.5
#define KD_YAW		0.01
#define N 			0.5 		// filter coefficient
#define TS 			0.01 		// PID sampling period

#define KP_SPEED	30
#define KI_SPEED	0
#define KD_SPEED	450

// Thresholds
#define ROTATION_FACTOR 			0.005
#define PITCH_ERROR_THRESHOLD		0.7 //[°] because of the noise of the imu
#define YAW_ERROR_THRESHOLD			4*10.0 //[mW/m²] because of the noise of the ir receiver
#define ACCELERATION_THRESHOLD		10.0
#define PITCH_THRESHOLD				0.4

// compute numerical pid terms
#define A0	(1+N*TS)
#define A1	-(2+N*TS)
#define A2	1
#define B0		(KP_YAW*A0 + KI_YAW*TS*A0 + KD_YAW*N)
#define B1		(KP_YAW*A1 - KI_YAW*TS - 2*KD_YAW*N)
#define B2		(KP_YAW + KD_YAW*N)
#define KU1	(A1/A0)
#define KU2	(A2/A0)
#define KE0	(B0/A0)
#define KE1	(B1/A0)
#define KE2	(B2/A0)


/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void drive_uphill(void);
float pi_yaw_correction(float speed);
float pd_speed(float angle);
float complementary_lowpass(float input1, float input2);
float angle_estimation(void);
void start_pid_regulator(void);
void stop_pid_regulator(void);
void start_assess_stability(void);
void stop_assess_stability(void);
bool get_equilibrium(void);
float get_distance(void);

#endif /* PID_REGULATOR_H */
