#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

// simple PD for speed control
#define KP_SPEED	5
#define KI_SPEED	0
#define KD_SPEED	550

// numerical PID for lateral positionning control
#define KP_YAW		10
#define KI_YAW		0.01
#define KD_YAW		0.4
#define N 			40			// filter coefficient
#define TS 			0.01 		// PID sampling period

//
#define INITIAL_SPEED				250
#define ROTATION_FACTOR 			0.05
#define STAB_SAMPLES				5
#define STAB_THRESHOLD				50*PITCH_THRESHOLD
#define STAB_CUTOFF					0.7
#define UPHILL_CUTOFF 				0.7
#define PD_SPEED_CUTOFF 			0.7
#define MESURE_POSITION_CUTOFF		0.5
#define LOWPASS_CUTOFF				0.85
#define LOWPASS_PONDERATION			0.9

// THRESHOLDS
#define IR_THRESHOLD 				10
#define PITCH_THRESHOLD				0.01 //[°] because of the noise of the imu
#define PITCH_RATE_THRESHOLD		0.25
#define ACCELERATION_THRESHOLD		10.0

// compute numerical PID terms
#define A0	(1+N*TS)
#define A1	-(2+N*TS)
#define A2	1
#define B0	(KP_YAW*A0 + KI_YAW*TS*A0 + KD_YAW*N)
#define B1	(KP_YAW*A1 - KI_YAW*TS - 2*KD_YAW*N)
#define B2	(KP_YAW + KD_YAW*N)
#define KU1	(A1/A0)
#define KU2	(A2/A0)
#define KE0	(B0/A0)
#define KE1	(B1/A0)
#define KE2	(B2/A0)

// @brief numerical PID regulator
void drive_uphill(void);

// @brief numerical PID regulator
float pid_yaw_correction(float speed);

// @brief simple PD regulator
float pd_speed(float angle);

// @brief merges two mesures and applies lowpass filter
float complementary_lowpass(float input1, float input2);

// @brief takes gyro and acc mesures to estimate angle
float angle_estimation(void);

// @brief returns if the EPuck has found a stable position
bool recursive_stability_check(float *angle, uint8_t pos);

// @brief rotates epuck to face carboard and returns position
void mesure_position(void);
// @brief returne equilibrium state
bool equilibrium_found(void);

// @brief returns mesured distance
uint16_t get_distance(void);

// @brief start and stop pid thread
void start_pid_regulator(void);
void stop_pid_regulator(void);

// @brief start and stop stability thread
void start_assess_stability(void);
void stop_assess_stability(void);


#endif /* PID_REGULATOR_H */
