#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

// Robot wide IPC bus
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

//constants for the differents parts of the project
#define KP_YAW		2
#define KI_YAW		0.5
#define KD_YAW		0

#define KP_SPEED	50
#define KI_SPEED	0
#define KD_SPEED	5

#define N 			20 		//filter coefficient
#define TS 			0.01 	// PID sampling period

#define SPEED_THRESHOLD 			(MOTOR_SPEED_LIMIT*4/5)
#define ROTATION_THRESHOLD 			(MOTOR_SPEED_LIMIT/5)
#define ANGLE_ERROR_THRESHOLD		1.0f //[°] because of the noise of the imu
#define YAW_ERROR_THRESHOLD			5.0f //[mW/m²] because of the noise of the ir receiver
#define ACCELERATION_THRESHOLD		9.5f

#define a0	(1+N*TS)
#define a1	-(2+N*TS)
#define a2	1

// PI constants for rotation correction
#define rb0		(KP_YAW*a0 + KI_YAW*TS*a0 + KD_YAW*N)
#define rb1		(KP_YAW*a1 - KI_YAW*TS - 2*KD_YAW*N)
#define rb2		(KP_YAW + KD_YAW*N)
#define rku1	(a1/a0)
#define rku2	(a2/a0)
#define rke0	(rb0/a0)
#define rke1	(rb1/a0)
#define rke2	(rb2/a0)

// PID constants for speed computation
#define sb0		(KP_SPEED*a0 + KI_SPEED*TS*a0 + KD_SPEED*N)
#define sb1		(KP_SPEED*a1 - KI_SPEED*TS - 2*KD_SPEED*N)
#define sb2		(KP_SPEED + KD_SPEED*N)
#define sku1	(a1/a0)
#define sku2	(a2/a0)
#define ske0	(sb0/a0)
#define ske1	(sb1/a0)
#define ske2	(sb2/a0)


/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void start_social_distancing(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void stop_social_distancing(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void straight_line(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void pid_regulator_start(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void pid_regulator_stop(void);


/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void drive_uphill_start(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void drive_uphill_stop(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
bool get_eq(void);

#endif /* PID_REGULATOR_H */
