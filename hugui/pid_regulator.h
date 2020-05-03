#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

// Robot wide IPC bus
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

//constants for the differents parts of the project

#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			0.1f

#define ERROR_THRESHOLD_ALIGN	10.0f//[mW/m²] because of the noise of the ir receiver
#define ERROR_THRESHOLD_ANGLE	1.0f//[°] because of the noise of the imu
#define KP_ALIGN				1.0f
#define KP_ANGLE				10.0f
#define KI_ALIGN				0.1f
#define KI_ANGLE				0.1f	//must not be zero
#define MAX_SUM_ERROR_ALIGN		(MOTOR_SPEED_LIMIT/3)
#define MAX_SUM_ERROR_ANGLE		(MOTOR_SPEED_LIMIT/3)

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
