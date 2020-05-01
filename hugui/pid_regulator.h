#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

// Robot wide IPC bus
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

//constants for the differents parts of the project

#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2

#define ERROR_THRESHOLD_ALIGN	0.25f//[mW/m²] because of the noise of the ir receiver
#define ERROR_THRESHOLD_ANGLE	0.01f//[°] because of the noise of the imu
#define KP_ALIGN				10.0f
#define KP_ANGLE				10.0f
#define KI_ALIGN				3.5f
#define KI_ANGLE				3.5f	//must not be zero
#define MAX_SUM_ERROR_ALIGN		(MOTOR_SPEED_LIMIT/KI_ALIGN)
#define MAX_SUM_ERROR_ANGLE		(MOTOR_SPEED_LIMIT/KI_ANGLE)

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
bool get_eq(void);

#endif /* PID_REGULATOR_H */
