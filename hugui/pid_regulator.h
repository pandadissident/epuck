#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

// Robot wide IPC bus
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

//constants for the differents parts of the project
//#define IMAGE_BUFFER_SIZE		640
//#define WIDTH_SLOPE				5
//#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
//#define PXTOCM					1570.0f //experimental value
//#define GOAL_DISTANCE 			10.0f
//#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD_ALIGN	0.1f
#define ERROR_THRESHOLD_ANGLE	0.1f//[cm] because of the noise of the camera
#define KP_ALIGN				800.0f
#define KP_ANGLE				800.0f
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
//void straight_line(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void pid_regulator_start(void);

#endif /* PID_REGULATOR_H */
