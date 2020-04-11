#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

// Robot wide IPC bus
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void calibrate_imu(void);

#define THRESHOLD 100
#define RED 100, 0, 0

#endif
