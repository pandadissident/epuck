#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define SAMPLES 8
#define THRESHOLD SAMPLES*100
#define MAX_ITERATIONS 65528 //= floor((2^16-1)/SAMPLES)*SAMPLES
#define RED 100, 0, 0
#define GREEN 0, 100, 0
#define BLUE 0, 0, 100


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

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void calibrate_tof(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
int get_originPos(void);

#endif
