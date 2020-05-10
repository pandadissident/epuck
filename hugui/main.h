#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"		// no use but avoiding warnings without changing the epuck-main-processor project
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

// Robot wide IPC bus
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

#define MEASUREMENT 0
#define SENSORS_CALIBRATION 4
#define BLUETOOTH_TEST 8
#define ORIGIN_CALIBRATION 12

#define STOP 0
#define OFF 0
#define ON 1
#define TOGGLE 2
#define ALL 4

// @brief fun starting animation
void startingAnimation(void);

// @brief animation to indicate that process suceeded
void readyAnimation(void);

#ifdef __cplusplus
}
#endif

#endif
