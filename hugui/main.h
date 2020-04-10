#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define MEASUREMENT 0
#define IMU_CALIBRATION 1
#define TOF_CALIBRATION 2

#define OFF 0
#define ON 1
#define TOGGLE 2
#define ALL 4

// Robot wide IPC bus
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

int main(void);

#ifdef __cplusplus
}
#endif

#endif
