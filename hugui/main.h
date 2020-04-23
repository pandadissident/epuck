#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

// Robot wide IPC bus
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

#define MEASUREMENT 0
#define CALIB_PHASE_1 1
#define CALIB_PHASE_2 2
#define MOTOR_TEST 3

#define STOP 0
#define OFF 0
#define ON 1
#define TOGGLE 2
#define ALL 4

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
int main(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void startingAnimation(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void readyAnimation(void);

#ifdef __cplusplus
}
#endif

#endif
