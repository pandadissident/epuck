#ifndef MASS_COMPUTATION_H_
#define MASS_COMPUTATION_H_

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define Me	140 			//masse epuck en g
#define L	500				// longueur du bras de mesure en mm

// Robot wide IPC bus
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void measure_mass(void);


#endif /* MASS_COMPUTATION_H_ */
