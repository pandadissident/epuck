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
void find_equilibrium_stop(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void find_equilibrium_start(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void measure_mass(void);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void compute_mass(float eqPos, float originPos);

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void send_mass(void);


#endif /* MASS_COMPUTATION_H_ */
