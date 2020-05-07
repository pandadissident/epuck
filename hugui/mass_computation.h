#ifndef MASS_COMPUTATION_H_
#define MASS_COMPUTATION_H_

#define M_EPUCK		140 			//masse epuck en g
#define L_BASCULE	500				// longueur de la bascule de mesure en mm

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */

void measure_mass(void);
void send_mass(void);

#endif /* MASS_COMPUTATION_H_ */
