#ifndef MASS_COMPUTATION_H_
#define MASS_COMPUTATION_H_

#define M_EPUCK		140 			//masse epuck en g
#define L_BASCULE	600				// longueur de la bascule de mesure en mm
#define CORRECTION  0.9

// @brief algorithm to mesure a given mass on the bascule
void measure_mass(void);

// @brief computes mass and send it per bluetooth
void send_mass(void);

#endif /* MASS_COMPUTATION_H_ */
