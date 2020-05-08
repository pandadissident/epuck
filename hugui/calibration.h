#ifndef CALIBRATION_H
#define CALIBRATION_H

#define SAMPLES 				8
#define STABILITY_THRESHOLD 	SAMPLES*75
#define MAX_ITERATIONS 			65528 //= floor((2^16-1)/SAMPLES)*SAMPLES

/*! @brief
 *
 *  @param
 *  @param values
 *  @warning
 */
void wait_for_stability(void);
void calibrate_imu_prox(void);
void calibrate_tof(void);
float get_pos_zero(void);

#endif
