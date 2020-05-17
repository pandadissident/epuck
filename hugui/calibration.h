#ifndef CALIBRATION_H
#define CALIBRATION_H

#define SAMPLES 				8
#define STABILITY_THRESHOLD 	SAMPLES*75
#define MAX_ITERATIONS 			65528 //= floor((2^16-1)/SAMPLES)*SAMPLES

// @brief waits for epuck to be on a stable support
void wait_for_stability(void);

// @brief calibrates imu and ir sensors aka proximity sensors
void calibrate_imu_prox(void);

// @brief calibrates the bascule and find origin
void calibrate_tof(void);

// @brief returns averaged value of distance mesured by the TOF sensor
uint16_t tof_distance(void);

// @brief returns calibrated origin
uint16_t get_pos_zero(void);

#endif
