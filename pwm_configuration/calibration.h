/*
 * calibration.h
 *
 *  Created on: 26.05.2021
 *      Author: dvarx
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_MEAS_FREQ 3500
#define MIN_MEAS_FREQ 2500
#define N_MEAS 100
#define CALIB_WAIT_CYCLES 20

extern uint16_t meas_freqs[N_MEAS];
extern uint16_t meas_currents[N_MEAS];
extern bool calibration_initialized;
extern uint16_t meas_number;
extern uint16_t calib_wait_counter;

void init_calibration(void);


#endif /* CALIBRATION_H_ */
