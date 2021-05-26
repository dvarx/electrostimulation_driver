/*
 * calibration.c
 *
 *  Created on: 26.05.2021
 *      Author: dvarx
 */

#include "calibration.h"
#include "coil_driver.h"

uint16_t meas_freqs[N_MEAS];
uint16_t meas_currents[N_MEAS];
bool calibration_initialized=false;
uint16_t meas_number=0;                 //variable holding the number of the frequency / current that is measured atm
uint16_t calib_wait_counter=0;
bool request_calibration=false;

void init_calibration(){
    uint16_t df=(MAX_MEAS_FREQ-MIN_MEAS_FREQ)/N_MEAS;
    int i=0;
    for(i=0; i<N_MEAS; i++){
        meas_freqs[i]=MIN_MEAS_FREQ+i*df;
    }
}
