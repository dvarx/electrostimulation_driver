/*
 * impedance_lookup.c
 *
 *  Created on: 15.03.2021
 *      Author: dvarx
 */

#include <frequency_lookup.h>

//calculates the frequency f where Z(f)=imp
float frequency_lookup(float current_amplitude){
    //if impedance requested is too high return -1.0
    if(current_amplitude>lookup_currents[0])
        return -1.0;
    //if impedance requested too low return -2.0
    if(current_amplitude<lookup_currents[num_entries-1])
        return -2.0;
    //otherwise perform binary search to find next two closest current amplitudes
    unsigned int upper_ind=num_entries-1;
    unsigned int lower_ind=0;
    while((upper_ind-lower_ind)>1){
        unsigned int mid_ind=(upper_ind+lower_ind)/2;
        if(current_amplitude>lookup_currents[mid_ind]){
            upper_ind=mid_ind;
        }
        else
            lower_ind=mid_ind;
    }
    //interpolate frequency
    return (current_amplitude-lookup_currents[lower_ind])/(lookup_currents[upper_ind]-lookup_currents[lower_ind])*(lookup_freqs[upper_ind]-lookup_freqs[lower_ind])+lookup_freqs[lower_ind];
}
