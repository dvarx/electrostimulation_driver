/*
 * impedance_lookup.c
 *
 *  Created on: 15.03.2021
 *      Author: dvarx
 */

#include "impedance_lookup.h"

//calculates the frequency f where Z(f)=imp
float inverse_impedance(float imp){
    //if impedance requested is too high return -1.0
    if(imp>imps[num_entries-1])
        return -1.0;
    //if impedance requested too low return -2.0
    if(imp<imps[0])
        return -2.0;
    //otherwise perform binary search to find next two closest impedances
    unsigned int upper_ind=num_entries-1;
    unsigned int lower_ind=0;
    while((upper_ind-lower_ind)>1){
        unsigned int mid_ind=(upper_ind+lower_ind)/2;
        if(imp>imps[mid_ind]){
            lower_ind=mid_ind;
        }
        else
            upper_ind=mid_ind;
    }
    //interpolate frequency
    return (imp-imps[lower_ind])/(imps[upper_ind]-imps[lower_ind])*(freqs[upper_ind]-freqs[lower_ind])+freqs[lower_ind];
}

#include "impedance_lookups/350hz_resonator.h"
