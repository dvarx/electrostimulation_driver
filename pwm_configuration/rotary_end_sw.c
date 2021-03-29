/*
 * rotary_end_sw.c
 *
 *  Created on: 29.03.2021
 *      Author: dvarx
 */

#include "rotary_enc_sw.h"

enum encoder_state enc_state;

void run_rotary_enc_fsm(bool dt_sig,bool clk_sig,uint8_t* was_rotated){
    *was_rotated=false;
    switch(enc_state){
    case INITROT:
        if(dt_sig&(!clk_sig))
            enc_state=CLOCKWISE_1;
        else if((!dt_sig)&clk_sig)
            enc_state=CCLOCKWISE_1;
        else
            enc_state=INITROT;
        return;
    // clockwise logic
    case CLOCKWISE_1:
        if(dt_sig&(!clk_sig))
            enc_state=CLOCKWISE_1;
        else if((!dt_sig)&(!clk_sig))
            enc_state=CLOCKWISE_2;
        else
            enc_state=INITROT;
        return;
    case CLOCKWISE_2:
        if((!dt_sig)&clk_sig)
            enc_state=CLOCKWISE_3;
        else if((!dt_sig)&(!clk_sig))
            enc_state=CLOCKWISE_2;
        else
            enc_state=INITROT;
        return;
    case CLOCKWISE_3:
        if(dt_sig&clk_sig){
            enc_state=INITROT;
            *was_rotated=1;
            return;
        }
        else if((!dt_sig)&clk_sig)
            enc_state=CLOCKWISE_3;
        else
            enc_state=INITROT;
        return;
    // ccwlockwise logic
    case CCLOCKWISE_1:
        if((!dt_sig)&(!clk_sig))
            enc_state=CCLOCKWISE_2;
        else if((!dt_sig)&clk_sig)
            enc_state=CCLOCKWISE_1;
        else
            enc_state=INITROT;
        return;
    case CCLOCKWISE_2:
        if(dt_sig&(!clk_sig))
            enc_state=CCLOCKWISE_3;
        else if((!dt_sig)&(!clk_sig))
            enc_state=CCLOCKWISE_2;
        else
            enc_state=INITROT;
        return;
    case CCLOCKWISE_3:
        if(dt_sig&clk_sig){
            enc_state=INITROT;
            *was_rotated=2;
            return;
        }
        else if(dt_sig&(!clk_sig))
            enc_state=CCLOCKWISE_3;
        else
            enc_state=INITROT;
        return;

    }
}

bool prev_sig=true;
bool detect_rising_edge(bool sig){
    bool rising_edge=false;
    if(sig&(!prev_sig))
        rising_edge=true;
    prev_sig=sig;
    return rising_edge;
}
