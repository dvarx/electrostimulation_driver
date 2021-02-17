/*
 * state_machine.h
 *
 *  Created on: Feb 8, 2021
 *      Author: dvarx
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <string.h>
#include <stdlib.h>

enum system_state{INIT,OPERATIONAL,DEBUGSTATE};

extern char input_buffer[];
const uint32_t res_freq=100000;
const uint32_t cl_pwm_freq=100000;
extern uint16_t duty;
extern float i_ref;
extern bool request_opmode_change;
extern bool start_control;
extern enum system_state state;
extern bool request_debug_state;
extern bool request_stop;

inline void set_pwm_freq(int freq);
extern void set_duty(uint16_t);

const char SET_FREQ_CMD[]="!frq";
const char SET_DUTY_CMD[]="!dut";
const char SWITCH_STATE[]="!sws";
const char SET_CURRENT[]="!cur";
const char DEBUG[]="!dbg";
const char STOP[]="!stp";

void parse_input(uint8_t buffer_size){
    //!frq command
    if(!strncmp(input_buffer,SET_FREQ_CMD,sizeof(SET_FREQ_CMD)/sizeof(char)-1)){
        //changing the frequency manually only works in debug state or resonant state
        if(state==DEBUGSTATE){
            uint32_t des_res_freq=(uint32_t)atoi(input_buffer+sizeof(SET_FREQ_CMD)/sizeof(char)-1);
            set_pwm_freq(des_res_freq);
        }
        return;
    }
    //!dut command
    else if(!strncmp(input_buffer,SET_DUTY_CMD,sizeof(SET_DUTY_CMD)/sizeof(char)-1)){
        //if not in debug state, we cannot set the frequency manually
        if(state!=DEBUGSTATE)
            return;
        duty=(uint32_t)atoi(input_buffer+sizeof(SET_FREQ_CMD)/sizeof(char)-1);
        set_duty(duty);
    }
    //!sws command
    else if(!strncmp(input_buffer,SWITCH_STATE,sizeof(SWITCH_STATE)/sizeof(char)-1)){
        request_opmode_change=true;
    }
    //!cur command
    else if(!strncmp(input_buffer,SET_CURRENT,sizeof(SET_CURRENT)/sizeof(char)-1)){
        i_ref=atoi(input_buffer+sizeof(SET_CURRENT)/sizeof(char)-1)*0.001;;
    }
    //!dbg command
    else if(!strncmp(input_buffer,DEBUG,sizeof(DEBUG)/sizeof(char)-1)){
        if(state==INIT)
            request_debug_state=true;
        if(state==DEBUGSTATE)
            request_debug_state=false;
    }
    //!stp command
    else if(!strncmp(input_buffer,STOP,sizeof(STOP)/sizeof(char)-1)){
        request_stop=true;
    }
}


#endif /* STATE_MACHINE_H_ */
