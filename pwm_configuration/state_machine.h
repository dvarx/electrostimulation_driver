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
#include "coil_driver.h"

enum system_state{INIT=0,OPERATIONAL=1,DEBUGSTATE=2,ERROR=3};

extern char input_buffer[];
const uint32_t res_freq_mhz=10000000;
const uint32_t cl_pwm_freq_mhz=10000000;
extern uint16_t duty;
extern uint32_t i_ref_ampl_ma;
extern bool request_opmode_change;
extern bool start_control;
extern enum system_state state;
extern bool request_debug_state;
extern bool request_stop;
extern uint32_t Ton;
extern uint32_t N_ONOFF_CYCLES;

inline void set_pwm_freq(unsigned int freq);
extern void set_duty(uint16_t);

const char SET_FREQ_CMD[]="!frq";
const char SET_DUTY_CMD[]="!dut";
const char SWITCH_STATE[]="!sws";
const char SET_CURRENT[]="!cur";
const char DEBUG[]="!dbg";
const char STOP[]="!stp";
const char GET_STATE[]="!gst";
const char GET_CURRENT[]="!gcr";
const char SET_ONOFF[]="!onoff";
const char SET_CYC[]="!cyc";

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
        i_ref_ampl_ma=atoi(input_buffer+sizeof(SET_CURRENT)/sizeof(char)-1);
    }
    //on/off time command
    else if(!strncmp(input_buffer,SET_ONOFF,sizeof(SET_ONOFF)/sizeof(char)-1)){
        Ton=atoi(input_buffer+sizeof(SET_ONOFF)/sizeof(char)-1);
    }
    //cyc time command
    else if(!strncmp(input_buffer,SET_CYC,sizeof(SET_CYC)/sizeof(char)-1)){
        N_ONOFF_CYCLES=atoi(input_buffer+sizeof(SET_CYC)/sizeof(char)-1);
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
    //!gst command
    else if(!strncmp(input_buffer,GET_STATE,sizeof(GET_STATE)/sizeof(char)-1)){
        MAP_UART_transmitData(EUSCI_A0_BASE, state);
    }
    //!gcr command
    else if(!strncmp(input_buffer,GET_CURRENT,sizeof(GET_CURRENT)/sizeof(char)-1)){
        uart_write_string((char*)(&i_ref_ampl_ma),sizeof(i_ref_ampl_ma));
    }
}


#endif /* STATE_MACHINE_H_ */
