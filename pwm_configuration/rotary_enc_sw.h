/*
 * rotary_enc_sw.h
 *
 *  Created on: 29.03.2021
 *      Author: dvarx
 */

#ifndef ROTARY_ENC_SW_H_
#define ROTARY_ENC_SW_H_

#include <stdint.h>
#include <stdbool.h>

enum encoder_state {INITROT,CLOCKWISE_1,CLOCKWISE_2,CLOCKWISE_3,CCLOCKWISE_1,CCLOCKWISE_2,CCLOCKWISE_3};

/*
 * dt_sig, clk_sig : encoder signals
 * was_rotated* : set to zero if not rotated, 1 if rotated clockwise , 2 if rotated counter-clockwise
 * was_pressed* : set if button was pressed
 */
void run_rotary_enc_fsm(bool dt_sig,bool clk_sig,uint8_t* was_rotated);

#endif /* ROTARY_ENC_SW_H_ */
