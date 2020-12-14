/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP432 CODE EXAMPLE DISCLAIMER
 *
 * MSP432 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see http://www.ti.com/tool/mspdriverlib for an API functional
 * library & https://dev.ti.com/pinmux/ for a GUI approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP432P401 Demo - Timer_A3, Toggle P8.0/TA1.0, Up Mode, 32kHz ACLK
//
//  Description: Toggle P8.0 using hardware TA1.0 output. Timer1_A is configured
//  for up mode with CCR0 defining period, TA1.0 also output on P8.0. In this
//  example, CCR0 is loaded with 10-1 and TA1.0 will toggle P8.0 at TACLK/10.
//  Thus the output frequency on P8.0 will be the TACLK/20. No CPU or software
//  resources required. Normal operating mode is LPM3.
//  As coded with TACLK = ACLK, P8.0 output frequency = 32768/20 = 1.6384kHz.
//  ACLK = TACLK = 32kHz, MCLK = default DCO ~3MHz
//
//                MSP432P401
//            -------------------
//        /|\|          XIN(PJ.0)|--
//         | |                   |  ~32768Hz
//         --|RST      XOUT(PJ.1)|--
//           |                   |
//           |         P8.0/TA1.0|--> ACLK/20
//
//
//   William Goh
//   Texas Instruments Inc.
//   June 2016 (updated) | June 2014 (created)
//   Built with CCSv6.1, IAR, Keil, GCC
//******************************************************************************
#include "ti/devices/msp432p4xx/inc/msp.h"

const uint16_t DUTY_MAX=255;

void error(void)
{
    volatile uint32_t i;

    while (1)
    {
        P1->OUT ^= BIT0;
        for(i = 20000; i > 0; i--);           // Blink LED forever
    }
}

void init_adc(void){
    // set the s&h time to 16 ADCCLK cycles
    ADC14->CTL0 |= ADC14_CTL0_SHT0_2;
    ADC14->CTL0 |= ADC14_CTL0_SHT1_2;

    // ADCCLK selection (we choose MCKL @ 48MHz)
    ADC14->CTL0 |= ADC14_CTL0_SSEL_3;

    /* source of sample signal SAMPCON, we want the output of the sampling timer for that
     * this signal determines when the sampling phase ends*/
    ADC14->CTL0 |= ADC14_CTL0_SHP;

    /* source for the sample trigger signal, we choose source #4 which corresponds to
     * TA1_C2.
     * */
    ADC14->CTL0 |= ADC14_CTL0_SHS_4;

    // set the ADC mode to repeat-single-channel
    ADC14->CTL0 |= ADC14_CTL0_CONSEQ_2;

    /*  by setting ADC14_CTL0_MSC (multiple sample and conversion) to zero, we explicitly demand another trigger signal
        before initiating the next ADC conversion (e.g. we wait for the next PWM cycle to measure the next current sample
        this bit is by default set to zero so the line is commented out */
    //ADC14->CTL0 &= (~ADC14_CTL0_MSC);

    // set the resolution to 14bits (16 clock cycle conversion time)
    ADC14->CTL1 |= ADC14_CTL1_RES_3;

    /* configure the ADC memory registers and pin multiplexing
     * for all ADC memory registers we choose the ADC A13 on Pin 4.0
     * */
    P4->DIR &= (~BIT0);     // P4.0 input
    P4->SEL0 |= BIT0;       // choose ADC Channel 13 as option for Pin 4.0
    P4->SEL1 |= BIT0;       //
    uint8_t i=0;
    for(i=0; i<32; i++){
        ADC14->MCTL[i] |= ADC14_MCTLN_INCH_13;
    }
    //output the adc trigger on another pin for debugging



    /* enable the ADC, enable conversion
     * Hint: it may be useful to disable the ADC conversion when processing the results (e.g. unset ADC14_CTL0_ENC)
     */
    ADC14->CTL0 |= ADC14_CTL0_ON | ADC14_CTL0_ENC;
}

void init_clk(void){
    uint32_t currentPowerState;

    WDT_A->CTL = WDT_A_CTL_PW |
                 WDT_A_CTL_HOLD;            // Stop WDT

    P1->DIR |= BIT0;                        // P1.0 set as output

    /* NOTE: This example assumes the default power state is AM0_LDO.
     * Refer to  msp432p401x_pcm_0x code examples for more complete PCM
     * operations to exercise various power state transitions between active
     * modes.
     */

    /* Step 1: Transition to VCORE Level 1: AM0_LDO --> AM1_LDO */

    /* Get current power state, if it's not AM0_LDO, error out */
    currentPowerState = PCM->CTL0 & PCM_CTL0_CPM_MASK;
    if (currentPowerState != PCM_CTL0_CPM_0)
        error();

    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    if (PCM->IFG & PCM_IFG_AM_INVALID_TR_IFG)
        error();                            // Error if transition was not successful
    if ((PCM->CTL0 & PCM_CTL0_CPM_MASK) != PCM_CTL0_CPM_1)
        error();                            // Error if device is not in AM1_LDO mode

    /* Step 2: Configure Flash wait-state to 1 for both banks 0 & 1 */
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) |
            FLCTL_BANK0_RDCTL_WAIT_1;
    FLCTL->BANK1_RDCTL  = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) |
            FLCTL_BANK1_RDCTL_WAIT_1;

    /* Step 3: Configure DCO to 48MHz, ensure MCLK uses DCO as source*/
    CS->KEY = CS_KEY_VAL ;                  // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_5;           // Set DCO to 48MHz
    /* Select MCLK = DCO, no divider */
    CS->CTL1 = (CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK)) |
            CS_CTL1_SELM_3;
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    /* Step 4: Output MCLK to port pin to demonstrate 48MHz operation */
    P4->DIR |= BIT3;
    P4->SEL0 |=BIT3;                        // Output MCLK
    P4->SEL1 &= ~(BIT3);
}

volatile uint16_t result;

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
            WDT_A_CTL_HOLD;

    //P8.0 -> TA1.0
    P8->DIR |= BIT0;                        // P8.0 output
    P8->SEL1 |= BIT0;                       // P8.0 option select
    P8->SEL0 &= (~BIT0);                    // P8.0 option select
    //P7.7 -> TA1.1
    P7->DIR |= BIT7;                        // P7.7 output
    P7->SEL0 |= BIT7;                       // P7.7 option select
    P7->SEL1 &= (~BIT7);                    // P7.7 option select

    init_clk();
    init_adc();

    // Configure Timer_A
    TIMER_A1->CTL = TIMER_A_CTL_TASSEL_2 |  // SMCLK
            TIMER_A_CTL_MC_1 |              // Up Mode
            TIMER_A_CTL_CLR |               // Clear TAR
            TIMER_A_CTL_ID_0;               // Clk /1
    TIMER_A1->EX0 = TIMER_A_EX0_TAIDEX_0;   // Clk /1
    //configure CCR TA1.0 (ceiling register)
    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_OUTMOD_4; // CCR4 toggle mode
    TIMER_A1->CCR[0] = 256 - 1;
    //configure CCR TA1.1 (duty register)
    TIMER_A1->CCR[1] = 100;                     //this value corresponds to the PWM value in [0...255]
    TIMER_A1->CCTL[1] = TIMER_A_CCTLN_OUTMOD_3; // CCR4 toggle mode for PWM generation
    /*  configure CCR TA1.2 (adc trigger register)
        TA1.2 will serve as the adc trigger. TA1.2 will have a positive edge (and trigger the adc) when
        the counter value reaches CCR[2]. In order to sample in between PWM switch events, we need to place CCR[2]
        as far away from an edge of CCR[1] as possible.
     * */
    if(TIMER_A1->CCR[1] > (DUTY_MAX/2))
        TIMER_A1->CCR[2] = (TIMER_A1->CCR[1])/2;
    else
        TIMER_A1->CCR[2] = (DUTY_MAX/2)+(TIMER_A1->CCR[1])/2;
    TIMER_A1->CCTL[2] = TIMER_A_CCTLN_OUTMOD_3; // CCR4 toggle mode for ADC triggering


    uint32_t counter=0;
    while(1){
        if((ADC14->IFGR0)&(ADC14_IFGR0_IFG0)){
            counter++;
            //adc result has been written to memory
            result=ADC14->MEM[0];
        }
    }

    __sleep();
    __no_operation();                       // For debugger
}


