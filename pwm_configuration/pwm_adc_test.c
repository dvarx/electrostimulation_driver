#include "ti/devices/msp432p4xx/inc/msp.h"
#include <stdbool.h>
#include "current_controller.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <math.h>

/*
 * Peripherals Overview
 * TA1.1    ->  duty                ->  P7.7
 * TA1.2    ->  adc trigger         ->  P7.6
 * TA1.3    ->  查uty               ->  P7.5    ! Make sure duty and 查uty are connected in the right order, otherwise current controller unstable
 * TA0.0    ->  main cl interrupt
 * ADC Input                        ->  P4.0
 * P1.1     ->  turn on switch      ->  P1.1
 * P1.4     ->  mode change request ->  P1.4
 * P2.4     ->  disable signal      ->  P2.4
 * P5.6     ->  heartbeat
 * P5.5     ->  ssr enable          ->  P5.5
 */

inline void set_disable(void){P2->OUT |= BIT4;};
inline void unset_disable(void){P2->OUT &= ~BIT4;};
inline void set_ssrdisable(void){P5->OUT |= BIT5;};
inline void unset_ssrdisable(void){P5->OUT &= ~BIT5;};
inline void toggle_heartbeat(void){P5->OUT ^= BIT6;};
inline void set_duty(uint16_t d){
    //set the duty CCR and the 查uty CCR
    TIMER_A1->CCR[1]=d;
    TIMER_A1->CCR[3]=d;
}

#define DEBUG

const uint16_t DUTY_MAX=255;    //maximum value of duty
uint16_t duty=255/2;               //duty cycle
bool run_main_loop=false;       //flag which triggers execution of one control loop cycle
bool start_control=false;        //this flag is set when the controller should start running
volatile uint16_t result;       //result of adc conversion
bool closed_loop=true;
//adc related parameters/variables
uint32_t adc_sum = 0;
uint16_t no_samples = 0;
const uint16_t va_max=0x3FFF;         //maximum output of the 14bit ADC
const uint16_t va_offset=6961;        //measured offset (needs to be calibrated) (nominal 0x1FFF=8192)
#define VCC 3.3
#define SENSITIVITY 0.286
#define VAOFFSET 8192.0
const float conv_const=VCC/(SENSITIVITY*VAOFFSET);
//control related parameters
const float vdc=30.0;
struct pi_controller_32 current_controller={0.0,0.0,0.0,4.0,8.256550961220959e+02,200e-6};
//uart related parameters
//parameters can be calculated for f(SMCLK)=48MHz at
//http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
const eUSCI_UART_ConfigV1 uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        26,                                     // BRDIV = 78
        0,                                       // UCxBRF = 2
        111,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
        EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
};
//state machine related parameters and variables
enum system_state{CLOSED_LOOP,CL_TO_RES,RES_TO_CL,RESONANT};
enum system_state state=CLOSED_LOOP;
enum system_state nextState=CLOSED_LOOP;
const uint16_t n_shutdown=20+2;                 //number of cycles to wait before energy is assumed to be fed back into DC link
uint16_t transition_counter=0;
bool request_opmode_change=false;                //this bit is set when a change of state from CLOSED_LOOP to RESONANT is requested

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

    /* enable the adc interrupt upon sampling completion
     *
     */
    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);   // Enable ADC interrupt in NVIC module
    ADC14->IER0 |= ADC14_IER0_IE0;              // Enable adc conv complete interrupt for adc memory location 1

    /* enable the ADC, enable conversion
     * Hint: it may be useful to disable the ADC conversion when processing the results (e.g. unset ADC14_CTL0_ENC)
     */
    ADC14->CTL0 |= ADC14_CTL0_ON;
}

// ADC14 interrupt service routine
void ADC14_IRQHandler(void) {
    //add the measured current to the running sum. read access to MEM[0] will also clear the associated interrupt flag
    adc_sum += ADC14->MEM[0];
    no_samples++;
}

void init_clk(void){
    uint32_t currentPowerState;

    WDT_A->CTL = WDT_A_CTL_PW |
                 WDT_A_CTL_HOLD;            // Stop WDT

    /* NOTE: This example assumes the default power state is AM0_LDO.
     * Refer to  msp432p401x_pcm_0x code examples for more complete PCM
     * operations to exercise various power state transitions between active
     * modes.
     */

    /* Step 1: Transition to VCORE Level 1: AM0_LDO --> AM1_LDO */

    /* Get current power state, if it's not AM0_LDO, error out */
    currentPowerState = PCM->CTL0 & PCM_CTL0_CPM_MASK;
    if (currentPowerState != PCM_CTL0_CPM_0)
        while(1){}

    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    if (PCM->IFG & PCM_IFG_AM_INVALID_TR_IFG)
        while(1){}                            // Error if transition was not successful
    if ((PCM->CTL0 & PCM_CTL0_CPM_MASK) != PCM_CTL0_CPM_1)
        while(1){}                            // Error if device is not in AM1_LDO mode

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

void init_gpio(void){
    //disable signal (P2.4)
    P2->DIR |= BIT4;
    P2->OUT |= BIT4;
    //P7.7  ->  TA1.1 (duty)
    P7->DIR |= BIT7;                        // P7.7 output
    P7->SEL0 |= BIT7;                       // P7.7 option select
    P7->SEL1 &= (~BIT7);                    // P7.7 option select
    //P7.6  ->  TA1.2 (adc trigger)
    P7->DIR |= BIT6;                        // P7.6 output
    P7->SEL0 |= BIT6;                       // P7.6 option select
    P7->SEL1 &= (~BIT6);                    // P7.6 option select
    //P7.5 查uty
    P7->SEL0 |= BIT5;
    P7->SEL1 &= (~BIT5);
    P7->DIR |= BIT5;
    //interrupt from buttons


    P1->DIR &= (~BIT1);                     // P1.1 input (pull down switch)
    P1->REN |= BIT1;                        // enable pull-up
    P1->IES = BIT1;                         // interrupt on falling edge
    P1->IE = BIT1;                          // enable interrupt of Port1.1


    P1->DIR &= (~BIT4);                     // P1.4 input (pull down switch)
    P1->REN |= BIT4;                        // enable pull-up
    P1->IES |= BIT4;                        // interrupt on falling edge
    P1->IFG=0;                              // ISSUE: for some reason we need to reset the interrupt register because
                                            // it would otherwise always throw an interrupt on 1.4 when starting up
    P1->IE |= BIT4;                         // enable interrupt of Port1.4

    NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);   //enable Port1 interrupt of ARM Processor
    //P5.6 - heartbeat
    P5->DIR |= BIT6;
    P5->SEL0 &=(~BIT6);
    P5->SEL1 &=(~BIT6);
    //P1.0 - error led
    P1->DIR |= BIT0;
    P1->OUT &= (~BIT0);
    //P5.5 - ssr enable
    P5->DIR |= BIT5;
}

//init the timer responsible for triggering the main control loop
void init_cl_timer(void){
    //TimerA0 uses SMCLK and operates in UPMODE (counts to CCR[0] and then restars from zero)
    //the interrupt enable bit of this counter is set (TIMER_A_CCTLN_CCIE)
    TIMER_A0->CTL |= TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC__UP | TIMER_A_CCTLN_CCIE;

    //enable TA0 interrupt in the ARM Nested Vector Interrupt Controller (NVIC)
    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);

    //configure Capture-Compare Register CCR
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;      //reset the capture-compare interrupt flag
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;         //enable the interrupt request of this cc register
    TIMER_A0->CCR[0] = 0x1FFF;

    // Enable global interrupt
    __enable_irq();
}

//init the pwm and the adc for synchronized sampling
void init_pwm_adc(void){
    // Configure Timer_A
    TIMER_A1->CTL = TIMER_A_CTL_TASSEL_2 |  // SMCLK
            TIMER_A_CTL_MC_1 |              // Up Mode
            TIMER_A_CTL_CLR |               // Clear TAR
            TIMER_A_CTL_ID_1;               // Clk /2
    TIMER_A1->EX0 = TIMER_A_EX0_TAIDEX_0;   // Clk /1
    //configure CCR TA1.0 (ceiling register)
    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_OUTMOD_4; // CCR4 toggle mode
    TIMER_A1->CCR[0] = DUTY_MAX;
    //configure CCR TA1.1 (duty register)
    TIMER_A1->CCR[1] = duty;                     //this value corresponds to the PWM value in [0...255]
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
    //configure CCR TA1.3 as the inverse duty cycle
    TIMER_A1->CCR[3] = duty;                     //this value corresponds to the PWM value in [0...255]
    TIMER_A1->CCTL[3] = TIMER_A_CCTLN_OUTMOD_6; // CCR4 toggle mode for PWM generation
}

//set the pwm frequency to the one used in cl mode

inline void set_pwm_freq_cl(void){
    //formula for PWM: f_pwm=f_0/CCR[0], f_0 at the moment is 12.8MHz
    TIMER_A1->CCR[0]=DUTY_MAX;
    TIMER_A1->CCR[1]=DUTY_MAX/2;
}

//set the pwm frequency to the one used in res mode
inline void set_pwm_freq_res(void){
    //formula for PWM: f_pwm=f_0/CCR[0], f_0 at the moment is 12.8MHz
    TIMER_A1->CCR[0]=DUTY_MAX/2;
    TIMER_A1->CCR[1]=DUTY_MAX/4;
}

void fatal_error(void){
    //set error led
    P1->OUT |= BIT0;
    //set disable pin
    P2->OUT |= BIT4;
    while(1){
        //loop forever
    }
}

void PORT1_IRQHandler(void){
    //enable the ADC conversions
    ADC14->CTL0 |= ADC14_CTL0_ENC;

    //check if Button 1 was pressed
    if(P1->IFG &= BIT1){
        //clear P1 interrupt flag (important: otherwise infinite interrupt loop)
        P1->IFG &= ~BIT1;
        //trigger a main control loop run
        start_control=true;
    }
    else{
        //clear P1 interrupt flag (important: otherwise infinite interrupt loop)
        P1->IFG &= ~BIT4;
        //set this bit to request a change of operational mode
        request_opmode_change=true;
    }
}

void TA0_0_IRQHandler(void){
    // Clear the compare interrupt flag
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    //set the flag to run the main loop
    run_main_loop=true;
}

void init_uart(){
//    /* Halting WDT  */
//    MAP_WDT_A_holdTimer();

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableMaster();
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
void EUSCIA0_IRQHandler(void)
{
    //get the interrupt status of the eUSCI
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    //check if the interrupt was generated by the UART interface, not SPI
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        MAP_UART_transmitData(EUSCI_A0_BASE, MAP_UART_receiveData(EUSCI_A0_BASE));
    }

}

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
            WDT_A_CTL_HOLD;

    init_clk();
    init_adc();
    init_pwm_adc();
    init_cl_timer();
    init_uart();
    init_gpio();

    //main control loop
    uint32_t counter_cl=0;
    float i_ref=0.2;

    #ifdef DEBUG
    uint32_t maxi=0;
    uint16_t max_nosamples=0;
    uint32_t amax=0;
    float imax=0.0;
    #endif

    while(1){
        if(run_main_loop&start_control){
            state=nextState;
            //------------------------------
            //run the Moore state machine, determine nextState
            //------------------------------
            switch(state){
            case CLOSED_LOOP:
                if(request_opmode_change){
                    nextState=CL_TO_RES;
                    request_opmode_change=false;
                }
                else{
                    nextState=CLOSED_LOOP;
                }
                break;
            case CL_TO_RES:
                if(transition_counter<n_shutdown){
                    transition_counter++;
                    nextState=CL_TO_RES;
                }
                else{
                    transition_counter=0;
                    nextState=RESONANT;
                }
                break;
            case RES_TO_CL:
                if(transition_counter<n_shutdown){
                    transition_counter++;
                    nextState=RES_TO_CL;
                }
                else{
                    transition_counter=0;
                    nextState=CLOSED_LOOP;
                }
                break;
            case RESONANT:
                if(request_opmode_change){
                    nextState=RES_TO_CL;
                    request_opmode_change=false;
                }
                else{
                    nextState=RESONANT;
                }
                break;
            }

            //------------------------------
            //set the outputs
            //------------------------------
            if(state==CLOSED_LOOP){
                ////////////
                //calculation of reference current
                ////////////
            //sinusoidal test current
//                float control_loop_dT=1.0/5000.0;
//                float time=counter_cl*control_loop_dT;
//                i_ref=1*sin(2*M_PI*50*time);
//                counter_cl++;
            //constant current test
                i_ref=3.0;
            //stepping current test
//              counter_cl=(counter_cl+1)%5000;
//              if(counter_cl>1000)
//                  i_ref=4.0;
//              else
//                  i_ref=-4.0;

                ////////////
                //compute the measured current
                ////////////
            //if we do not have enough samples, skip control cycle
                if(no_samples<8)
                    continue;
            //calculate the average current signal measured
                uint32_t adc_avg = adc_sum/no_samples;
                int32_t adc_avg_wooff = adc_avg-va_offset;
                float i_meas=-conv_const*(float)adc_avg_wooff;
                #ifdef DEBUG
                if(i_meas>imax){
                    imax=i_meas;
                    amax=adc_sum;
                    maxi=counter_cl;
                    max_nosamples=no_samples;
                }
                #endif
            //reset adc variables
                adc_sum=0;
                no_samples=0;

                ////////////
                //calculate and execute control law
                ////////////
                float err=i_ref-i_meas;
                float u_norm=current_controller.kp/vdc*err;
            //set the duty cycle
                if(u_norm>1.0)
                    duty=0;
                else if(u_norm<-1.0)
                    duty=255;
                else
                    duty=DUTY_MAX/2-(u_norm)*(DUTY_MAX/2);

                ////////////
                //set output signals
                ////////////
                set_duty(duty);
                toggle_heartbeat();     //toggle the heartbeat
                unset_disable();        //set the disable bit to low
                unset_ssrdisable();        //enable ssr to bypass the cap
                run_main_loop=false;
            } //end closed loop
            else if(state==RESONANT){ //open loop resonant actuation
                ////////////
                //set output signals
                ////////////
                toggle_heartbeat();         //toggle the heartbeat
                unset_disable();            //set the disable bit to low
                set_ssrdisable();          //unset ssrenable
                run_main_loop=false;
            } //end open loop resonant actuation
            else if(state==CL_TO_RES){
                ////////////
                //set output signals
                ////////////
                toggle_heartbeat();     //toggle the heartbeat
                set_disable();          //set the disable bit to high, disabling all MOSFETs
                unset_ssrdisable();        //enable ssr to bypass the cap
                set_pwm_freq_res();     //set the PWM frequency to the resonant frequency (50% duty)
                run_main_loop=false;
            }
            else if(state==RES_TO_CL){
                ////////////
                //set output signals
                ////////////
                toggle_heartbeat();         //toggle the heartbeat
                set_disable();              //set the disable bit to high, disabling all MOSFETs
                set_ssrdisable();          //keep ssr open such that oscillation continues
                set_pwm_freq_cl();          //set the PWM frequency to the closed-loop frequency (50% duty)
                run_main_loop=false;
            }
            }
    }
}
