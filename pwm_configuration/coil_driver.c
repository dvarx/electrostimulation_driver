#include <frequency_lookup.h>
#include "ti/devices/msp432p4xx/inc/msp.h"
#include <stdbool.h>
#include "current_controller.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "state_machine.h"
#include <math.h>
#include "libs/hd44780.h"
#include "stdio.h"
#include "rotary_enc_sw.h"
#include "coil_driver.h"
#include "calibration.h"
#include "frequency_lookup.h"

/*
 * Peripherals Overview
 * TA1.1    ->  duty                ->  P7.7
 * TA1.2    ->  adc trigger         ->  P7.6
 * TA1.3    ->  �duty               ->  P7.5    ! Make sure duty and �duty are connected in the right order, otherwise current controller unstable
 * TA0.0    ->  main cl interrupt
 * ADC Input                        ->  P5.0
 * P1.1     ->  turn on switch      ->  P1.1
 * P1.4     ->  mode change request ->  P1.4
 * P2.4     ->  disable signal      ->  P2.4
 * P5.6     ->  TA2_CCR1
 * P4.0-P4.7    ->  LCD Output
 * P6.5         ->  LCD ENABLE
 * P6.0         ->  LCD RS
 * P3.5         ->  SW
 * P3.7         ->  DT
 * P5.2         ->  CLK
 */

inline void set_disable(void){P2->OUT |= BIT4;};
inline void unset_disable(void){P2->OUT &= ~BIT4;};
inline void toggle_debugging(void){P5->OUT ^= BIT7;};

#define DEBUG

uint16_t duty=512;               //duty cycle
bool run_main_loop=false;       //flag which triggers execution of one control loop cycle
volatile uint16_t result;       //result of adc conversion
bool closed_loop=true;
//adc related parameters/variables
int32_t avg_abs_current_acc_nmeas=0;
const int Nmeas=1000;           //number of measurements in a single current average measurement
const int Mmeas=20;           //number of averaged measurements
unsigned int nmeas_counter=0;           //number of single current samples in the current average current measurement
int32_t avgd_abs_current_meass[Mmeas];         //array of current average measurements
int32_t last_current_meass[Nmeas];              //last adc current measurements for debugging
int32_t last_current_meass_offset=0;            //offset into last_current_meass_offset
int32_t main_acc_avg_abs_current=0;                //accumulator for absolute average current measurements
int32_t main_avg_abs_current_est=0;
uint16_t avgd_current_meass_offset=0;     //offset into the current_avg_meas array
float imeas_hat=0.0;                    //measured current amplitude
const int32_t va_offset=7415;           //measured offset (needs to be calibrated) (nominal 0x1FFF=8192)
//ireal_ma = alpha*main_avg_abs_current_est+beta
//these values were linearly fitted
//const float alpha=11.761;
//const float beta=134.28;
//---- 1150hz resonator
const float alpha=7.766;
const float beta=250.78;
//control related parameters
#define V_DC 30.0
#define CONTROLLER_DT 200E-6
uint32_t i_ref_ampl_ma=1000;                   //reference amplitude value [mA]
const uint32_t i_ref_amp_max=7500;         //maximum current [mA]
const float v_in_hat=V_DC*4.0/M_PI;
struct pi_controller_32 current_controller={0.0,0.0,0.0,8.0,0.0,CONTROLLER_DT};
float res_kp=5.0;
float res_ki=1.0;
float err_i_hat_integral=0.0;   //integral of pi current controller
float des_freq_controller=10000.0;
float des_imp=10000.0;
uint32_t des_freq_mhz=3400000;     //desired frequency for constant frequency mode
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
char input_buffer[128];
char send_buffer[128];
uint8_t input_pointer=0;
uint8_t send_pointer=0;
//state machine related parameters and variables
enum system_state state=INIT;
enum system_state nextState=INIT;
enum operational_mode opmode=CONSTANT_CURRENT;
enum init_screen in_screen=START;
enum resonator resonator_num=RES2500HZ;
const uint16_t n_shutdown=20+2;                 //number of cycles to wait before energy is assumed to be fed back into DC link
uint16_t transition_counter=0;
bool request_opmode_change=false;                //this bit is set when a change of state from CLOSED_LOOP to RESONANT is requested
bool command_to_be_processed=false;
bool request_debug_state=false;
bool request_stop=false;
//lcd related parameters
char buffer_0[20]="";
char buffer_1[20]="";
char buffer_2[20]="";
char buffer_3[20]="";
uint8_t selection_wheel_start=0;                //selection wheel for start screen in INIT state
//rotary encoder / switch related parameters
int32_t rotary_counter=0;
uint8_t was_rotated;            //rotary encoder variable (0 if not rotated, 1 if rotated cw, 2 if rotated ccw)
uint8_t was_pressed;            //press button variable (1 if was pressed, 0 if not)
uint8_t cursor_position=1;        //position of the selection cursor [5.28 , first digit 0 second digit 1 third digit 2]
//debug related variables
extern uint32_t debug_frequency_mhz;

void init_adc(void){
    // set the s&h time to 16 ADCCLK cycles
    ADC14->CTL0 |= ADC14_CTL0_SHT0_2;
    ADC14->CTL0 |= ADC14_CTL0_SHT1_2;

    // ADCCLK selection (we choose MCKL @ 48MHz)
    ADC14->CTL0 |= ADC14_CTL0_SSEL_3;

    /* source of sample signal SAMPCON, we want the output of the sampling timer for that
     * this signal determines when the sampling phase ends*/
    ADC14->CTL0 |= ADC14_CTL0_SHP;

    /* source for the sample trigger signal, we choose source #5 which corresponds to
     * TA2_C1.
     * */
    ADC14->CTL0 |= ADC14_CTL0_SHS_5;

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
    P5->DIR &= (~BIT0);     // P5.0 input
    P5->SEL0 |= BIT0;       // choose ADC Channel 5 as option for Pin 5.0
    P5->SEL1 |= BIT0;       //
    uint8_t i=0;
    for(i=0; i<32; i++){
        ADC14->MCTL[i] |= ADC14_MCTLN_INCH_5;
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


inline void enable_adc(){
    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

inline void disable_adc(){
    ADC14->CTL0 &= (~ADC14_CTL0_ENC);
}

// ADC14 interrupt service routine
void ADC14_IRQHandler(void) {
    //-- log some adc acquisitions for debugging
    last_current_meass[last_current_meass_offset]=ADC14->MEM[0];
    last_current_meass_offset=(last_current_meass_offset+1)%Nmeas;
    //--
    avg_abs_current_acc_nmeas+=abs(ADC14->MEM[0]-va_offset);
    nmeas_counter++;
    if(nmeas_counter>=Nmeas){
        int32_t new_avgd_abs_current_meas=abs(avg_abs_current_acc_nmeas/Nmeas);
        //update the main current estimate for average absolute current (add the newest measurement, subtract the oldest measurement)
        main_acc_avg_abs_current+=new_avgd_abs_current_meas-avgd_abs_current_meass[avgd_current_meass_offset];
        main_avg_abs_current_est=main_acc_avg_abs_current/Mmeas;
        //save the newest avgd_current_meas and discard the oldest avgd_current_meas
        avgd_abs_current_meass[avgd_current_meass_offset]=new_avgd_abs_current_meas;
        avg_abs_current_acc_nmeas=0;
        //update the offset pointer
        avgd_current_meass_offset=(avgd_current_meass_offset+1)%Mmeas;
        //reset the counter
        nmeas_counter=0;
    }
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
//    P4->DIR |= BIT3;
//    P4->SEL0 |=BIT3;                        // Output MCLK
//    P4->SEL1 &= ~(BIT3);
}

void init_gpio(void){
    //disable signal (P2.4)
    P2->DIR |= BIT4;
    P2->OUT |= BIT4;
    //P7.7  ->  TA1.1 (duty)
    P7->DIR |= BIT7;                        // P7.7 output
    P7->SEL0 |= BIT7;                       // P7.7 option select
    P7->SEL1 &= (~BIT7);                    // P7.7 option select
    //P7.5 �duty
    P7->SEL0 |= BIT5;
    P7->SEL1 &= (~BIT5);
    P7->DIR |= BIT5;
    //interrupt from buttons

    //P5.7 as debugging output
    P5->DIR |= BIT7;

    //P2.7 as mode change request button
    P2->DIR &= (~BIT7);                     //P2.7 as input
    P2->OUT |= BIT7;
    P2->REN |= BIT7;                        //enable pull up
    P2->IES |= BIT7;                        //interrupt on falling edge
    P2->IFG = 0;
    P2->IE |= BIT7;                         //enable P2.7 interrupt
    NVIC->ISER[1] = 1 << ((PORT2_IRQn) & 31);   //enable Port1 interrupt of ARM Processor


    P1->DIR &= (~BIT1);                     // P1.1 input (pull down switch)
    P1->REN |= BIT1;                        // enable pull-up
    P1->IES |= BIT1;                         // interrupt on falling edge
    P1->IE |= BIT1;                          // enable interrupt of Port1.1


    P1->DIR &= (~BIT4);                     // P1.4 input (pull down switch)
    P1->REN |= BIT4;                        // enable pull-up
    P1->IES |= BIT4;                        // interrupt on falling edge
    P1->IFG=0;                              // ISSUE: for some reason we need to reset the interrupt register because
                                            // it would otherwise always throw an interrupt on 1.4 when starting up
    P1->IE |= BIT4;                         // enable interrupt of Port1.4

    NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);   //enable Port1 interrupt of ARM Processor
    //P5.6 - ADC trigger
    P5->DIR |= BIT6;        //output
    P5->SEL0 |= BIT6;
    P5->SEL1 &=(~BIT6);
    //P1.0 - error led
    P1->DIR |= BIT0;
    P1->OUT &= (~BIT0);

    //setup lcd display
    P6->DIR |= BIT0;        //P6.0  -   RS
    P6->DIR |= BIT5;        //P6.5  -   ENABLE
    P4->DIR = 0xFF;        //P4.0 to P4.7  -   B0 to B7$

    //rotary encoder & switch
    P3->DIR &= (~0x20);     //P3.5 input sw
    P3->DIR &= (~0x80);     //P3.7 input dt
    P5->DIR &= (~0x02);     //P5.2 input clk
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

//dead time in units of Timer_A clock cycles. Timer_A clock is 12MHz therefore one deadtime unit corresponds to 83nsns.
const uint16_t deadtime=1;
//init the pwm and the adc for synchronized sampling
void init_pwm_adc(void){
    // Configure Timer_A1 (this timer is used for duty generation)
    TIMER_A1->CTL = TIMER_A_CTL_TASSEL_2 |  // SMCLK
            TIMER_A_CTL_MC_3 |              // Up-Down Mode
            TIMER_A_CTL_CLR |               // Clear TAR
            TIMER_A_CTL_ID_2;               // Clk /8
    TIMER_A1->EX0 = TIMER_A_EX0_TAIDEX_0;   // Clk /1
    //configure CCR TA1.0 (ceiling register)
    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_OUTMOD_6; // toggle/set mode
    TIMER_A1->CCR[0] = 1024;
    //configure CCR TA1.1 (duty register)
    TIMER_A1->CCR[1] = duty+deadtime;                    //this value corresponds to the PWM value in [0...255]
    TIMER_A1->CCTL[1] = TIMER_A_CCTLN_OUTMOD_6;          // CCR4 toggle mode for PWM generation
    //configure CCR TA1.3 as the inverse duty cycle
    TIMER_A1->CCR[3] = duty-deadtime;                   //this value corresponds to the PWM value in [0...255]
    TIMER_A1->CCTL[3] = TIMER_A_CCTLN_OUTMOD_2;         // CCR4 toggle mode for PWM generation


    //legacy implementation
//    /*  configure CCR TA1.2 (adc trigger register)
//        TA1.2 will serve as the adc trigger. TA1.2 will have a positive edge (and trigger the adc) when
//        the counter value reaches CCR[2]. In order to sample in between PWM switch events, we need to place CCR[2]
//        as far away from an edge of CCR[1] as possible.
//     * */
//    if(TIMER_A1->CCR[1] > (1024/2))
//        TIMER_A1->CCR[2] = (TIMER_A1->CCR[1])/2;
//    else
//        TIMER_A1->CCR[2] = (1024/2)+(TIMER_A1->CCR[1])/2;
//    TIMER_A1->CCTL[2] = TIMER_A_CCTLN_OUTMOD_3; // CCR4 toggle mode for ADC triggering

    //new implementation for electrostimulation setup
    //configure Timer_A2 to generate the ADC trigger samples for sampling the current
    TIMER_A2->CTL = TIMER_A_CTL_TASSEL_2 |  // SMCLK
            TIMER_A_CTL_MC_1 |              // Up Mode
            TIMER_A_CTL_CLR |               // Clear TAR
            TIMER_A_CTL_ID_1;               // Clk /8
    TIMER_A2->EX0 = TIMER_A_EX0_TAIDEX_0;   // Clk /1
    //configure ceiling register
    TIMER_A2->CCTL[0] = TIMER_A_CCTLN_OUTMOD_4; // CCR4 toggle mode
    TIMER_A2->CCR[0]=256;
    //configure TA2_CCR2, the output of the CCR will trigger the ADC
    TIMER_A2->CCR[1] = 128;
    TIMER_A2->CCTL[1] = TIMER_A_CCTLN_OUTMOD_3; // CCR4 toggle mode for generating ADC trigger

    enable_adc();
}


//set the duty cycle (10bit)
inline void set_duty(uint16_t duty){
    uint16_t counter_duty=(TIMER_A1->CCR[0]*duty)/1024;
    TIMER_A1->CCR[1]=counter_duty;        //counter toggles at CCR[1]
    //set inverted duty cycle as well
    TIMER_A1->CCR[3] =counter_duty;
    /*  configure CCR TA1.2 (adc trigger register)
        TA1.2 will serve as the adc trigger. TA1.2 will have a positive edge (and trigger the adc) when
        the counter value reaches CCR[2]. In order to sample in between PWM switch events, we need to place CCR[2]
        as far away from an edge of CCR[1] as possible.
     * */
    if(TIMER_A1->CCR[1] > (TIMER_A1->CCR[1]/2))
        TIMER_A1->CCR[2] = (TIMER_A1->CCR[1])/2;
    else
        TIMER_A1->CCR[2] = (TIMER_A1->CCR[1])+(TIMER_A1->CCR[1])/2;
}

//set the pwm frequency to the one used in res mode
//frequency in mHz
inline void set_pwm_freq(unsigned int freq){
    //formula for PWM: f_pwm=f_timera/CCR[0]*2, f_timera at the moment is 12MHz (clk of timer a)
    uint16_t counter_limit=12000000000/freq/2;
    //we set CCR[0]:=800, therefore the output frequency is 30kHz
    TIMER_A1->CCR[0]=counter_limit;        //counter counts to CCR[0]
    //set duty cycle to 50%
    TIMER_A1->CCR[1]=counter_limit/2+deadtime;        //counter toggles at CCR[1]
    //set inverted duty cycle to 50% as well
    TIMER_A1->CCR[3] =counter_limit/2-deadtime;
    //set CCR[2] which is needed to trigger the ADC
    TIMER_A1->CCR[2] =counter_limit/4;
}

void fatal_error(void){
    //set error led
    P1->OUT |= BIT0;
    //set disable pin
    P2->OUT |= BIT4;
    set_disable();
    nextState=ERROR;
}

void PORT1_IRQHandler(void){
    //enable the ADC conversions
    ADC14->CTL0 |= ADC14_CTL0_ENC;

    //check if Button 1 was pressed
    if(P1->IFG &= BIT1){
        //clear P1 interrupt flag (important: otherwise infinite interrupt loop)
        P1->IFG &= ~BIT1;
        //trigger a main control loop run
        //start_control=true;
    }
    else{
        //clear P1 interrupt flag (important: otherwise infinite interrupt loop)
        P1->IFG &= ~BIT4;
        //set this bit to request a change of operational mode
        //request_opmode_change=true;
    }
}

//timer 32 used for button debounce
void init_timer32(void){
    TIMER32_1->CONTROL |= TIMER32_CONTROL_ONESHOT;          // set the one-shot bit
    TIMER32_1->CONTROL |= TIMER32_CONTROL_PRESCALE_2;       // set the prescaler to 256
    TIMER32_1->CONTROL |= TIMER32_CONTROL_IE;               // set the interrupt enable bit
    NVIC->ISER[0] = 1 << ((T32_INT1_IRQn) & 31);   //enable T32_INT1 interrupt of ARM Processor

}

//timer 32 interrupt service routine, reenables interrupt for P2.7 for debounce purpose
void T32_INT1_IRQHandler(void){
    if(!((P2->IN)&BIT7)){
        if(state==INIT)
            request_opmode_change=true;
        else if(state==OPERATIONAL||state==ERROR)
            request_stop=true;
    }
    TIMER32_1->INTCLR|=BIT0;        //clear interrupt for timer32
}

void PORT2_IRQHandler(void){
    P2->IFG &= (~BIT7);                 //clear the interrupt flag

    //set up a timer to check if the output is low 10ms from now
    TIMER32_1->LOAD=20000;
    TIMER32_1->CONTROL|=TIMER32_CONTROL_ENABLE;       //set the enable bit
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

/* EUSCI A0 UART ISR */
void EUSCIA0_IRQHandler(void)
{
    //get the interrupt status of the eUSCI
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    //check if the interrupt was generated by the UART interface, not SPI
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        input_buffer[input_pointer]=MAP_UART_receiveData(EUSCI_A0_BASE);
        //check if \n or \r received, indicating an end of command
        if(input_buffer[input_pointer]=='\n'||input_buffer[input_pointer]=='\r'){
            command_to_be_processed=true;
            //check if there is a second \n or \r character
            if(input_buffer[input_pointer-1]=='\r'||input_buffer[input_pointer-1]=='\n')
                input_buffer[input_pointer-1]='\0';
            else
                input_buffer[input_pointer]='\0';
        }
        else
            input_pointer++;
    }
    else if(status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG){
        //check if end of string to be transmitted reached
        if(send_pointer==0){
            UART_clearInterruptFlag(EUSCI_A0_BASE,0x0002);
            send_pointer=0;
            return;
        }
        else{
            MAP_UART_transmitData(EUSCI_A0_BASE, send_buffer[send_pointer]);
            send_pointer--;
        }
    }

}

void uart_write_string(char* string_ptr,uint8_t num){
    //copy string to buffer
    memcpy(send_buffer,string_ptr,num);
    send_pointer=num;
    //enable transmit interrupt (interrupt will be set when character shifted out or UART buffer empty)
    //after the interrupt is enabled, the interrupt is immediately triggered due to an empty UART send register
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
    //send the first character
    send_pointer--;
    MAP_UART_transmitData(EUSCI_A0_BASE, send_buffer[0]);
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
    init_timer32();

    uint32_t counter_disp=0;
    const uint32_t counter_disp_max=20;
    const uint32_t counter_disp_max2=4096;
    uint32_t counter_res_cl=0;
    bool prev_sw_sig=true;

    while(1){
        if(run_main_loop){
            state=nextState;
            //------------------------------
            //run the Moore state machine, determine nextState
            //------------------------------
            switch(state){
            case INIT:
                if(request_opmode_change){
                    nextState=OPERATIONAL;
                    set_pwm_freq(cl_pwm_freq_mhz);
                    request_opmode_change=false;
                }
                else if(request_debug_state){
                    nextState=DEBUGSTATE;
                    set_pwm_freq(cl_pwm_freq_mhz);
                    request_debug_state=false;
                }
                else if(request_calibration){
                    nextState=CALIBRATION;
                }
                else
                    nextState=INIT;
                break;
            case OPERATIONAL:
                if(request_stop){
                    nextState=INIT;
                    request_stop=false;
                }
                break;
            case DEBUGSTATE:
                if(request_stop){
                    nextState=INIT;
                    request_stop=false;
                }
                else
                    nextState=DEBUGSTATE;
                break;
            case ERROR:
                if(request_stop){
                    //unset error led
                    P1->OUT &= (~BIT0);
                    nextState=INIT;
                    request_stop=false;
                }
            case CALIBRATION:
                if(request_stop)
                    nextState=INIT;
            }

            //------------------------------
            //check buttons and rotary switch
            //------------------------------
            //check the rotary switch and update reference current
            run_rotary_enc_fsm((P3->IN)&BIT7,(P5->IN)&BIT2,&was_rotated);
            if(was_rotated==1){
                if(state==OPERATIONAL){
                    if(opmode==CONSTANT_CURRENT){
                        if(cursor_position==0)
                            i_ref_ampl_ma+=1000;
                        else if(cursor_position==1)
                            i_ref_ampl_ma+=100;
                        else if(cursor_position==2)
                            i_ref_ampl_ma+=10;
                    }
                    else{
                        if(des_freq_mhz<maxfreqmhz_cfctrl)
                            des_freq_mhz+=10000;
                    }
                }
                else if(state==INIT){
                    if(in_screen==SELECT_MODE)
                        opmode=(opmode+1)%2;
                    else if(in_screen==SELECT_RESONATOR)
                        resonator_num=(resonator_num+1)%2;
                    else
                        selection_wheel_start=(selection_wheel_start+1)%2;
                }
            }
            else if(was_rotated==2){
                if(state==OPERATIONAL){
                    if(opmode==CONSTANT_CURRENT){
                        if(cursor_position==0)
                            i_ref_ampl_ma-=1000;
                        else if(cursor_position==1)
                            i_ref_ampl_ma-=100;
                        else if(cursor_position==2)
                            i_ref_ampl_ma-=10;
                    }
                    else{
                        if(des_freq_mhz>minfreqmhz_cfctrl)
                            des_freq_mhz-=10000;
                    }
                }
                else if(state==INIT){
                    if(in_screen==SELECT_MODE)
                        opmode=(opmode-1)%2;
                    else if(in_screen==SELECT_RESONATOR)
                        resonator_num=(resonator_num-1)%2;
                    else
                        selection_wheel_start=(selection_wheel_start+1)%2;
                }
            }
            //check max/min limits for reference current amplitude
            if(i_ref_ampl_ma>i_ref_amp_max)
                i_ref_ampl_ma=i_ref_amp_max;
            else if(i_ref_ampl_ma<0)
                i_ref_ampl_ma=0;
            //check if rotary knob (switch signal sw (P3.5)) has had a rising edge
            bool sw_sig=((P3->IN&BIT5)==0);
            if(sw_sig&(!prev_sw_sig)){
                if(state==OPERATIONAL)
                    cursor_position=(cursor_position+1)%3;
                else if(state==INIT){
                    if(in_screen==START){
                        in_screen=(selection_wheel_start==0) ? SELECT_MODE : SELECT_RESONATOR;
                    }
                    else{
                        in_screen=START;
                    }
                }
            }
            prev_sw_sig=sw_sig;

            //------------------------------
            //parse input command & set display output
            //------------------------------
            //process uart commands
            if(command_to_be_processed){
                parse_input(input_pointer);
                input_pointer=0;
                command_to_be_processed=false;
            }
            //update display
            counter_disp++;
            if((counter_disp%counter_disp_max)==0){
                hd44780_timer_isr();
                toggle_debugging();
            }
            if((counter_disp%counter_disp_max2)==0){
                hd44780_clear_screen();

                if(state==OPERATIONAL){
                    if(opmode==CONSTANT_CURRENT){
                        sprintf(buffer_0,"CONST. CURRENT MODE",(float)i_ref_ampl_ma*1e-3);
                        sprintf(buffer_1,"Des. Curr: %05.3f",(float)i_ref_ampl_ma*1e-3);
                        sprintf(buffer_2,"Current:   %05.3f",imeas_hat);
                        sprintf(buffer_3,"Freq:      %05.3f",des_freq_controller);

                        hd44780_write_string(buffer_0,1,1,NO_CR_LF);
                        hd44780_write_string(buffer_1,2,1,NO_CR_LF);
                        hd44780_write_string(buffer_2,3,1,NO_CR_LF);
                        hd44780_write_string(buffer_3,4,1,NO_CR_LF);
                    }
                    else{
                        sprintf(buffer_0,"CONST. FREQU. MODE  ",(float)i_ref_ampl_ma*1e-3);
                        sprintf(buffer_1,"                    ",(float)i_ref_ampl_ma*1e-3);
                        sprintf(buffer_2,"Current:   %05.3f",imeas_hat);
                        sprintf(buffer_3,"Freq:      %5d",des_freq_mhz/1000);

                        hd44780_write_string(buffer_0,1,1,NO_CR_LF);
                        hd44780_write_string(buffer_1,2,1,NO_CR_LF);
                        hd44780_write_string(buffer_2,3,1,NO_CR_LF);
                        hd44780_write_string(buffer_3,4,1,NO_CR_LF);
                    }
                }
                else if(state==ERROR){
                    hd44780_write_string("ERROR",1,1,NO_CR_LF);
                    hd44780_write_string("Click Button",2,1,NO_CR_LF);
                    hd44780_write_string("For Reset",3,1,NO_CR_LF);
                }
                else if(state==DEBUGSTATE){
                    sprintf(buffer_0,"DEBUG MODE",(float)i_ref_ampl_ma*1e-3);
                    sprintf(buffer_1,"Freq:      %5d",debug_frequency_mhz/1000);
                    sprintf(buffer_2,"Current:   %05.3f",imeas_hat);

                    hd44780_write_string(buffer_0,1,1,NO_CR_LF);
                    hd44780_write_string(buffer_1,2,1,NO_CR_LF);
                    hd44780_write_string(buffer_2,3,1,NO_CR_LF);
                }
                else if(state==INIT){
                    if(in_screen==START){
                        hd44780_write_string("Ctrl Ready      v0.1",1,1,NO_CR_LF);
                        hd44780_write_string("                    ",2,1,NO_CR_LF);
                        hd44780_write_string("[ ] Select Mode     ",3,1,NO_CR_LF);
                        hd44780_write_string("[ ] Select Resonator",4,1,NO_CR_LF);
                        if(selection_wheel_start==0)
                            hd44780_write_string("x",3,2,NO_CR_LF);
                        else
                            hd44780_write_string("x",4,2,NO_CR_LF);
                    }
                    else if(in_screen==SELECT_MODE){
                        hd44780_write_string("Mode Select         ",1,1,NO_CR_LF);
                        hd44780_write_string("                    ",2,1,NO_CR_LF);
                        hd44780_write_string("[ ] Const. Current  ",3,1,NO_CR_LF);
                        hd44780_write_string("[ ] Const. Frequency",4,1,NO_CR_LF);
                        if(opmode==CONSTANT_CURRENT)
                            hd44780_write_string("x",3,2,NO_CR_LF);
                        else
                            hd44780_write_string("x",4,2,NO_CR_LF);
                    }
                    //else if(inscreen==SELECT_RESONATOR)
                }
            }

            //------------------------------
            //set the outputs / process measurements
            //------------------------------
            if(state==INIT){
                ////////////
                //set output signals
                ////////////
                set_disable();            //set the disable bit to low
            }
            else if(state==DEBUGSTATE){
                imeas_hat=main_avg_abs_current_est*alpha+beta;
                unset_disable();
            }
            else if(state==OPERATIONAL){
                unset_disable();
                imeas_hat=main_avg_abs_current_est*alpha+beta;

                if(opmode==CONSTANT_CURRENT){
                    counter_res_cl=(counter_res_cl+1)%100;
                    //counter makes sure this loop is only executed every 100th interrupt
                    if(counter_res_cl==0){
                        //determine the necessary frequency for the desired current amplitude from the lookup table
                        des_freq_controller=frequency_lookup(i_ref_ampl_ma);

                        //error signal
                        float err_i_hat=1e-3*((float)i_ref_ampl_ma-imeas_hat);
                        err_i_hat_integral+=err_i_hat;
                        des_freq_controller=des_freq_controller-(res_kp*err_i_hat+res_ki*err_i_hat_integral);

                        //check if there was an error calculating the des_freq
                        if(des_freq_controller<minfreq_cfctrl){
                            set_pwm_freq(1000000);
                            i_ref_ampl_ma=1000;
                            fatal_error();
                        }
                        else
                            set_pwm_freq((unsigned int)1000.0*des_freq_controller);
                    }
                }
                else{
                    set_pwm_freq(des_freq_mhz);
                }
            }

            else if(state==CALIBRATION){
                unset_disable();
                if(!calibration_initialized){
                    init_calibration();
                    calibration_initialized=true;
                }
                else{
                    set_pwm_freq(meas_freqs[meas_number]*1000);
                    calib_wait_counter++;
                    if(calib_wait_counter>=CALIB_WAIT_CYCLES){
                        //compute the current in mA
                        imeas_hat=main_avg_abs_current_est*alpha+beta;
                        meas_currents[meas_number]=(uint16_t)(imeas_hat);
                        meas_number++;
                        calib_wait_counter=0;
                    }
                    if(meas_number==N_MEAS-1){
                        set_disable();
                        set_pwm_freq(100000000);
                        request_calibration=false;
                        request_stop=true;
                    }
                }

            }
            run_main_loop=false;

            }
    }
}
