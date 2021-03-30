#include "ti/devices/msp432p4xx/inc/msp.h"
#include <stdbool.h>
#include "current_controller.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "state_machine.h"
#include <math.h>
#include "libs/hd44780.h"
#include "impedance_lookup.h"
#include "stdio.h"
#include "rotary_enc_sw.h"

/*
 * Peripherals Overview
 * TA1.1    ->  duty                ->  P7.7
 * TA1.2    ->  adc trigger         ->  P7.6
 * TA1.3    ->  ¬duty               ->  P7.5    ! Make sure duty and ¬duty are connected in the right order, otherwise current controller unstable
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
const uint16_t clk_divider_cnt=8;   //clock division factor for counter
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
const int32_t va_offset=8192;           //measured offset (needs to be calibrated) (nominal 0x1FFF=8192)
const float conv_const=0.001359167287240;        //conversion from
//control related parameters
#define V_DC 30.0
#define CONTROLLER_DT 200E-6
uint32_t i_ref_ampl_ma=1000;                   //reference amplitude value [mA]
const uint32_t i_ref_amp_max=5000;         //maximum current [mA]
const float v_in_hat=V_DC*4.0/M_PI;
struct pi_controller_32 current_controller={0.0,0.0,0.0,8.0,0.0,CONTROLLER_DT};
float res_kp=5.0;
float res_ki=1.0;
float err_i_hat_integral=0.0;   //integral of pi current controller
float des_freq=1000.0;
float des_imp=1000.0;
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
uint8_t input_pointer=0;
//state machine related parameters and variables
enum system_state state=INIT;
enum system_state nextState=INIT;
const uint16_t n_shutdown=20+2;                 //number of cycles to wait before energy is assumed to be fed back into DC link
uint16_t transition_counter=0;
bool request_opmode_change=false;                //this bit is set when a change of state from CLOSED_LOOP to RESONANT is requested
bool command_to_be_processed=false;
bool request_debug_state=false;
bool request_stop=false;
//lcd related parameters
char top_buffer[16]="";
char bottom_buffer[16]="";
//rotary encoder / switch related parameters
int32_t rotary_counter=0;
uint8_t was_rotated;            //rotary encoder variable (0 if not rotated, 1 if rotated cw, 2 if rotated ccw)
uint8_t was_pressed;            //press button variable (1 if was pressed, 0 if not)
uint8_t cursor_position=1;        //position of the selection cursor [5.28 , first digit 0 second digit 1 third digit 2]

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
    //P7.5 ¬duty
    P7->SEL0 |= BIT5;
    P7->SEL1 &= (~BIT5);
    P7->DIR |= BIT5;
    //interrupt from buttons

    //P5.7 as debugging output
    P5->DIR |= BIT7;


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

//init the pwm and the adc for synchronized sampling
void init_pwm_adc(void){
    // Configure Timer_A1 (this timer is used for duty generation)
    TIMER_A1->CTL = TIMER_A_CTL_TASSEL_2 |  // SMCLK
            TIMER_A_CTL_MC_1 |              // Up Mode
            TIMER_A_CTL_CLR |               // Clear TAR
            TIMER_A_CTL_ID_3;               // Clk /8
    TIMER_A1->EX0 = TIMER_A_EX0_TAIDEX_0;   // Clk /1
    //configure CCR TA1.0 (ceiling register)
    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_OUTMOD_4; // CCR4 toggle mode
    TIMER_A1->CCR[0] = 1024;
    //configure CCR TA1.1 (duty register)
    TIMER_A1->CCR[1] = duty;                     //this value corresponds to the PWM value in [0...255]
    TIMER_A1->CCTL[1] = TIMER_A_CCTLN_OUTMOD_3; // CCR4 toggle mode for PWM generation
    //configure CCR TA1.3 as the inverse duty cycle
    TIMER_A1->CCR[3] = duty;                     //this value corresponds to the PWM value in [0...255]
    TIMER_A1->CCTL[3] = TIMER_A_CCTLN_OUTMOD_6; // CCR4 toggle mode for PWM generation


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
    //formula for PWM: f_pwm=f_0/CCR[0]*2, f_0 at the moment is 12.8MHz
    uint16_t counter_limit=12000000000/clk_divider_cnt/freq*4;
    //we set CCR[0]:=800, therefore the output frequency is 30kHz
    TIMER_A1->CCR[0]=counter_limit;        //counter counts to CCR[0]
    //set duty cycle to 50%
    TIMER_A1->CCR[1]=counter_limit/2;        //counter toggles at CCR[1]
    //set inverted duty cycle to 50% as well
    TIMER_A1->CCR[3] =counter_limit/2;
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
        //MAP_UART_transmitData(EUSCI_A0_BASE, MAP_UART_receiveData(EUSCI_A0_BASE));
        input_buffer[input_pointer]=MAP_UART_receiveData(EUSCI_A0_BASE);
        //check if newline character received. each command ends with a newline character
        //we replace the newline character with a string termination character for further processing
        if(input_buffer[input_pointer]=='\r'){
            command_to_be_processed=true;
            input_buffer[input_pointer]='\0';
            input_pointer=0;
        }
        else
            input_pointer++;
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

    uint32_t counter_disp=0;
    const uint32_t counter_disp_max=20;
    const uint32_t counter_disp_max2=4096;
    uint32_t counter_res_cl=0;
    bool sw_sig=true;
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
            }

            //------------------------------
            //check buttons and rotary switch
            //------------------------------
            //check the rotary switch and update reference current
            run_rotary_enc_fsm((P3->IN)&BIT7,(P5->IN)&BIT2,&was_rotated);
            if(was_rotated==1){
                if(cursor_position==0)
                    i_ref_ampl_ma+=1000;
                else if(cursor_position==1)
                    i_ref_ampl_ma+=100;
                else if(cursor_position==2)
                    i_ref_ampl_ma+=10;
            }
            else if(was_rotated==2){
                if(cursor_position==0)
                    i_ref_ampl_ma-=1000;
                else if(cursor_position==1)
                    i_ref_ampl_ma-=100;
                else if(cursor_position==2)
                    i_ref_ampl_ma-=10;
            }
            //check max/min limits for reference current amplitude
            if(i_ref_ampl_ma>i_ref_amp_max)
                i_ref_ampl_ma=i_ref_amp_max;
            else if(i_ref_ampl_ma<0)
                i_ref_ampl_ma=0;
            //check if switch signal sw (P3.5) has had a rising edge
            bool sw_sig=((P3->IN&BIT5)==0);
            if(sw_sig&(!prev_sw_sig))
                cursor_position=(cursor_position+1)%3;
            prev_sw_sig=sw_sig;

            //------------------------------
            //parse input command & set display output
            //------------------------------
            //process uart commands
            if(command_to_be_processed){
                parse_input(input_pointer);
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
                    sprintf(top_buffer,"Current: %05.3f",imeas_hat);
                    sprintf(bottom_buffer,"Freq:    %05.3f",des_freq);

                    hd44780_write_string(top_buffer,1,1,NO_CR_LF);
                    hd44780_write_string(bottom_buffer,2,1,NO_CR_LF);
                }
                else{
                    hd44780_write_string("Controller Ready",1,1,NO_CR_LF);
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
                unset_disable();
            }
            else if(state==OPERATIONAL){
                unset_disable();

                counter_res_cl=(counter_res_cl+1)%100;

                //counter makes sure this loop is only executed every 100th interrupt
                if(counter_res_cl==0){
                    //calculate the frequency for desired current amplitude i_ref_ampl
                    //first calculate the necessary impedance
                    des_imp=1e3*v_in_hat/i_ref_ampl_ma;
                    des_freq=inverse_impedance(des_imp);


                    //run PI current controller
                    imeas_hat=main_avg_abs_current_est*conv_const;
                    //error signal
                    float err_i_hat=1e-3*(float)i_ref_ampl_ma-imeas_hat;
                    err_i_hat_integral+=err_i_hat;
                    des_freq=des_freq-(res_kp*err_i_hat+res_ki*err_i_hat_integral);

                    //check if there was an error calculating the des_freq
                    if(des_freq<minfreq){
                        set_pwm_freq(1000000);
                        fatal_error();
                    }
                    else
                        set_pwm_freq((unsigned int)1000.0*des_freq);
                }
            }
            run_main_loop=false;

            }
    }
}
