// Implementing Motor and IR Sensor
// Xavier A. Portillo-Catalan
// UTA ID: 1001779115

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "adc0.h"

// Pins
#define I2C1SCL         PORTA, 6 // I2C 1 SCL
#define I2C1SDA         PORTA, 7 // I2C 1 SDA

#define OUT_ENABLE      PORTE, 0 // P30

#define OUT_PWM_1       PORTC, 4 // M0PWM6
#define OUT_PWM_2       PORTC, 5 // M0PWM7
#define OUT_PWM_3       PORTB, 6 // M0PWM0
#define OUT_PWM_4       PORTB, 7 // M0PWM1

#define TIMER_IN_L      PORTD, 6 // WT5CCP0
#define TIMER_IN_R      PORTC, 6 // WT1CCP0

#define TIMER_IN_IR     PORTB, 1 // T2CCP1

#define RED_LED         PORTF, 1
#define GREEN_LED       PORTF, 3
#define BLUE_LED        PORTF, 2

#define PB_1            PORTF, 4
#define PB_2            PORTF, 0

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initPWM(void){
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0; // Pg. 354
    _delay_cycles(3);

    GPIO_PORTC_PCTL_R &= GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M;    // enable PWM
    GPIO_PORTB_PCTL_R &= GPIO_PCTL_PB6_M | GPIO_PCTL_PB7_M;    // enable PWM
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_M0PWM6 | GPIO_PCTL_PC5_M0PWM7;
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB6_M0PWM0 | GPIO_PCTL_PB7_M0PWM1;

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module // Pg. 330
    SYSCTL_SRPWM_R = 0;                              // leave reset state // Pg. 330

    PWM0_0_CTL_R = 0;                                // turn-off PWM0 generator 0 (drives outs 0 and 1) // Pg. 1266
    PWM0_3_CTL_R = 0;                                // turn-off PWM0 generator 3 (drives outs 6 and 7) // Pg. 1266

    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE; // output 0 on PWM0, gen 0a, cmpa // Pg. 1282
    PWM0_0_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE; // output 1 on PWM0, gen 0b, cmpb // Pg. 1282
    PWM0_3_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE; // output 6 on PWM0, gen 3a, cmpa // Pg. 1282
    PWM0_3_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE; // output 7 on PWM0, gen 3b, cmpb // Pg. 1282

    PWM0_0_LOAD_R = 1024; // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_3_LOAD_R = 1024; // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz

    // invert outputs so duty cycle increases with increasing compare values
    PWM0_INVERT_R = PWM_INVERT_PWM0INV | PWM_INVERT_PWM1INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV; // Pg. 1249

    PWM0_0_CMPA_R = 0; // M0PWM0 // PB6
    PWM0_0_CMPB_R = 0; // M0PWM1 // PB7
    PWM0_3_CMPA_R = 0; // M0PWM6 // PC4
    PWM0_3_CMPB_R = 0; // M0PWM7 // PC5

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE;    // turn-on PWM0 generator 2
    PWM0_3_CTL_R = PWM_0_CTL_ENABLE;    // turn-on PWM0 generator 2

    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN; // enable outputs
}

/*
void initPWM(void)
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0; // Pg. 354
    _delay_cycles(3);

    setPinAuxFunction(OUT_PWM_1, GPIO_PCTL_PC4_M0PWM6) // M0PWM6 // PC4
    setPinAuxFunction(OUT_PWM_2, GPIO_PCTL_PC5_M0PWM7) // M0PWM7 // PC5
    setPinAuxFunction(OUT_PWM_3, GPIO_PCTL_PB6_M0PWM0) // M0PWM0 // PB6
    setPinAuxFunction(OUT_PWM_4, GPIO_PCTL_PB7_M0PWM1) // M0PWM1 // PB7

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state

    PWM0_0_CTL_R = 0;                                // turn-off PWM0 generator 0 (drives outs 0 and 1)
    PWM0_3_CTL_R = 0;                                // turn-off PWM0 generator 3 (drives outs 6 and 7)

    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE; // output 0 on PWM0, gen 0a, cmpa // Pg. 1282
    PWM0_0_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE; // output 1 on PWM0, gen 0b, cmpb // Pg. 1282
    PWM0_3_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE; // output 6 on PWM0, gen 3a, cmpa // Pg. 1282
    PWM0_3_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE; // output 7 on PWM0, gen 3b, cmpb // Pg. 1282

    PWM0_0_LOAD_R = 1024; // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_3_LOAD_R = 1024; // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz

    // invert outputs so duty cycle increases with increasing compare values
    PWM0_INVERT_R = PWM_INVERT_PWM0INV | PWM_INVERT_PWM1INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;

   PWM0_0_CMPA_R = 0; // M0PWM0 // PB6
   PWM0_0_CMPB_R = 0; // M0PWM1 // PB7
   PWM0_3_CMPA_R = 0; // M0PWM6 // PC4
   PWM0_3_CMPB_R = 0; // M0PWM7 // PC5 // (0=always low, 1023=always high)

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 0
    PWM0_3_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 3

    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN| PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
}
*/

//-----------------------------------------------------------------------------
// Initialize Hardware
//-----------------------------------------------------------------------------

void initHw(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    _delay_cycles(3);

    enablePort(PORTA);
    enablePort(PORTB);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);
    
    selectPinPushPullOutput(OUT_ENABLE);

    selectPinDigitalInput(TIMER_IN_L);
    selectPinDigitalInput(TIMER_IN_R);
    selectPinDigitalInput(TIMER_IN_IR);



    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);

    selectPinDigitalInput(PB_1);
    enablePinPullup(PB_1);

    setPinCommitControl(PB_2);
    selectPinDigitalInput(PB_2);
    enablePinPullup(PB_2);
}

void changeState(uint8_t side)
{
    switch (side)
    {
    case 1:
        //setPinValue(OUT_PWM_A1, 1);
        //setPinValue(OUT_PWM_A2, 0);
        //setPinValue(OUT_PWM_B1, 0);
        //setPinValue(OUT_PWM_B2, 0);

        PWM0_0_CMPA_R = 500;
        PWM0_0_CMPB_R = 0;
        PWM0_3_CMPA_R = 0;
        PWM0_3_CMPB_R = 0;
        break;

    case 2:
        //setPinValue(OUT_PWM_A1, 0);
        //setPinValue(OUT_PWM_A2, 0);
        //setPinValue(OUT_PWM_B1, 1);
        //setPinValue(OUT_PWM_B2, 0);

        PWM0_0_CMPA_R = 0;
        PWM0_0_CMPB_R = 0;
        PWM0_3_CMPA_R = 500;
        PWM0_3_CMPB_R = 0;
        break;

    case 3:
        //setPinValue(OUT_PWM_A1, 0);
        //setPinValue(OUT_PWM_A2, 1);
        //setPinValue(OUT_PWM_B1, 0);
        //setPinValue(OUT_PWM_B2, 0);

        PWM0_0_CMPA_R = 0;
        PWM0_0_CMPB_R = 500;
        PWM0_3_CMPA_R = 0;
        PWM0_3_CMPB_R = 0;
        break;

    case 4:
        //setPinValue(OUT_PWM_A1, 0);
        //setPinValue(OUT_PWM_A2, 0);
        //setPinValue(OUT_PWM_B1, 0);
        //setPinValue(OUT_PWM_B2, 1);

        PWM0_0_CMPA_R = 0;
        PWM0_0_CMPB_R = 0;
        PWM0_3_CMPA_R = 0;
        PWM0_3_CMPB_R = 500;
        break;
    }
    waitMicrosecond(10);  // Wait 1 second
}

void setPwmDutyCycle(uint8_t side, uint16_t pwmA, uint16_t pwmB)
{
    switch(side)
    {
        case 0:
            PWM0_1_CMPA_R = pwmA;
            PWM0_1_CMPB_R = pwmB;
            break;
        case 1:
            PWM0_2_CMPA_R = pwmA;
            PWM0_2_CMPB_R = pwmB;
            break;
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();
    initUart0();

    //waitMicrosecond(1000000);

    setPinValue(OUT_ENABLE, 1);

    PWM0_0_CMPA_R = 0; //
    PWM0_0_CMPB_R = 0; //
    PWM0_3_CMPA_R = 0; //
    PWM0_3_CMPB_R = 0; //

    setPwmDutyCycle();


    while (true)
    {

    }
}













