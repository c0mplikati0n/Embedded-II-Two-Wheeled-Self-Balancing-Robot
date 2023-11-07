// Motor Library
// Xavier

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration: -

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include "motorControl.h"
#include "tm4c123gh6pm.h"
#include "gpio.h"

#define OUT_PWM_1       PORTC, 4 // M0PWM6
#define OUT_PWM_2       PORTC, 5 // M0PWM7
#define OUT_PWM_3       PORTB, 6 // M0PWM0
#define OUT_PWM_4       PORTB, 7 // M0PWM1

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
// Motor Data
//-----------------------------------------------------------------------------

    //setPwmDutyCycle(0, 1000, 0); // Left wheel moves forward
    //setPwmDutyCycle(1, 0, 1000); // Right Wheel moves forward
    //setPwmDutyCycle(0, 0, 1000); // Left wheel moves backwards
    //setPwmDutyCycle(1, 1000, 0); // Right wheel moves backwards

    //setDirection(0, 0, 1000, 1000, 0); // Both wheels go backwards
    //setDirection(1, 1000, 0, 0, 1000); // Both wheels go forwards

    //setPwmDutyCycle(0, 750, 0); // Left wheel moves forward    // Lowest value = 750
    //setPwmDutyCycle(1, 0, 760); // Right Wheel moves forward   // Lowest value = 760
    //setPwmDutyCycle(0, 0, 740); // Left wheel moves backwards  // Lowest value = 740
    //setPwmDutyCycle(1, 750, 0); // Right wheel moves backwards // Lowest value = 750

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initPWM(void)
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0; // Pg. 354
    _delay_cycles(3);

    setPinAuxFunction(OUT_PWM_1, GPIO_PCTL_PC4_M0PWM6); // M0PWM6 // PC4
    setPinAuxFunction(OUT_PWM_2, GPIO_PCTL_PC5_M0PWM7); // M0PWM7 // PC5
    setPinAuxFunction(OUT_PWM_3, GPIO_PCTL_PB6_M0PWM0); // M0PWM0 // PB6
    setPinAuxFunction(OUT_PWM_4, GPIO_PCTL_PB7_M0PWM1); // M0PWM1 // PB7

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
    PWM0_3_CMPB_R = 0; // M0PWM7 // PC5 // (0 = always low, 1023 = always high)

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 0
    PWM0_3_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 3

    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
}

void setPwmDutyCycle(uint8_t side, uint16_t pwmA, uint16_t pwmB)
{
    switch(side)
    {
        case 0: // Left Wheel
            PWM0_0_CMPA_R = pwmA;
            PWM0_0_CMPB_R = pwmB;
            break;
        case 1: // Right Wheel
            PWM0_3_CMPA_R = pwmA;
            PWM0_3_CMPB_R = pwmB;
            break;
    }
}

void setDirection(uint8_t side, uint16_t pwmAL, uint16_t pwmBL, uint16_t pwmAR, uint16_t pwmBR)
{
    switch(side)
    {
        case 0: // Backwards
            PWM0_0_CMPA_R = pwmAL;
            PWM0_0_CMPB_R = pwmBL;
            PWM0_3_CMPA_R = pwmAR;
            PWM0_3_CMPB_R = pwmBR;
            break;
        case 1: // Forward
            PWM0_0_CMPA_R = pwmAL;
            PWM0_0_CMPB_R = pwmBL;
            PWM0_3_CMPA_R = pwmAR;
            PWM0_3_CMPB_R = pwmBR;
            break;
    }
}

void setDirectionNew(uint8_t direction, uint16_t pwmL, uint16_t pwmR)
{
    switch(direction)
    {
        case 0: // Backwards
            PWM0_0_CMPA_R = 0;
            PWM0_0_CMPB_R = pwmL;
            PWM0_3_CMPA_R = pwmR;
            PWM0_3_CMPB_R = 0;
            break;
        case 1: // Forward
            PWM0_0_CMPA_R = pwmL;
            PWM0_0_CMPB_R = 0;
            PWM0_3_CMPA_R = 0;
            PWM0_3_CMPB_R = pwmR;
            break;
    }
}

void turnOffAll(void)
{
    PWM0_0_CMPA_R = 0; // Left Wheel
    PWM0_0_CMPB_R = 0; // Left Wheel
    PWM0_3_CMPA_R = 0; // Right Wheel
    PWM0_3_CMPB_R = 0; // Right Wheel
}



