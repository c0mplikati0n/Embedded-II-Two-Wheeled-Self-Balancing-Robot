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
#include "conversion.h"

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

uint32_t lastTime = 0; // Last captured time
uint32_t pulseWidth;

typedef enum
{
    NEC_IDLE,
    NEC_START,
    NEC_DATA
} NEC_State;

NEC_State currentState = NEC_IDLE;
uint32_t data = 0; // Stores the decoded data
uint8_t bitCount = 0; // Bit counter for the 32-bits of data

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

/*
void enableTimerMode() // Time Enable
{
    // ISR // PB1 // T2CCP1
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = 40000000;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts

    NVIC_EN0_R |= 1 << (INT_TIMER2B-16);             // turn-on interrupt 40 (TIMER2B)
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}
*/

///*
void enableTimerMode() // Time Enable
{
    // ISR // PB1 // T2CCP1
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_CAP;             // configure for capture mode
    //TIMER2_CTL_R |= GPTM_CTL_TACM;                  // configure for edge-time mode and count up on rising edge
    TIMER2_TAILR_R = 40000000;                       // initialize the load register
    TIMER2_IMR_R = TIMER_IMR_CAEIM;                  // enable capture match interrupt

    NVIC_EN0_R |= 1 << (INT_TIMER2B-16);             // turn-on interrupt 39 (TIMER2A)
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}
//*/

//-----------------------------------------------------------------------------
// Initialize Hardware
//-----------------------------------------------------------------------------

void initHw(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0 | SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2; // Enable clock for Timer 2
    _delay_cycles(3);

    enablePort(PORTA);
    enablePort(PORTB);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);
    
    selectPinPushPullOutput(OUT_ENABLE);

    selectPinPushPullOutput(OUT_PWM_1);
    selectPinPushPullOutput(OUT_PWM_2);
    selectPinPushPullOutput(OUT_PWM_3);
    selectPinPushPullOutput(OUT_PWM_4);

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

void IRdecoder(void)
{
    uint32_t currentTime;

    // Get the current time
    currentTime = TIMER2_TAR_R;

    // Calculate the pulse width
    pulseWidth = lastTime - currentTime;
    lastTime = currentTime;

    printfUart0("\nPulse Width =     %d \n", pulseWidth);
    printfUart0("\nCurrent Time =    %d \n", currentTime);
    printfUart0("\nLast Time =       %d \n", lastTime);


    // Decode NEC protocol based on the pulseWidth and currentState
    switch(currentState)
    {
        case NEC_IDLE:
            if(pulseWidth >= 9000 && pulseWidth <= 9100) // Roughly check for 9ms
                currentState = NEC_START;
            break;

        case NEC_START:
            if(pulseWidth >= 4500 && pulseWidth <= 4600) // Roughly check for 4.5ms
            {
                currentState = NEC_DATA;
                data = 0;
                bitCount = 0;
            }
            else
                currentState = NEC_IDLE;
            break;

        case NEC_DATA:
            if(pulseWidth >= 500 && pulseWidth <= 600) // Roughly check for 562.5µs
            {
                data <<= 1; // Shift data left
                bitCount++;
            }
            else if(pulseWidth >= 1600 && pulseWidth <= 1700) // Roughly check for 1.6875ms
            {
                data <<= 1;
                data |= 1; // Set the least significant bit
                bitCount++;
            }

            if(bitCount == 32)
            {
                // Here, the variable 'data' has the 32-bit NEC data
                // Process or store the data as required
                currentState = NEC_IDLE;
            }
            break;
    }
    TIMER2_ICR_R = TIMER_ICR_CAECINT; // Clear the interrupt
}

void turnOffAll()
{
    PWM0_0_CMPA_R = 0; // Left Wheel
    PWM0_0_CMPB_R = 0; // Left Wheel
    PWM0_3_CMPA_R = 0; // Right Wheel
    PWM0_3_CMPB_R = 0; // Right Wheel
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    enableTimerMode();
    initPWM();

    //waitMicrosecond(1000000);

    setPinValue(OUT_ENABLE, 1);

    turnOffAll();

    //setPwmDutyCycle(0, 1000, 0); // Left wheel moves forward
    //setPwmDutyCycle(1, 0, 1000); // Right Wheel moves forward
    //setPwmDutyCycle(0, 0, 1000); // Left wheel moves backwards
    //setPwmDutyCycle(1, 1000, 0); // Right wheel moves backwards

    //setDirection(0, 0, 1000, 1000, 0); // Both wheels go backwards
    //setDirection(1, 1000, 0, 0, 1000); // Both wheels go forwards

    //printfUart0("\nInitialization Success\n\n");

    while (true)
    {
        //do nothing?
        //printfUart0("\n Success\n");
    }
}













