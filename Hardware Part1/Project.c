
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
#include "motorControl.h"
#include "nvic.h"

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

#define TIMER_IN_IR     PORTD, 2 // WT3CCP0

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



//-----------------------------------------------------------------------------
// Initialize Hardware
//-----------------------------------------------------------------------------

void initHw(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0 | SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2 | SYSCTL_RCGCTIMER_R3; // Enable clock for Timer 2
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0 | SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R2 | SYSCTL_RCGCWTIMER_R3;
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
    setPinAuxFunction(TIMER_IN_IR, GPIO_PCTL_PD2_WT3CCP0);

    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);

    selectPinDigitalInput(PB_1);
    enablePinPullup(PB_1);

    setPinCommitControl(PB_2);
    selectPinDigitalInput(PB_2);
    enablePinPullup(PB_2);
}

void enableTimerMode()
{
    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER3_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER3_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER3_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER3_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER3_TAV_R = 0;                               // zero counter for first period
    WTIMER3_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER3A-16-96);         // turn-on interrupt 116 (WTIMER3A)
}

volatile uint32_t timeElapsed = 0;  // global variable to store time
volatile uint32_t currentTime = 0;

void IRdecoder(void) //fine tweak still
{
    // Calculate the pulse width in microseconds
    pulseWidth = WTIMER3_TAV_R / 40;
    //printfUart0("Pulse Width: %d\n", pulseWidth);
    WTIMER3_TAV_R = 0;

    // Noise Filtering
    if (pulseWidth < 100 || pulseWidth > 100000)
    {
        WTIMER3_ICR_R = TIMER_ICR_CAECINT;
        return;
    }

    // Decode NEC protocol
    switch(currentState)
    {
        case NEC_IDLE:
            if(pulseWidth >= 3000 && pulseWidth <= 10000)
            {
                //printfUart0("Entered NEC_START\n");
                currentState = NEC_START;
            }
            break;

        case NEC_START:
            if(pulseWidth >= 500 && pulseWidth <= 5000)
            {
                //printfUart0("Transitioning to NEC_DATA\n");
                currentState = NEC_DATA;
                data = 0;
                bitCount = 0;
            }
            else
                currentState = NEC_IDLE;
            break;

        case NEC_DATA:
            if(pulseWidth >= 400 && pulseWidth <= 1200) // '0'
            {
                data <<= 1;
                bitCount++;
                //printfUart0("Detected '0'\n");
            }
            else if(pulseWidth >= 1200 && pulseWidth <= 3050) // Adjusted tolerance for '1'
            {
                data <<= 1;
                data |= 1;
                bitCount++;
                //printfUart0("Detected '1'\n");
            }
            else if (pulseWidth >= 2800) // Handle longer pulses separately
            {
                //printfUart0("Long pulse detected\n");
            }
            else
            {
                currentState = NEC_IDLE;
                printfUart0("Unexpected pulse, resetting...\n");
                return;
            }

            if(bitCount == 32)
            {
                printfUart0("Decoded Data: %u\n", data);
                currentState = NEC_IDLE;
            }
            break;
    }

    WTIMER3_ICR_R = TIMER_ICR_CAECINT;
}


void wideTimer3Isr()
{
    togglePinValue(GREEN_LED);
    IRdecoder();
}

/*
void wideTimer3Isr()
{
    pulseWidth = WTIMER3_TAV_R / 40; // Convert to microseconds
    printfUart0("Pulse Width: %d\n", pulseWidth);
    WTIMER3_TAV_R = 0;
    WTIMER3_ICR_R = TIMER_ICR_CAECINT; // Clear the interrupt
}
*/

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

    printfUart0("\n\nInitialization Success\n\n");

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

    uint32_t testing = 10;

    while (true)
    {
        //do nothing?
        //printfUart0("\n Success\n");
        //printfUart0("Edge Count: %d\n", edgeCount);
        //printfUart0("Time Elapsed: %d\n", timeElapsed);
//        waitMicrosecond(testing);
//        timeElapsed = WTIMER3_TAV_R;                    // read counter input
//        printfUart0("Time Elapsed: %d\n", timeElapsed);
//        WTIMER3_TAV_R = 0;                              // zero counter for next edge
//        testing++;
//
//        if(testing>1000)
//        {
//            testing = 1;
//        }
    }
}
