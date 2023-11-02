
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
uint32_t pulseWidth = 0;

typedef enum
{
    NEC_IDLE,
    NEC_START,
    NEC_DATA
} NEC_State;

typedef enum {
    NONE = 0,
    FORWARD_FAST   = 16722135,
    FORWARD_NORMAL = 16754775,
    FORWARD_SLOW   = 16738455,
    BACK_FAST      = 16713975,
    BACK_NORMAL    = 16746615,
    BACK_SLOW      = 16730295,
    ROTATE_LEFT    = 16771095,
    ROTATE_RIGHT   = 16762935
    // Add other buttons
} ButtonAction;

typedef enum
{
    BUTTON_RELEASED,
    BUTTON_PRESSED,
    BUTTON_HELD
} ButtonState;

ButtonAction currentButtonAction = NONE;
ButtonState currentButtonState = BUTTON_RELEASED;
NEC_State currentState = NEC_IDLE;

uint32_t data = 0; // Stores the decoded data
uint8_t bitCount = 0; // Bit counter for the 32-bits of data

uint32_t lastDecodedData = 0; // Stores the last valid decoded data

uint32_t noSignalCounter = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void processDecodedData(uint32_t data);
void handleButtonAction(void);

//-----------------------------------------------------------------------------
// Initialize Hardware
//-----------------------------------------------------------------------------

void initHw(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    SYSCTL_RCGCTIMER_R  |= SYSCTL_RCGCTIMER_R0  | SYSCTL_RCGCTIMER_R1  | SYSCTL_RCGCTIMER_R2  | SYSCTL_RCGCTIMER_R3;
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
    //printfUart0("Pulse Width: %u\n", pulseWidth);
    WTIMER3_TAV_R = 0;

    // Check for repeated signal pattern for button held down
    if (pulseWidth >= 95000 && pulseWidth <= 150000)
    {
        noSignalCounter = 0;
        currentButtonState = BUTTON_HELD;
        processDecodedData(lastDecodedData);
        WTIMER3_ICR_R = TIMER_ICR_CAECINT;
        return;
    } else if (pulseWidth >= 2000 && pulseWidth <= 3000) {
        // This is part of the repeated signal
        WTIMER3_ICR_R = TIMER_ICR_CAECINT;
        return;
    }

    // Decode NEC protocol
    switch(currentState)
    {
        case NEC_IDLE:
            if(pulseWidth >= 1000 && pulseWidth <= 10000)
            {
                //printfUart0("Entered NEC_START\n");
                currentState = NEC_START;
            }
        break;

        case NEC_START:
            if(pulseWidth >= 200 && pulseWidth <= 5000) // Unsure
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
            if(pulseWidth >= 1 && pulseWidth <= 1200) // '0'
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
            else
            {
                currentState = NEC_IDLE;
                //printfUart0("Unexpected pulse, resetting...\n");
                return;
            }

            if(bitCount == 31)
            {
                printfUart0("Decoded Data: %u\n", data);
                lastDecodedData = data;
                processDecodedData(data);
                currentState = NEC_IDLE;
                currentButtonState = BUTTON_PRESSED;
            }
        break;
    }
    WTIMER3_ICR_R = TIMER_ICR_CAECINT;
}

void wideTimer3Isr()
{
    togglePinValue(GREEN_LED);
    IRdecoder();
    //printfUart0("Bit Count:   %u\n", bitCount);
}

void processDecodedData(uint32_t data)
{
    switch(data)
    {
        case FORWARD_FAST:
            currentButtonAction = FORWARD_FAST;
        break;
        case FORWARD_NORMAL:
            currentButtonAction = FORWARD_NORMAL;
        break;
        case FORWARD_SLOW:
            currentButtonAction = FORWARD_SLOW;
        break;
        case BACK_FAST:
            currentButtonAction = BACK_FAST;
        break;
        case BACK_NORMAL:
            currentButtonAction = BACK_NORMAL;
        break;
        case BACK_SLOW:
            currentButtonAction = BACK_SLOW;
        break;
        // Handle other buttons similarly when you have their decoded data
        default:
            currentButtonAction = NONE;
    }
}

// This is what is called from main
void handleButtonAction(void)
{
    switch(currentButtonAction)
    {
        case NONE:
            // We aint doin nothin brub
        break;

        case FORWARD_FAST:
            if (currentButtonState == BUTTON_HELD)
            {
                //setDirection(1, 1000, 0, 0, 1000); // Both wheels go forwards
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                //turnOffAll();
            }
        break;

        default:
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
        /*
        noSignalCounter++;
        if (noSignalCounter > 1000000)  // This threshold needs to be adjusted based on experimentation
        {
            printfUart0("\n\nBUTTON RELEASED\n\n");
            currentButtonState = BUTTON_RELEASED;
            noSignalCounter = 0;  // Reset the counter
        }
        */

        handleButtonAction();
    }
}
