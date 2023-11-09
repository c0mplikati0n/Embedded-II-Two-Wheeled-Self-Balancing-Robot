// Implementing I2C direction thingy, and wheel sensors that make it go straight
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
#include "motorControl.h"
#include "nvic.h"
#include "i2c1.h"

// Pins
//#define I2C1SCL         PORTA, 6 // I2C 1 SCL
//#define I2C1SDA         PORTA, 7 // I2C 1 SDA

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

    ROTATE_LEFT_B  = 16734375,
    ROTATE_LEFT_F  = 16742535,
    ROTATE_RIGHT_B = 16767015,
    ROTATE_RIGHT_F = 16775175,

    SPINNING_BOI_1 = 16718565,
    SPINNING_BOI_2 = 16751205
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
    if ((currentButtonState == BUTTON_PRESSED) || (currentButtonState == BUTTON_HELD))
    {
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
    }

    // Decode NEC protocol
    switch(currentState)
    {
        case NEC_IDLE:
            if(pulseWidth >= 1000 && pulseWidth <= 10000)
            {
                currentState = NEC_START;
            }
        break;

        case NEC_START:
            if(pulseWidth >= 200 && pulseWidth <= 5000) // Unsure
            {
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
            }
            else if(pulseWidth >= 1200 && pulseWidth <= 3050) // Adjusted tolerance for '1'
            {
                data <<= 1;
                data |= 1;
                bitCount++;
            }
            else
            {
                currentState = NEC_IDLE;
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

//*
void wideTimer3Isr()
{
    togglePinValue(GREEN_LED);
    IRdecoder();
    //printfUart0("Bit Count:   %u\n", bitCount);
}
//*/

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

        case ROTATE_LEFT_B:
            currentButtonAction = ROTATE_LEFT_B;
        break;
        case ROTATE_LEFT_F:
            currentButtonAction = ROTATE_LEFT_F;
        break;

        case ROTATE_RIGHT_B:
            currentButtonAction = ROTATE_RIGHT_B;
        break;
        case ROTATE_RIGHT_F:
            currentButtonAction = ROTATE_RIGHT_F;
        break;

        case SPINNING_BOI_1:
            currentButtonAction = SPINNING_BOI_1;
        break;
        case SPINNING_BOI_2:
            currentButtonAction = SPINNING_BOI_2;
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
                setDirection(1, 1023, 1023); // Both wheels go forwards
                //printfUart0("\nFORWARD_FAST\n");
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                setDirection(1, 1000, 1000); // Both wheels go forwards
                waitMicrosecond(100000);
                setDirection(1, 950, 950); // Both wheels go forwards
                waitMicrosecond(100000);
                setDirection(1, 850, 850); // Both wheels go forwards
                waitMicrosecond(100000);
                setDirection(1, 800, 800); // Both wheels go forwards
                waitMicrosecond(100000);
                turnOffAll();
                //printfUart0("\nOFF\n");
            }
        break;
        case FORWARD_NORMAL:
            if (currentButtonState == BUTTON_HELD)
            {
                setDirection(1, 1023, 1023); // Both wheels go forwards
                waitMicrosecond(100000);
                setDirection(1, 900, 900); // Both wheels go forwards
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                turnOffAll();
            }
        break;
        case FORWARD_SLOW:
            if (currentButtonState == BUTTON_HELD)
            {
                setDirection(1, 1023, 1023); // Both wheels go forwards
                waitMicrosecond(100000);
                setDirection(1, 800, 800); // Both wheels go forwards
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                turnOffAll();
            }
        break;

        case BACK_FAST:
            if (currentButtonState == BUTTON_HELD)
            {
                setDirection(0, 1023, 1023); // Both wheels go backwards
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                setDirection(0, 1000, 1000); // Both wheels go forwards
                waitMicrosecond(100000);
                setDirection(0, 950, 950); // Both wheels go forwards
                waitMicrosecond(100000);
                setDirection(0, 850, 850); // Both wheels go forwards
                waitMicrosecond(100000);
                setDirection(0, 800, 800); // Both wheels go forwards
                turnOffAll();
            }
        break;
        case BACK_NORMAL:
            if (currentButtonState == BUTTON_HELD)
            {
                setDirection(0, 1023, 1023); // Both wheels go backwards
                waitMicrosecond(100000);
                setDirection(0, 900, 900); // Both wheels go backwards
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                turnOffAll();
            }
        break;
        case BACK_SLOW:
            if (currentButtonState == BUTTON_HELD)
            {
                setDirection(0, 1023, 1023); // Both wheels go backwards
                waitMicrosecond(100000);
                setDirection(0, 790, 790); // Both wheels go backwards
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                turnOffAll();
            }
        break;

        case ROTATE_LEFT_B:
            if (currentButtonState == BUTTON_HELD)
            {
                setPwmDutyCycle(0, 0, 1023); // Left wheel moves backwards
                waitMicrosecond(100000);
                setPwmDutyCycle(0, 0, 1000); // Left wheel moves backwards
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                turnOffAll();
            }
        break;
        case ROTATE_LEFT_F:
            if (currentButtonState == BUTTON_HELD)
            {
                setPwmDutyCycle(0, 1023, 0); // Left wheel moves forward
                waitMicrosecond(100000);
                setPwmDutyCycle(0, 1000, 0); // Left wheel moves forward
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                turnOffAll();
            }
        break;
        case ROTATE_RIGHT_B:
            if (currentButtonState == BUTTON_HELD)
            {
                setPwmDutyCycle(1, 1023, 0); // Right wheel moves backwards
                waitMicrosecond(100000);
                setPwmDutyCycle(1, 1000, 0); // Right wheel moves backwards
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                turnOffAll();
            }
        break;
        case ROTATE_RIGHT_F:
            if (currentButtonState == BUTTON_HELD)
            {
                setPwmDutyCycle(1, 0, 1023); // Right Wheel moves forward
                waitMicrosecond(100000);
                setPwmDutyCycle(1, 0, 1000); // Right Wheel moves forward
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                turnOffAll();
            }
        break;

        case SPINNING_BOI_1:
            if (currentButtonState == BUTTON_HELD)
            {
                //setPwmDutyCycle(0, 0, 1023);
                //setPwmDutyCycle(1, 0, 1023);
                setDirectionOld(0, 0, 1023, 0, 1023);
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                turnOffAll();
            }
        break;
        case SPINNING_BOI_2:
            if (currentButtonState == BUTTON_HELD)
            {
                //setPwmDutyCycle(0, 1023, 0);
                //setPwmDutyCycle(1, 1023, 0);
                setDirectionOld(0, 1023, 0, 1023, 0);
            }
            else if (currentButtonState == BUTTON_RELEASED)
            {
                turnOffAll();
            }
        break;

        default:
            turnOffAll();
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

    setPinValue(OUT_ENABLE, 1);

    turnOffAll();

    printfUart0("\n\nInitialization Success\n\n");

    initI2c1();

    // Verify we can see the MPU-6050 (6-dof IMU) // b110100X // 01101000 -> 0x68
    bool confirmADDR = pollI2c1Address(0x68);

    printfUart0("\n\nPolling I2C Address");

    if(confirmADDR)
    {
        printfUart0("\n\nAddress is 0x68\n\n");
    } else {
        while(true){
            printfUart0("Address is NOT 0x68\n");
        }
    }

    waitMicrosecond(1000000);

    while (true)
    {
        if((WTIMER3_TAV_R / 40) > 200000)
        {
            currentButtonState = BUTTON_RELEASED;
        }

        handleButtonAction();
    }
}
