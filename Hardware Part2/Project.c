
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
//#include "irDecoder.h"

// Pins
//#define I2C1SCL         PORTA, 6 // I2C 1 SCL
//#define I2C1SDA         PORTA, 7 // I2C 1 SDA

#define OUT_ENABLE      PORTE, 0 // P30

#define OUT_PWM_1       PORTC, 4 // M0PWM6
#define OUT_PWM_2       PORTC, 5 // M0PWM7
#define OUT_PWM_3       PORTB, 6 // M0PWM0
#define OUT_PWM_4       PORTB, 7 // M0PWM1

#define TIMER_IN_L      PORTC, 6 // WT1CCP0
#define TIMER_IN_R      PORTD, 6 // WT5CCP0

#define TIMER_IN_IR     PORTD, 2 // WT3CCP0

#define RED_LED         PORTF, 1
#define GREEN_LED       PORTF, 3
#define BLUE_LED        PORTF, 2

#define PB_1            PORTF, 4
#define PB_2            PORTF, 0

#define MPU6050         0x68  // 110 1000 = 0x68 = ADDR is logic low

#define MAX_SPEED 1023
#define MIN_SPEED 850

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

void rotate(uint8_t degrees, bool direction);

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

typedef enum
{
    NEC_IDLE,
    NEC_START,
    NEC_DATA
} NEC_State;

// LED Remote
typedef enum
{
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

    ROTATE_CW_90   = 16722645,
    ROTATE_CW_180  = 16714485,

    ROTATE_CCW_90  = 16755285,
    ROTATE_CCW_180 = 16747125,

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

uint32_t lastTime = 0; // Last captured time
uint32_t pulseWidth = 0;

ButtonAction currentButtonAction = NONE;
ButtonState currentButtonState = BUTTON_RELEASED;
NEC_State currentState = NEC_IDLE;

uint32_t data = 0; // Stores the decoded data
uint8_t bitCount = 0; // Bit counter for the 32-bits of data

uint32_t lastDecodedData = 0; // Stores the last valid decoded data

uint32_t noSignalCounter = 0;

bool actionHeldExecuted = false;
bool actionReleasedExecuted = false;

bool goStraight = false;

uint16_t leftWheelSpeed;
uint16_t rightWheelSpeed;
uint16_t currentDirection;

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
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0 | SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R2 | SYSCTL_RCGCWTIMER_R3 | SYSCTL_RCGCWTIMER_R4 | SYSCTL_RCGCWTIMER_R5;
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
    setPinAuxFunction(TIMER_IN_L, GPIO_PCTL_PC6_WT1CCP0);

    selectPinDigitalInput(TIMER_IN_R);
    setPinAuxFunction(TIMER_IN_R, GPIO_PCTL_PD6_WT5CCP0);

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
    // IR sensor // TSOP13438 38kHz AGC4
    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER3_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER3_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER3_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER3_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER3_TAV_R = 0;                               // zero counter for first period
    WTIMER3_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER3A-16-96);         // turn-on interrupt 116 (WTIMER3A)

    // Left Wheel // OPB876N55 Optical Interrupter // PC6 // WT1CCP0
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_NEG;           // measure time from negative edge to negative edge
    WTIMER1_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER1A-16-96);         // turn-on interrupt 112 (WTIMER1A)

    // Right Wheel // OPB876N55 Optical Interrupter // PD6 // WT5CCP0
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_NEG;           // measure time from negative edge to negative edge
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER5A-16-96);         // turn-on interrupt 120 (WTIMER5A)

    // Configure Timer 2 for PID controller
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = 4000;                          // set load value to 40000 for 1000 Hz interrupt rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);              // turn-on interrupt 39 (TIMER2A)
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
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

                actionHeldExecuted = false;
                actionReleasedExecuted = false;
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
            leftWheelSpeed = 1023;
            rightWheelSpeed = 1023;
            currentDirection = 1;
        break;
        case FORWARD_NORMAL:
            currentButtonAction = FORWARD_NORMAL;
            leftWheelSpeed = 900;
            rightWheelSpeed = 900;
            currentDirection = 1;
        break;
        case FORWARD_SLOW:
            currentButtonAction = FORWARD_SLOW;
            leftWheelSpeed = 850;
            rightWheelSpeed = 850;
            currentDirection = 1;
        break;

        case BACK_FAST:
            currentButtonAction = BACK_FAST;
            leftWheelSpeed = 1023;
            rightWheelSpeed = 1023;
            currentDirection = 0;
        break;
        case BACK_NORMAL:
            currentButtonAction = BACK_NORMAL;
            leftWheelSpeed = 900;
            rightWheelSpeed = 900;
            currentDirection = 0;
        break;
        case BACK_SLOW:
            currentButtonAction = BACK_SLOW;
            leftWheelSpeed = 850;
            rightWheelSpeed = 850;
            currentDirection = 0;
        break;

        case ROTATE_LEFT_B:
            currentButtonAction = ROTATE_LEFT_B;
            leftWheelSpeed = 850;
            rightWheelSpeed = 850;
        break;
        case ROTATE_LEFT_F:
            currentButtonAction = ROTATE_LEFT_F;
            leftWheelSpeed = 850;
            rightWheelSpeed = 850;
        break;

        case ROTATE_RIGHT_B:
            currentButtonAction = ROTATE_RIGHT_B;
            leftWheelSpeed = 850;
            rightWheelSpeed = 850;
        break;
        case ROTATE_RIGHT_F:
            currentButtonAction = ROTATE_RIGHT_F;
            leftWheelSpeed = 850;
            rightWheelSpeed = 850;
        break;

        case ROTATE_CW_90:
            currentButtonAction = ROTATE_CW_90;
        break;
        case ROTATE_CW_180:
            currentButtonAction = ROTATE_CW_180;
        break;
        case ROTATE_CCW_90:
            currentButtonAction = ROTATE_CCW_90;
        break;
        case ROTATE_CCW_180:
            currentButtonAction = ROTATE_CCW_180;
        break;

        case SPINNING_BOI_1:
            currentButtonAction = SPINNING_BOI_1;
            leftWheelSpeed = 900;
            rightWheelSpeed = 900;
        break;
        case SPINNING_BOI_2:
            currentButtonAction = SPINNING_BOI_2;
            leftWheelSpeed = 900;
            rightWheelSpeed = 900;
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
            actionHeldExecuted = false;
            actionReleasedExecuted = false;
        break;

        case FORWARD_FAST:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setDirection(currentDirection, leftWheelSpeed, rightWheelSpeed); // Both wheels go forwards
                goStraight = true;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                slowDown(1, 1023, 1023);
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
                //currentButtonState = BUTTON_RELEASED;
                //currentButtonAction = NONE;
            }
        break;
        case FORWARD_NORMAL:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setDirection(1, 1023, 1023); // Both wheels go forwards
                waitMicrosecond(100000);
                setDirection(currentDirection, leftWheelSpeed, rightWheelSpeed); // Both wheels go forwards
                goStraight = true;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
                //currentButtonState = BUTTON_RELEASED;
                                //currentButtonAction = NONE;
            }
        break;
        case FORWARD_SLOW:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setDirection(1, 1023, 1023); // Both wheels go forwards
                waitMicrosecond(100000);
                setDirection(currentDirection, leftWheelSpeed, rightWheelSpeed); // Both wheels go forwards
                goStraight = true;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
                //currentButtonState = BUTTON_RELEASED;
                                //currentButtonAction = NONE;
            }
        break;

        case BACK_FAST:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setDirection(currentDirection, leftWheelSpeed, rightWheelSpeed); // Both wheels go backwards
                goStraight = true;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                slowDown(0, 1023, 1023);

                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
                //currentButtonState = BUTTON_RELEASED;
                                //currentButtonAction = NONE;
            }
        break;
        case BACK_NORMAL:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setDirection(0, 1023, 1023); // Both wheels go backwards
                waitMicrosecond(100000);
                setDirection(currentDirection, leftWheelSpeed, rightWheelSpeed); // Both wheels go backwards
                goStraight = true;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
                //currentButtonState = BUTTON_RELEASED;
                                //currentButtonAction = NONE;
            }
        break;
        case BACK_SLOW:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setDirection(0, 1023, 1023); // Both wheels go backwards
                waitMicrosecond(100000);
                setDirection(currentDirection, leftWheelSpeed, rightWheelSpeed); // Both wheels go backwards
                goStraight = true;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
                //currentButtonState = BUTTON_RELEASED;
                                //currentButtonAction = NONE;
            }
        break;

        case ROTATE_LEFT_B:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setPwmDutyCycle(0, 0, 1023); // Left wheel moves backwards
                waitMicrosecond(100000);
                setPwmDutyCycle(0, 0, 850); // Left wheel moves backwards
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_LEFT_F:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setPwmDutyCycle(0, 1023, 0); // Left wheel moves forward
                waitMicrosecond(100000);
                setPwmDutyCycle(0, 850, 0); // Left wheel moves forward
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_RIGHT_B:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setPwmDutyCycle(1, 1023, 0); // Right wheel moves backwards
                waitMicrosecond(100000);
                setPwmDutyCycle(1, 850, 0); // Right wheel moves backwards
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_RIGHT_F:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setPwmDutyCycle(1, 0, 1023); // Right Wheel moves forward
                waitMicrosecond(100000);
                setPwmDutyCycle(1, 0, 850); // Right Wheel moves forward
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;

        case ROTATE_CW_90:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                rotate(90, true);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_CW_180:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                rotate(180, true);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_CCW_90:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                rotate(90, false);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_CCW_180:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                rotate(180, false);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;

        case SPINNING_BOI_1:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setDirectionOld(0, 0, 900, 0, 900);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case SPINNING_BOI_2:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setDirectionOld(0, 900, 0, 900, 0);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;

        default:
            turnOffAll();
        break;
    }
}

void rotate(uint8_t degrees, bool direction)
{
    //uint32_t wheelRotationsNeeded = calculateRotationsForDegrees(degrees);
    if(direction) //CW
    {
        //setWheelSpeed(leftWheel, wheelRotationsNeeded * wheelSpeedPerRotation);
        //setWheelSpeed(rightWheel, -wheelRotationsNeeded * wheelSpeedPerRotation);
    }
    else // COUNTERCLOCKWISE
    {
        //setWheelSpeed(leftWheel, -wheelRotationsNeeded * wheelSpeedPerRotation);
        //setWheelSpeed(rightWheel, wheelRotationsNeeded * wheelSpeedPerRotation);
    }
    //waitUntilRotationComplete(); // You need to implement this based on your sensors
}


int32_t leftWheelOpticalInterrupt = 0;
int32_t rightWheelOpticalInterrupt = 0;

int32_t leftWheelDistanceTraveled = 0;
int32_t rightWheelDistanceTraveled = 0;

// Left Wheel // OPB876N55 Optical Interrupter // PC6 // WT1CCP0
void wideTimer1Isr()
{
    leftWheelOpticalInterrupt++;
    leftWheelDistanceTraveled = leftWheelOpticalInterrupt;
    //printfUart0("left Wheel Optical Interrupt:  %d \n", leftWheelOpticalInterrupt);
    if(rightWheelOpticalInterrupt == 40) // 40 tabs on wheel // 1 tab detected = 1 cm
    {
        //rightWheelOpticalInterrupt = 0;
        //printfUart0("left Wheel Full Rotation\n");
    }
    // 1m = 100cm = 2FullRotation + 20cm
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

// Right Wheel // OPB876N55 Optical Interrupter // PD6 // WT5CCP0 // 1 tab detected = 1 cm
void wideTimer5Isr()
{
    rightWheelOpticalInterrupt++;
    rightWheelDistanceTraveled = rightWheelOpticalInterrupt;
    //printfUart0("Right Wheel Optical Interrupt: %d \n", rightWheelOpticalInterrupt);
    if(rightWheelOpticalInterrupt == 40) // 40 tabs on wheel
    {
        //rightWheelOpticalInterrupt = 0;
        //printfUart0("Right Wheel Full Rotation\n");
    }
    // 1m = 100cm = 2FullRotation + 20cm
    WTIMER5_ICR_R = TIMER_ICR_CAECINT;
}

/*
void goStraightISR()
{
    if((currentButtonState != BUTTON_RELEASED) && (goStraight == true))
    {
        int interruptDifference = leftWheelOpticalInterrupt - rightWheelOpticalInterrupt;

        // Adjust the speed of the wheels based on the difference in interrupts
        if (interruptDifference > 5)
        {
            // Left wheel has more interrupts, slow it down
            setDirection(currentDirection, (leftWheelSpeed - 100), rightWheelSpeed);
        }
        else if (interruptDifference < 0)
        {
            // Right wheel has more interrupts, slow it down
            setDirection(currentDirection, leftWheelSpeed, (rightWheelSpeed - 100));
        }
        // If interruptDifference is 0, both wheels are moving at the same speed, so no change is needed
    }
    TIMER2_ICR_R = TIMER_ICR_TATOCINT; // Clear timer interrupt
}
*/

// pid calculation of u
float coeffKp = 1; // Proportional coefficient
float coeffKi = 0.1; // Integral coefficient // should get me most of the way there // should be 1/100th to maybe 1/20th of kp
float coeffKd = 0; // Derivative coefficient

int32_t coeffKo = 0;
//int32_t coeffK = 100; // denominator used to scale Kp, Ki, and Kd
int32_t integral = 0;
int32_t iMax = 100; // 100
int32_t diff;
int32_t error;
int32_t u = 0;
int32_t deadBand = 0;

int32_t prevLeftWheelOpticalInterrupt = 0;

void pidISR()
{
    static int32_t lastError = 0;

    if (leftWheelOpticalInterrupt - prevLeftWheelOpticalInterrupt > 3) {
        leftWheelOpticalInterrupt = prevLeftWheelOpticalInterrupt;
    } else {
        prevLeftWheelOpticalInterrupt = leftWheelOpticalInterrupt;
    }

    // Calculate error (difference in wheel rotations)
    error = leftWheelOpticalInterrupt - rightWheelOpticalInterrupt;

    if( error > 0 )
    {
      // left motor is turning faster
      //Serial.println("Turning Right");
    }
    else if( error < 0 )
    {
      // right motor is turning faster
      //Serial.println("Turning Left");
    }

    // Integral term with windup guard
    integral += error;
    if (integral > iMax) integral = iMax;
    if (integral < -iMax) integral = -iMax;

    // Derivative term
    int32_t derivative = error - lastError;

    // PID calculation
    int32_t output = (coeffKp * error) + (coeffKi * integral) + (coeffKd * derivative);

    // Scale down the output to avoid large jumps in speed
    //output = output / 10; // Adjust this scaling factor as needed

    // Speed limit checks
    int32_t newLeftSpeed = leftWheelSpeed - output;
    int32_t newRightSpeed = rightWheelSpeed + output;

    //newLeftSpeed = MAX(MIN(newLeftSpeed, MAX_SPEED), MIN_SPEED);
    //newRightSpeed = MAX(MIN(newRightSpeed, MAX_SPEED), MIN_SPEED);

    newLeftSpeed = (newLeftSpeed > MAX_SPEED) ? MAX_SPEED : ((newLeftSpeed < MIN_SPEED) ? MIN_SPEED : newLeftSpeed);
    newRightSpeed = (newRightSpeed > MAX_SPEED) ? MAX_SPEED : ((newRightSpeed < MIN_SPEED) ? MIN_SPEED : newRightSpeed);

    // Apply new speeds
    if (goStraight == true)
    {
        setDirection(currentDirection, newLeftSpeed, newRightSpeed);
        ///*
        printfUart0("Left = %d   Right = %d   ", newLeftSpeed, newRightSpeed);
        printfUart0("Error = %d   LastError = %d   Integral = %d   ", error, lastError, integral);
        printfUart0("derivative = %d   output = %d   ", derivative, output);
        printfUart0("LWOI = %d   RWOI = %d\n", leftWheelOpticalInterrupt, rightWheelOpticalInterrupt);
        ///*/

    }

    // Prepare for next iteration
    lastError = error;

    // Clear timer interrupt
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}



// MPU6050 Registers

#define MPU6050_ADDRESS 0x68

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_SENSITIVITY 0x1C
#define MPU6050_GYRO_SENSITIVITY 0x1B

#define MPU6050_OUT_TEMP_H 0x41
#define MPU6050_OUT_TEMP_L 0x42

#define MPU6050_OUTX_H_G 0x43
#define MPU6050_OUTX_L_G 0x44
#define MPU6050_OUTY_H_G 0x45
#define MPU6050_OUTY_L_G 0x46
#define MPU6050_OUTZ_H_G 0x47
#define MPU6050_OUTZ_L_G 0x48

#define MPU6050_OUTX_H_XL 0x3B
#define MPU6050_OUTX_L_XL 0x3C
#define MPU6050_OUTY_H_XL 0x3D
#define MPU6050_OUTY_L_XL 0x3E
#define MPU6050_OUTZ_H_XL 0x3F
#define MPU6050_OUTZ_L_XL 0x40

#define PI 3.1415

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43

void readMPU6050(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t data[14];
    readI2c1Registers(MPU6050, MPU6050_ACCEL_XOUT_H, data, 14);

    *ax = (data[0] << 8) | data[1];
    *ay = (data[2] << 8) | data[3];
    *az = (data[4] << 8) | data[5];
    *gx = (data[8] << 8) | data[9];
    *gy = (data[10] << 8) | data[11];
    *gz = (data[12] << 8) | data[13];
}

void readMPU6050Data()
{
    int16_t ax, ay, az, gx, gy, gz;

    //uint8_t readMPU = readI2c1Data(MPU6050);
    readMPU6050(&ax, &ay, &az, &gx, &gy, &gz);

    printfUart0("Read Data ax =    %d\n", ax);
    printfUart0("Read Data ay =    %d\n", ay);
    printfUart0("Read Data az =    %d\n", az);
    printfUart0("Read Data gx =    %d\n", gx);
    printfUart0("Read Data gy =    %d\n", gy);
    printfUart0("Read Data gz =    %d\n\n", gz);
    //printfUart0("Read Data =    %u\n", readMPU);
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
    bool confirmADDR = pollI2c1Address(MPU6050);
    printfUart0("\n\nPolling I2C Address");
    if(confirmADDR)
    {
        printfUart0("\n\nAddress is 0x68\n\n");
    } else {
        while(true){
            printfUart0("Address is NOT 0x68\n");
        }
    }

    // START condition (S) on the bus, which is defined as a HIGH-to-LOW transition of the SDA line while SCL line is HIGH
    //writeI2c1Data(MPU6050, 1);
    writeI2c1Register(MPU6050, MPU6050_PWR_MGMT_1, 0x00);
    waitMicrosecond(10000);

    // The bus is considered to be busy until the master puts

    // A STOP condition (P) on the bus is defined as a LOW to HIGH transition on the SDA line while SCL is HIGH
    //writeI2c1Data(MPU6050, 0);
    //waitMicrosecond(10000);

    waitMicrosecond(1000000);

    while (true)
    {
        if((WTIMER3_TAV_R / 40) > 200000)
        {
            currentButtonState = BUTTON_RELEASED;
            //currentButtonAction = NONE; // this breaks the code

            //printfUart0("Button Released\n");
            //actionHeldExecuted = false;
            //actionReleasedExecuted = false;

            leftWheelOpticalInterrupt = 0;
            rightWheelOpticalInterrupt = 0;
            goStraight = false;

            //pidcount = 0;
        }

        handleButtonAction();

        //readMPU6050Data();
        //waitMicrosecond(1000000);
    }
}
