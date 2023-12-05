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
#include <math.h>
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

#define PI 3.1415

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

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
    SPINNING_BOI_2 = 16751205,

    BALANCE        = 167124459, // remove the 9 to reactivate

    FORWARD_1M     = 16726725,
    BACK_1M        = 16759365
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

uint32_t data = 0;    // Stores the decoded IR data
uint8_t bitCount = 0; // Bit counter for the 32-bits of IR data

uint32_t lastDecodedData = 0; // Stores the last valid decoded data

uint32_t noSignalCounter = 0;

bool actionHeldExecuted = false;
bool actionReleasedExecuted = false;

bool goStraight = false;    // if true robot will go straight
bool goBalance = true;      // if true robot will balance

bool amRotate = false;

uint16_t leftWheelSpeed;
uint16_t rightWheelSpeed;
uint16_t currentDirection;

int32_t leftWheelOpticalInterrupt = 0;
int32_t rightWheelOpticalInterrupt = 0;

int32_t leftWheelDistanceTraveled = 0;
int32_t rightWheelDistanceTraveled = 0;

int16_t ax, ay, az, gx, gy, gz;
float fax, fay, faz, fgx, fgy, fgz;

float velocityX, velocityY, velocityZ;
float distanceX, distanceY, distanceZ;

float currentGyroRotation = 0.0; // Total rotation angle in degrees

bool isMovingCommand = false;

float distanceTraveledX = 0.0;
float distanceTraveledY = 0.0;
float distanceTraveledZ = 0.0;
float lastAx = 0.0;
float lastAy = 0.0;
float lastAz = 0.0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void processDecodedData(uint32_t data);
void handleButtonAction(void);
void rotate(uint8_t degrees, bool direction);
void readMPU6050();

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

    ///*
    // Using WT2CCP0 for Timing stuff
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER2_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER2_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from negative edge to negative edge
    WTIMER2_TAV_R = 0;                               // zero counter for first period
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    //*/


    // Time in Seconds = (load / 40,000,000)
    // Hz = (40,000,000 / load)

    // Configure Timer 1 for PID controller (Balance)
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 1000000;                        // set load value to 40,000 for 1000 Hz interrupt rate // original was 4,000,000 = 10 Hz
                                                     // I think 5,000,000 would be good, 8 Hz
                                                     // 1,000,000 = 40 Hz = 25ms

    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

    // Configure Timer 2 for PID controller (Driving Straight)
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = 40000;                          // set load value to 40000 for 1000 Hz interrupt rate / once per ms
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);              // turn-on interrupt 39 (TIMER2A)
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
            if(pulseWidth >= 1 && pulseWidth <= 1200) // 0
            {
                data <<= 1;
                bitCount++;
            }
            else if(pulseWidth >= 1200 && pulseWidth <= 3050) // Adjusted tolerance for 1
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
                printfUart0("\nDecoded Data: %u\n", data);
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

void wideTimer3Isr()
{
    togglePinValue(GREEN_LED);
    IRdecoder();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

        case BALANCE:
            currentButtonAction = BALANCE;
        break;

        case FORWARD_1M:
            currentButtonAction = FORWARD_1M;
            leftWheelSpeed = 850;
            rightWheelSpeed = 850;
            currentDirection = 1;
        break;
        case BACK_1M:
            currentButtonAction = BACK_1M;
            leftWheelSpeed = 1000;
            rightWheelSpeed = 1000;
            currentDirection = 0;
        break;
        // Handle other buttons similarly when you have their decoded data
        default:
            currentButtonAction = NONE;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This is what is called from main
void handleButtonAction(void)
{
    switch(currentButtonAction)
    {
        case NONE:
            // We aint doin nothin brub
            //actionHeldExecuted = false;
            //actionReleasedExecuted = false;
            goStraight = false;

        break;

        case FORWARD_FAST:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                setDirection(currentDirection, leftWheelSpeed, rightWheelSpeed); // Both wheels go forwards
                goStraight = true;
                leftWheelOpticalInterrupt = 0;
                rightWheelOpticalInterrupt = 0;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                goStraight = false;
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
                leftWheelOpticalInterrupt = 0;
                rightWheelOpticalInterrupt = 0;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                goStraight = false;
                //slowDown(currentDirection, leftWheelSpeed, rightWheelSpeed);
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
                leftWheelOpticalInterrupt = 0;
                rightWheelOpticalInterrupt = 0;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                goStraight = false;
                //slowDown(currentDirection, leftWheelSpeed, rightWheelSpeed);
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
                leftWheelOpticalInterrupt = 0;
                rightWheelOpticalInterrupt = 0;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                goStraight = false;
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
                leftWheelOpticalInterrupt = 0;
                rightWheelOpticalInterrupt = 0;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                goStraight = false;
                //slowDown(currentDirection, leftWheelSpeed, rightWheelSpeed);
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
                //amRotate = true;
                setDirection(0, 1023, 1023); // Both wheels go backwards
                waitMicrosecond(100000);
                setDirection(currentDirection, leftWheelSpeed, rightWheelSpeed); // Both wheels go backwards
                goStraight = true;
                leftWheelOpticalInterrupt = 0;
                rightWheelOpticalInterrupt = 0;
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                goStraight = false;
                //amRotate = false;
                //slowDown(currentDirection, leftWheelSpeed, rightWheelSpeed);
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
                amRotate = true;
                setPwmDutyCycle(0, 0, 1023); // Left wheel moves backwards
                waitMicrosecond(100000);
                setPwmDutyCycle(0, 0, 850); // Left wheel moves backwards
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                amRotate = false;
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_LEFT_F:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                amRotate = true;
                setPwmDutyCycle(0, 1023, 0); // Left wheel moves forward
                waitMicrosecond(100000);
                setPwmDutyCycle(0, 850, 0); // Left wheel moves forward
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                amRotate = false;
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_RIGHT_B:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                amRotate = true;
                setPwmDutyCycle(1, 1023, 0); // Right wheel moves backwards
                waitMicrosecond(100000);
                setPwmDutyCycle(1, 850, 0); // Right wheel moves backwards
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                amRotate = false;
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_RIGHT_F:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                amRotate = true;
                setPwmDutyCycle(1, 0, 1023); // Right Wheel moves forward
                waitMicrosecond(100000);
                setPwmDutyCycle(1, 0, 850); // Right Wheel moves forward
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                amRotate = false;
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;

        case ROTATE_CW_90:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                amRotate = true;
                rotate(90, true);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                amRotate = false;
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_CW_180:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                amRotate = true;
                rotate(180, true);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                amRotate = false;
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_CCW_90:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                amRotate = true;
                rotate(90, false);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                amRotate = false;
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case ROTATE_CCW_180:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                amRotate = true;
                rotate(180, false);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                amRotate = false;
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;

        case SPINNING_BOI_1:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                amRotate = true;
                setDirectionOld(0, 0, 850, 0, 850);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                amRotate = false;
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;
        case SPINNING_BOI_2:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                amRotate = true;
                setDirectionOld(0, 850, 0, 850, 0);
                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                amRotate = false;
                turnOffAll();
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;

        case BALANCE:
            if (currentButtonState == BUTTON_HELD && !actionHeldExecuted)
            {
                // toggle a global variable to turn balance on/off for testing

                goBalance ^= 1;

                actionHeldExecuted = true;
                actionReleasedExecuted = false;
            }
            else if (currentButtonState == BUTTON_RELEASED && !actionReleasedExecuted)
            {
                actionReleasedExecuted = true;
                actionHeldExecuted = false;
            }
        break;

        case FORWARD_1M:
            if (!actionHeldExecuted)
            {
                printfUart0("Starting Moving 1 Meter Forward \n");
                //isMovingCommand = true;
                goStraight = true;
                goBalance = false;
                setDirection(currentDirection, leftWheelSpeed, rightWheelSpeed);
                WTIMER2_TAV_R = 0;
                actionHeldExecuted = true;
            }
            else
            {
                if ((WTIMER2_TAV_R/40000000) >= 2.0)
                {
                    printfUart0("Finished Moving 1 Meter Forward \n");
                    goBalance = true;
                    goStraight = false;
                    turnOffAll();
                    currentButtonAction = NONE;
                    actionHeldExecuted = false;
                }
            }
        break;
        case BACK_1M:
            if (!actionHeldExecuted)
            {
                printfUart0("Starting Moving 1 Meter Backward \n");
                //isMovingCommand = true;
                goStraight = true;
                goBalance = false;
                setDirection(currentDirection, leftWheelSpeed, rightWheelSpeed);
                WTIMER2_TAV_R = 0;
                actionHeldExecuted = true;
            }
            else
            {
                if ((WTIMER2_TAV_R/40000000) >= 2.0)
                {
                    printfUart0("Finished Moving 1 Meter Backward \n");
                    goBalance = true;
                    goStraight = false;
                    turnOffAll();
                    currentButtonAction = NONE;
                    actionHeldExecuted = false;
                }
            }
        break;

        default:
            turnOffAll();
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

volatile float currentRotation = 0.0;

void rotate(uint8_t degrees, bool direction) {
    // Reset rotation angle
    currentRotation = 0.0;

    amRotate = true;

    if (direction) {
        setDirectionOld(0, 0, 1000, 0, 1000);
        currentGyroRotation -= degrees; //CCW
        degrees -= 10;
    } else {
        setDirectionOld(0, 1000, 0, 1000, 0);
        currentGyroRotation += degrees; // CW
        degrees -= 11;
    }

    printfUart0("currentGyroRotation = %f \n", &currentGyroRotation);

    //uint32_t startTime = (WTIMER2_TAV_R/40);
    //uint32_t timeout = 2000000;

    // Wait until the desired angle is reached
    while (fabs(currentRotation) < (degrees/2))
    {
        /*
        if (((WTIMER2_TAV_R/40) - startTime) > timeout) {
            printfUart0("Rotation timeout\n");
            break; // Break out of the loop if timeout is reached
        }
        */
        //printfUart0("currentRotation = %f \n", &currentRotation);
        //waitMicrosecond(10000); // 10ms
    }

    turnOffAll();
    amRotate = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Left Wheel // OPB876N55 Optical Interrupter // PC6 // WT1CCP0
void wideTimer1Isr()
{
    leftWheelOpticalInterrupt++;
    leftWheelDistanceTraveled = leftWheelOpticalInterrupt;
    //printfUart0("left Wheel Optical Interrupt:  %d \n", leftWheelOpticalInterrupt);
    if(leftWheelOpticalInterrupt == 40) // 40 tabs on wheel // 1 tab detected = 1 cm
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float coeffKp = 2.5;  // Proportional coefficient // 2.5 is okay
float coeffKi = 0.5; // Integral coefficient // should get me most of the way there // should be 1/100th to maybe 1/20th of kp
float coeffKd = 0;   // Derivative coefficient

int32_t integral = 0;
int32_t iMax = 100; // 100

int32_t prevLeftWheelOpticalInterrupt = 0;

// Configure Timer 2 for PID controller (Driving Straight)
void pidISR()
{
    static float lastGyroError = 0;
    float gyroError;
    int32_t output;
    int32_t newLeftSpeed;
    int32_t newRightSpeed;

    // Error is the rate of rotation around z axis
    gyroError = fgz; // deg/sec

    // deadband?
    if (fabs(gyroError) < 5)
    {
        //gyroError = 0;
    }

    integral += gyroError;
    if (integral > iMax) integral = iMax;
    if (integral < -iMax) integral = -iMax;

    float derivative = gyroError - lastGyroError;
    output = coeffKp * gyroError + coeffKi * integral + coeffKd * derivative;

    if (currentDirection == 1)
    {
        newLeftSpeed = leftWheelSpeed + output;
        newRightSpeed = rightWheelSpeed - output;
    } else {
        newLeftSpeed = leftWheelSpeed - output;
        newRightSpeed = rightWheelSpeed + output;
    }

    newLeftSpeed = MAX(MIN(newLeftSpeed, MAX_SPEED), MIN_SPEED);
    newRightSpeed = MAX(MIN(newRightSpeed, MAX_SPEED), 500);

    if (goStraight == true)
    {
        setDirection(currentDirection, newLeftSpeed, newRightSpeed);
        /*
        printfUart0("ax: %f  ay: %f  az: %f  gx: %f  gy: %f  gz: %f\n", &fax, &fay , &faz, &fgx, &fgy, &fgz);

        printfUart0("Left = %d   Right = %d   ", newLeftSpeed, newRightSpeed);
        printfUart0("Error = %f   LastError = %f   Integral = %d   ", &gyroError, &lastGyroError, integral);
        printfUart0("derivative = %f   output = %d \n", &derivative, output);
        //waitMicrosecond(100000);
        //printfUart0("LWOI = %d   RWOI = %d\n", leftWheelOpticalInterrupt, rightWheelOpticalInterrupt);
        */
    }

    lastGyroError = gyroError;

    // Clear timer interrupt
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readMPU6050()
{
    uint8_t data[14];
    readI2c1Registers(MPU6050, 0x3B, data, 14);

    ax = (data[0] << 8) | data[1];
    ay = (data[2] << 8) | data[3];
    az = (data[4] << 8) | data[5];

    // data[6] and data[7] are temperature, not needed for now

    gx = (data[8] << 8) | data[9];
    gy = (data[10] << 8) | data[11];
    gz = (data[12] << 8) | data[13];

    // Convert to g
    fax = (ax/16384.0);
    fay = (ay/16384.0);
    faz = (az/16384.0);

    fgx = (gx/131.0);
    fgy = (gy/131.0);
    fgz = (gz/131.0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float balanceKp = 2; // Proportional coefficient
float balanceKi = 0; // Integral coefficient // should get me most of the way there // should be 1/100th to maybe 1/20th of kp
float balanceKd = 0; // Derivative coefficient

int32_t balanceIntegral = 0;
int32_t balanceiMax = 100; // 100

float calculateTiltAngle()
{
    return atan2(fax, faz) * 180.0 / PI;
}

// Configure Timer 1 for PID controller (Balance)
void balancePID()
{
    static int32_t balanceLastError = 0;

    readMPU6050();

    currentRotation += fgz * 0.025; // 25ms

    float tiltAngle = calculateTiltAngle();
    int32_t error = 0 - tiltAngle; // Desired angle is 0

    balanceIntegral += error;
    if (balanceIntegral > balanceiMax) balanceIntegral = balanceiMax;
    if (balanceIntegral < -balanceiMax) balanceIntegral = -balanceiMax;

    int32_t derivative = error - balanceLastError;

    int32_t output = balanceKp * error + balanceKi * balanceIntegral + balanceKd * derivative;

    // Base speed for balancing, may need to tweak this
    int32_t baseSpeed = 800;

    if(goStraight == false)
    {
        leftWheelSpeed = baseSpeed;
        rightWheelSpeed = baseSpeed;
    }

    bool direction = (tiltAngle < 0); // Forward for negative tilt, backward for positive tilt

    // Adjust wheel speeds based on PID output
    int32_t newLeftSpeed = (direction ? leftWheelSpeed + output : leftWheelSpeed - output);
    int32_t newRightSpeed = (direction ? rightWheelSpeed + output : rightWheelSpeed - output);

    newLeftSpeed = MAX(MIN(newLeftSpeed, MAX_SPEED), MIN_SPEED);
    newRightSpeed = MAX(MIN(newRightSpeed, MAX_SPEED), MIN_SPEED);

    float balanceThreshold = 20.0; // Adjust
    if (((fabs(tiltAngle) < balanceThreshold) || (fabs(tiltAngle) > 80)) && (isMovingCommand == false)) // the robot seems to currently tilt a bit forward when balanced so maybe change the conditions here
    {
        newLeftSpeed = 0; // Turn off motors when balanced
        newRightSpeed = 0; // Turn off motors when balanced
    }
    if ((goBalance == true) && (amRotate == false))
    {
        setDirection(direction, newLeftSpeed, newRightSpeed);
        /*
        printfUart0("ax: %f  ay: %f  az: %f  gx: %f  gy: %f  gz: %f  Tilt Angle = %f\n", &fax, &fay , &faz, &fgx, &fgy, &fgz, &tiltAngle);

        printfUart0("Left = %d   Right = %d   ", newLeftSpeed, newRightSpeed);
        printfUart0("Error = %d   LastError = %d   Integral = %d   ", error, balanceLastError, balanceIntegral);
        printfUart0("derivative = %d   output = %d\n", derivative, output);
        waitMicrosecond(100000);
        */
        //printfUart0("Left = %d   Right = %d\n", newLeftSpeed, newRightSpeed);
    }

    balanceLastError = error;

    // Clear timer interrupt
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Initialize and Configure MPU-6050
void initMPU6050()
{
    // Wake up the MPU6050 - write 0
    writeI2c1Register(MPU6050, 0x6B, 0x00);
    waitMicrosecond(10000);

    // Set sensitivity to 2g
    // 0x00 for +/- 2g
    // 0x08 for +/- 4g
    // 0x10 for +/- 8g
    // 0x18 for +/- 16g
    writeI2c1Register(MPU6050, 0x1C, 0x00);

    // Set sensitivity to 250 degrees/second
    // 0x00 for +/- 250 deg/sec         = (250 / 360) * 60  = 41.6667  RPM
    // 0x08 for +/- 500 deg/sec         = (500 / 360) * 60  = 83.3333  RPM
    // 0x10 for +/- 1000 deg/sec        = (1000 / 360) * 60 = 166.6667 RPM
    // 0x18 for +/- 2000 deg/sec        = (2000 / 360) * 60 = 333.3333 RPM
    writeI2c1Register(MPU6050, 0x1B, 0x00);
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
    initMPU6050();

    while (true)
    {
        if((WTIMER3_TAV_R / 40) > 200000)
        {
            currentButtonState = BUTTON_RELEASED;
            //currentButtonAction = NONE; // this breaks the code

            //leftWheelOpticalInterrupt = 0;
            //rightWheelOpticalInterrupt = 0;
            //goStraight = false;
        }

        handleButtonAction();

        //waitMicrosecond(1000000);
    }
}
