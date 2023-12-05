// Custom Functions
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
#include "customFunctions.h"
#include "gpio.h"

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
// ???
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void readNprintMPU6050(void)
{
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050(&ax, &ay, &az, &gx, &gy, &gz); // Read MPU6050 data

    //float tiltAngle = calculateTiltAngle(ax, ay, az); // Calculate the tilt angle
    float tiltAngle = calculateTiltAngle(ax, ay, az); // Calculate the tilt angle

    float fax = (ax/16384.0);
    float fay = (ay/16384.0);
    float faz = (az/16384.0);

    float fgx = (gx/131.0);
    float fgy = (gy/131.0);
    float fgz = (gz/131.0);

    printfUart0("Read Data ax =    %f\n", &fax);
    printfUart0("Read Data ay =    %f\n", &fay);
    printfUart0("Read Data az =    %f\n\n", &faz);

    printfUart0("Read Data gx =    %f\n", &fgx);
    printfUart0("Read Data gy =    %f\n", &fgy);
    printfUart0("Read Data gz =    %f\n\n", &fgz);

    printfUart0("tilt Angle   =    %f\n\n", &tiltAngle);
}

// 250 deg/sec
//fgx = (gx/131.0);
//fgy = (gy/131.0);
//fgz = (gz/131.0);

// 1000 deg/sec
//fgx = (gx/32.8);
//fgy = (gy/32.8);
//fgz = (gz/32.8);

// 2000 deg/sec
//fgx = (gx/16.4);
//fgy = (gy/16.4);
//fgz = (gz/16.4);

/*
float coeffKp = 10;  // Proportional coefficient
float coeffKi = .06; // Integral coefficient // should get me most of the way there // should be 1/100th to maybe 1/20th of kp
float coeffKd = 0;   // Derivative coefficient

int32_t integral = 0;
int32_t iMax = 100; // 100

int32_t prevLeftWheelOpticalInterrupt = 0;

void pidISR()
{
    static int32_t lastError = 0;
    static int32_t lastLeftWheelInterrupt = 0;
    static int32_t lastRightWheelInterrupt = 0;

    /
    if (leftWheelOpticalInterrupt - prevLeftWheelOpticalInterrupt > 4) {
        leftWheelOpticalInterrupt = prevLeftWheelOpticalInterrupt;
    } else {
        prevLeftWheelOpticalInterrupt = leftWheelOpticalInterrupt;
    }

    if (leftWheelOpticalInterrupt > (rightWheelOpticalInterrupt + 10)) {
        leftWheelOpticalInterrupt = rightWheelOpticalInterrupt;
    }
    /

    // Calculate the difference in the number of interrupts since the last check
    int32_t leftWheelInterruptDelta = leftWheelOpticalInterrupt - lastLeftWheelInterrupt;
    int32_t rightWheelInterruptDelta = rightWheelOpticalInterrupt - lastRightWheelInterrupt;

    // Error is the difference in distance traveled by each wheel
    int32_t error = leftWheelInterruptDelta - rightWheelInterruptDelta;

    integral += error;
    if (integral > iMax) integral = iMax;
    if (integral < -iMax) integral = -iMax;

    int32_t derivative = error - lastError;

    int32_t output = coeffKp * error + coeffKi * integral + coeffKd * derivative;

    // Adjusting the motor speed based on PID output
    int32_t newLeftSpeed = leftWheelSpeed + output;
    int32_t newRightSpeed = rightWheelSpeed - output;

    newLeftSpeed = MAX(MIN(newLeftSpeed, MAX_SPEED), MIN_SPEED);
    newRightSpeed = MAX(MIN(newRightSpeed, MAX_SPEED), MIN_SPEED);

    // Apply new speeds
    if (goStraight == true)
    {
        setDirection(currentDirection, newLeftSpeed, newRightSpeed);

        // Debugging prints
        /
        printfUart0("Left = %d   Right = %d   ", newLeftSpeed, newRightSpeed);
        printfUart0("Error = %d   LastError = %d   Integral = %d   ", error, lastError, integral);
        printfUart0("derivative = %d   output = %d   ", derivative, output);
        printfUart0("LWOI = %d   RWOI = %d\n", leftWheelOpticalInterrupt, rightWheelOpticalInterrupt);
        /
    }

    // Save the current state for the next iteration
    lastLeftWheelInterrupt = leftWheelOpticalInterrupt;
    lastRightWheelInterrupt = rightWheelOpticalInterrupt;
    lastError = error;

    // Clear timer interrupt
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}
 */

