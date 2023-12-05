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



