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

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <stdint.h>

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initPWM(void);

void setPwmDutyCycle(uint8_t side, uint16_t pwmA, uint16_t pwmB);
void setDirectionOld(uint8_t side, uint16_t pwmAL, uint16_t pwmBL, uint16_t pwmAR, uint16_t pwmBR);
void setDirection(uint8_t direction, uint16_t pwmL, uint16_t pwmR);
void turnOffAll(void);

#endif
