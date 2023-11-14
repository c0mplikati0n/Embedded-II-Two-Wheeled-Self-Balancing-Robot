// IR Decoder Library
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

#ifndef IRDECODER_H_
#define IRDECODER_H_

#include <stdint.h>

// General Defines


// Structs
typedef enum
{
    NEC_IDLE,
    NEC_START,
    NEC_DATA
} NEC_State;

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

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------



#endif
