// Uart Conversion Library
// Zachary Trumbaturi

// Use this to convert any possible unit to a printable format for Uart0

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#define CHAR_BUFF_SIZE 80
char retVal[80];

#define UPPERCASE 1
#define LOWERCASE 0


// This function should not be directly called.
// Instead, use the middleman functions below this function.
char* uint2strhex(uint32_t num, uint8_t size, bool uppercase)
{
    uint8_t i;
    for (i = CHAR_BUFF_SIZE; i > (CHAR_BUFF_SIZE - (size * 2)); i--)
    {
        uint8_t tempBits = (num >> (4 * (CHAR_BUFF_SIZE - i))) & 0xF;
        switch(tempBits)
        {
            case 0x0:
            case 0x1:
            case 0x2:
            case 0x3:
            case 0x4:
            case 0x5:
            case 0x6:
            case 0x7:
            case 0x8:
            case 0x9:
            {
                retVal[i-1] = tempBits + 48;
                break;
            }
            case 0xA:
            case 0xB:
            case 0xC:
            case 0xD:
            case 0xE:
            case 0xF:
            {
                if (uppercase)
                {
                    retVal[i - 1] = tempBits + 55;
                }
                else
                {
                    retVal[i - 1] = tempBits + 87;
                }
                break;
            }
        }
    }

    retVal[i - 1] = 'x';
    retVal[i - 2] = '0';

    return &retVal[i - 2];
}


char* uint8_to_strhex(uint8_t num, bool uppercase)
{
    return uint2strhex(num, 1, uppercase);
}

char* uint16_to_strhex(uint16_t num, bool uppercase)
{
    return uint2strhex(num, 2, uppercase);
}

char* uint32_to_strhex(uint32_t num, bool uppercase)
{
    return uint2strhex(num, 4, uppercase);
}


char* uint_to_str(uint32_t num)
{
    if (num == 0)
    {
        retVal[17] = '0';
        return &retVal[17];
    }
    uint8_t i;

    for (i = 17; num != 0; i--)
    {
        retVal[i] = (num % 10) + 48;
        num = num / 10;
    }
    return &retVal[i + 1];
}

char* int_to_str(int num)
{
    bool negative;
    if (num < 0)
    {
        negative = true;
        num = num * -1;
    }
    else if (num > 0)
        negative = false;
    else
    {
        retVal[17] = '0';
        return &retVal[17];
    }

    uint8_t i;

    for (i = 17; num != 0; i--)
    {
        retVal[i] = (num % 10) + 48;
        num = num / 10;
    }

    if (negative)
    {
        retVal[i] = '-';
        return &retVal[i];
    }
    else
    {
        return &retVal[i + 1];
    }
}

// NOTE Only use if stack is high enough.
char* float_to_str(float* x)
{
    sprintf(retVal, "%f", *x);
    return retVal;
}


