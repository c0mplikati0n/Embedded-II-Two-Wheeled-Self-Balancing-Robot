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
char* uint2strhex(uint32_t num, uint8_t size, bool uppercase);

char* uint8_to_strhex(uint8_t num, bool uppercase);

char* uint16_to_strhex(uint16_t num, bool uppercase);

char* uint32_to_strhex(uint32_t num, bool uppercase);


char* uint_to_str(uint32_t num);

char* int_to_str(int num);

// NOTE Only use if stack is high enough.
char* float_to_str(float* x);


