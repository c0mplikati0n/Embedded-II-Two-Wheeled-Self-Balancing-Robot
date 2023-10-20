// UART0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef UART0_H_
#define UART0_H_

// General Defines
#define MAX_CHARS 80
#define MAX_FIELDS 5

// Structs
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint32_t fieldCount;
    uint32_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart0();
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart0(char c);
void putsUart0(char* str);
char getcUart0();
bool kbhitUart0();

char* itostr(uint32_t number);
void getsUart0(USER_DATA* data);
void parseFields(USER_DATA* data);
bool customStrcmp (const char strCommand[], const char argument[]);
char* getFieldString(USER_DATA* data, uint32_t fieldNumber);
int32_t getFieldInteger(USER_DATA* data, uint32_t fieldNumber);
double getFieldDouble(USER_DATA* data, uint32_t fieldNumber);
bool  isCommand(USER_DATA* data, const char strCommand[], uint32_t minArguments);
void printfUart0(char* format, ...);

//void enableCounterMode();    //Frequency Enable
//void disableCounterMode();   // Frequency Disable
//void enableTimerMode();      // Time Enable
//void disableTimerMode();     // Time Disable

#endif
