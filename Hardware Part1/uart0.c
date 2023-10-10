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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart0()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART0
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart0()
{
    return !(UART0_FR_R & UART_FR_RXFE);
}

//------------------------------------------------------------------------------------------------------------------------------------

char* itostr(uint32_t number)
{
    uint32_t i = 0;
    uint32_t j = 0;
    //char string[MAX_CHARS];
    char string[MAX_CHARS] = '\0';
    char temp = '\0';

    do
    {
        string[j] = (number % 10) + '0';
        j++;

        number = (number / 10);
    }
    while (number > 0);

    j--;

    //reverse string
    while(i < j)
    {
        temp = string[j];
        string[j] = string[i];
        string[i] = temp;

        i++;
        j--;
    }

    return string;
}

//------------------------------------------------------------------------------------------------------------------------------------

void getsUart0(USER_DATA* data)
{
    uint32_t count = 0;
    char c;

    while(true)
    {
        c = getcUart0(); //b

        if(c == 8 || c == 127) //c
        {
            if (count > 0)
            {
                count--;
            }
        }

        else if(c == 13) //d
        {
            data->buffer[count] = '\0';
            break;
        }

        else if(c >= 32) //e
        {
            data->buffer[count] = c;
            count++;

            if(count == MAX_CHARS)
            {
                data->buffer[count] = '\0';
                break;
            }
        }
    }
}

//------------------------------------------------------------------------------------------------------------------------------------

// Alpha is a-z and A-Z, numeric is 0-9 and optionally hyphen and period (or comma in some localizations), and everything else is a delimiter
// Alpha a-z (97-122), A-Z (65-90), numeric (48-57)
// Delimiter (1-47) (58-64) (91-96) (123-127)
void parseFields(USER_DATA* data)
{
    uint32_t i = 0;
    uint32_t count = 0;

    uint32_t d, a, n;

    while(true)
    {
        d = ((data->buffer[i] >= 1 && data->buffer[i] <= 47) || (data->buffer[i] >= 58 && data->buffer[i] <= 64) || (data->buffer[i] >= 91 && data->buffer[i] <= 96) || (data->buffer[i] >= 123 && data->buffer[i] <= 127));
        a = ((data->buffer[i] >= 97 && data->buffer[i] <= 122) || (data->buffer[i] >= 65 && data->buffer[i] <= 90));
        n = (data->buffer[i] >= 48 && data->buffer[i] <= 57);

        if (d)
        {
            if((data->buffer[i-1] >= 1 && data->buffer[i-1] <= 47) || (data->buffer[i-1] >= 58 && data->buffer[i-1] <= 64) || (data->buffer[i-1] >= 91 && data->buffer[i-1] <= 96) || (data->buffer[i-1] >= 123 && data->buffer[i-1] <= 127))
            {
                //data->buffer[i-1] = '\0';
            }

            i++;

            if(data->buffer[i] == '\0')
            {
                data->fieldCount = count;
                putsUart0("\n");
                break;
            }
            //continue;
        }

        if(a || n)
        {
            if(a)
            {
                data->fieldType[count] = 'a';
            }

            else if(n)
            {
                data->fieldType[count] = 'n';
            }

            data->fieldPosition[count] = i;

            if(((data->buffer[i-1] >= 1 && data->buffer[i-1] <= 47) || (data->buffer[i-1] >= 58 && data->buffer[i-1] <= 64) || (data->buffer[i-1] >= 91 && data->buffer[i-1] <= 96) || (data->buffer[i-1] >= 123 && data->buffer[i-1] <= 127)) && (a || n))
            {
                //data->buffer[i-1] = '\0';
                count++;
            }

            if (i == 0)
            {
                count++;
            }

            i++;

            if(data->buffer[i] == '\0' || count == MAX_FIELDS)
            {
                data->fieldCount = count;
                putsUart0("\n");
                break;
            }
        }
    }
}

//------------------------------------------------------------------------------------------------------------------------------------

bool customStrcmp (const char strCommand[], const char argument[])
{
    uint32_t i = 0;

    while(strCommand[i] == argument[i])
    {
        if(strCommand[i] == '\0' || argument[i] == '\0')
            break;

        i++;
    }

    if(strCommand[i] == '\0' && argument[i] == '\0')
        return true;

    else
        return false;
}

//------------------------------------------------------------------------------------------------------------------------------------

char* getFieldString(USER_DATA* data, uint32_t fieldNumber)
{
    return &data->buffer[data->fieldPosition[fieldNumber]];
}

//------------------------------------------------------------------------------------------------------------------------------------

int32_t getFieldInteger(USER_DATA* data, uint32_t fieldNumber)
{
    return atoi(getFieldString(data, fieldNumber));
}


//------------------------------------------------------------------------------------------------------------------------------------

double getFieldDouble(USER_DATA* data, uint32_t fieldNumber)
{
    return atof(getFieldString(data, fieldNumber));
}

//------------------------------------------------------------------------------------------------------------------------------------

bool  isCommand(USER_DATA* data, const char strCommand[], uint32_t minArguments)
{
    if(customStrcmp(strCommand, &data->buffer[data->fieldPosition[0]]) && data->fieldCount >= minArguments)
    {
        return true;
    }

    return false;
}

