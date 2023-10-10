// THIS FILE IS FOR TESTING
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
#include "adc0.h"

#define RED_LED         PORTF, 1
#define GREEN_LED       PORTF, 3
#define BLUE_LED        PORTF, 2

#define PB_1            PORTF, 4
#define PB_2            PORTF, 0

// Initialize Hardware
void initHw(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    _delay_cycles(3);

    enablePort(PORTA);
    enablePort(PORTB);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);
    
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);

    /*
    selectPinDigitalInput(PB_1);
    enablePinPullup(PB_1);

    setPinCommitControl(PB_2);
    selectPinDigitalInput(PB_2);
    enablePinPullup(PB_2);
    */
}

int main(void)
{
    // Initialize hardware
    initHw();
    initUart0();

    //waitMicrosecond(1000000);

    setPinValue(RED_LED, 0);
    setPinValue(GREEN_LED, 0);
    setPinValue(BLUE_LED, 0);


    while (true)
    {

    }
}













