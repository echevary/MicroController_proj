#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.h"
//********************

typedef struct{
    uint32_t adcBuffer[4];
    uint32_t AVG;
    uint32_t C;
    uint32_t F;
}Temp_t;


typedef enum{
    WAIT,
    RED, BLUE, GREEN,
    red, blue, green,
    TEMP,
    temp,
    STAT

}State_enum;


typedef struct{

    unsigned char input;

}UART_t;

void check_status(uint32_t status, const char* string);
void send_status(const char* string, bool status);
void calculate_avg_temperature(Temp_t *temp_t);
//*******************

#define GREEN_LED GPIO_PIN_3
#define BLUE_LED  GPIO_PIN_2
#define RED_LED   GPIO_PIN_1

//*******************
int main(void)
{

      //Config System Clock
      SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_XTAL_16MHZ | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN );

     Temp_t temp_t;

     volatile unsigned char status_of_leds;

     volatile State_enum state, nextState = WAIT;

     volatile UART_t uart_t;

     volatile uint32_t period;




     //Config UART
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);


        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);


        UARTStdioConfig(0, 115200, SysCtlClockGet());
     //

     //Config ADC
     SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

     ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
     ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_TS);
     ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_TS);
     ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_TS);
     ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_TS | ADC_CTL_END | ADC_CTL_IE);
     ADCSequenceEnable(ADC0_BASE, 1);

     //Config LEDs
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
     GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
     GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);



     //Next State for UI
    while(1)
    {

        state = nextState;

        switch(state){

        case WAIT:

            while(!UARTCharsAvail(UART0_BASE)){};

          uart_t.input = UARTCharGet(UART0_BASE);

                 if(uart_t.input == 'R') nextState = RED;
            else if(uart_t.input == 'B') nextState = BLUE;
            else if(uart_t.input == 'G') nextState = GREEN;
            else if(uart_t.input == 'r') nextState = red;
            else if(uart_t.input == 'b') nextState = blue;
            else if(uart_t.input == 'g') nextState = green;
            else if(uart_t.input == 'T') nextState = TEMP;
            else if(uart_t.input == 't') nextState = temp;
            else if(uart_t.input == 'S') nextState = STAT;
            else                         nextState = WAIT;
            break;

        case RED:
            GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
            printm("Turning on Red LED");
            nextState = WAIT;
            break;

        case red:
            GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
            printm("Turning off Red LED");
            nextState = WAIT;
            break;

        case BLUE:
            GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, BLUE_LED);
            printm("Turning on Blue LED");
            nextState = WAIT;
            break;
        case blue:
            GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
            printm("Turning off Blue LED");
            nextState = WAIT;
            break;

        case GREEN:
            GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, GREEN_LED);
            printm("Turning On Green LED");
            nextState = WAIT;
            break;

        case green:
            GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
            printm("Turning off Green LED");
            nextState = WAIT;
            break;

        case TEMP:
            calculate_avg_temperature(&temp_t);
            UARTprintf("C %3d\t", temp_t.C);
            UARTprintf("\n");
            nextState = WAIT;
            break;

        case temp:
            calculate_avg_temperature(&temp_t);
            UARTprintf("F: %3d\t",temp_t.F);
            UARTprintf("\n");
            nextState = WAIT;
            break;

        case STAT:
            status_of_leds = GPIOPinRead(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED);
            check_status((status_of_leds&RED_LED)   >> 1, "Red:   ");
            check_status((status_of_leds&BLUE_LED)  >> 2, "Blue:  ");
            check_status((status_of_leds&GREEN_LED) >> 3, "Green: ");
            nextState = WAIT;
            break;

        default:
           nextState = WAIT;
           continue;

        }

    }
}


void check_status(uint32_t status, const char* string){
    if(status)
       send_status(string, 1);
    else
       send_status(string, 0);
}

void send_status (const char* string, bool status){

    UARTprintf(string);

    if(status)
        UARTprintf("ON");
    else
       UARTprintf("OFF");

    UARTprintf("\n");
}

//Calculate the average
void calculate_avg_temperature(Temp_t *temp_t){

    //Trigger ADC and wait for interrupt
    ADCProcessorTrigger(ADC0_BASE, 1);

    while(!ADCIntStatus(ADC0_BASE, 1, false)){};
    ADCIntClear(ADC0_BASE, 1);

    ADCSequenceDataGet(ADC0_BASE, 1, temp_t->adcBuffer);

    //Calculate Average
    temp_t->AVG = (temp_t->adcBuffer[0] + temp_t->adcBuffer[1] + temp_t->adcBuffer[2] + temp_t->adcBuffer[3])/4;

    temp_t->C = (1475 - ((2475 * temp_t->AVG)) / 4096)/10;
    temp_t->F = ((temp_t->C * 9) + 160) / 5;

}
