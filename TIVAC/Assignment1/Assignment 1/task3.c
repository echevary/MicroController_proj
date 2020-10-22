#include "inc/tm4c123gh6pm.h"
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_adc.h"
#include "inc/hw_types.h"
#include "inc/hw_udma.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "utils/uartstdio.h"

#define ADC_SAMPLE_BUF_SIZE     1

#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//Structs
enum BUFFER_STATUS
{
    EMPTY,
    FULL
};

typedef struct{
    uint16_t adcBuffer[ADC_SAMPLE_BUF_SIZE];
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
///////////////////////////////////////////
//Variables
volatile static enum BUFFER_STATUS BufferStatus;
volatile Temp_t temp_t;
volatile uint32_t Count;
///////////////////////////////////////////
//Prototype
void CalculateTemperatureAvg(void);
void ConfigureUART(void);
void ConfigureADC(void);
void ConfigureUDMA(void);
void check_status(uint32_t status, const char* string);
void send_status(const char* string, bool status);
///////////////////////////////////////////
#define GREEN_LED GPIO_PIN_3
#define BLUE_LED  GPIO_PIN_2
#define RED_LED   GPIO_PIN_1
///////////////////////////////////////////
void printm(char *str)
{
    //While it's not NULL
    while(*str != '\0')
    {
        UARTCharPut(UART0_BASE,*str);
        ++str;
    }
}

int main(void){

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_INT |   SYSCTL_XTAL_16MHZ);

    volatile unsigned char status_of_leds;

    volatile State_enum state, nextState = WAIT;

    volatile UART_t uart_t;
    BufferStatus = EMPTY;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    //Config LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    ConfigureUART();
    ConfigureUDMA();
    ConfigureADC();

    IntMasterEnable();

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
                    CalculateTemperatureAvg();
                    UARTprintf("C %3d\t", temp_t.C);
                    UARTprintf("\n");
                    nextState = WAIT;
                    break;


               case temp:
                    CalculateTemperatureAvg();
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
                    break;
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

void CalculateTemperatureAvg(void){

    ADCProcessorTrigger(ADC0_BASE, 0);

    while(BufferStatus == EMPTY){};

                temp_t.AVG = 0;

                for(Count = 0; Count < ADC_SAMPLE_BUF_SIZE; Count++)
                {
                    temp_t.AVG += temp_t.adcBuffer[Count];
                    temp_t.adcBuffer[Count] = 0;
                }

                BufferStatus = EMPTY;

                uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                                       UDMA_MODE_BASIC,
                                       (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                                       &temp_t.adcBuffer, ADC_SAMPLE_BUF_SIZE);

                uDMAChannelEnable(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT);

                temp_t.AVG = ((temp_t.AVG +
                                      (ADC_SAMPLE_BUF_SIZE / 2)) /
                                       ADC_SAMPLE_BUF_SIZE);

                temp_t.C = (1475 - ((2475 * temp_t.AVG)) / 4096)/10;
                temp_t.F = ((temp_t.C * 9) + 160) / 5;
}

void ConfigureUART(void){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);


    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);


    UARTStdioConfig(0, 115200, SysCtlClockGet());
}


void ConfigureADC(void){

        IntDisable(INT_ADC0SS0);

        ADCIntDisable(ADC0_BASE, 0);


        ADCSequenceDisable(ADC0_BASE, 0);


        ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);


        ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_TS | ADC_CTL_END |
                                 ADC_CTL_IE);


        ADCSequenceEnable(ADC0_BASE, 0);

        ADCIntClear(ADC0_BASE, 0);

        ADCSequenceDMAEnable(ADC0_BASE, 0);

        ADCIntEnable(ADC0_BASE, 0);

        IntEnable(INT_ADC0SS0);


}


void ConfigureUDMA(void){

       uDMAEnable();


       uDMAControlBaseSet(pui8ControlTable);

       uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC0,
                                   UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY |
                                   UDMA_ATTR_REQMASK);


       uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_SIZE_16 |
                             UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_64);


       uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                              UDMA_MODE_BASIC,
                              (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                              &temp_t.adcBuffer, ADC_SAMPLE_BUF_SIZE);


       uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC0, UDMA_ATTR_USEBURST);


       uDMAChannelEnable(UDMA_CHANNEL_ADC0);
}

void ADCSeq0Handler(void){

    ADCIntClear(ADC0_BASE, 0);

    if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT) == UDMA_MODE_STOP))
    {
        BufferStatus = FULL;

    }

}

