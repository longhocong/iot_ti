/*
 * UART.c
 *
 *  Created on: Apr 15, 2016
 *      Author: Satu
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "led_task.h"
#include "switch_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#define TARGET_IS_BLIZZARD_RB1
#include "driverlib/rom.h"
#include "drivers/led.h"
#include "drivers/switches.h"
#include "driverlib/interrupt.h"
#include "inc/hw_nvic.h"
#include "UART.h"
static char Buff[100];
typedef enum
{
    LEDON = 0, LEDOFF
} ledState_t;
static ledState_t state = LEDOFF;

typedef struct
{
    unsigned int buttonStatus;
} Data_t;

void ledRedStateMachineUpdate(void)
{
    uint8_t a, b = 0;
    Data_t data;
    switch (state)
    {
    case LEDON:
    {
        char x1[7] = { 'O', 'F', 'F', 'L', 'E', 'D', 0x0a };
        for (b = 0; b < 7; b++)
        {
            if (Buff[b] != x1[b])
                return;
        }
        state = LEDOFF;
        //  GPIOPinWrite(led_port,ledblue,0);
    }
    case LEDOFF:
    {
        char x2[6] = { 'O', 'N', 'L', 'E', 'D', 0x0a };
        for (a = 0; a < 6; a++)
        {
            if (Buff[a] != x2[a])
                return;
        }
        state = LEDON;
        //     GPIOPinWrite(led_port,ledblue,ledblue );}

        data.buttonStatus = state;
    }
    }
}

/*  static sosanh(void){
 uint8_t a,b;
 char x1[6]= {'O','N','L','E','D',0x0a};
 char x2[7]= {'O','F','F','L','E','D',0x0a};

 for (a=0;a<7;a++) {
 if(Buff[a]!=x2[a])
 {
 for (b=0;b<6;b++){
 if(Buff[b]!=x1[b]){return;}
 }
 GPIOPinWrite(led_port,ledblue,ledblue );
 return;}
 }

 GPIOPinWrite(led_port,ledblue,0);
 }  */
static void Reset_Buffer(char *pBuff)
{
    while (*pBuff != 0x0a)
    {
        *pBuff = 0;
        pBuff++;
    }
}
void UARTGetBuffer(char *pBuff)
{
    static uint16_t i = 0;
    char c;
    if (i == 0)
        Reset_Buffer(pBuff);
    while (UARTCharsAvail(UART0_BASE))
    {
        c = UARTCharGet(UART0_BASE);
        *(pBuff + i) = c;
        UARTCharPut(UART0_BASE, c);
        i++;
    }
    if (c == 0x0a)
    {
        ledRedStateMachineUpdate();
        // sosanh();

        i = 0;
    }
}
static void UART_ISR(void)
{
    UARTIntClear(UART0_BASE, UARTIntStatus(UART0_BASE, true));
    UARTGetBuffer(&Buff[0]);
}

void Config_UART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);

    UARTConfigSetExpClk(
            UART0_BASE, SysCtlClockGet(), 115200,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTIntRegister(UART0_BASE, &UART_ISR);
    IntEnable(INT_UART1);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

