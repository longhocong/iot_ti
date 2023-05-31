// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

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
#include "../drivers/buttons.h"
#include "../drivers/led.h"
#include "../include.h"
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>FreeRTOS Example (freertos_demo)</h1>
//!
//! This application demonstrates the use of FreeRTOS on Launchpad.
//!
//! The application blinks the user-selected LED at a user-selected frequency.
//! To select the LED press the left button and to select the frequency press
//! the right button.  The UART outputs the application status at 115,200 baud,
//! 8-n-1 mode.
//!
//! This application utilizes FreeRTOS to perform the tasks in a concurrent
//! fashion.  The following tasks are created:
//!
//! - An LED task, which blinks the user-selected on-board LED at a
//!   user-selected rate (changed via the buttons).
//!
//! - A Switch task, which monitors the buttons pressed and passes the
//!   information to LED task.
//!
//! In addition to the tasks, this application also uses the following FreeRTOS
//! resources:
//!
//! - A Queue to enable information transfer between tasks.
//!
//! - A Semaphore to guard the resource, UART, from access by multiple tasks at
//!   the same time.
//!
//! - A non-blocking FreeRTOS Delay to put the tasks in blocked state when they
//!   have nothing to do.
//!
//! For additional details on FreeRTOS, refer to the FreeRTOS web page at:
//! http://www.freertos.org/
//
//*****************************************************************************


#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}
xQueueHandle xQueue;
typedef struct
{
    unsigned int buttonStatus;
} Data_t;
//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
static void UART_ISR(void);
static uint8_t status;
static char Buff[100];
static float TempValueC ;
static float v1 ;
static uint8_t aoto=1, fan=0, pi,ds;
typedef     enum {LEDON=0,LEDOFF}  ledState_t;
static ledState_t  state = LEDOFF;


static void UART_ISR(void)
 { uint8_t data;  static uint16_t i=0; uint8_t *pBuff;
 char c;
 portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    UARTIntClear(UART7_BASE,UARTIntStatus(UART7_BASE,true));



       while(UARTCharsAvail(UART7_BASE))
       {
           c=UARTCharGet(UART7_BASE);
           Buff[i]=c;
         //  UARTCharPut(UART7_BASE,c);
           i++;
       }
       if(c==0x0a){  uint8_t a,b=0;

           { char x1[7]= {'O','F','F','L','E','D'};
           for ( b=0;b<6;b++){
                if(Buff[b]!=x1[b])
                   goto ab1; }
           state=LEDOFF; ds=1;
           status =0 ;}
      ab1:
           {char x2[6]= {'O','N','L','E','D'};
           for ( a=0;a<5;a++){
                        if(Buff[a]!=x2[a])
                            goto ab2; }
                   state=LEDON; ds=1;
                   status = 1;}


   ab2:     {char x2[6]= {'A','U','T','O'};
                 for ( a=0;a<4;a++){
                              if(Buff[a]!=x2[a])
                                  goto ab3; }

                         aoto = 1;  UARTCharPut(UART7_BASE,55);}


     ab3:  {char x2[6]= {'O','F','F','A','T'};
                 for ( a=0;a<5;a++){
                              if(Buff[a]!=x2[a])
                                  goto ab4; }

                         aoto = 0; UARTCharPut(UART7_BASE,56);}
    ab4:  {char x2[6]= {'O','N','F','A','N'};
                      for ( a=0;a<5;a++){
                                   if(Buff[a]!=x2[a])
                                       goto ab5; }

                              fan = 1;}
ab5:       {char x2[6]= {'O','F','F','F','A','N'};
                  for ( a=0;a<6;a++){
                      if(Buff[a]!=x2[a])
                 goto ab; }

        fan = 0;}

   ab:    for (i=0;i<30;i++) {Buff[i]= 0 ;}
i=0;
                  }



    data=  status ;
    xQueueSendFromISR( xQueue, &data, &xHigherPriorityTaskWoken );
   portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
 }


static  void vReceiverTask( void *pvParameters );
static  void fancute( void *pvParameters );
static  void temp1( void *pvParameters );
static  void light( void *pvParameters );
static  void utemp( void *pvParameters );
//static  void vReceiverTask1( void *pvParameters1 );
/* Define the structure type that will be passed on the queue. */


//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
static void switch_isr(void);
static uint8_t status;
int main(void)
{

        SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
      Config_UART();

      GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_5);
      GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_4); // cau hinh pf1 la ouput
          GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_4, 0);
          GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5, 0);
     // IntMasterDisable();
     //   switchInit();
        ledInit();
        config_tem();
        config_adc();
      //  swIntEnable();
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
         GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_INT_PIN_0);
          GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_0); // cau hinh pf4 la input
          GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU); //sach tr267
          GPIOIntRegister( GPIO_PORTF_BASE,&switch_isr); //khai bao ngat
          GPIOIntEnable(GPIO_PORTF_BASE,GPIO_INT_PIN_0); //kich hoatj ngat o chan nao
          IntEnable(INT_GPIOF); // KICH HAOT CAP PORT
        IntMasterEnable();

        xQueue = xQueueCreate( 3, sizeof( uint8_t) );

    xTaskCreate( vReceiverTask, "ReceiverTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
    xTaskCreate( fancute, "fancute", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
    xTaskCreate( temp1, "temp1", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
    xTaskCreate( light, "light", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
   // xTaskCreate( utemp, "utemp", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
   // xTaskCreate( vReceiverTask1, "ReceiverTask1", 240, NULL, 1, NULL );
    vTaskStartScheduler();
    while(1)
       {
       }
}

static void utemp( void *pvParameters )
{

    for(;;) {
    UARTCharPut(UART7_BASE,TempValueC);
    UARTCharPut(UART0_BASE,TempValueC);
     }
}

static void vReceiverTask( void *pvParameters )
{
uint8_t  data;

static uint8_t s=0;
    portBASE_TYPE xStatus;

   while(1)
    {

         xQueueReceive( xQueue, &data, 100 );
         if (aoto==0) {  if (1) {
        if (data==0) {ledControl(LEDRED,OFF);GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5, 0) ; UARTCharPut(UART7_BASE,51);}
        if (data==1) {ledControl(LEDRED,ON);GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5, GPIO_PIN_5) ;  UARTCharPut(UART7_BASE,52);}
        ds==0 ;}
         }
    }
}
static void fancute( void *pvParameters ) {
    while(1)
       {   if (aoto==0) { UARTCharPut(UART7_BASE,56); if (fan==0) {GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0) ; UARTCharPut(UART7_BASE,54);}
       if (fan==1) {GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); UARTCharPut(UART7_BASE,53);}
       vTaskDelay(500);
       }

       }
}
//static  void vReceiverTask1( void *pvParameters1 )

static void temp1( void *pvParameters )
{
    while(1) {  ADCProcessorTrigger(ADC1_BASE,3);
    if (aoto==1) {if (TempValueC >= 20) {GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4) ;UARTCharPut(UART7_BASE,53);   } else { GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0) ;UARTCharPut(UART7_BASE,54);}}
    vTaskDelay(1000);
    pi= (char)TempValueC;
    UARTCharPut(UART7_BASE,pi);
      UARTCharPut(UART0_BASE,pi);
    }
}
static void light( void *pvParameters )
{
    while(1) {  ADCProcessorTrigger(ADC0_BASE,1);
     if (v1 >= 4.2) { UARTCharPut(UART7_BASE,58);  if (aoto==1) {ledControl(LEDRED,ON);  GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_5, GPIO_PIN_5) ;  UARTCharPut(UART7_BASE,52);  vTaskDelay(200); UARTCharPut(UART7_BASE,55); }}

     else {UARTCharPut(UART7_BASE,57); if (aoto==1) { ledControl(LEDRED,OFF);GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0) ; UARTCharPut(UART7_BASE,51);vTaskDelay(200);}
     }

}}
static void adc_isr(void)
{   ADCIntClear(ADC0_BASE,1);

    uint32_t  data1[1]; // ham toi uw ow trn
    ADCSequenceDataGet(ADC0_BASE,1,(uint32_t*)&data1); // uint32_t  data[3]; // cos 3 mau du lieu voltage = (float)((data[0]+data[1]+data[2])/3)*5/4096;

    v1 = (float)(data1[0])*5/4096;
}
static void tem_isr(void)
{   ADCIntClear(ADC1_BASE,3);
uint32_t tem[1];
    float TempValueF ;
    ADCSequenceDataGet(ADC1_BASE,3,(uint32_t*)&tem);
   TempValueC = (float) (1475 - ((2475 * tem[0])) / 4096)/10; // cong thuc cua sach
  //TempValueF = ((TempValueC * 9) + 160) / 5;
}
extern config_adc(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);   //kich hoat adc0
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // kich hoat poet e
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3); //kich hoat chan hoatj dong

    ADCHardwareOversampleConfigure(ADC0_BASE,64); // tinh gia tri trung binh bien v1
    ADCSequenceConfigure(ADC0_BASE,1,ADC_TRIGGER_PROCESSOR,1);    // loaiadc, so bo lay mau, xung , muwcs uu tien 0, 1 2 3
   // ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH0); //loaiadc, so mau dc lay, step, vi pe3 laf kenh0
    //ADCSequenceStepConfigure(ADC0_BASE,1,1,ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH0|ADC_CTL_END|ADC_CTL_IE);  //vi lay 3 mau maf ss1 cos 4 mau nen dung ADC_CTL_END de ko use mau 4 , co ie de ngat

    ADCSequenceEnable(ADC0_BASE,1); // bat adc

    ADCIntRegister(ADC0_BASE,1,&adc_isr); // chi toi ham ngat
    ADCIntEnable(ADC0_BASE,1); // bat ngat
}
void config_tem(void)
        {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);   //kich hoat adc0


        ADCHardwareOversampleConfigure(ADC1_BASE,64); // tinh gia tri trung binh bien v1
        ADCSequenceConfigure(ADC1_BASE,3,ADC_TRIGGER_PROCESSOR,1);    // loaiadc, so bo lay mau, xung , muwcs uu tien 0, 1 2 3

        ADCSequenceStepConfigure(ADC1_BASE,3,0,ADC_CTL_TS|ADC_CTL_END|ADC_CTL_IE);  //vi lay 3 mau maf ss1 cos 4 mau nen dung ADC_CTL_END de ko use mau 4 , co ie de ngat

        ADCSequenceEnable(ADC1_BASE,3); // bat adc

        ADCIntRegister(ADC1_BASE,3,&tem_isr); // chi toi ham ngat
        ADCIntEnable(ADC1_BASE,3); // bat ngat
        }

void Config_UART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinConfigure(GPIO_PE0_U7RX);
    GPIOPinConfigure(GPIO_PE1_U7TX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART7_BASE, UART_CLOCK_SYSTEM);

    UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 9600,UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE);
    UARTIntRegister(UART7_BASE,&UART_ISR);
   IntEnable (INT_UART7);
    UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
       GPIOPinConfigure(GPIO_PA0_U0RX);
       GPIOPinConfigure(GPIO_PA1_U0TX);
       GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
       UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);

       UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
}
static void switch_isr(void)
 {
    GPIOIntClear(GPIO_PORTF_BASE,GPIOIntStatus(GPIO_PORTF_BASE,true));

uint8_t data;
 portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
if ( status ==1)
        status = 0;
else   status = 1;

        data = status;

    xQueueSendFromISR( xQueue, &data, &xHigherPriorityTaskWoken );
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
 }
