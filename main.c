#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/sysctl.c"
#include "driverlib/interrupt.h"
#include "driverlib/interrupt.c"
#include "driverlib/gpio.h"
#include "driverlib/gpio.c"
#include "driverlib/timer.h"
#include "driverlib/timer.c"
#include "driverlib/uart.h"
#include "driverlib/uart.c"
//#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "grlib/grlib.h"
#include <string.h>
/**
 * main.c
 */
void UARTIntHandler(void);
void Timer0IntHandler(void);
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
int estaEncendido = false;
char color='a';
char antecolor='a';
int cambio=0;
int main(void)
{
    SysCtlClockSet ( SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ ) ;
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF ) ;
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}
    //UART
    SysCtlPeripheralEnable (SYSCTL_PERIPH_UART0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)){}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0))
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    IntMasterEnable();
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTDisable(UART0_BASE);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    IntEnable (INT_UART0);
    //UARTIntEnable (UART0_BASE, UART_INT_TX | UART_INT_RX);
    UARTIntEnable (UART0_BASE, UART_INT_RX);
    UARTEnable (UART0_BASE);
    IntPrioritySet(INT_UART0, 0x0);
    IntRegister(INT_UART0, UARTIntHandler);
    UARTFIFOEnable(UART0_BASE);
    UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);
    //IntMasterEnable ();
    //UARTprintf("Hello, world!\n");
    //UARTSend((uint8_t *)"Enter text: ", 12);

    //TIMER
    SysCtlPeripheralEnable (SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){}
    SysCtlPeripheralReset (SYSCTL_PERIPH_TIMER0);
    SysCtlDelay (5);
    TimerDisable(TIMER0_BASE, TIMER_A|TIMER_B);
    TimerConfigure (TIMER0_BASE,TIMER_CFG_PERIODIC);
    TimerLoadSet (TIMER0_BASE, TIMER_A, 20000000 -1);
    //TimerMatchSet (TIMER0_BASE, TIMER_A, 20000000); // PWM
    TimerEnable (TIMER0_BASE, TIMER_A|TIMER_B);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        //Establecer la ISR
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
        //Habilitar interrupción por parte del NVIC
    IntEnable(INT_TIMER0A);

        //Habilitar temporizador
    TimerEnable(TIMER0_BASE, TIMER_A);
    //Configuración puertos salidas
    GPIOPinTypeGPIOOutput (GPIO_PORTF_BASE,GPIO_PIN_1) ;//led rojo
    GPIOPinTypeGPIOOutput (GPIO_PORTF_BASE, GPIO_PIN_2) ;//led azul
    GPIOPinTypeGPIOOutput (GPIO_PORTF_BASE, GPIO_PIN_3) ;//led verde

        //uint32_t tiempo=0;

    while(1){}
}
void Timer0IntHandler(){
    TimerIntClear(TIMER0_BASE,  TIMER_TIMA_TIMEOUT);
        if (estaEncendido){
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x0);
        }else{
            switch(color){
                            case 'r':
                                GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x02);
                                antecolor='r';
                                break;
                            case 'g':
                                GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x08);
                                antecolor='g';
                                break;
                            case 'b':
                                GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x04);
                                antecolor='b';
                                break;
                            case 'o':
                                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x0);
                                antecolor='o';
                                break;
                                }

        }
        estaEncendido = !estaEncendido;


}
void UARTIntHandler(){
    uint32_t ui32Status;

        //
        // Get the interrrupt status
        ui32Status = UARTIntStatus(UART0_BASE, true);
        //UARTCharPutNonBlocking(UART0_BASE,ui32Status);
        //
        // Clear the asserted interrupts.
        //
        UARTIntClear(UART0_BASE, ui32Status);

        //
        // Loop while there are characters in the receive FIFO.
        //
        while(UARTCharsAvail(UART0_BASE))
        {

            color=UARTCharGet(UART0_BASE);
            UARTCharPutNonBlocking(UART0_BASE,color);
            if(color==antecolor){color='o';}
        }


}
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}
