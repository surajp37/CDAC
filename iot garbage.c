/*This is IOT based garbage monitoring system where we are using ultrasonic sensor (HC-sr04) and LCD display (16*2)
 Ultrasonic sensor HC-sr04 pin out
 vcc - +5v
 gnd - 0
 echo - PA4
 trigger - PA5

 LCD connection
 1 - gnd
 2 - +5v
 3 - vee  we can connect it to GND or 1K pot
 4 - RS (PN5)
 5 - R/W = (WRITE=0,READ=1)
 6 - EN  (PN4)
 7 - PK0
 14 - PK7
 15 - VCC
 16 - GND

*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"

#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"

#include "driverlib/rom_map.h"
#include "utils/uartstdio.c"


// PA4=echo  PA5=trigger;
//This is LCD 16*2 code


//PORT K=DATA LINES
//PN4=EN;
//PN5=RS;

void UARTTransmitCommand( char *p);  // send from UART
void inputInt();       // timer start
void Captureinit();     // timer capture
void InitConsole(void);  // UART 0
//Function Prototypes for lcd
void LCD_init(void);                    //Function to initialise the LCD
void LCD_command(unsigned char cmd);    //Function to pass command to the LCD
void LCD_data(unsigned char data);        //Function to write character to the LCD
void LCD_write_string(char *str);//Function to write string to the LCD
//This is to avoid doing the math everytime you do a reading
const double temp = 1.0/16.0;   // 16Mhz

char msg[10];
//Stores the pulse length
volatile  long pulse=0;

//Tells the main code if the a pulse is being read at the moment
volatile uint8_t echowait=0;


void
UART5IntHandler()
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART5_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART5_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART5_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        UARTCharPutNonBlocking(UART5_BASE,
                                   UARTCharGetNonBlocking(UART5_BASE));


    }
}


int main()
{
 //Set system clock to 16Mhz
     char var1[] = "SEC 62  BLOCK B";//Declare message to be displayed
     char var2[] = "BIN   IS    FULL";
     char var3[] = "BIN IS HALF FULL";//Declare message to be displayed
     char var4[] = "BIN   IS   EMPTY";

     /*Set the clocking to directly run from the crystal at 16MHz*/
     SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

     /* Set the clock for the GPIO Port F */
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
         SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);

         IntMasterEnable();


     /* Set the type of the GPIO Pin */

  // configure LED for indication
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Check if the peripheral access is enabled.
        //
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
        {
        }
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));


        GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5);
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_0);
        GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
      //  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,GPIO_PIN_4|GPIO_PIN_5);


  //Configures the UART
 // InitConsole();


             GPIOPinConfigure(GPIO_PC6_U5RX);
            GPIOPinConfigure(GPIO_PC7_U5TX);
            GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

            UARTConfigSetExpClk(UART5_BASE, 16000000, 9600,
                                       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                        UART_CONFIG_PAR_NONE));

               //
               // Enable the UART interrupt.
               //
               UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
               IntEnable(INT_UART5);
               UARTEnable(UART5_BASE);

  //Configures the timer
  Captureinit();

  //Configure Trigger pin as a output
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlDelay(3);
  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);

  //Configure Echo pin as a input
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlDelay(3);
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);
  GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_4);
  GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4,GPIO_BOTH_EDGES);
  GPIOIntRegister(GPIO_PORTA_BASE,inputInt);

      LCD_init();                                   // call function to initialise of LCD
      SysCtlDelay(55333);         // delay of 10 mili seconds


  while(1)
  {

    //Checks if a pulse read is in progress
    if(echowait != 1){
      //Does the required pulse of 10uS
      GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
      SysCtlDelay(54);
      GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);

      /*
        This makes the code wait for a reading to finish
        You can omit this part if you want the code to be non-blocking but
        reading is only ready when echowait=0.
      */
      while(echowait != 0);

      //Converts the counter value to cm.
      pulse =(uint32_t)(temp* pulse);
      pulse = (pulse/58 );

      //Prints out the distance measured.
    //  UARTprintf("distance = %2dcm \n" , pulse);

      ltoa(pulse,msg);

      UARTTransmitCommand(msg);




      if(pulse<=4)                                                       // bin is full
         {
                     LCD_write_string(var1);                    //Display message on first line
                     SysCtlDelay(55333);

                     LCD_command(0xC0);          // initiate cursor to second line
                     LCD_write_string(var2);//Display message on second line


             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
             GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
             SysCtlDelay(3);
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
             GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00);
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);


         }
         else if(pulse>=5 && pulse<=15){                  // bin is half empty
                       LCD_write_string(var1);                    //Display message on first line
                        SysCtlDelay(55333);

                        LCD_command(0xC0);          // initiate cursor to second line
                        LCD_write_string(var3);    //Display message on second line
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
             GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
             SysCtlDelay(3);
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
             GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x00);
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);


         }
         else{                                                          // bin is empty

                       LCD_write_string(var1);                    //Display message on first line
                        SysCtlDelay(55333);

                        LCD_command(0xC0);          // initiate cursor to second line
                        LCD_write_string(var4);//Display message on second line
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
             SysCtlDelay(3);
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
             GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x00);
             GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00);
         }
    }




   //wait about 10ms until the next reading.
      SysCtlDelay(400000);
  //  SysCtlDelay(16000000/3);


  }
}

void inputInt(){
  //Clear interrupt flag. Since we only enabled on this is enough
  GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_4);

  /*
    If it's a rising edge then set he timer to 0
    It's in periodic mode so it was in some random value
  */
  if ( GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) == GPIO_PIN_4){
    HWREG(TIMER2_BASE + TIMER_O_TAV) = 0; //Loads value 0 into the timer.
    TimerEnable(TIMER2_BASE,TIMER_A);
    echowait=1;
  }
  /*
    If it's a falling edge that was detected, then get the value of the counter
  */
  else{
    pulse = TimerValueGet(TIMER2_BASE,TIMER_A); //record value
    TimerDisable(TIMER2_BASE,TIMER_A);
    echowait=0;
  }


}

void Captureinit(){
  /*
    Set the timer to be periodic.
  */
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  SysCtlDelay(3);
  TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
  TimerEnable(TIMER2_BASE,TIMER_A);
}

void InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // : change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // : change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 9600, 16000000);

}

void LCD_init(void)        // Function to initialise the LCD
{
    LCD_command(0x38);      // initialization of 16X2 LCD in 8bit mode
    SysCtlDelay(53333);
    LCD_command(0x01);      // clear LCD
    SysCtlDelay(53333);
    LCD_command(0x0C);      // cursor off
    SysCtlDelay(53333);
    LCD_command(0x80);      // go to first line and 0th position
    SysCtlDelay(53333);
}

void LCD_command(unsigned char cmd) //Function to pass command to the LCD
{
    /* Writing Command on Port B*///Send data on LCD command bus
   GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5,0);//RS = 0 since command to LCD
   GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,cmd);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4,GPIO_PIN_4);//Generate High to low pulse on EN
    SysCtlDelay(1333);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4,0);


}

void LCD_data(unsigned char data)//Function to write data to the LCD
{
    /* Writing Command on Port B*///Send data on LCD data bus
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5,GPIO_PIN_5);//RS = 1 since data to LCD
       GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,data);
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4,GPIO_PIN_4);//Generate High to low pulse on EN
        SysCtlDelay(1333);
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4,0);
}
//Function to write string to LCD
void LCD_write_string(char *str)
{
    int i = 0;
    while (str[i] != '\0')
    {
        LCD_data(str[i]);      // sending data on LCD byte by byte
        //SysCtlDelay(5533);
        i++;
    }
}


void UARTTransmitCommand( char *p)
{
while(*p!='\0')
{
    UARTCharPutNonBlocking(UART5_BASE, *p);
p++;
}
}
