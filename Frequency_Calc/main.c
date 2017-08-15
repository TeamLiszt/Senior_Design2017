/*
 * main.c
 *
 *  Last Edited: August 15, 2017
 *      Author: James Bowen and Cameron Bailey
 *
 *  MSP430 USERS_MANUAL URL
 *    http://www.ti.com/lit/ug/slau049f/slau049f.pdf
 *
 */

#include  <msp430x26x.h>
#include <stdlib.h>   // atoi()
#include <stdio.h>    // sprintf()
#include <string.h>
#include "Gfa.h"
#include "time.h"

int  Assign(char option, long value, int Count);
void InitSystem(void);
void InitTimers(void);
void InitUart(void);
int  ParseString(char * theString, int index);
void PrintDate(int month, int day, int year);
void PrintDec(long value);
void PrintTime(int hour, int min, int sec);
void PrintString(char * theString);
void SerialReply(char option);
int  ServiceSerial(void);
void Set_DCO(void);
void ShowSingleVariable(char option);
int  Uart1TxChar(uint8 c);
long CalcPeriod(void);
float CalcFrequency (float period);
void ResetTimerA(void);


// Global Variables
int TxCharCount = 0, TxReadIndex = 0, TxWriteIndex = 0;
int RxWriteIndex = 0, RxReadIndex = 0;
char TxBuffer[SER_BUFFER_SIZE], RxBuffer[SER_BUFFER_SIZE];
int ISR_Flag = 0;
tm_t clock_Time = {0, 0, 12, 7, 11, 2016, 1};   //(sec, min, hr, day, mon, year,0);

int buffer0[Num_of_Results];
char message[SER_BUFFER_SIZE];
long array[];
long print_flag = 0;

/****************************************************************************/
/*  Main function                                                           */
/*  Function : main                                                         */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void main(void)
{
  long Period = 5333333, PeriodAvg = 5333333, ChargeCost = 0;
  int battCurrent = 0, battSOC = 20, TotWsec = 0;
  float period = 0;
  float frequency = 0;

  InitSystem();
  InitUart();
  InitTimers();                 // Enable tic timer
  P1IES &= ~BIT4;                        //Initialize the system to look for a rising edge on port 1

  __enable_interrupt(); // Enable Global Interrupts

   while(1)
   {
       if(print_flag == 2)              // Flag is only set in P1_ISR, flag is cleared after this if
       {
           __disable_interrupt();       // Disable interrupts so flag is unchanged until done printing
           // P1IE &= (~BIT1);          // Turn off P1 interrupts so we can use UART
           P1OUT ^= BIT0;               // Toggle LED to show we getting interrupts
           period = CalcPeriod();       // Calculates period and returns a float
           frequency = CalcFrequency(period);       // Calculates frequency and returns a float

           //----------------PRINT OUT ACROSS UART---------------------
           tm2time(&clock_Time);
           sprintf(message,"X= %ld, %ld, %f, %f, %d, %ld, ", array[0], array[1], period, frequency, TotWsec, ChargeCost);        // SOC
           PrintString(message);
           sprintf(message, "%02d:%02d:%02d %02d/%02d/%02d\r\n", (int)clock_Time.tm_hour, (int)clock_Time.tm_min, (int)clock_Time.tm_sec, (int)clock_Time.tm_mon, (int)clock_Time.tm_mday, (int)clock_Time.tm_year);        PrintString(message);
           clock_Time.tm_sec += 1;

           print_flag = 0;            // Reset P1 interrupt flag
//           _delay_cycles(1000000);      // slow output data and blinks an LED
           ResetTimerA();               // Resets TimerA to prevent roll over calculation errors

           __enable_interrupt();    //Re-enable interrupts
       }
   }
}// main()

/************************************   Assign ******************************
Description: Assign takes the parsed commands and changes the operation of
             the instrument.
****************************************************************************/
int Assign(char option, long value, int Count)
{
  int status = ERR_OK;
  char message[SER_BUFFER_SIZE];

  switch (option)       {
    case 'd':  // Update Clock
    case 'D':
        if      (Count == 1)    clock_Time.tm_hour = (uint8)(value);
        else if (Count == 2)    clock_Time.tm_min  = (uint8)(value);
        else if (Count == 3)    clock_Time.tm_sec  = (uint8)(value);
        else if (Count == 4)    clock_Time.tm_mday = (uint8)(value);
        else if (Count == 5)    clock_Time.tm_mon  = (uint8)(value);
        else if (Count == 6)    clock_Time.tm_year = value;
        else if (Count == 7)    clock_Time.tm_isdst= (uint8)value;
        break;
      case 'f':             //  SampleRate
      case 'F':
//        TBCCR2 = (int)value;
//        TBCCR0 = (int)value;
        break;
      case 'h':             //  HeadPhone ON/OFF control
      case 'H':
//          if      (value == 1) P2OUT &= ~BIT6;
//          else if (value == 0) P2OUT |= BIT6;
          break;
      default:
          sprintf (message, "%c:%d Invalid Assignment\r\n", option, value);
          PrintString(message);
          status = ERR_VALUE;
          break;
  }
  return (status);
}

/*****************************************************************************
    InitUART function - set constants & I/O
   Description: Echo a received character, RX ISR used. Normal mode is LPM0.
   USCI_A1 RX interrupt triggers TX Echo.
   Baud rate divider with 1MHz = 1MHz/115200 = ~8.7 - MUST CHANGE if DCOCTL changes
*****************************************************************************/
void InitUart(void) {
  P3SEL = 0xC0;                             // P3.6,7 = USCI_A1 TXD/RXD
  UCA1CTL1 |= UCSSEL_2;                     // SMCLK
  UCA1BR0 = 69;                              // 8MHz 115200
  UCA1BR1 = 0;                              // 8MHz 115200
  UCA1MCTL = UCBRS2;                        // Modulation UCBRSx = 4 for 8MHz
  UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UC1IE |= UCA1RXIE;                          // Enable USCI_A0 RX interrupt
}


/*****************************************************************************
    InitSystem function - set constants & I/O
*****************************************************************************/
void InitSystem(void) {
  unsigned int i;

  WDTCTL = WDTPW + WDTHOLD;     // Stop watchdog timer

  BCSCTL1 = CALBC1_8MHZ;                    // set first functionality to all ports
  DCOCTL = CALDCO_8MHZ;                       // Set DCO to 1MHz , this should be 8 Mhz and the clock is divided to 1Mhz in inittimer function
  BCSCTL2 = 0;                  // SMCLK = MCLK = DCOCLK / 1
  for (i = 0xfffe; i > 0; i--);             // Delay for XTAL stabilization

  P1SEL = 0x00;
  P2SEL = 0x00;     // P2SEL must have BIT6/7 HIGH to enable 32kHz crystal, ACLK Active, P2.3 - P2.4 TA
  P3SEL = 0x00;
  P4SEL = 0x00;
  P5SEL = 0x00;
  P6SEL = 0x01;
  P7SEL = 0x00;
  P8SEL = 0x00;

  P1SEL &= (~BIT1); // Set P1.1 SEL as GPIO (Setting to a 1 disables P1 interrupts, setting to 0 enables interrupts, currently set to 0)
  P1DIR |= (BIT0);   // Set P1.0 as Output (pin 12 on the board)
  P1DIR &= (~BIT1); // Set P1.1 SEL as Input (pin 13 on the board)
  P1IES |= (BIT0); // Rising Edge Interrupt
  P1IFG &= (~BIT1); // Clear interrupt flag
  P1IE |= (BIT1); // Enable interrupt for Port 1

  P2DIR |= BIT0; // Set P1.0 output direction
  P2OUT &= ~BIT0; // Set the LEDs off

  TACCTL0 = 0x00;                // TACCR1 toggle, interrupt enabled
  TACCTL1 = 0x00;
  TACCTL2 = 0x00;
  TBCCTL0 = 0x00;
  TBCCTL1 = 0x00;
  TBCCTL2 = 0x00;

  ADC12MCTL0 = 0x0001+SREF_1;                       //
  ADC12IFG = 0x00;
  ADC12CTL1 = SHS_3 + CONSEQ_2 + ADC12SSEL_3;             // S&H TB.OUT1, rep. single chan, Use SMCLK
  ADC12CTL0 = REF2_5V + REFON + ADC12ON + ENC;    // VRef ADC12 on, enabled

  DMACTL0 = DMA0TSEL_6;

  DMA0SA = (void (*)())&ADC12MEM0;          // DMA source address is the adc12 module
  DMA0DA = (void (*)())&buffer0[0];         // DMA source desitination is buffer 0
  DMA0SZ = 0x800;                               // set DMA transfer size to 2048
  DMA0CTL = DMADSTINCR_3 + DMADT_4 + DMAIE + DMAEN;                // Sng rpt, config
}

/***************************************************************************
                       InitTimers
    Description: This function initializes the tic timer
                 TimerA is set as a up count timer with ACLK (32KHz)
                   as source, count up to value in TACCR0,
                   interrupt on compare (2 Hz)

    This must be called after SetDCO
***************************************************************************/
void InitTimers(void)
{
  TACTL = 0;                            // Resets all bits in the Timer A register
  TACTL = MC_2 + TASSEL_2 +TACLR + ID_3;

//  TBCCTL0 = CCIE;
//  TBCTL = TBSSEL_2 + MC_1; // Set the timer A to SMCLCK, UpMode
  // Clear the timer and enable timer interrupt
//  TBCCR0 = 5333333;
}

/****************************************************************************
                       ResetTimerA
    Description: This function resets TimerA to account for the roll over
                 TimerA will exerpience once the 16bit register is full
 ****************************************************************************/
void ResetTimerA(void)
{
    TACTL = 0;                            // Resets all bits in the Timer A register
    TACTL = MC_2 + TASSEL_2 +TACLR + ID_3;
}

/****************************************************************************
                            ParseString
Description: Separate leading character (command) and terminator '=' from
             the data.  Data is comma separated.
****************************************************************************/
int ParseString(char * theString, int index)
{
    long value;
    int Count = 0;
    char action[SER_BUFFER_SIZE + 1];
    char *pTest;

    pTest = strtok(theString, ",");
    do  {
        strncpy(&action[0], pTest, (unsigned long)index + 1);
        if (Count == 0) value = atoi(&action[2]);
        else            value = atoi(&action[0]);
        Assign((char)theString[0], value, Count);
        Count++;
        pTest = strtok(NULL,",");
    }   while (pTest != NULL);
    return (Count);
}


/****************************************************************************
                            PrintDate
Description: Prints a formatted date.
****************************************************************************/
void PrintDate(int month, int day, int year)
{
    int i, j;
    char digit[11];

    digit[0] = ' ';
    digit[1] = '0' + (unsigned char) (month / 10);
    digit[2] = '0' + (unsigned char) (month % 10);
    digit[3] = '/';
    digit[4] = '0' + (unsigned char) (day / 10);
    digit[5] = '0' + (unsigned char) (day % 10);
    digit[6] = '/';
    digit[7] = '0' + (unsigned char) (year / 1000);
    i = year / 1000;
    i *= 1000;
    j = year - i;
    digit[8] = '0' + (unsigned char) (j / 100);
    i = j / 100;
    i *= 100;
    j -= i;
    digit[9] = '0' + (unsigned char) (j / 10);
    digit[10] = '0' + (unsigned char) (j % 10);
    strcat(message, digit);   //  PrintString(digit);
}


/****************************************************************************
                            PrintDec
Description: Prints a long value as a decimal.
****************************************************************************/
void PrintDec(long value)
{
    #define MAX_DECIMAL 12

    int i, NegFlag = 0;
    long iInt = value;
    char digit[MAX_DECIMAL + 1];

    iInt = value;
    if (iInt & 0x80000000)      {
            iInt ^= 0xFFFFFFFF;
            iInt += 1;
            NegFlag = 1;
    }
    digit[MAX_DECIMAL] = '\0';
    for (i = MAX_DECIMAL - 1; i >= 0; i--)  {
        digit[i] = '0' + (unsigned char) (iInt % 10);
        iInt = iInt / 10;
        if (iInt == 0) break;
    }
    if (NegFlag)   digit[--i] = '-';
    strcat(message, &digit[i]); // PrintString(&digit[i]);
}


/****************************************************************************
                            PrintTime
Description: Prints a formatted time.
****************************************************************************/
void PrintTime(int hour, int min, int sec)
{
    char digit[9];

    digit[0] = ' ';
    digit[1] = '0' + (unsigned char) (hour / 10);
    digit[2] = '0' + (unsigned char) (hour % 10);
    digit[3] = ':';
    digit[4] = '0' + (unsigned char) (min / 10);
    digit[5] = '0' + (unsigned char) (min % 10);
    digit[6] = ':';
    digit[7] = '0' + (unsigned char) (sec / 10);
    digit[8] = '0' + (unsigned char) (sec % 10);
    strcat(message, digit);     // PrintString(digit);
}


/******************************* PrintString ********************************

Description: Print null terminated ascii string to USB port
****************************************************************************/
void PrintString(char * theString)
{
  unsigned int i;
  int stringLength = strlen(theString);

  for (i = 0; i < stringLength; i++)    {
    if (theString[i] == '\0') break;
    while (!(UC1IFG & UCA1TXIFG));
    UCA1TXBUF = theString[i];
    // Uart1TxChar(theString[i]);
  }
}

// *********************** SerialReply **************************************
//  Provide a mechanism to query the system
// **************************************************************************
void SerialReply(char option)
{
  switch (option)
  {
      case 'p': // Recording Period Dwell
      case 'P':
          PrintString("P=");
          PrintString("%\r\n");
          break;

      default:
       PrintString("---\r\n");
       break;
  }
}

/****************************************************************************
                      ServiceSerial
Description: Process received USB data
****************************************************************************/
int ServiceSerial(void)
{
  static int menuFlag = 1, CommaCnt = 0;
  static uint8 theChar, index = 0;
  static char rxBuff[SER_BUFFER_SIZE];
  int status = 0;

  while (RxWriteIndex != RxReadIndex)   {
    theChar = RxBuffer[RxReadIndex];
    RxReadIndex = ((RxReadIndex + 1) & (SER_BUFFER_SIZE - 1));
    switch(theChar)
    {
      case 0x00:
          break;
      case CR:    // carriage return
          menuFlag = 0;
          PrintString("\r\n");
          rxBuff[index] = (char)0x00;   // Null terminate the string
          if (rxBuff[1] == '=') {
             status = ParseString((char *) rxBuff, index);
          }
          else if (rxBuff[1] == '?')    {
             SerialReply((char)rxBuff[0]);
          }
          else  status = -1;
            for (index = 0; index < (SER_BUFFER_SIZE - 1); index++) rxBuff[index] = '\0';
            index = 0;
            break;
      case ',':
          rxBuff[index] = (char)theChar;
          index++;
          CommaCnt++;
          Uart1TxChar(theChar);
          break;
      case BS:    // backspace
          if (index > 0)        {
            index--;
            rxBuff[index] = (char) 0x00;
          }
          Uart1TxChar(theChar);
          break;
      default:
          rxBuff[index] = (char) theChar;
          index++;
          index &= (SER_BUFFER_SIZE - 1);
          Uart1TxChar(theChar);
          break;
    }
    if (0 == menuFlag) break;
  } // while (RxWriteIndex != RxReadIndex);
  return ( (int) status);
}

// Load TxBuffer and enable TX interrupt to send character out the USB Serial Port
int Uart1TxChar(uint8 c)
{
  UC1IE &= ~UCA1TXIE;
  TxBuffer[TxWriteIndex++] = c;
  TxWriteIndex &= (SER_BUFFER_SIZE - 1);
//  TxCharCount++;
  UC1IE |= UCA1TXIE;
  return (int)TxWriteIndex;
}

//***************************************************************************
// Timer A1 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
  __disable_interrupt();

  __enable_interrupt();

  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}

//***************************************************************************
#pragma vector=USCIAB1RX_VECTOR
__interrupt void USCI1RX_ISR(void)
{
  ISR_Flag |= ISRFLG_USB_BIT;
  while (!(UC1IFG & UCA1TXIFG));                // USCI_A0 TX buffer ready?

  RxBuffer[RxWriteIndex++] = UCA1RXBUF;
  RxWriteIndex &= (SER_BUFFER_SIZE - 1);

  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}


//***************************************************************************
#pragma vector=USCIAB1TX_VECTOR
__interrupt void USCI1TX_ISR(void)
{
  if (TxReadIndex != TxWriteIndex)  {
    UCA1TXBUF = TxBuffer[TxReadIndex++];
    TxReadIndex &= (SER_BUFFER_SIZE -1);
//    TxCharCount--;
  }
  else UC1IE &= ~UCA1TXIE;
  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}


// DMA interrupt service routine
#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void)
{
  DMA0CTL &= ~DMAEN;                        // Disables DMA
  DMA0CTL &= ~DMAIFG;                       // Clear DMA1 interrupt flag
//  ISR_Flag |= ISRFLG_DMA_BIT;

  DMA0DA = (void(*)())&buffer0[0];

  DMA0CTL |= ~DMAEN;                        // Enable DMA

  _bic_SR_register_on_exit(LPM0_bits);
}

#ifdef UART_PRINTF
int fputc(int _c, register FILE *_fp)
{
    while (!(UC1IFG & UCA1TXIFG));                // USCI_A0 TX buffer ready?
    UCA1TXBUF = (unsigned char) _c;
    return (unsigned char)(_c);
}

int fputs(const char *_ptr, register FILE *_fp)
{
    unsigned int i, len;

    len = strlen(_ptr);
    for (i=0; i<len; i++)
    {
        while (!(UC1IFG & UCA1TXIFG));                // USCI_A0 TX buffer ready?
        UCA1TXBUF = (unsigned char) _ptr[i];
    }
    return len;
}
#endif


// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    __disable_interrupt();

    if(print_flag == 0)
    {
       array[0] = TAR;
        //array[0] = TAR;
    }
    else if (print_flag == 1)
    {
        array[1] = TAR;
        //array[1] = TAR;
    }

    print_flag = print_flag + 1;       // Flag for calculating values and sending across UART in main

    P1IFG &= (~BIT1); // P1.1 IFG clear

    __enable_interrupt();
}

/*
#pragma vector=TIMERB0_VECTOR
__interrupt void Timer_B (void)
{
    __disable_interrupt();

    P2OUT ^= BIT0;          // Toggles Port 2 pin 0 to create a square wave, fed into port 1

    __enable_interrupt();
}
*/

long CalcPeriod(void)
{
    float period = 0;

    period = array[1] - array[0];

    return period;
}

float CalcFrequency (float period)
{
    float frequency = 0;

    frequency = period / 1000000;

    frequency = 1 / frequency;

    return frequency;

}
