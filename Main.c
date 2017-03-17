/*
 * main.c
 *
 *  Created on: March 2016
 *      Author: Aaron Miyasaki and Jacob Greig-Prine
 */

#include <msp430.h>
#include <stdlib.h>   // atoi()
#include <stdio.h>    // sprintf()
#include <string.h>
#include "sel.h"
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

// Global Variables
int TxCharCount = 0, TxReadIndex = 0, TxWriteIndex = 0;
int RxWriteIndex = 0, RxReadIndex = 0;
char TxBuffer[SER_BUFFER_SIZE], RxBuffer[SER_BUFFER_SIZE];
int ISR_Flag = 0;
tm_t clock_Time = {30, 59, 23, 7, 12, 2016, 1}; //(sec, min, hr, day, mon, year,0);

int offset[6] = {0};
int buffer0[100];
int buffer1[100];
int buffer2[100];
int buffer3[100];
int buffer4[100];
int zeros[100];
long frequency[100];
int count = 0;
int buf_num = 4;
int tick[1] = {1};

#define UART_PRINTF  // routes printf to I/O pins instead of console
#ifdef UART_PRINTF
int fputc(int _c, register FILE * _fp);
int fputs(const char *_ptr, register FILE *_fp);
#endif

/****************************************************************************/
/*  Main function                                                           */
/*  Function : main                                                         */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void main(void)
{
  char message[SER_BUFFER_SIZE];
  long Period = 5333333, PeriodAvg = 5333333, ChargeCost = 0;
  int battCurrent = 0, battSOC = 20, TotWsec = 0;
  int LoopCount = 0;


  InitSystem();
  InitUart();
  InitTimers();                 // Enable tic timer

  while (1) {
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts
    if (ISR_Flag & ISRFLG_USB_BIT)  {
      ISR_Flag &= ~ISRFLG_USB_BIT;
      ServiceSerial();
    }
    if (ISR_Flag & ISRFLG_TWOHZ) {
      LoopCount++;
      ISR_Flag &= ~ISRFLG_TWOHZ;
    }

    if (LoopCount == 0x02) {
        P1OUT ^= BIT0;
        LoopCount = 0;
        tm2time(&clock_Time);
//    if ((IO0PIN & DEMO_GFA_EVENT) == 0)   Period += 2000;     // about 0.02 Hz
//          PrintString("X= ");
//      PrintDec(Period);
//      sprintf(message, "X= %ld, ", Period);
//      PrintString(message);
//      PrintString(", ");
//      PrintDec(PeriodAvg);
        sprintf(message,"X= %ld, %ld, %d, %d, %d, %ld, ", Period, PeriodAvg, battCurrent, battSOC, TotWsec, ChargeCost);        // SOC
        PrintString(message);
        sprintf(message, "%02d:%02d:%02d %02d/%02d/%02d\r\n", (int)clock_Time.tm_hour, (int)clock_Time.tm_min, (int)clock_Time.tm_sec, (int)clock_Time.tm_mon, (int)clock_Time.tm_mday, (int)clock_Time.tm_year);
//      PrintDec(ChargeCost);
//      PrintString(",");
//      PrintTime((int)clock_Time.tm_hour, (int)clock_Time.tm_min, (int)clock_Time.tm_sec);
//      PrintDate((int)clock_Time.tm_mon, (int)clock_Time.tm_mday, (int)clock_Time.tm_year);
        PrintString(message);
//      PrintString("\r\n");
        clock_Time.tm_sec += 1;
        printf("Hello World\r\n");
    }
  }  // while(1)
}  // main()

/************************************ Assign ******************************
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

/***************************************************************************
                     freqCalc
     Description: This function takes the digital voltage values read
                  into the chip and converts it into frequency based
                  off of the sample rate (78125 Hz) of the ADC.
****************************************************************************/
void freqCalc(volatile unsigned int voltage[Num_of_Results], int *offset, long frequency[Num_of_Results], int *tick)
{
    int count = 0, i = 0, cur = 0, prev = 0, next = 0, start = 0, zeros[Num_of_Results], cur2 = 0, prev2 = 0, upper = 500, lower = 100;
    long period = 0;

    if (offset[0] != 0)
    {
        zeros[count] = offset[0];
        count++;
        zeros[count] = offset[1];
        count++;
        frequency[0] = frequency[Num_of_Results - 1];
        start = 1;
    }

    else
    {
        frequency[0] = 60;
        frequency[1] = 60;
        frequency[2] = 60;
        frequency[3] = 60;
        start = 4;
    }

    for (i = start; i < Num_of_Results; i++)
    {
        if (i == 0)
        {
            cur = offset[4];
            prev = offset[2];
        }

        else if (i == 1)
        {
            cur = offset[5];
            prev = offset[3];
        }

        else if (i == 2)
        {
            cur = voltage[i - 2];
            prev = offset[4];
        }

        else if (i == 3)
        {
            cur = voltage[i - 2];
            prev = offset[5];
        }

        else
        {
            cur = voltage[i - 2];
            prev = voltage[i - 4];
        }

        next = voltage[i];

        if ((cur < prev) && (next > cur) && (cur < 3000))
        {
            zeros[count] = i;
            if (count < 1)
            {
                count++;
            }

            else if (abs(zeros[count] - zeros[count -1]) > 40)
            {
                count++;
            }

            if (count > 2)
            {
                cur2 = zeros[count-1];
                prev2 = zeros[count-3];
                period = (cur2-prev2);
                if ((upper>period)&&(lower<period)){
                frequency[i] =  (tick[0] * period);
                    /*if(tick[0] == 1){
                        tick[0] = 5333333/frequency[i];
                    } */

                }
                else if(period>upper){
                    frequency[i]=upper;
                }
                else if (period<lower){
                    frequency[i]=lower;
                }

            }

            else
            {
                frequency[i] = frequency[i-1];
            }
        }

        else
        {
            frequency[i] = frequency[i-1];
        }
    }

    offset[0] = zeros[count - 2] - Num_of_Results;
    offset[1] = zeros[count - 1] - Num_of_Results;
    offset[2] = voltage[Num_of_Results - 4];
    offset[3] = voltage[Num_of_Results - 3];
    offset[4] = voltage[Num_of_Results - 2];
    offset[5] = voltage[Num_of_Results - 1];
}

double voltConv(volatile unsigned int voltage[Num_of_Results])
{
    int peak = 0, i = 0;
    double vrms = 0.0;

    for (i = 0; i < Num_of_Results; i++)
    {
        if (peak < voltage[i]) peak = voltage[i];
    }

    vrms = 0.000732*peak*sqrt(2);

    return vrms;
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
  UCA1BR0 = 8;                              // 1MHz 115200
  UCA1BR1 = 0;                              // 1MHz 115200
  UCA1MCTL = UCBRS2 + UCBRS0;               // Modulation UCBRSx = 5
  UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UC1IE |= UCA1RXIE;                          // Enable USCI_A0 RX interrupt
}


/*****************************************************************************
    InitSystem function - set constants & I/O
*****************************************************************************/
void InitSystem(void) {
  unsigned int i;

  WDTCTL = WDTPW + WDTHOLD;     // Stop watchdog timer
  // set first functionality to all ports
  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
  DCOCTL = CALDCO_1MHZ;
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

  //set pin direction and initialize outputs
  P1DIR |= 0x09;                            // P1.3 output
  TACCTL0 = 0x00;                // TACCR1 toggle, interrupt enabled
  TACCTL1 = 0x00;
  TACCTL2 = 0x00;
  TBCCTL0 = 0x00;
  TBCCTL1 = 0x00;
  TBCCTL2 = 0x00;

    ADC12MCTL0 = 0x0001;                       //
    ADC12IFG = 0x00;
    ADC12CTL1 = SHS_3 + CONSEQ_2;             // S&H TB.OUT1, rep. single chan
    ADC12CTL0 = REF2_5V + REFON + ADC12ON + ENC;    // VRef ADC12 on, enabled

    DMACTL0 = DMA0TSEL_6;

          DMA0SA = (void (*)())&ADC12MEM0;                       // Src address = ADC12 module (start block address)
          DMA0DA = (void (*)())&buffer0[0];                      // Destination Block address
          DMA0SZ = 0x64;                                         // Size set to 64 bits or 1 word
          DMA0CTL = DMADSTINCR_3 + DMADT_4 + DMAIE + DMAEN;      // Sng rpt, config
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
  TACTL = 0;
  TACTL = TASSEL_2 + TACLR + ID_3;      // SMCLK/8, clear TAR
  TACCTL0 = CCIE;                       // CCR0 interrupt enabled
  TACCR0 = 50000;
  TACTL |= MC1;                         // Start Timer_a in upmode

  TBCCR0 = 100;                             // Init TBCCR0 in up mode w/ sample period (what are we setting the period to?)
  TBCCR1 = 100 - 50;                        // Trigger for ADC12 SC
  TBCCTL1 = OUTMOD_7;                       // Reset OUT1 on EQU1, set on EQU0

  TBCTL = TBSSEL_2 + MC_1 + TBCLR;          // SMCLK, clear TBR, up mode
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

    PrintString(digit);
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
//  int zeroFlag = 0;

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
    PrintString(&digit[i]);
}


/****************************************************************************
                            PrintTime
Description: Prints a formatted time.
****************************************************************************/
void PrintTime(int hour, int min, int sec)
{
    char digit[9];

    digit[0] = ' ';
    digit[1] = '0' + (char) (hour / 10);
    digit[2] = '0' + (char) (hour % 10);
    digit[3] = ':';
    digit[4] = '0' + (char) (min / 10);
    digit[5] = '0' + (char) (min % 10);
    digit[6] = ':';
    digit[7] = '0' + (char) (sec / 10);
    digit[8] = '0' + (char) (sec % 10);

    PrintString(digit);
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
    Uart1TxChar(theString[i]);
  }
}

// *********************** SerialReply **************************************
//  Provide a mechanism to query the system
// **************************************************************************
void SerialReply(char option)
{
//  uint16 crc = 0;

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
  TxCharCount++;
  UC1IE |= UCA1TXIE;
  return (int)TxWriteIndex;
}

//***************************************************************************
// Timer A1 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
  P1OUT ^= 0x08;                            // Toggle P1.3
  ISR_Flag |= ISRFLG_TWOHZ;
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
  if (TxCharCount > 0)  {
    UCA1TXBUF = TxBuffer[TxReadIndex++];
    TxReadIndex &= (SER_BUFFER_SIZE -1);
    TxCharCount--;
  }
  if (TxCharCount == 0) UC1IE &= ~UCA1TXIE;
  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}

// Port 1 interrupt service routine *******************************
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  P1IFG &= ~0x01;                          // P1.0 IFG cleared
  ISR_Flag |= BIT1;
  __bic_SR_register_on_exit(LPM4_bits + GIE);     // Exit LPMx mode
}


// DMA interrupt service routine
#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void)
{
  DMA0CTL &= ~DMAEN;                        // Disables DMA
  DMA0CTL &= ~DMAIFG;                       // Clear DMA1 interrupt flag
 
  //Any Change of Address's or Memory Allocation should be done below while DMAEN is disabled
  
  DMA0CTL |= DMAEN;                         

  //_BIC_SR_IRQ(LPM0_bits);                   
  _bic_SR_register_on_exit(LPM0_bits);      //Turns off Low Power Mode and re-enables CPU
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
/*#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR (void)
{

    __bic_SR_register_on_exit(LPM0_bits)

} */
