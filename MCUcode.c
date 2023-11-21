#include <msp430.h> 


/**
 * main.c
 */

void SetupClock();
void SetupPins();
//void SetupLEDs()
//void SetupSwitchs();
void SetupUART();
void SetupTimerB0();
void SetupTimerB1();
void SetupTimerB2();
void SetupTimerA0();
void SetupTimerA1();
//void SetupADC();
//void SelectADCChannel();
void SetupAnalogInputs();

void UpdateCoils();
void UpdateMotors();
void RecordEncoderData();
void TransmitEncoderData();
void ControlSystem();

//Setup Circular buffer
void Enqueue();
void Dequeue();

char Queue[50];

volatile unsigned char bufferSize=0, packetSize = 0, stopIndex = 0, startIndex = 0, packetFlag = 0, packetIndex = 0, stepperDirFlag = 1, updateCoilsFlag = 0, fullPacketFlag = 0, transmitFlag = 0, state = 0, updateEncoderFlag = 0, encoderPositiveFlag = 1,enablePositionControleFlag = 0, targetPositiveFlag = 1;

volatile unsigned int packetByte = 0, stepSize = 0xffff/4, encoderPosition = 0, encoderPacketByte1 = 0, encoderPacketByte2 = 0, timerCount = 0, targetPosition = 0;
//const unsigned int FULL_ROTATION = 48;
int error = 0;
unsigned int Kp = 600;

int main(void)
{
    //Write microprocessor code to generate PWM waveforms for the H-bridge motor drivers.
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    SetupClock();

    //Run the timer in continuous mode so that the PWM duty cycle could be set with full 16-bit precision.
    SetupTimerB2();

    //Set up Stepper sources
    SetupTimerB0();
    SetupTimerB1();

    //Set up Encoder sources
    SetupTimerA0();
    SetupTimerA1();


    SetupPins();
    SetupUART();

    _EINT();    //General interrupt enable

    while(1)
    {
        //To do every cycle
        if(updateCoilsFlag)
        {
            updateCoilsFlag = 0;
            UpdateCoils();
        }
        if(transmitFlag)
        {
            transmitFlag = 0;
            TransmitEncoderData();
        }
        if(updateEncoderFlag)
        {
            updateEncoderFlag = 0;
            RecordEncoderData();

            if(enablePositionControleFlag)
            {
                ControlSystem();
            }
        }
    }

    return 0;
}

void SetupClock()
{
    CSCTL0 = 0xA500;
    CSCTL1 = DCOFSEL0 + DCOFSEL1;
    CSCTL2 = SELM0 + SELM1 + SELA0 +SELA1 + SELS0 + SELS1;
    //CSCTL3 = DIVA_4+ DIVS_4 + DIVM_4; //500kHz
}

void SetupTimerB2()
{
    //TB2CTL = CNTL_0 + TBSSEL_1 + MC_2 + TBCLR;

    //Timer control register
    TB2CTL |= TBSSEL__SMCLK  // Clock source: SMLK,
            + MC_2          // Mode: cont
            //+ ID__8         //Input clock divider so this will give a 1MHz clock
            //+ TAIE          //Enables the TAIFG interrupt (TAIFG set whenever timer hits 0, useful for continuous mode)
            + TBCLR;

    TB2CCTL1 = OUTMOD_7;
    //TB2EX0 = TBIDEX_7; //this makes it 100Khz;
    TB2CCR1 = 0xFFFF/4;
    TB2CCR2 = 0xFFFF;
    TB2CCTL2 |= CCIE;


}

void SetupTimerB0()
{
    //Set up Timer B0
    //Configure Timer period
    TB0CCR0 = 1000; // Set period to generate 500 Hz signal (assuming a 1 MHz clock)
    /****************************************************************************
     * Formula is Fcs/Ftarget - 1
     ****************************************************************************/

    //TAxCCTLn: Timer_Ax Capture/Compare Control n Register
    TB0CCTL1 = OUTMOD_7; // Set TA0.1 to set and reset on CCR1 match (useful for PWM)
          //+ CCIE;     // Capture/compare interrupt enable NOTE: CCR0 has a different vector than all other CCRx
          //+ CAP       // Set to capture mode
          //+ CCIS_0;   // Set input port
    TB0CCTL2 = OUTMOD_7;
    TB0CCR1 = TB0CCR0/4; // Set TA0.1 duty cycle to 50%
    TB0CCR2 = TB0CCR0/4;

    /****************************************************************************
     *  OUTMOD RW 0h Output mode. Modes 2, 3, 6, and 7 are not useful for TAxCCR0
            000b = OUT bit value
            001b = Set
            010b = Toggle/reset
            011b = Set/reset
            100b = Toggle
            101b = Reset
            110b = Toggle/set
            111b = Reset/set
     *  CCIS: Capture/compare input select. These bits select the TAxCCR0 input signal.
            00b = CCIxA     See Mixed-Signal guide secion 6.10.10 for input pin numbers
            01b = CCIxB
            10b = GND
            11b = VCC
     ****************************************************************************/

    //Timer control register
    TB0CTL |= TBSSEL__SMCLK  // Clock source: SMLK,
            + MC_1          // Mode: up
            // + ID__8         //Input clock divider
          //+ TAIE          //Enables the TAIFG interrupt (TAIFG set whenever timer hits 0, useful for continuous mode)
            + TBCLR;        //Clear TAR, the clock divider logic, and the count direction

    /****************************************************************************
     * MC: Mode control. Setting MC = 00h when Timer_A is not in use conserves power.
            00b = Stop mode: Timer is halted
            01b = Up mode: Timer counts up to TAxCCR0
            10b = Continuous mode: Timer counts up to 0FFFFh
            11b = Up/down mode: Timer counts up to TAxCCR0 then down to 0000h
     ****************************************************************************/
}
void SetupTimerB1()
{
    //Set up Timer B0
    //Configure Timer period
    TB1CCR0 = 1000; // Set period to generate 500 Hz signal (assuming a 1 MHz clock)
    /****************************************************************************
     * Formula is Fcs/Ftarget - 1
     ****************************************************************************/

    //TAxCCTLn: Timer_Ax Capture/Compare Control n Register
    TB1CCTL1 = OUTMOD_7; // Set TA0.1 to set and reset on CCR1 match (useful for PWM)
          //+ CCIE;     // Capture/compare interrupt enable NOTE: CCR0 has a different vector than all other CCRx
          //+ CAP       // Set to capture mode
          //+ CCIS_0;   // Set input port
    TB1CCTL2 = OUTMOD_7;
    TB1CCR1 = TB1CCR0/4; // Set TA0.1 duty cycle to 50%
    TB1CCR2 = TB1CCR0/4;

    //Timer control register
    TB1CTL |= TBSSEL__SMCLK  // Clock source: SMLK,
            + MC_1          // Mode: up
          //+ ID__8         //Input clock divider
          //+ TAIE          //Enables the TAIFG interrupt (TAIFG set whenever timer hits 0, useful for continuous mode)
            + TBCLR;        //Clear TAR, the clock divider logic, and the count direction
}
void SetupTimerA0()
{
    //TB2CTL = CNTL_0 + TBSSEL_1 + MC_2 + TBCLR;

    //Timer control register
    TA0CTL |= TASSEL__TACLK  // Clock source: SMLK,
            + MC_2          // Mode: continuous
            //+ ID__8         //Input clock divider
            //+ TAIE          //Enables the TAIFG interrupt (TAIFG set whenever timer hits 0, useful for continuous mode)
            + TACLR;

    TA0CCTL1 = OUTMOD_7;
}
void SetupTimerA1()
{
    //TB2CTL = CNTL_0 + TBSSEL_1 + MC_2 + TBCLR;

   //Timer control register
   TA1CTL |= TASSEL__TACLK  // Clock source: SMLK,
           + MC_2          // Mode: continuous
           //+ ID__8         //Input clock divider
           //+ TAIE          //Enables the TAIFG interrupt (TAIFG set whenever timer hits 0, useful for continuous mode)
           + TACLR;

   TA1CCTL1 = OUTMOD_7;
}

void SetupPins()
{
    //DC motor dircetion pins
    P3DIR |= BIT7 + BIT6;
    //Start turned off
    P3OUT &= ~(BIT7 + BIT6);

    //TB0.1 Pin (AIN2_DRV1)
    P1DIR |= BIT4;
    P1OUT &= ~BIT4;
    P1SEL0 |= BIT4;

    //TB0.2 Pin (AIN1_DRV1)
    P1DIR |= BIT5;
    P1OUT &= ~BIT5;
    P1SEL0 |= BIT5;

    //TB1.1 Pin (BIN2_DRV1)
    P3DIR |= BIT4;
    P3OUT &= ~BIT4;
    P3SEL0 |= BIT4;

    //TB1.2 Pin (BIN1_DRV1)
    P3DIR |= BIT5;
    P3OUT &= ~BIT5;
    P3SEL0 |= BIT5;

    //Setup input pins

    //Setup special pins
    P2DIR |= BIT1; // Set P2.1 as outputs
    P2SEL0 &= ~(BIT1);
    P2SEL0 |= BIT1; // P2.1 options select to output TB2.1

    //Setup external TA1CLK
    P1DIR &= ~BIT1;
    P1SEL1 |= BIT1;

    //Setup external TA0CLK
    P1DIR &= ~BIT2;
    P1SEL1 |= BIT2;
}

void SetupUART()
{
    P2SEL0 &= ~(BIT5 + BIT6);
    P2SEL1 |= BIT5 +BIT6;

    UCA1CTLW0 |= UCSWRST;                   // Put the UART in software reset
    UCA1CTLW0 |= UCSSEL0;                    // Run the UART using ACLK
    UCA1MCTLW = UCOS16 + UCBRF0 + 0x4900;   // Baud rate = 9600 from an 8 MHz clock
    UCA1BRW = 52;
    UCA1CTLW0 &= ~UCSWRST;                  // release UART for operation
    UCA1IE |= UCRXIE;                       // Enable UART Rx interrupt
}

void Dequeue()
{
    //Check errors
    if (bufferSize == 0)
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 'N';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 'o';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = ' ';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 'i';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 't';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 'e';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 'm';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 's';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = ' ';
    }
    else
    {
        //Decrement buffer
        bufferSize--;

        //Decrement packet
        if(packetFlag)
            packetSize--;

        //Save and transmit data
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = Queue[startIndex];

        //If start index at beginning of buffer roll over
        if(startIndex == 49)
        {
            startIndex = 0;
        }
        else // Else increment
            startIndex++;
    }
}

void Enqueue()
{
    //Check errors

    if (bufferSize == 50)
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 'N';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 'o';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = ' ';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 's';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 'p';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 'a';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 'c';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 'e';
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = ' ';
    }
    else if(packetFlag)
    {

        //Increment buffer
        bufferSize++;

        //Increment Packet counter
        packetSize++;

        //Save and transmit data
        //UCA1TXBUF = UCA1RXBUF;
        Queue[stopIndex] = UCA1RXBUF;

        //If stop index at end of buffer roll over
        if(stopIndex == 49)
        {
            stopIndex = 0;
        }
        else // Else increment
            stopIndex++;
    }
    else if(UCA1RXBUF == 0xff)
    {
        packetFlag = 1;             //This is the start of a bit
        //Increment buffer
        bufferSize++;

        //Increment Packet counter
        packetSize++;

        packetIndex = stopIndex;    //Save package index

        //Transmit and save data
        //UCA1TXBUF = UCA1RXBUF;
        Queue[stopIndex] = UCA1RXBUF;

        //If stop index at end of buffer roll over
        if(stopIndex == 49)
        {
            stopIndex = 0;
        }
        else // Else increment
            stopIndex++;
    }
    //If you have a full packet
    if(packetSize == 5)
    {
        UpdateMotors();
    }
}

void UpdateCoils()
{

    switch(state)
    {
        case 0:
            {
                //TB0.1 Pin (AIN2_DRV1)
                //P1DIR &= ~BIT4;
                //TB0CCR1 = 0;
                P1SEL0 &= ~BIT4;

                //TB0.2 Pin (AIN1_DRV1)
                //P1DIR |= BIT5;
                //TB0CCR2 = TB0CCR0/4;
                P1SEL0 |= BIT5;

                //TB1.1 Pin (BIN2_DRV1)
                //P3DIR &= ~BIT4;
                //TB1CCR1 = 0;
                P3SEL0 &= ~BIT4;

                //TB1.2 Pin (BIN1_DRV1)
                //P3DIR &= ~BIT5;
                //TB0CCR2 = 0;
                P3SEL0 &= ~BIT5;
            }
        break;
        case 1:
            {
                //TB0.1 Pin (AIN2_DRV1)
                //P1DIR &= ~BIT4;
                //TB0CCR1 = 0;
                P1SEL0 &= ~BIT4;

                //TB0.2 Pin (AIN1_DRV1)
                //TB0CCR2 = TB0CCR0/4;
                //P1DIR |= BIT5;
                P1SEL0 |= BIT5;

                //TB1.1 Pin (BIN2_DRV1)
                //P3DIR &= ~BIT4;
                //TB1CCR1 = 0;
                P3SEL0 &= ~BIT4;

                //TB1.2 Pin (BIN1_DRV1)
                //P3DIR |= BIT5;
                //TB1CCR2 = TB1CCR0/4;
                P3SEL0 |= BIT5;

            }
        break;
        case 2:
            {
                //TB0.1 Pin (AIN2_DRV1)
                //P1DIR &= ~BIT4;
                //TB0CCR1 = 0;
                P1SEL0 &= ~BIT4;

                //TB0.2 Pin (AIN1_DRV1)
                //P1DIR &= ~BIT5;
                //TB0CCR2 = 0;
                P1SEL0 &= ~BIT5;

                //TB1.1 Pin (BIN2_DRV1)
                //P3DIR &= ~BIT4;
                //TB1CCR1 = 0;
                P3SEL0 &= ~BIT4;

                //TB1.2 Pin (BIN1_DRV1)
                //P3DIR |= BIT5;
                //TB1CCR2 = TB1CCR0/4;
                P3SEL0 |= BIT5;
            }
        break;
        case 3:
            {
                //TB0.1 Pin (AIN2_DRV1)
                //P1DIR |= BIT4;
                //TB0CCR1 = TB0CCR0/4;
                P1SEL0 |= BIT4;

                //TB0.2 Pin (AIN1_DRV1)
                //P1DIR &= ~BIT5;
                //TB0CCR2 = 0;
                P1SEL0 &= ~BIT5;

                //TB1.1 Pin (BIN2_DRV1)
                //P3DIR &= ~BIT4;
                //TB1CCR1 = 0;
                P3SEL0 &= ~BIT4;

                //TB1.2 Pin (BIN1_DRV1)
                //P3DIR |= BIT5;
                //TB1CCR2 = TB1CCR0/4;
                P3SEL0 |= BIT5;
            }
        break;
        case 4:
            {
                //TB0.1 Pin (AIN2_DRV1)
                //P1DIR |= BIT4;
                //TB0CCR1 = TB0CCR0/4;
                P1SEL0 |= BIT4;

                //TB0.2 Pin (AIN1_DRV1)
                //P1DIR &= ~BIT5;
                //TB0CCR2 = 0;
                P1SEL0 &= ~BIT5;

                //TB1.1 Pin (BIN2_DRV1)
                //P3DIR &= ~BIT4;
                //TB1CCR1 = 0;
                P3SEL0 &= ~BIT4;

                //TB1.2 Pin (BIN1_DRV1)
                //P3DIR &= ~BIT5;
                //TB1CCR2 = 0;
                P3SEL0 &= ~BIT5;
            }
        break;
        case 5:
            {
                //TB0.1 Pin (AIN2_DRV1)
                //P1DIR |= BIT4;
                //TB0CCR1 = TB0CCR0/4;
                P1SEL0 |= BIT4;

                //TB0.2 Pin (AIN1_DRV1)
                //P1DIR &= ~BIT5;
                //TB0CCR2 = 0;
                P1SEL0 &= ~BIT5;

                //TB1.1 Pin (BIN2_DRV1)
                //P3DIR |= BIT4;
                //TB1CCR1 = TB1CCR0/4;
                P3SEL0 |= BIT4;

                //TB1.2 Pin (BIN1_DRV1)
                //P3DIR &= ~BIT5;
                //TB1CCR2 = 0;
                P3SEL0 &= ~BIT5;
            }
        break;
        case 6:
            {
                //TB0.1 Pin (AIN2_DRV1)
                //P1DIR &= ~BIT4;
                //TB0CCR1 = 0;
                P1SEL0 &= ~BIT4;

                //TB0.2 Pin (AIN1_DRV1)
                //P1DIR &= ~BIT5;
                //TB0CCR2 = 0;
                P1SEL0 &= ~BIT5;

                //TB1.1 Pin (BIN2_DRV1)
                //P3DIR |= BIT4;
                //TB1CCR1 = TB1CCR0/4;
                P3SEL0 |= BIT4;

                //TB1.2 Pin (BIN1_DRV1)
                //P3DIR &= ~BIT5;
                //TB1CCR2 = 0;
                P3SEL0 &= ~BIT5;
            }
        break;
        case 7:
            {
                //TB0.1 Pin (AIN2_DRV1)
                //P1DIR &= ~BIT4;
                //TB0CCR1 = 0;
                P1SEL0 &= ~BIT4;

                //TB0.2 Pin (AIN1_DRV1)
                //P1DIR |= BIT5;
                //TB0CCR2 = TB0CCR0/4;
                P1SEL0 |= BIT5;

                //TB1.1 Pin (BIN2_DRV1)
                //P3DIR |= BIT4;
                //TB1CCR1 = TB1CCR0/4;
                P3SEL0 |= BIT4;

                //TB1.2 Pin (BIN1_DRV1)
                //P3DIR &= ~BIT5;
                //TB1CCR2 = 0;
                P3SEL0 &= ~BIT5;
            }
        break;
        default:
            {
                //TB0.1 Pin (AIN2_DRV1)
                //P1DIR &= ~BIT4;
                //TB0CCR1 = 0;
                P1SEL0 &= ~BIT4;

                //TB0.2 Pin (AIN1_DRV1)
                //P1DIR &= ~BIT5;
                //TB0CCR2 = 0;
                P1SEL0 &= ~BIT5;

                //TB1.1 Pin (BIN2_DRV1)
                //P3DIR &= ~BIT4;
                //TB1CCR1 = 0;
                P3SEL0 &= ~BIT4;

                //TB1.2 Pin (BIN1_DRV1)
                //P3DIR &= ~BIT5;
                //TB1CCR2 = 0;
                P3SEL0 &= ~BIT5;
            }
        break;
    }
}
void UpdateMotors()
{
    //Reset Packet flag and Packet size counter
   packetFlag = 0;
   packetSize = 0;
   enablePositionControleFlag = 0;

   //Record data bytes
   unsigned int DB1, DB2;

   //Check ESC byte
   if(Queue[packetIndex + 4] & BIT0)
       DB1 = 255;
   else
       DB1 = Queue[packetIndex + 2];

   if(Queue[packetIndex + 4] & BIT1)
      DB2 = 255;
   else
       DB2 = Queue[packetIndex + 3];

   //Save Packet byte
  packetByte = DB1<<8;
  packetByte += DB2;    //Save packet

  //Forward DC motor control
   if(Queue[packetIndex + 1] == 2)
  {
     //Forward direction
       P3OUT |= BIT7;
       P3OUT &= ~(BIT6);

       //Update DC Motor speed
       TB2CCR1 = packetByte;
  }
   //Backward DC motor control
  else if(Queue[packetIndex + 1] == 3)
  {
      //Reverse direction
      P3OUT |= BIT6;
      P3OUT &= ~(BIT7);
      //Update DC Motor speed
      TB2CCR1 = packetByte;
  }
   //Shutdown DC motor control
  else if(Queue[packetIndex + 1] == 1)
  {
        //Stop DC motor
        P3OUT &= ~(BIT6);
        P3OUT &= ~(BIT7);
  }
   //Forward stepper motor control
  else if(Queue[packetIndex + 1] == 4)
    {
      //Stepper Forward
      stepperDirFlag = 0;

      //Update Freq
      stepSize = packetByte;
    }
   //Backward stepper motor control
  else if(Queue[packetIndex + 1] == 5)
    {
      //Stepper Backward
        stepperDirFlag = 1;

        //Update Freq
      stepSize = packetByte;
    }
   //Forward single step control
  else if(Queue[packetIndex + 1] == 6)
  {
    //Stepper step forward
      stepperDirFlag = 2;
  }
   //Backward signle step control
  else if(Queue[packetIndex + 1] == 7)
  {
    //Stepper step backward
      stepperDirFlag = 3;
  }
   //Shutdown Stepper motor control
  else if(Queue[packetIndex + 1] == 8)
  {
    //Stepper Stop
      stepperDirFlag = 4;
  }
   //Positive DC motor position control
  else if(Queue[packetIndex + 1] == 9)
   {
       enablePositionControleFlag = 1;
       targetPosition = packetByte;
       targetPositiveFlag = 1;
   }
   //Negative DC motor position control
  else if(Queue[packetIndex + 1] == 10)
 {
     enablePositionControleFlag = 1;
     targetPosition = packetByte;
     targetPositiveFlag = 0;
 }
   //setting Kp
  else if(Queue[packetIndex + 1] == 11)
   {

       Kp = packetByte;

   }


    int i;
    for(i=0; i < 5; i++)
        Dequeue();
}
void RecordEncoderData()
{
    //Send Data bytes
     if(TA0R > TA1R)
     {
         packetByte = TA0R - TA1R;

         //Indicate positive direction with control bit
         encoderPositiveFlag = 1;
     }
     else
     {
         packetByte = TA1R - TA0R;

         //Indicate negative direction with control bit
         encoderPositiveFlag = 0;
     }
}


void TransmitEncoderData()
{
    unsigned char DB = 0, endByte = 0;

    //Send start Byte
    while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = 255;

    if(encoderPositiveFlag)
    {
        while(!(UCA1IFG & UCTXIFG));
          UCA1TXBUF = 0;
    }
    else
    {
        while(!(UCA1IFG & UCTXIFG));
          UCA1TXBUF = 1;
    }

    DB = packetByte>>8;
    if(DB == 255)
    {
        endByte |= BIT1;
        while(!(UCA1IFG & UCTXIFG));
            UCA1TXBUF = 0;
    }
    else
    {
        while(!(UCA1IFG & UCTXIFG));
            UCA1TXBUF = DB;
    }

    DB = packetByte & 0xff;
    if(DB == 255)
    {
        endByte |= BIT0;
        while(!(UCA1IFG & UCTXIFG));
            UCA1TXBUF = 0;
    }
    else
    {
        while(!(UCA1IFG & UCTXIFG));
            UCA1TXBUF = DB;
    }

    //Send end byte
    while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = endByte;
}
void ControlSystem()
{
    unsigned int command;
    //Calculate error (signed int)
    if(targetPositiveFlag && encoderPositiveFlag)
        error = targetPosition - packetByte;
    else if(!targetPositiveFlag && !encoderPositiveFlag)
        error = targetPosition - packetByte;
    else
        error = targetPosition + packetByte;

    if (abs(error)<=0xFFFF/Kp){

        command = Kp*abs(error);
    }
    else
        command = 0xFFFF;
    
    TB2CCR1 = command;

    //Update DC motor direction
    if(targetPositiveFlag && error>=0)
     {
        //Forward direction
          P3OUT |= BIT7;
          P3OUT &= ~(BIT6);
     }
    else if (!targetPositiveFlag && error<=0)
    {
        //Forward direction
          P3OUT |= BIT7;
          P3OUT &= ~(BIT6);
     }
    else
    {
        //Backward direction
          P3OUT &= ~BIT7;
          P3OUT |= (BIT6);
    }

    //Update DC motor duty cycle


}
#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{

        if(UCA1RXBUF == 13)
        {
            Dequeue();
        }
      else
        {
            Enqueue();
        }
}

#pragma vector = TIMER2_B1_VECTOR
__interrupt void Timer0_B1_ISR(void)
{
    //Reset interrupt flag
    TB2IV &= ~TB0IV_TBCCR2;

    //Freq
    TB2CCR2 += stepSize;

    if(stepperDirFlag == 0) //If moving forward
    {
        //Increment State
        if(state >= 7)
            state = 0;
        else
            state++;
    }
    else if(stepperDirFlag == 1)    //If moving backward
    {
        if(state == 0)
            state = 7;
        else
            state--;
    }
    else if (stepperDirFlag == 2){
        if(state >= 7)
            state = 0;
        else
            state++;
        stepperDirFlag = 4;

    }
    else if (stepperDirFlag == 3){
        if(state == 0)
                    state = 7;a
                else
                    state--;
            stepperDirFlag = 4;
    }
    else
        state == 8;

    updateCoilsFlag = 1;

    //Update encoder count
    updateEncoderFlag = 1;
    //Transmit Encoder Data
    if(timerCount == 3)
    {
        timerCount = 0;
        transmitFlag = 1;
        //TransmitEncoderData();
    }

    timerCount ++;
}
