#include <msp430.h>
#include <stdint.h>

const float Kp = 0.5;                           // Proportional Term
const float Ki = 0.07;                          // Integral Term
const float Kd = 0.3;                           // Derivative Term
const int OutMax = 998, OutMin = 0;             // Max/Min Duty Cycle Possible
float Up;
int Out;
int uart_count = 0;
int seconds_count = 10;
int ADC_count = 0;
float x1 = 0, x2 = 0, x3 = 0, x4 = 0, x5 = 0, x6 = 0, x7 = 0, x8 = 0, x9 = 0, x10 = 0, y;
//bi constants set for FIR filter
const float a1 = 0.25;
const float a2 = 0.20;
const float a3 = 0.15;
const float a4 = 0.1;
const float a5 = 0.05;
const float a6 = 0.05;
const float a7 = 0.05;
const float a8 = 0.05;
const float a9 = 0.05;
const float a10 = 0.05;

volatile float tempVolt, potVolt;
float temp, temp1;
float ADC_Voltage, Vp;
float Ud, error, i_error, OutPreSat, p_error;
float Ui = 0;
float desiredTemp, prevTemp;                     // Desired temperature
uint16_t test;

void TimerA0Setup(){
    //Set P1.0, P4.7, P1.2 to the output direction.
    P4DIR |= BIT0;
    P4DIR |= BIT7;
    P1DIR |= BIT2;
    P1SEL |= BIT2;
    TA0CCTL1 = OUTMOD_7; // Enable interrupt in compare mode
    TA0CCR0 = 999;              // Set period of CCR0 1000 microseconds.
    TA0CCR1 = 500;              // Set duty cycle to 50%
    TA0CTL = TACLR;             // Clear flag
    TA0CTL = TASSEL_1 + MC_1;   // SMCLK, UP
}

int updatePWM(void)
{
    // Temperature calculations
    tempVolt = ADC12MEM0;
    potVolt = ADC12MEM2;

    potVolt = ((potVolt*3.3)/4096);
    desiredTemp = ((potVolt)*30);
    temp1 = ((tempVolt*3.3)/4096);
    temp1 = (temp1*100);

    // samples are shifted up
    x10 = x9;
    x9 = x8;
    x8 = x7;
    x7 = x6;
    x6 = x5;
    x5 = x4;
    x4 = x3;
    x3 = x2;
    x2 = x1;
    x1 = temp1;

    // FIR filter input: x  output: temp
    temp = (x1*a1) + (x2*a2) + (x3*a3) + (x4*a4) + (x5*a5) + (x6*a6) + (x7*a7) + (x8*a8) + (x9*a9) + (x10*a10);

    p_error = error;                        // Track previous Up and current Up
    error = temp - desiredTemp;             // Compute the error
    Up = Kp * error;                        // Compute Proportional Control
    Ui += (Ki * (error * 0.025));           // Compute Integral Control
    Ud = Kd * (((error - p_error)) / 0.025);// Compute Derivative Control
    Out = (Up + Ui + Ud)*30;              // Output before saturation is taken into account

    //simple bang bang to speed up system process between large values
    if(desiredTemp-temp>3){
        Out = OutMin;
    }else if (desiredTemp-temp<-3){
        Out = OutMax;
    } else {
        Out = Out;
    }
    uart_count += 1;
    switch (uart_count)
    {
        case 20: // send volt regulator temp
        {
            test = temp;
            UCA1TXBUF = test+4;
            break;
        }
        case 40: // send potentiometer temp
        {
            test = desiredTemp;
            UCA1TXBUF = test;
            uart_count = 0;
            break;
        }
        default: break;
    }
    return Out;
}

int main(void)
{
  WDTCTL = WDTPW+WDTHOLD;                   // Stop watchdog timer

  TimerA0Setup();                           // Setup TimerA0

  P6SEL = 0x1F;                             // Enable A/D channel inputs
  ADC12CTL0 = ADC12ON+ADC12MSC+ADC12SHT0_2; // Turn on ADC12, set sampling time
  ADC12CTL1 = ADC12SHP+ADC12CONSEQ_1;       // Use sampling timer, single sequence
  ADC12MCTL0 = ADC12INCH_0;                 // ref+=AVcc, channel = A0
  ADC12MCTL2 = ADC12INCH_2;                 // ref+=AVcc, channel = A2
  ADC12IE = 0x10;                           // Enable ADC12IFG.4
  ADC12CTL0 |= ADC12ENC;                    // Enable conversions
  P4SEL |= BIT4;                            // UART TX
  P4SEL |= BIT5;                            // UART RX
  UCA1CTL1 |= UCSWRST;                      // Clears the UART control register 1
  UCA1CTL1 |= UCSSEL_2;                     // Sets SMCLK
  UCA1BR0 = 104;                            // For baud rate of 9600
  UCA1BR1 = 0;                              // For baud rate of 9600
  UCA1MCTL |= UCBRS_2;                      // set modulation pattern to high on bit 1 & 5
  UCA1CTL1 &= ~UCSWRST;                     // initialize USCI
  UCA1IE |= UCRXIE;                         // enable USCI_A1 RX interrupt
  UCA1IFG &= ~UCRXIFG;                      // clears interrupt flags


  while(1)
  {
      ADC12CTL0 |= ADC12SC;
      __bis_SR_register(GIE);              // Enables Global Interrupt - ADC/UART interrupt support
      __no_operation();
  }
}

#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
    TA0CCR1 = updatePWM();
}
