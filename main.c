#include <msp430.h>
#include <stdint.h>

const float Kp = 0.5;   // Proportional Control Constant
const float Ki = 0.07;  // Integral Control Constant
const float Kd = 0.3;   // Derivative Control Constant
const int MaxOut = 998, MinOut = 0; // Maximum/Minimum Duty Cycle

float Up;
int Out;
int uart_count = 0;
int seconds_count = 10;
int ADC_count = 0;
float k1 = 0, k2 = 0, k3 = 0, k4 = 0, k5 = 0, k6 = 0, k7 = 0, k8 = 0, k9 = 0, k10 = 0, y;

//ai constants set for FIR filter
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

volatile float temp_Voltage, pot_Voltage;   // Voltage out from the PTAT (temp_Voltage) and the potentiometer (pot_Voltage)
float temp, temp1;
float ADC_Voltage, Vp;
float Ud, error, i_error, OutPreSat, p_error;
float Ui = 0;
float setTemp, prevTemp;    // Desired/Set Temperature and Previous Temperature
uint16_t test;  // Test Variable

void TimerA0Setup(){
    //Set P1.0, P4.7, P1.2 to the output direction.
    P4DIR |= BIT0;
    P4DIR |= BIT7;
    P1DIR |= BIT2;
    P1SEL |= BIT2;
    TA0CCTL1 = OUTMOD_7;        // Enable interrupt in compare mode
    TA0CCR0 = 999;              // Set period of CCR0 1000 microseconds.
    TA0CCR1 = 500;              // Set duty cycle to 50%
    TA0CTL = TACLR;             // Clear flag
    TA0CTL = TASSEL_1 + MC_1;   // TimerA0 Control: SMCLK, UP Mode
}

int updatePWM(void)
{
    // Temperature calculations
    temp_Voltage = ADC12MEM0;   // Stores PTAT voltage in MEM0 of ADC
    pot_Voltage = ADC12MEM2;    // Stores Potentiometer voltage in MEM2 of ADC

    pot_Voltage = ((pot_Voltage*3.3)/4096);     // Calculation for potentiometer voltage reading
    setTemp = ((pot_Voltage)*30);               // Conversion from PTAT reading to temperature
    temp1 = ((temp_Voltage*3.3)/4096);
    temp1 = (temp1*100);

    // Samples shifted up by 1 and temp1 is shifted in
    k10 = k9;
    k9 = k8;
    k8 = k7;
    k7 = k6;
    k6 = k5;
    k5 = k4;
    k4 = k3;
    k3 = k2;
    k2 = k1;
    k1 = temp1;

    // FIR filter input: x  output: temp
    temp = (k1*a1) + (k2*a2) + (k3*a3) + (k4*a4) + (k5*a5) + (k6*a6) + (k7*a7) + (k8*a8) + (k9*a9) + (k10*a10);

    p_error = error;                        // Track previous Up and Current Up
    error = temp - setTemp;                 // Calculation for the error
    Up = Kp * error;                        // Calculation for Proportional Control
    Ui += (Ki * (error * 0.025));           // Calculation for Integral Control
    Ud = Kd * (((error - p_error)) / 0.025);    // Calculation for Derivative Control
    Out = (Up + Ui + Ud)*30;              // Output before saturation is taken into account

    // Simple bang bang control to speed up system process between large values
    if(setTemp-temp>3){
        Out = MinOut;
    } else if (setTemp-temp<-3){
        Out = MaxOut;
    } else {
        Out = Out;
    }
    uart_count += 1;        // Increment UART counter
    switch (uart_count)
    {
        case 20: // Send volt regulator temperature
        {
            test = temp;
            UCA1TXBUF = test+4; // Insert test variable into Transmit Buffer for UART
            break;
        }
        case 40: // Send potentiometer temperature
        {
            test = setTemp;     // Set test to desired/set temperature
            UCA1TXBUF = test;   // Insert test variable into Transmit Buffer for UART
            uart_count = 0;     // Set UART counter to zero
            break;
        }
        default: break;
    }
    return Out;
}

int main(void)
{
  WDTCTL = WDTPW+WDTHOLD;                   // Disable the Watchdog timer

  TimerA0Setup();                           // Setup TimerA0

  P6SEL = 0x1F;                             // Enable A/D channel inputs
  P4SEL |= BIT4;                            // UART TX
  P4SEL |= BIT5;                            // UART RX

  UCA1CTL1 |= UCSWRST;                      // Clears the UART control register 1
  UCA1CTL1 |= UCSSEL_2;                     // Sets SMCLK
  UCA1BR0 = 104;                            // Baud rate 9600
  UCA1BR1 = 0;                              // Baud rate 9600
  UCA1MCTL |= UCBRS_2;                      // Set modulation pattern to high on bit 1 & 5
  UCA1CTL1 &= ~UCSWRST;                     // Initialize USCI
  UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt
  UCA1IFG &= ~UCRXIFG;                      // Clears interrupt flags

  ADC12CTL0 = ADC12ON+ADC12MSC+ADC12SHT0_2; // Turn on ADC12, set sampling time
  ADC12CTL1 = ADC12SHP+ADC12CONSEQ_1;       // Use sampling timer, single sequence
  ADC12MCTL0 = ADC12INCH_0;                 // ref+=AVcc, channel = A0
  ADC12MCTL2 = ADC12INCH_2;                 // ref+=AVcc, channel = A2
  ADC12IE = 0x10;                           // Enable ADC12IFG.4
  ADC12CTL0 |= ADC12ENC;                    // Enable conversions

// Infinite While loop
  while(1)
  {
      ADC12CTL0 |= ADC12SC;
      __bis_SR_register(GIE);              // Enables Global Interrupt - ADC/UART interrupt support
      __no_operation();
  }
}

// Interrupt vector for ADC
#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
    TA0CCR1 = updatePWM();
}
