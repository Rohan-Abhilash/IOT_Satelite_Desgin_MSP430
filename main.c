#include <msp430.h>

#define REF_VOLTAGE 3.3
#define THRESHOLD_50 (4095 * 0.5)  // 50% of 3.3V
#define THRESHOLD_30 (4095 * 0.3)  // 30% of 3.3V
#define LORA_EN BIT0 //P4.0 pin for enabling the Lora
#define SKYSAT_EN BIT3 //GPIO4 p5.3 5v to SkySat switch
#define PIZERO_EN BIT4 //GPIO1 p1.4 5V to PI zero switch
#define BURN_EN BIT0 //P3.0 for enabling the burn wire
#define BAT_VOLT BIT6 //P1.6 for receiving the voltage of the battery
#define TIMER_THRESHOLD 50000

void configureADC(void);
void configurePins(void);
void configureTimer(void);
unsigned int readADC(void);
void switch_control(unsigned int);
void resetPIZero(void);

volatile unsigned int timerCounter = 0;

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    // Configure the 12 bit ADC
    configurePins();
    configureADC();
    configureTimer();

    __BIS_SR(GIE);

    // Main loop
    while(1)
    {
        unsigned int adc_value = readADC();

        switch_control(adc_value);

        if(timerCounter >= TIMER_THRESHOLD){
            timerCounter = 0;
            resetPIZero();
        }


        __delay_cycles(100000); // Delay for demonstration purposes
    }
}

void configureADC(void)
{
    // Configure P1.0 for ADC
    P1SEL0 |= BIT6;
    P1SEL1 |= BIT6;

    // Configure ADC12_B
    ADC12CTL0 &= ~ADC12ENC;              // Disable ADC12_B before configuration
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;   // Sampling time, ADC12 on
    ADC12CTL1 = ADC12SHP;                // Use sampling timer
    ADC12CTL2 = ADC12RES_2;              // 12-bit conversion results
    ADC12MCTL0 = ADC12INCH_0;            // A0 ADC input select; Vref=AVCC
    ADC12IER0 = 0;                       // Disable ADC interrupts
    ADC12CTL0 |= ADC12ENC;               // Enable ADC12_B
}

void configurePins(void){
    P1DIR |= PIZERO_EN;
    P4DIR |= LORA_EN;
    P5DIR |= SKYSAT_EN;

    P1OUT &= ~PIZERO_EN;
    P4OUT &= ~LORA_EN;
    P5OUT &= ~SKYSAT_EN;

    // Configure P5.0 as input for watchdog signal
    P5DIR &= ~BIT0;
    P5REN |= BIT0;  // Enable pull-up/pull-down resistor
    P5OUT |= BIT0;  // Select pull-up mode

    // Configure P5.1 as output for reset signal
    P5DIR |= BIT1;
    P5OUT &= ~BIT1;  // Ensure the reset signal is initially low
}

void configureTimer(void)
{
    // Configure Timer_A
    TA0CCTL0 = CCIE;                          // Enable interrupt for CCR0
    TA0CCR0 = 32768;                          // Set CCR0 value for ~1 second delay at ACLK 32.768 kHz
    TA0CTL = TASSEL_1 | MC_1 | TACLR;         // ACLK, up mode, clear TAR

    // Enable interrupt for P5.0 (watchdog signal)
    //Error here undefined p5ie , p5ies and p5ifg; to be solved
    P5IE |= BIT0;
    P5IES &= ~BIT0;  // Trigger on rising edge
    P5IFG &= ~BIT0;  // Clear interrupt flag
}

unsigned int readADC(void)
{
    ADC12CTL0 |= ADC12SC;                // Start sampling/conversion
    while (ADC12CTL1 & ADC12BUSY);       // Wait for conversion to complete
    return ADC12MEM0;                    // Return ADC value
}

void switch_control(unsigned int adc_value){
    if(adc_value >= THRESHOLD_50)
        {
            // If ADC value is >= 50% of 3.3V
            P1OUT |= PIZERO_EN;
            P4OUT |= LORA_EN;
            P5OUT |= SKYSAT_EN;
        }
        else if(adc_value < THRESHOLD_30)
        {
            // If ADC value is < 30% of 3.3V
            P1OUT &= ~PIZERO_EN;
            P4OUT &= ~LORA_EN;
            P5OUT |= SKYSAT_EN;
        }
        else
        {
            // If ADC value is >= 30% but < 50% of 3.3V
            P1OUT &= ~PIZERO_EN;
            P4OUT |= LORA_EN;
            P5OUT |= SKYSAT_EN;
        }
}

void resetPIZero(void)
{
    // Send reset signal to the other board through P5.1
    P5OUT |= BIT1;    // Set P5.1 high
    __delay_cycles(100000);  // Delay to ensure reset signal is recognized (adjust as necessary)
    P5OUT &= ~BIT1;   // Set P5.1 low
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    timerCounter++;
}

#pragma vector=PORT5_VECTOR
__interrupt void Port_5(void)
{
    if(P5IFG & BIT0)  // Check if interrupt was triggered by P5.0
    {
        timerCounter = 0;  // Reset timer counter when watchdog signal is received
        P5IFG &= ~BIT0;    // Clear interrupt flag
    }
}
