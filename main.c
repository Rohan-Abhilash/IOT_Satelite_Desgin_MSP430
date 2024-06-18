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


#define WATCHDOG_SIGNAL_PIN BIT6 // P2.6 changed as pin number 5.0 cannot have interrupts enabled
#define RESET_SIGNAL_PIN    BIT7 // P2.7

void configureADC(void);
void configurePins(void);
void configureTimer(void);
unsigned int readADC(void);
void switch_control(unsigned int);
void sendResetSignal(void);

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    // Configure the 12 bit ADC
    configurePins();
    configureADC();
    configureTimer();

    _BIS_SR(GIE);

    // Main loop
    while(1)
    {
        unsigned int adc_value = readADC();

        switch_control(adc_value);

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

void configurePins(void)
{
    // Configure P2.6 as input for watchdog signal
    P2DIR &= ~WATCHDOG_SIGNAL_PIN;
    P2REN |= WATCHDOG_SIGNAL_PIN;
    P2OUT |= WATCHDOG_SIGNAL_PIN;

    // Configure P2.7 as output for reset signal
    P2DIR |= RESET_SIGNAL_PIN;
    P2OUT &= ~RESET_SIGNAL_PIN;

    // Configure interrupt for watchdog signal on rising edge
    P2IE |= WATCHDOG_SIGNAL_PIN;
    P2IES &= ~WATCHDOG_SIGNAL_PIN;  // Rising edge
    P2IFG &= ~WATCHDOG_SIGNAL_PIN;  // Clear interrupt flag
}

void configureTimer(void)
{
    // Configure Timer_A0
    TA0CCTL0 = CCIE;                       // Enable Timer_A interrupt
    TA0CCR0 = 32768;                       // Set timer period (1 second at 32.768 kHz)
    TA0CTL = TASSEL_1 | MC_1 | TACLR;      // ACLK, up mode, clear TAR
}

void sendResetSignal(void)
{
    P2OUT |= RESET_SIGNAL_PIN;   // Set reset signal high
    __delay_cycles(100000);      // Delay to ensure reset signal is recognized
    P2OUT &= ~RESET_SIGNAL_PIN;  // Set reset signal low
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

//Port 2 ISR
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    if (P2IFG & WATCHDOG_SIGNAL_PIN)
    {
        // Clear interrupt flag
        P2IFG &= ~WATCHDOG_SIGNAL_PIN;

        // Reset Timer_A0
        TA0CTL |= TACLR;
    }
}

// Timer_A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    // Timeout occurred, send reset signal
    sendResetSignal();
}


