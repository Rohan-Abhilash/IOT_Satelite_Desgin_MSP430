#include <msp430.h>
#include <stdint.h>

#define REF_VOLTAGE 3.3
#define THRESHOLD_50 (4095 * 0.5)  // 50% of 3.3V
#define THRESHOLD_30 (4095 * 0.3)  // 30% of 3.3V
#define LORA_EN BIT0 //P4.0 pin for enabling the Lora
#define SKYSAT_EN BIT3 //GPIO4 p5.3 5v to SkySat switch
#define PIZERO_EN BIT4 //GPIO1 p1.4 5V to PI zero switch
#define BURN_EN BIT0 //P3.0 for enabling the burn wire
#define BAT_VOLT_SDA BIT6 //P1.6 for receiving the voltage of the battery
#define BAT_VOLT_SCL BIT7
#define TIMER_THRESHOLD 50000


#define WATCHDOG_SIGNAL_PIN BIT6 // P2.6 changed as pin number 5.0 cannot have interrupts enabled
#define RESET_SIGNAL_PIN    BIT7 // P2.7

void configurePins(void);
void configureTimer(void);
void switch_control(unsigned int);
void sendResetSignal(void);
void I2C_init(void);
void I2C_Read(uint8_t , uint8_t* , uint8_t );

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    // Configure the 12 bit ADC
    configurePins();
    configureTimer();

    uint8_t data[2];

    _BIS_SR(GIE);

    // Main loop
    while(1)
    {
        I2C_Read(0x02,data,2);

        uint16_t voltage_raw = (data[1] << 8) | data[0];
        float voltage = voltage_raw * 1.25 / 1000;

        switch_control(voltage);

        __delay_cycles(100000); // Delay for demonstration purposes
    }
}

void I2C_init(void) {
    // Configure I2C pins
    P1SEL0 |= BAT_VOLT_SDA | BAT_VOLT_SCL;      // P1.6 = SDA, P1.7 = SCL
    P1SEL1 &= ~(BAT_VOLT_SDA| BAT_VOLT_SCL);

    // Configure USCI_B0 for I2C
    UCB0CTLW0 |= UCSWRST;       // Put eUSCI_B in reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC; // I2C master mode, synchronous mode
    UCB0CTLW0 |= UCSSEL__SMCLK; // SMCLK
    UCB0BRW = 10;               // Set clock frequency to 100 kHz (assuming SMCLK is 1 MHz)
    UCB0CTLW0 &= ~UCSWRST;      // Release eUSCI_B from reset
}

void I2C_Read(uint8_t reg, uint8_t *data, uint8_t length) {
    // Set slave address
    UCB0I2CSA = MAX17048G_ADDR;

    // Generate start condition and send register address
    UCB0CTLW0 |= UCTR + UCTXSTT; // I2C TX, start condition
    while (UCB0CTLW0 & UCTXSTT); // Wait until start condition is sent
    UCB0TXBUF = reg;             // Send register address
    while (!(UCB0IFG & UCTXIFG0)); // Wait for TX buffer to be ready

    // Generate repeated start condition and initiate read
    UCB0CTLW0 &= ~UCTR;          // I2C RX
    UCB0CTLW0 |= UCTXSTT;        // Repeated start condition
    while (UCB0CTLW0 & UCTXSTT); // Wait until repeated start condition is sent

    // Read data
    int i = 0;
    for (i = 0; i < length; i++) {
        while (!(UCB0IFG & UCRXIFG0)); // Wait for RX buffer to be ready
        data[i] = UCB0RXBUF;        // Read received data
        if (i == length - 1) {
            UCB0CTLW0 |= UCTXSTP;   // Generate stop condition
        }
    }
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


