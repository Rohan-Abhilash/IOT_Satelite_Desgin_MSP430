#include <msp430.h>
#include <stdint.h>

#define REF_VOLTAGE 3.3
#define THRESHOLD_50 (4095 * 0.5)  // 50% of 3.3V
#define THRESHOLD_30 (4095 * 0.3)  // 30% of 3.3V

#define BURN_EN BIT0 //P3.0 for enabling the burn wire
#define BAT_VOLT BIT6 //P1.6 for receiving the voltage of the battery

#define SKYSAT_EN BIT3 //GPIO4 p5.3 5v to SkySat switch
#define SKYSATCOM_UART_TX BIT3 //4.3 used for transmitting data to the skysatcom
#define SKYSATCOM_UART_RX BIT2 //4.2 used for receiving data from the skysatcom

#define PIZERO_EN BIT4 //GPIO1 p1.4 5V to PI zero switch
#define PIZERO_MOSI BIT0 //2.0 used for getting input from the pizero
#define PIZERO_MISO BIT1 //2.1 used for sending output to the pizero
#define PIZERO_CS BIT3 //2.3 used for selecting the mode of operation
#define PIZERO_SCK BIT2 //2.2 used for selecting the clock for communication
#define WATCHDOG_SIGNAL_PIN BIT6 // P2.6 changed as pin number 5.0 cannot have interrupts enabled
#define RESET_SIGNAL_PIN    BIT7 // P2.7

#define LORA_EN BIT0 //P4.0 pin for enabling the Lora
#define LORA_G0 BIT4 // 2.4 used for incoming interrupt requests from the LORA board
#define LORA_RST BIT5 //2.5 used for reseting the LORA board in case of any glitches or hangs
#define LORA_MISO BIT5 //3.5 used for getting data from Lora
#define LORA_MOSI BIT6 //3.4 used for sending the data to the Lora board
#define LORA_CS BIT7 //3.7 used for choosing the mode of operation
#define LORA_SCK BIT6 //3.6 used for selecting the clock used for the communication

void configureADC(void);
void configurePins(void);
void configureTimer(void);
void sendResetSignal(void);
void enable_burn_wire(void);
unsigned int readADC(void);
void switch_control(unsigned int);
void SPI_init(void);
void Lora_init(void);
void Lora_reset(void);
void Lora_enable(void);
void SPI_send(uint8_t);
uint8_t SPI_receive(void);

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    // Configure the 12 bit ADC
    configurePins();
    configureADC();

    enable_burn_wire(); //enables the burn wire to deploy antennas for communication

    SPI_init();
    Lora_init();

    _BIS_SR(GIE);

    // Main loop
    while(1)
    {
        unsigned int adc_value = readADC();

        switch_control(adc_value);

        __delay_cycles(100000);
    }
}

void configureADC(void)
{
    P1SEL0 |= BIT6;
    P1SEL1 |= BIT6;

    ADC12CTL0 &= ~ADC12ENC;              // Disable ADC12_B before configuration
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;   // Sampling time, ADC12 on

    ADC12CTL1 = ADC12SHP;                // Use sampling timer
    ADC12CTL2 = ADC12RES_2;              // 12-bit conversion results
    ADC12MCTL0 = ADC12INCH_0;
    ADC12IER0 = 0;                       // Disable ADC interrupts
    ADC12CTL0 |= ADC12ENC;               // Enable ADC12_B
}

void configurePins(void){
    P1DIR |= PIZERO_EN;
    P4DIR |= LORA_EN;
    P5DIR |= SKYSAT_EN;
    P3DIR |= BURN_EN; // setting the burnwire enable pin as output

    P1OUT &= ~PIZERO_EN;
    P4OUT &= ~LORA_EN;
    P5OUT &= ~SKYSAT_EN;
    P3OUT &= ~BURN_EN;  // setting the burn wire to low till antenna has to be deployed

    // setting the pins for the pizero watchdog and reset
    // configuring 2.6 as the input of the watchdog signal from pizero
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

void SPI_init(void){
    P2DIR |= LORA_G0 | LORA_RST;
    P3DIR |= LORA_CS ;
    P4DIR |= LORA_EN;

    P2OUT &= ~LORA_RST;
    P3OUT |= LORA_CS;
    P4OUT &= ~LORA_EN;

    // configuring the SPI communication ports
    UCB0CTLW0 |= UCSWRST;      // Put eUSCI state machine in reset
    UCB0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB; // 3-pin, 8-bit SPI master
    UCB0CTLW0 |= UCSSEL__SMCLK; // SMCLK
    UCB0BRW = 0x02;             // /2 clock divider
    UCB0CTLW0 &= ~UCSWRST;      // Initialize eUSCI state machine
    UCB0IE |= UCRXIE;

    P2SEL0 |= LORA_MOSI | LORA_MISO;
    P3SEL0 |= LORA_SCK;

}

void Lora_init(void){
    Lora_reset();
    Lora_enable();
}

void Lora_reset(void){
    P2OUT &= ~LORA_RST; //active low reset
    __delay_cycles(10000);
    P2OUT |= LORA_RST;  //setting it back to its inactive position
}

void Lora_enable(void){
    P4OUT |= LORA_EN;
}

void SPI_send(uint8_t data){
    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = data;
    while(!(UCB0IFG & UCRXIFG));
    (void)UCB0RXBUF;
}

uint8_t SPI_receive(void){
    while(!(UCB0IFG & UCRXIFG));
    UCB0TXBUF = 0x00;   //dummy read t0 generate clock signal
    while(!(UCB0IFG & UCRXIFG));
    return UCB0RXBUF;
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

void enable_burn_wire(void){
    P3OUT |= BURN_EN;
    __delay_cycles(100000);
    P3OUT &= ~BURN_EN;
}

unsigned int readADC(void)
{
    ADC12CTL0 |= ADC12SC;                // Start sampling
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
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER_A0(void){
    sendResetSignal();
}

#pragma vector = PORT2_VECTOR
__interrupt void PORT_2(void){

    if(P2IFG & WATCHDOG_SIGNAL_PIN){
        TA0CTL |= TACLR;
        P2IFG &= ~WATCHDOG_SIGNAL_PIN;
    }
}

