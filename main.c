#include <msp430.h>

#define REF_VOLTAGE 3.3
#define THRESHOLD_50 (4095 * 0.5)  // 50% of 3.3V
#define THRESHOLD_30 (4095 * 0.3)  // 30% of 3.3V
#define LORA_EN BIT0 //P4.0 pin for enabling the Lora
#define SKYSAT_EN BIT3 //GPIO4 p5.3 5v to SkySat switch
#define PIZERO_EN BIT4 //GPIO1 p1.4 5V to PI zero switch
#define BURN_EN BIT0 //P3.0 for enabling the burn wire
#define BAT_VOLT BIT6 //P1.6 for receiving the voltage of the battery
#define SKYSATCOM_UART_TX BIT3 //4.3 used for transmitting data to the skysatcom
#define SKYSATCOM_UART_RX BIT2 //4.2 used for receiving data from the skysatcom
#define PIZERO_MOSI BIT0 //2.0 used for getting input from the pizero
#define PIZERO_MISO BIT1 //2.1 used for sending output to the pizero
#define LORA_MISO BIT5 //3.5 used for getting data from Lora
#define LORA_MOSI BIT6 //3.4 used for sending the data to the Lora board
#define LORA_CS BIT7 //3.7 used for choosing the mode of operation
#define LORA_SCK BIT6 //3.6 used for selecting the clock used for the communication
#define PIZERO_CS BIT3 //2.3 used for selecting the mode of operation
#define PIZERO_SCK BIT2 //2.2 used for selecting the clock for communication
#define LORA_G0 BIT4 // 2.4 used for incoming interrupt requests from the LORA board
#define LORA_RST BIT5 //2.5 used for reseting the LORA board in case of any glitches or hangs

void configureADC(void);
void configurePins(void);
void enable_burn_wire(void);
unsigned int readADC(void);
void switch_control(unsigned int);

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    // Configure the 12 bit ADC
    configurePins();
    configureADC();

    enable_burn_wire(); //enables the burn wire to deploy antennas for communication

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

