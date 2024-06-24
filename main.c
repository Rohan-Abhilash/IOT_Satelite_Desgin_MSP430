#include <msp430.h>
#include <stdint.h>

#define MAX17048_I2C_address 0x36
#define MAX17048_REG_SOC 0x04

// INA3221 I2C address
#define INA3221_I2C_ADDR 0x40

// Register addresses
#define INA3221_REG_CONFIG 0x00
#define INA3221_REG_SHUNT_VOLTAGE_1 0x01
#define INA3221_REG_BUS_VOLTAGE_1 0x02
#define INA3221_REG_SHUNT_VOLTAGE_2 0x03
#define INA3221_REG_BUS_VOLTAGE_2 0x04
#define INA3221_REG_SHUNT_VOLTAGE_3 0x05
#define INA3221_REG_BUS_VOLTAGE_3 0x06
// Configuration register value
#define CONFIG_VALUE 0x7127

#define REF_VOLTAGE 3.3
#define THRESHOLD_50 (REF_VOLTAGE * 0.5)  // 50% of 3.3V
#define THRESHOLD_30 (REF_VOLTAGE * 0.3)  // 30% of 3.3V

#define BURN_EN BIT0 //P3.0 for enabling the burn wire
#define BAT_VOLT_SDA BIT6 //P1.6 for receiving the voltage of the battery
#define BAT_VOLT_SCL BIT7 //p1.7 for the clock signal which els when the data is ready

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



void configurePins(void);
void configureTimer(void);
void sendResetSignal(void);
void enable_burn_wire(void);
void switch_control(float);

void Lora_init(void);
void Lora_reset(void);
void Lora_enable(void);

void VC_Sensor_configure_ina3221(void);
void VC_Sensor_I2C_init(void);
void VC_Sensor_I2C_write_word(uint8_t , uint16_t);
uint16_t VC_Sensor_I2C_read_word(uint8_t);
float VC_Sensor_get_shunt_voltage(uint8_t);
float VC_Sensor_get_bus_voltage(uint8_t);

void SPI_init(void);
void SPI_send(uint8_t);
uint8_t SPI_receive(void);

void cell_I2C_init(void);
float cell_I2C_read_data(void);

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    // Configure the 12 bit ADC
    configurePins();

    cell_I2C_init();

    VC_Sensor_I2C_init();
    VC_Sensor_configure_ina3221();

    SPI_init();
    Lora_init();

    enable_burn_wire(); //enables the burn wire to deploy antennas for communication

    _BIS_SR(GIE);
    // Main loop
    while(1)
    {
        //reading the value coming from the voltage current sensor
        uint8_t channel;

        for(channel = 1;channel <=3 ; channel++){
            float shunt_voltage = VC_Sensor_get_shunt_voltage(channel); // shunt voltage of each channel
            float bus_voltage = VC_Sensor_get_bus_voltage(channel); //bus voltage of each channel
        }

        float stateOfCharge = cell_I2C_read_data();

        switch_control(stateOfCharge);

        __delay_cycles(100000);
    }
    return 0;
}

void cell_I2C_init(void){
    P1SEL0 |= BIT6 | BIT7;
    P1SEL1 &= ~(BIT6 | BIT7);

    UCB0CTLW0 |= UCSWRST;
    UCB0CTLW0 |= UCMST | UCMODE_3 | UCSYNC;
    UCB0CTLW1 |= UCASTP_2;
    UCB0BRW = 10; // baudrate = 10
    UCB0I2CSA = MAX17048_I2C_address;
    UCB0CTLW0 &= ~UCSWRST;
    UCB0IE |= UCNACKIE;
}

float cell_I2C_read_data(){

    uint16_t data;
    while(~(UCB0CTLW0 & UCTXSTP));
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while(~(UCB0IFG & UCTXIFG0));
    UCB0TXBUF = MAX17048_REG_SOC;

    while(~(UCB0CTL0 & UCTXSTP));
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;

    while(UCB0CTLW0 & UCTXSTT);
    while(~(UCB0IFG & UCRXIFG0));
    data = UCB0RXBUF << 8;

    UCB0CTLW0 |= UCTXSTP;

    while(~(UCB0IFG & UCRXIFG0));
    data |= UCB0RXBUF;

    float result = (data >> 8) + ((data & 0xFF) / 256.0);

    return result;
}

void VC_Sensor_configure_ina3221(void) {
    VC_Sensor_I2C_write_word(INA3221_REG_CONFIG, CONFIG_VALUE);
}

void VC_Sensor_I2C_init(void) {
    // Configure pins 3.1 and 3.2 for I2C functionality
    P3SEL0 |= BIT1 | BIT2;
    P3SEL1 &= ~(BIT1 | BIT2);

    // Initialize eUSCI_B0 for I2C master mode
    UCB0CTLW0 = UCSWRST;                      // Enable SW reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC;   // I2C master mode, synchronous mode
    UCB0CTLW1 |= UCASTP_2;                    // Automatic STOP condition
    UCB0BRW = 10;                             // baudrate = SMCLK / 10
    UCB0I2CSA = INA3221_I2C_ADDR;             // Slave address
    UCB0CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
    UCB0IE |= UCNACKIE;                       // Enable NACK interrupt
}

void VC_Sensor_I2C_write_word(uint8_t reg, uint16_t data) {
    while (UCB0CTLW0 & UCTXSTP);              // Ensure stop condition got sent
    UCB0CTLW0 |= UCTR | UCTXSTT;              // I2C TX, start condition

    while (!(UCB0IFG & UCTXIFG0));            // Wait for TX buffer to be ready
    UCB0TXBUF = reg;                          // Send register address

    while (!(UCB0IFG & UCTXIFG0));            // Wait for TX buffer to be ready
    UCB0TXBUF = data >> 8;                    // Send MSB

    while (!(UCB0IFG & UCTXIFG0));            // Wait for TX buffer to be ready
    UCB0TXBUF = data & 0xFF;                  // Send LSB

    while (!(UCB0IFG & UCTXIFG0));            // Wait for TX buffer to be ready
    UCB0CTLW0 |= UCTXSTP;                     // I2C stop condition
}

uint16_t VC_Sensor_I2C_read_word(uint8_t reg) {

    uint16_t data;

    while (UCB0CTLW0 & UCTXSTP);              // Ensure stop condition got sent
    UCB0CTLW0 |= UCTR | UCTXSTT;              // I2C TX, start condition

    while (!(UCB0IFG & UCTXIFG0));            // Wait for TX buffer to be ready
    UCB0TXBUF = reg;                          // Send register address

    while (!(UCB0IFG & UCTXIFG0));            // Wait for TX buffer to be ready
    UCB0CTLW0 &= ~UCTR;                       // I2C RX
    UCB0CTLW0 |= UCTXSTT;                     // I2C repeated start condition

    while (UCB0CTLW0 & UCTXSTT);              // Wait for start condition to be sent
    while (!(UCB0IFG & UCRXIFG0));            // Wait for RX buffer to be ready
    data = UCB0RXBUF << 8;                    // Read MSB

    UCB0CTLW0 |= UCTXSTP;                     // I2C stop condition
    while (!(UCB0IFG & UCRXIFG0));            // Wait for RX buffer to be ready
    data |= UCB0RXBUF;                        // Read LSB

    return data;
}

float VC_Sensor_get_shunt_voltage(uint8_t channel) {
    uint8_t reg;

    switch (channel) {
        case 1:
            reg = INA3221_REG_SHUNT_VOLTAGE_1;
            break;
        case 2:
            reg = INA3221_REG_SHUNT_VOLTAGE_2;
            break;
        case 3:
            reg = INA3221_REG_SHUNT_VOLTAGE_3;
            break;
        default:
            return -1.0;
    }

    uint16_t raw_value = VC_Sensor_I2C_read_word(reg);
    return raw_value * 40e-6;  // Convert raw value to voltage (in volts)
}

float VC_Sensor_get_bus_voltage(uint8_t channel) {
    uint8_t reg;

    switch (channel) {
        case 1:
            reg = INA3221_REG_BUS_VOLTAGE_1;
            break;
        case 2:
            reg = INA3221_REG_BUS_VOLTAGE_2;
            break;
        case 3:
            reg = INA3221_REG_BUS_VOLTAGE_3;
            break;
        default:
            return -1.0;
    }

    uint16_t raw_value = VC_Sensor_I2C_read_word(reg);
    return raw_value * 8e-3;  // Convert raw value to voltage (in volts)
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

void switch_control(float voltage){
    if(voltage >= THRESHOLD_50)
        {
            // If ADC value is >= 50% of 3.3V
            P1OUT |= PIZERO_EN;
            P4OUT |= LORA_EN;
            P5OUT |= SKYSAT_EN;
        }
        else if(voltage < THRESHOLD_30)
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

