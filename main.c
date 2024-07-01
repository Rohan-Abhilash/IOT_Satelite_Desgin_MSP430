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

#define LORA_MOSI BIT4   //3.4 used for sending the data to the Lora board
#define LORA_MISO BIT5 //3.5 used for getting data from Lora
#define LORA_SCK BIT6 //3.6 used for the clock of for the communication
#define LORA_CS BIT7 //3.7 used for choosing the chip


void configurePins(void);
void configureTimer(void);
void sendResetSignal(void);
void enable_burn_wire(void);
void switch_control(float);

void Lora_init(void);
void Lora_reset(void);
void Lora_enable(void);

void VC_Sensor_I2C_init(void);
uint16_t VC_Sensor_I2C_read_word(void);
float VC_Sensor_get_shunt_voltage(uint8_t);
float VC_Sensor_get_bus_voltage(uint8_t);

void pizero_spi_init(void);
void pizero_spi_write(uint8_t);
int pizero_spi_read(void);

void Lora_spi_init(void);
void Lora_spi_send(uint8_t);
int Lora_spi_receive(void);

void cell_I2C_init(void);
float cell_I2C_read_data(void);


int read_I2C_data_cell;
uint16_t read_I2C_data_VC_sensor;
uint8_t reg;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;
    // Configure the 12 bit ADC
    configurePins();

    configureTimer();

    enable_burn_wire(); //enables the burn wire to deploy antennas for communication

    __enable_interrupt();
    // Main loop
    while(1)
    {
        //reading the value coming from the voltage current sensor
        uint8_t channel;
        VC_Sensor_I2C_init();
        for(channel = 1;channel <=3 ; channel++){
            float shunt_voltage = VC_Sensor_get_shunt_voltage(channel); // shunt voltage of each channel
            float bus_voltage = VC_Sensor_get_bus_voltage(channel); //bus voltage of each channel
        }

        cell_I2C_init();
        float stateOfCharge = cell_I2C_read_data();
        switch_control(stateOfCharge);

        //pizero communication module
        pizero_spi_init();
        uint8_t pizero_data = pizero_spi_read();

        __delay_cycles(100000);
    }
    return 0;
}


void pizero_spi_init(void) {
    // Set P2.3 as output (CS pin)
    UCA0CTLW0 |= UCSWRST;

    UCA0CTLW0 |= UCMST;
    UCA0CTLW0 |= UCMODE_2;  // mode 1 is for 4 pin-spi communication with active low CS bit
    UCA0CTLW0 |= UCSSEL__SMCLK; //using the sub master clock
    UCA0BRW    = 10;    // 100khz clock
    UCA0CTLW0 |= UCSYNC;
    UCA0CTLW0 |= UCSTEM;
    UCA0CTLW0 |= UCMSB;

    P2SEL0 |= PIZERO_MOSI | PIZERO_MISO | PIZERO_SCK | PIZERO_CS;
    P2SEL1 &= ~(PIZERO_MOSI | PIZERO_MISO | PIZERO_SCK | PIZERO_CS);

    PM5CTL0 &= ~LOCKLPM5;

    UCA0CTLW0 &= ~UCSWRST;
}

void pizero_spi_write(uint8_t data) {
    UCA0TXBUF = data;
}

int pizero_spi_read(void) {
    int temp = UCA0RXBUF;
    return temp; // Return received data
}

void cell_I2C_init(void){
    UCB0CTLW0 |= UCSWRST;
    UCB0CTLW0 |= UCSSEL__SMCLK | UCMST | UCMSB | UCMODE_3;
    UCB0BRW = 10;
    UCB0I2CSA = MAX17048_I2C_address;
    UCB0CTLW1 |= UCASTP_2; // auto stop mode based on the byte counter
    UCB0TBCNT = 1; // reading one byte from the slave

    P1SEL0 |= BIT6 | BIT7;
    P1SEL1 &= ~(BIT6 | BIT7);
    PM5CTL0 &= ~LOCKLPM5;
    UCB0CTLW0 &= ~UCSWRST;

    UCB0IE |= UCRXIE0 | UCTXIE0;
    UCB0IFG &= ~(UCRXIFG0 | UCTXIFG0);
}

float cell_I2C_read_data(void){

    //Transmit register address from which we want to read the state of charge
    UCB0CTLW0 |= UCTR;
    UCB0CTLW0 |= UCTXSTT;

    while((UCB0IFG & UCSTPIFG) == 0); //waiting for the stp flag to fire to ensure that the max17048 address has been sent and acknowledged by the slave
    UCB0IFG &= ~UCSTPIFG;
    //from here it will go to the interrupt service routine
    //Receive the data from that register containing the state of charge
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    //restarting the communication for receiving the data from the specific register
    while((UCB0IFG & UCSTPIFG) == 0);
    UCB0IFG &= ~UCSTPIFG;

    //converting the result into voltage
    float result = (read_I2C_data_cell >> 8) + ((read_I2C_data_cell & 0xFF) / 256.0);

    return result;
}

void VC_Sensor_I2C_init(void) {
    // Configure pins 3.1 and 3.2 for I2C functionality
    UCB1CTLW0 |= UCSWRST;

    UCB1CTLW0 |= UCSSEL__SMCLK | UCMST | UCMSB | UCMODE_3;
    UCB1BRW = 10;
    UCB1I2CSA = INA3221_I2C_ADDR;
    UCB1CTLW1 |= UCASTP_2; // auto stop mode based on the byte counter
    UCB1TBCNT = 1; // reading one byte from the slave

    P3SEL0 |= BIT1 | BIT2;
    P3SEL1 &= ~(BIT1 | BIT2);

    PM5CTL0 &= ~LOCKLPM5;
    UCB1CTLW0 &= ~UCSWRST;
    UCB1IE |= UCRXIE0 | UCTXIE0;
    UCB1IFG &= ~(UCRXIFG0 | UCTXIFG0);
}

uint16_t VC_Sensor_I2C_read_word(void) {

    //Transmit register address from which we want to read the state of charge
    UCB1CTLW0 |= UCTR;
    UCB1CTLW0 |= UCTXSTT;

    while((UCB1IFG & UCSTPIFG) == 0); //waiting for the stp flag to fire to ensure that the max17048 address has been sent and acknowledged by the slave
    UCB1IFG &= ~UCSTPIFG;
    //from here it will go to the interrupt service routine
    //Receive the data from that register containing the state of charge
    UCB1CTLW0 &= ~UCTR;
    UCB1TBCNT = 2;
    UCB1CTLW0 |= UCTXSTT;

    //restarting the communication for receiving the data from the specific register
    while((UCB1IFG & UCSTPIFG) == 0);
    UCB1IFG &= ~UCSTPIFG;                    // Read LSB

    return read_I2C_data_VC_sensor;

}

float VC_Sensor_get_shunt_voltage(uint8_t channel) {

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

    uint16_t raw_value = VC_Sensor_I2C_read_word();
    return raw_value * 40e-6;  // Convert raw value to voltage (in volts)
}

float VC_Sensor_get_bus_voltage(uint8_t channel) {

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
    uint16_t raw_value = VC_Sensor_I2C_read_word();
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

void Lora_spi_init(void){
    // configuring the SPI communication ports
    UCA1CTLW0 |= UCSWRST;

    UCA1CTLW0 |= UCSSEL__SMCLK;
    UCA1BRW   |= 10; //100Khz communication
    UCA1CTLW0 |= UCMODE_2;
    UCA1CTLW0 |= UCMSB | UCMST;
    UCA1CTLW0 |= UCSYNC;
    UCA1CTLW0 |= UCSTEM;

    UCA1CTLW0 &= ~UCSWRST;

    P3SEL0 |= LORA_MOSI | LORA_MISO | LORA_SCK | LORA_CS;
    P3SEL1 &= ~(LORA_MOSI | LORA_MISO | LORA_SCK | LORA_CS);

}

void Lora_init(void){
    P2DIR |= LORA_RST;
    P2OUT &= ~LORA_RST;
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

void Lora_spi_send(uint8_t data){
    UCA1TXBUF = data;
}

int Lora_spi_receive(void){
    int temp = UCA1RXBUF;
    return temp;
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

//---------------------------------------------------------------------------------------------------------
//------------------------------------Interupt Service Routines--------------------------------------------

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
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void){
    if(UCB0IFG & UCTXIFG0){
        UCB0TXBUF = MAX17048_REG_SOC;
    }
    else if (UCB0IFG & UCRXIFG0){
        read_I2C_data_cell = UCB0RXBUF;
    }
}
#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void){
    if(UCB1IFG & UCTXIFG0){
        UCB1TXBUF = reg;
    }
    else if (UCB1IFG & UCRXIFG0){
        read_I2C_data_VC_sensor = UCB1RXBUF;
    }
}
