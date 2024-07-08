#include <msp430.h>
#include <stdint.h>

//-----------------------------------------------------------------------------------------------------------------
//---------------------------------------Defining The Constants----------------------------------------------------
#define DELAY_TIME 10000000 //tentative delay time 10s here as clock is 1MHz
#define SHUNT_RESISTANCE 5000 //here the shunt resistance is taken from the EPS as 5K ohm

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
#define THRESHOLD_50 50  // 50% of 3.3V
#define THRESHOLD_37 37
#define THRESHOLD_32 32

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

//-----------------------------------------------------------------------------------------------------------------
//------------------------------------Declaring The Functions------------------------------------------------------

void configurePins(void);
void configureTimer(void);
void sendResetSignal(void);
void enable_burn_wire(void);
void switch_control(float);
int check_switch_status(float,float);

void Lora_init(void);
void Lora_reset(void);
void Lora_enable(void);

void VC_Sensor_I2C_init(void);
uint16_t VC_Sensor_I2C_read_word(void);
float VC_Sensor_get_shunt_voltage(uint8_t);
float VC_Sensor_get_bus_voltage(uint8_t);

void pizero_spi_init(void);
void pizero_spi_write(float*);
uint16_t pizero_spi_read(void);

void Lora_spi_init(void);
void Lora_spi_send(float*);
uint16_t Lora_spi_receive(void);

void cell_I2C_init(void);
float cell_I2C_read_data(void);

//-----------------------------------------------------------------------------------------------------------------
//------------------------------------Initializing Global Variables------------------------------------------------

uint16_t read_I2C_data_cell ;
uint16_t read_I2C_data_VC_sensor;
uint8_t reg;
int power_mode = 0;
int sun_mode = 0;
uint16_t Lora_data_store;
uint16_t pizero_data_store;
float data_beacon_store[7];     /*The format of the stored data : (index,value)
                                (0 = state Of charge) (1 = voltage of channel 1) (2 = voltage of channel 2)
                                (3 = voltage of channel 3)
                                (4 = current of channel 1) (5 = current of channel 2) (6 = current of channel 3)*/
//-----------------------------------------------------------------------------------------------------------------
//-------------------------------------------Main Function---------------------------------------------------------

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    configurePins();            // Configure the 12 bit ADC

    configureTimer();

    __enable_interrupt();

//----------------------------------------POWER ON-----------------------------------------------------------------

    cell_I2C_init();
    VC_Sensor_I2C_init();
    pizero_spi_init();
    Lora_spi_init();

    float stateOfCharge = cell_I2C_read_data();

    while(1){
        if(stateOfCharge > THRESHOLD_50){
            enable_burn_wire();         //enables the burn wire to deploy antennas for communication
            break;
        }
        else{
            __delay_cycles(DELAY_TIME);
            stateOfCharge = cell_I2C_read_data();
        }
    }

//-----------------------------------------------Main Loop---------------------------------------------------------
    while(1){
        stateOfCharge = cell_I2C_read_data();
        switch_control(stateOfCharge);  //Three switches implemented LORA , PIZERO and SKYSATCOM (also resetting the switch states)

        int count_cycles = 0; // reseting the count of operations

        while(count_cycles < 3){

            //check if switches are in expected state
            //checking if the pizero and skysatcom switches are in the correct state
            float shunt_voltage_channel_2 = VC_Sensor_get_shunt_voltage(2); // channel 2
            float shunt_voltage_channel_3 = VC_Sensor_get_shunt_voltage(3); // channel 3
            float vc_current_2 = shunt_voltage_channel_2 / SHUNT_RESISTANCE;    //current from channel 2
            float vc_current_3 = shunt_voltage_channel_3 / SHUNT_RESISTANCE;    //current from channel 3

            if(!check_switch_status(vc_current_2,vc_current_3)){
                break;
            }
            //Power Mode operations
            switch(power_mode){
                case 0:{ // Full power mode operations
                    uint8_t channel;
                    VC_Sensor_I2C_init();

                    float data_beacon[7];   //packaging the data as an array and storing it in RAM temporarily till it is transmitted

                    data_beacon[0] = stateOfCharge;

                    for(channel = 1;channel <=3 ; channel++){

                        float shunt_voltage = VC_Sensor_get_shunt_voltage(channel); // shunt voltage is used for measuring the current
                                                                                  //if the value of the shunt resistance is known ie: ohms law
                                                                                    // I = shunt voltage / shunt resistance
                        float bus_voltage = VC_Sensor_get_bus_voltage(channel); //bus voltage is the voltage at the terminal

                        float vc_current = shunt_voltage / SHUNT_RESISTANCE;    //here the current is calculated from the shunt voltage and shunt resistance using OHM's law

                        //storing t data from the VC sensor in the beacon array to be transmitted
                        data_beacon[channel] = bus_voltage;

                        data_beacon[channel + 3] = vc_current;
                    }
                    int i;
                    for(i = 0;i<sizeof(data_beacon_store);i++){
                        data_beacon_store[i] = data_beacon[i]; //storing the packaged data (array) in the global variable
                    }
                    pizero_spi_init();
                    pizero_spi_write(data_beacon);
                    __delay_cycles(DELAY_TIME); //waiting for the data to be sent;
                    break;
                }
                case 1:{    //power saving mode
                    uint8_t channel;
                    VC_Sensor_I2C_init();

                    float data_beacon[7];   //packaging the data as an array and storing it in RAM temporarily till it is transmitted

                    data_beacon[0] = stateOfCharge;

                    for(channel = 1;channel <=3 ; channel++){

                        float shunt_voltage = VC_Sensor_get_shunt_voltage(channel); // shunt voltage is used for measuring the current
                                                                                  //if the value of the shunt resistance is known ie: ohms law
                                                                                    // I = shunt voltage / shunt resistance
                        float bus_voltage = VC_Sensor_get_bus_voltage(channel); //bus voltage is the voltage at the terminal

                        float vc_current = shunt_voltage / SHUNT_RESISTANCE;    //here the current is calculated from the shunt voltage and shunt resistance using OHM's law

                        //storing t data from the VC sensor in the beacon array to be transmitted
                        data_beacon[channel] = bus_voltage;

                        data_beacon[channel + 3] = vc_current;
                    }
                    int i;
                    for(i = 0;i<sizeof(data_beacon_store);i++){
                        data_beacon_store[i] = data_beacon[i]; //storing the packaged data (array) in the global variable
                    }

                    Lora_spi_send(data_beacon);
                    __delay_cycles(DELAY_TIME); //waiting for the data to be sent and waiting for receiving package
                    uint16_t lora_received_data = Lora_spi_receive(); //data received is stored in lora_received_data variable (RAM)

                    if(lora_received_data != 0x0000){
                        Lora_data_store = lora_received_data;   //obtained data is stored in the global variable
                    }
                    //handle data decoding depending on what type of data is being received here
                    __delay_cycles(DELAY_TIME); //waiting for reception to end
                    break;
                }
                case 2:{    //Ultra power saving mode
                    uint8_t channel;
                    VC_Sensor_I2C_init();

                    float data_beacon[7];   //packaging the data as an array and storing it in RAM temporarily till it is transmitted

                    data_beacon[0] = stateOfCharge;

                    for(channel = 1;channel <=3 ; channel++){

                        float shunt_voltage = VC_Sensor_get_shunt_voltage(channel); // shunt voltage is used for measuring the current
                                                                                  //if the value of the shunt resistance is known ie: ohms law
                                                                                    // I = shunt voltage / shunt resistance
                        float bus_voltage = VC_Sensor_get_bus_voltage(channel); //bus voltage is the voltage at the terminal

                        float vc_current = shunt_voltage / SHUNT_RESISTANCE;    //here the current is calculated from the shunt voltage and shunt resistance using OHM's law

                        //storing t data from the VC sensor in the beacon array to be transmitted
                        data_beacon[channel] = bus_voltage;

                        data_beacon[channel + 3] = vc_current;
                    }
                    int i;
                    for(i = 0;i<sizeof(data_beacon_store);i++){
                        data_beacon_store[i] = data_beacon[i]; //storing the packaged data (array) in the global variable
                    }

                    __delay_cycles(DELAY_TIME);

                    break;
                }
                default:{
                    break;
                }
            }
            count_cycles += 1;
        }
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------
//-----------------------------Raspberry Pi Zero SPI Communication module-----------------------------------------
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

void pizero_spi_write(float* data) {
    int i ;
    for(i = 0 ; i < sizeof(data)/sizeof(float); i++){
        UCA0TXBUF = (uint8_t)data[i];
        __delay_cycles(20);
    }

}

uint16_t pizero_spi_read(void) {
    UCA0TXBUF = 0x00;   //dummy write to get the contents of the TX buffer of the slave
    uint16_t temp = UCA0RXBUF;
    return temp;        // Return received data
}

//----------------------------------------------------------------------------------------------------------------
//-------------------------------------MAX17048 Cell Gauge module-------------------------------------------------

void cell_I2C_init(void){
    UCB0CTLW0 |= UCSWRST;
    UCB0CTLW0 |= UCSSEL__SMCLK | UCMST | UCMSB | UCMODE_3;
    UCB0BRW = 10;
    UCB0I2CSA = MAX17048_I2C_address;
    UCB0CTLW1 |= UCASTP_2; // auto stop mode based on the byte counter
    UCB0TBCNT = 1;          // reading one byte from the slave

    P1SEL0 |= BIT6 | BIT7;
    P1SEL1 &= ~(BIT6 | BIT7);

    PM5CTL0 &= ~LOCKLPM5;
    UCB0CTLW0 &= ~UCSWRST;
    UCB0IFG &= ~(UCRXIFG0 | UCTXIFG0);
}

float cell_I2C_read_data(void){

                        //Transmit register address from which we want to read the state of charge
    UCB0IE |= UCRXIE0 | UCTXIE0;

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

    UCB0IE &= ~(UCRXIE0 | UCTXIE0);
                        //converting the result into voltage
    float result = (float)(read_I2C_data_cell >> 8) + (float)((read_I2C_data_cell & 0xFF) / 256.0);
    return result;
}

//----------------------------------------------------------------------------------------------------------------
//------------------------------Three channel Voltage current sensor module---------------------------------------

void VC_Sensor_I2C_init(void) {
                        // Configure pins 3.1 and 3.2 for I2C functionality
    UCB1CTLW0 |= UCSWRST;

    UCB1CTLW0 |= UCSSEL__SMCLK | UCMST | UCMSB | UCMODE_3;
    UCB1BRW = 10;
    UCB1I2CSA = INA3221_I2C_ADDR;
    UCB1CTLW1 |= UCASTP_2; // auto stop mode based on the byte counter
    UCB1TBCNT = 1;      // reading one byte from the slave

    P3SEL0 |= BIT1 | BIT2;
    P3SEL1 &= ~(BIT1 | BIT2);

    PM5CTL0 &= ~LOCKLPM5;
    UCB1CTLW0 &= ~UCSWRST;
    UCB1IFG &= ~(UCRXIFG0 | UCTXIFG0);
}

uint16_t VC_Sensor_I2C_read_word(void) {
                    //Transmit register address from which we want to read the state of charge
    UCB1IE |= UCRXIE0 | UCTXIE0;

    UCB1CTLW0 |= UCTR;
    UCB1TBCNT = 1;
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

    UCB1IE &= ~(UCTXIE0 | UCRXIE0);

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
    return ((int)raw_value) * 40e-6;  // Convert raw value to voltage (in volts)
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
    return ((int)raw_value) * 8e-3;  // Convert raw value to voltage (in volts)
}

//----------------------------------------------------------------------------------------------------------------
//----------------------Configuring the enable , reset and other miscellaneous pins-------------------------------
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

//----------------------------------------------------------------------------------------------------------------
//----------------------------LORA SPI Communication and Enable Reset module--------------------------------------

void Lora_spi_init(void){
    // configuring the SPI communication ports
    UCA1CTLW0 |= UCSWRST;

    UCA1CTLW0 |= UCSSEL__SMCLK;
    UCA1BRW   |= 10; //100Khz communication
    UCA1CTLW0 |= UCMODE_2;
    UCA1CTLW0 |= UCMSB | UCMST;
    UCA1CTLW0 |= UCSYNC;
    UCA1CTLW0 |= UCSTEM;

    P3SEL0 |= LORA_MOSI | LORA_MISO | LORA_SCK | LORA_CS;
    P3SEL1 &= ~(LORA_MOSI | LORA_MISO | LORA_SCK | LORA_CS);

    PM5CTL0 &= ~LOCKLPM5;
    UCA1CTLW0 &= ~UCSWRST;

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

void Lora_spi_send(float* data){
    int i ;
    for(i = 0 ; i < sizeof(data)/sizeof(float) ; i++){
        UCA1TXBUF = (uint8_t)data[i];
        __delay_cycles(20);
    }
}

uint16_t Lora_spi_receive(void){
    UCA1TXBUF = 0x00;   //dummy write to get the contents of the TX buffer of the slave
    uint16_t temp = UCA1RXBUF;
    return temp;
}

//----------------------------------------------------------------------------------------------------------------
//--------------------------Raspberry Pi Zero Watchdog Timer Monitoring module------------------------------------

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

//----------------------------------------------------------------------------------------------------------------
//------------------------------------Burn Wire switch enable module----------------------------------------------

void enable_burn_wire(void){
    P3OUT |= BURN_EN;
    __delay_cycles(10000000);
    P3OUT &= ~BURN_EN;
}

//---------------------------------------------------------------------------------------------------------------
//---------------------------------------Checking switch status--------------------------------------------------

int check_switch_status(float vc_current_2,float vc_current_3){
    switch(power_mode){
        case 0:{
            if(vc_current_3 != 0 && vc_current_3 - vc_current_2 > 0.035){//here we are checking if the current difference is greater than 35 milliamps to see weather the lora board is enabled
                return 1;
            }
            else return 0;
            break;
        }
        case 1:{
            if(vc_current_3 == 0 && vc_current_3 - vc_current_2 > 0.035){ //here we are checking if the current difference is greater than 35 milliamps to see weather the lora board is enabled
                return 1;
            }
            else return 0;
            break;
        }
        case 2:{
            if(vc_current_3 == 0 && vc_current_3 - vc_current_2 < 0.005){ //here we are checking if the current difference is lesser than 5 milliamps to see weather the lora board is disabled
                return 1;
            }
            else return 0;
            break;
        }
        default:{
            return 0;
            break;
        }
    }
}
//----------------------------------------------------------------------------------------------------------------
//-------------------------------------Power Mode Switching module------------------------------------------------

void switch_control(float soc){
    VC_Sensor_I2C_init();
    float bus_voltage = VC_Sensor_get_bus_voltage(1); //bus voltage is the voltage at the channel 1
    if(bus_voltage > 4){
        sun_mode = 1;
    }
    else if (bus_voltage <= 4){
        sun_mode = 0;
    }

    switch(power_mode){
        case 0:{ //mode 0 is full power mode
            if(soc < THRESHOLD_50 && sun_mode == 0){
                P1OUT &= ~PIZERO_EN;
                P4OUT |= LORA_EN;
                P5OUT &= ~SKYSAT_EN;
                power_mode = 1;
            }
            else{
                P1OUT |= PIZERO_EN;
                P4OUT |= LORA_EN;
                P5OUT |= SKYSAT_EN;
            }
            break;
        }
        case 1:{   // case 1 is power saving mode
            if(soc < THRESHOLD_37 && sun_mode == 1){
                P1OUT |= PIZERO_EN;
                P4OUT |= LORA_EN;
                P5OUT |= SKYSAT_EN;
                power_mode = 0;
            }
            else if (soc < THRESHOLD_32 && sun_mode == 0 ){
                P1OUT &= ~PIZERO_EN;
                P4OUT &= ~LORA_EN;
                P5OUT &= ~SKYSAT_EN;
                power_mode = 2;
            }
            else{
                P1OUT &= ~PIZERO_EN;
                P4OUT |= LORA_EN;
                P5OUT &= ~SKYSAT_EN;
            }
            break;
        }
        case 2:{    // case 2 is ultra power saving mode
            if(soc > THRESHOLD_37 && sun_mode == 1){
                P1OUT |= PIZERO_EN;
                P4OUT |= LORA_EN;
                P5OUT |= SKYSAT_EN;
                power_mode = 0;
            }
            else{
                P1OUT &= ~PIZERO_EN;
                P4OUT &= ~LORA_EN;
                P5OUT &= ~SKYSAT_EN;
            }
            break;
        }

    }
}


//--------------------------------------------END OF FUNCTIONS----------------------------------------------------

//----------------------------------------------------------------------------------------------------------------
//---------------------------------------Interrupt Service Routines-----------------------------------------------
//----------------------------------------------------------------------------------------------------------------

#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER_A0(void){
    sendResetSignal();
}

//----------------------------Raspberry Pi Zero Watchdog Interrupt module-----------------------------------------
#pragma vector = PORT2_VECTOR
__interrupt void PORT_2(void){

    if(P2IFG & WATCHDOG_SIGNAL_PIN){
        TA0CTL |= TACLR;
        P2IFG &= ~WATCHDOG_SIGNAL_PIN;
    }
}

//------------------------------I2C USCB0 Communication Interrupt module Cell Gauge--------------------------------
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void){
    if(UCB0IFG & UCTXIFG0){
        UCB0TXBUF = MAX17048_REG_SOC;
    }
    else if (UCB0IFG & UCRXIFG0){
        read_I2C_data_cell = UCB0RXBUF;
    }
}

//------------------------------I2C USCB1 Communication Interrupt module VC Sensor--------------------------------
#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void){
    read_I2C_data_VC_sensor = read_I2C_data_VC_sensor << 8;
    if(UCB1IFG & UCTXIFG0){
        UCB1TXBUF = reg;
    }
    else if (UCB1IFG & UCRXIFG0){
        read_I2C_data_VC_sensor = UCB1RXBUF;
    }
}
