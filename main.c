#include <msp430.h>
#include <stdint.h>

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

void i2c_init(void) {
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

void i2c_write_word(uint8_t reg, uint16_t data) {
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

uint16_t i2c_read_word(uint8_t reg) {
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

void configure_ina3221(void) {
    i2c_write_word(INA3221_REG_CONFIG, CONFIG_VALUE);
}

float get_shunt_voltage(uint8_t channel) {
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

    uint16_t raw_value = i2c_read_word(reg);
    return raw_value * 40e-6;  // Convert raw value to voltage (in volts)
}

float get_bus_voltage(uint8_t channel) {
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

    uint16_t raw_value = i2c_read_word(reg);
    return raw_value * 8e-3;  // Convert raw value to voltage (in volts)
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

    i2c_init();
    configure_ina3221();

    while (1) {
        for (uint8_t channel = 1; channel <= 3; ++channel) {
            float shunt_voltage = get_shunt_voltage(channel);
            float bus_voltage = get_bus_voltage(channel);

            // Print results (replace with your preferred method of displaying the data)
            printf("Channel %d: Shunt Voltage = %.6f V, Bus Voltage = %.3f V\n", channel, shunt_voltage, bus_voltage);
        }
        __delay_cycles(1000000);  // Delay for a while
    }
}
