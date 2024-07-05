# Embedded System Code for MSP430 Microcontroller

## Introduction

This project covers the code implementation for an MSP430 microcontroller-based system. The code is designed to control various peripheral devices, including a voltage and current sensor, battery gauge, communication modules (LoRa, Raspberry Pi Zero), and other subsystems. The implementation includes pin configuration, I2C and SPI communication, sensor data acquisition, and power management.

## Constants and Macros

- `DELAY_TIME`: Defines a delay time of 10 seconds.
- `SHUNT_RESISTANCE`: Shunt resistance value is set to 5000 ohms.
- `MAX17048_I2C_address`: I2C address for the MAX17048 battery gauge.
- `MAX17048_REG_SOC`: Register address for state of charge in the MAX17048.
- `INA3221_I2C_address`: I2C address for the INA3221 voltage and current sensor.
- `INA3221 Register addresses`: Register addresses for the INA3221 sensor's configuration and voltage readings.
- `CONFIG_VALUE`: Configuration value for INA3221 sensor.
- `REF_VOLTAGE`: Reference voltage is set to 3.3V.
- `THRESHOLD_50`, `THRESHOLD_37`, `THRESHOLD_32`: Threshold values for battery voltage in percentages.
- **Pin Definitions**: Various pins are defined for enabling different modules and communication interfaces.

## Function Declarations

The following functions are declared in the code:

- `configurePins`: Configures the I/O pins.
- `configureTimer`: Configures the timer for periodic operations.
- `sendResetSignal`: Sends a reset signal to a specific pin.
- `enable_burn_wire`: Enables the burn wire for deploying antennas.
- `switch_control`: Controls the switching of different power modes.
- `check_switch_status`: Checks the status of switches for various modules.
- `Lora_init`, `Lora_reset`, `Lora_enable`: Initializes, resets, and enables the LoRa module.
- `VC_Sensor_I2C_init`, `VC_Sensor_I2C_read_word`, `VC_Sensor_get_shunt_voltage`, `VC_Sensor_get_bus_voltage`: Functions related to INA3221 sensor initialization and data reading.
- `pizero_spi_init`, `pizero_spi_write`, `pizero_spi_read`: SPI communication functions for Raspberry Pi Zero.
- `cell_I2C_init`, `cell_I2C_read_data`: I2C communication functions for MAX17048 battery gauge.

## Global Variables

The following global variables are used in the code:

- `read_I2C_data_cell`: Stores data read from the battery gauge.
- `read_I2C_data_VC_sensor`: Stores data read from the INA3221 sensor.
- `reg`: Register address for I2C communication.
- `power_mode`: Current power mode of the system.
- `sun_mode`: Current sun mode of the system.
- `Lora_data_store`: Stores data received from the LoRa module.
- `pizero_data_store`: Stores data received from the Raspberry Pi Zero.
- `data_beacon_store`: Stores sensor data for transmission.

## Main Function

The main function performs the following tasks:

1. **Initialization**:
   - Disables the watchdog timer.
   - Configures I/O pins and timer.
   - Enables global interrupts.
   - Initializes I2C and SPI interfaces for various sensors and modules.
   - Reads initial state of charge from the battery gauge.

2. **Power-On Sequence**:
   - If the state of charge is above a certain threshold, the burn wire is enabled to deploy antennas.

3. **Main Loop**:
   - Continuously reads the state of charge and controls switching of power modes.
   - Performs operations based on the current power mode (full power, power saving, ultra power saving).

## Raspberry Pi Zero SPI Communication

- `pizero_spi_init`: Initializes SPI communication with Raspberry Pi Zero.
- `pizero_spi_write`: Writes data to the Raspberry Pi Zero via SPI.
- `pizero_spi_read`: Reads data from the Raspberry Pi Zero via SPI.

## MAX17048 Battery Gauge

- `cell_I2C_init`: Initializes I2C communication with the MAX17048 battery gauge.
- `cell_I2C_read_data`: Reads the state of charge from the MAX17048.

## INA3221 Voltage and Current Sensor

- `VC_Sensor_I2C_init`: Initializes I2C communication with the INA3221 sensor.
- `VC_Sensor_I2C_read_word`: Reads a word from the INA3221 sensor.
- `VC_Sensor_get_shunt_voltage`: Reads the shunt voltage from a specified channel.
- `VC_Sensor_get_bus_voltage`: Reads the bus voltage from a specified channel.

## I/O Pin Configuration

- `configurePins`: Configures the I/O pins for enabling various modules and handling communication interfaces.

## LoRa Communication

- `Lora_spi_init`: Initializes SPI communication with the LoRa module.
- `Lora_init`: Initializes the LoRa module.
- `Lora_reset`: Resets the LoRa module.
- `Lora_enable`: Enables the LoRa module.
- `Lora_spi_send`: Sends data to the LoRa module via SPI.
- `Lora_spi_receive`: Receives data from the LoRa module via SPI.

## Watchdog Timer Monitoring

- `configureTimer`: Configures a timer for monitoring the watchdog signal from the Raspberry Pi Zero.
- `sendResetSignal`: Sends a reset signal if necessary.

## Conclusion

The code implements a comprehensive control system for an MSP430 microcontroller, managing various sensors, communication modules, and power modes. The system is designed for efficient power management and reliable communication with peripheral devices. Each function and module is configured to perform specific tasks, ensuring the overall functionality of the system.

| Image 1 | Image 2 |
|---------|---------|
| [c9023179-a3c8-4c38-90eb-c276bd74acbb](https://github.com/Rohan-Abhilash/IOT_Satelite_Desgin_MSP430/assets/120775348/10e363a7-41c3-43e0-b0c6-9a26bab3f093) | ![e71b741f-7d53-4334-8360-45105a6cc90e](https://github.com/Rohan-Abhilash/IOT_Satelite_Desgin_MSP430/assets/120775348/defef4be-4416-4976-b487-2f9e2ca21513)|
| Image 3 | Image 4 |
|---------|---------|
| ![0e512ddf-700d-46b4-b5ec-27a5f56ef8e8](https://github.com/Rohan-Abhilash/IOT_Satelite_Desgin_MSP430/assets/120775348/3a0479ba-cd2d-4477-9f97-db47b74fab10) | ![fccfae70-35e2-4118-9d13-2aa8207dfda1](https://github.com/Rohan-Abhilash/IOT_Satelite_Desgin_MSP430/assets/120775348/75a9c9d4-fc3c-4163-b8fa-274e231ff9dd) |
