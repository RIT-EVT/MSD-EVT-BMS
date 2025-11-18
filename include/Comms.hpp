#ifndef COMMS_HPP
#define COMMS_HPP

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file comms.hpp
 * @brief Implementation of communication interfaces for the Battery Management System (BMS).
 * 
 * This file provides functions for initializing and managing UART, ISO-UART, SPI, and I2C
 * communication protocols.
 *
 * Project: MSD_BMS
 * Author: Key'Mon Jenkins
 * Date: October 2025
 */

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>

// UART communication functions
void comms_uart_init(uint32_t baudrate);
int  comms_uart_send(const uint8_t *data, size_t length);
int  comms_uart_receive(uint8_t *buffer, size_t length);

// ISO-UART communication functions
void comms_isouart_init(uint32_t baudrate, uint8_t parity);
int  comms_isouart_send(const uint8_t *data, size_t length);
int  comms_isouart_receive(uint8_t *buffer, size_t length);

// SPI communication functions
void comms_spi_init(uint32_t clock_speed);
int  comms_spi_transfer(const uint8_t *tx_buffer, uint8_t *rx_buffer, size_t length);

// I2C communication functions
void comms_i2c_init(uint32_t speed_mode);
int  comms_i2c_write(uint8_t device_address, const uint8_t *data, size_t length);
int  comms_i2c_read(uint8_t device_address, uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif // COMMS_HPP