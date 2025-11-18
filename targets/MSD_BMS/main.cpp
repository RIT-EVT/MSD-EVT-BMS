/**
    * @file main.cpp
 */

#include <EVT/io/UART.hpp>
#include <EVT/io/I2C.hpp>
#include <EVT/io/SPI.hpp>
#include <EVT/core/platform.hpp>
#include <EVT/manager.hpp>
#include <EVT/io/pin.hpp>
#include "dev/BQ34.hpp"

namespace IO = EVT::core::IO;

int main() {
    // Initializations
    EVT::core::platform::init(); // system init
    EVT::manager::init();        // manager init
    

    // communication interface setup - UART, I2C, SPI
    IO::UART& uart = IO::getUART<IO::Pin::UART_TX, IO::Pin::UART_RX>(9600);
    IO::I2C& i2c = IO::getI2C<IO::Pin::I2C_SCL, IO::Pin::I2C_SDA>(100000);
    IO::SPI& spi = IO::getSPI<IO::Pin::SPI_MOSI, IO::Pin::SPI_MISO, IO::Pin::SPI_SCK>(1000000);

    BQ34 bq34(&i2c);

    // variables for BQ34 readings
    uint16_t voltage = 0;     // in mV
    uint16_t temperature = 0; // convert to C: (temp / 10) - 273.15
    int16_t current = 0;      // in mA; positive = discharging, negative = charging
    uint16_t soc = 0;         // state of charge in %
    uint16_t soh = 0;         // state of health in %

    while (1) {
        bq34.getVoltage(voltage);
        bq34.getTemperature(temperature);
        bq34.getCurrent(current);
        bq34.getSOC(soc);
        bq34.getSOH(soh);

        EVT::manager::delay_ms(5000); // wait 5 seconds before next read
        
    }
}
