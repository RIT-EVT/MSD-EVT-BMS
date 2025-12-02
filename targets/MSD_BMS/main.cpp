/**
    * @file main.cpp
 */

#include "core/io/UART.hpp"
#include "core/io/I2C.hpp"
#include "core/io/SPI.hpp"
#include "core/manager.hpp"
#include "core/io/pin.hpp"
#include "core/platform/f4xx/stm32f4xx.hpp"
#include "dev/BQ34.hpp"

namespace IO = core::io;

int main() {
    // Initializations
    core::platform::init(); // system init
    // core::dev;        // manager init


    // communication interface setup - UART, I2C, SPI
    IO::UART& uart = IO::getUART<IO::Pin::UART_TX, IO::Pin::UART_RX>(9600);
    IO::I2C& i2c = IO::getI2C<IO::Pin::I2C_SCL, IO::Pin::I2C_SDA>();
    // IO::SPI& spi = IO::getSPI<IO::Pin::SPI_SCK, IO::Pin::SPI_MOSI, IO::Pin::SPI_MISO>();

    BQ34 bq34(&i2c);

    // variables for BQ34 readings
    uint16_t voltage = 0;     // in mV
    uint16_t temperature = 0; // convert to C: (temp / 10) - 273.15
    int16_t current = 0;      // in mA; positive = discharging, negative = charging
    uint16_t soc = 0;         // state of charge in %
    uint16_t soh = 0;         // state of health in %

    while (true) {
        // --- Read measurements ---
        bool okV  = bq34.getVoltage(voltage);
        bool okT  = bq34.getTemperature(temperature);
        bool okI  = bq34.getCurrent(current);
        bool okSOC = bq34.getSOC(soc);
        bool okSOH = bq34.getSOH(soh);

        // check if the bq34 is detected
        if (!(okV && okT && okI && okSOC && okSOH)) {
            uart.puts("BQ34 read error (one or more values invalid)\r\n");
        } else {
            // print values
            uart.printf("Voltage: "
                "[V={} mV] [T={} C] [I={} mA] [SOC={} %] [SOH={} %]\r\n",
                voltage,
                temperature,
                current,
                soc,
                soh
            );
        }

        // --- Wait for next read ---
        core::time::wait(5000); // 5 seconds

    }
}
