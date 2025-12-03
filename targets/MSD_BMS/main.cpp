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
    uint16_t timeout = 0;
    uint16_t flags = 0;
    uint16_t voltage_raw = 0;

    uart.puts("Starting BQ34Z100 initialization...\r\n");

    // wait until the bq34z100 is detected
    bool detected = false;

    uart.puts("Scan complete.\r\n\r\n");

    while (!detected) {
        uart.puts("Searching for BQ34Z100...\r\n");
        // Try to read the CONTROL register to verify device presence
        uint16_t controlStatus = 0;
        if (timeout == 10) {
            uart.puts("BQ34Z100 not found\r\n");
            core::time::wait(100000); // long delay for now
            timeout = 0;
            uart.puts("Attempting to find BQ34Z100 again...\r\n");
        }
        if (bq34.readWord(CONTROL, controlStatus)) {
            uart.printf("BQ34Z100 detected! Control status: 0x%X\r\n", controlStatus);
            detected = true;
        } else {
            uart.puts("BQ34Z100 not found. Retrying in 2 seconds...\r\n");
            core::time::wait(2000);
            timeout ++;
        }
    }

    uart.puts("BQ34Z100 ready. Reading measurements...\r\n\r\n");

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

        if (bq34.getFlags(flags)) {
            uart.printf("flags: 0x%X\r\n", flags);
        }

        if (bq34.getVoltageRaw(voltage_raw)) {
            uart.printf("voltage_raw: 0x%X\r\n", voltage_raw);
        }

        // --- Wait for next read ---
        core::time::wait(5000); // 5 seconds

    }
}
