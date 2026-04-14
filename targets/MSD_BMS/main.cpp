/**
    * @file main.cpp
 */

#include "BMS.hpp"

namespace IO = core::io;

static IO::GPIO& RELAY_CTRL_PIN = IO::getGPIO<core::io::Pin::PC_2>();

int main() {
    /** Initializations and Configuration (POST)
     * Initialize the system and communication devices
     * Load configuration data for each IC
     * Perform device detection and verify slave integrity
     */

    // initialization
    core::platform::init();
    auto& bms = msd::bms::BmsMaster::instance();
    // safety delay
    core::time::wait(5000);
    bms.init();

    // if init fails, run again after N seconds
    if (!bms.is_initialized()) {

    }
    // bms update at normal update time
    // should be changed to watchdog or reliable timer in the future
    else {
        static uint32_t last = 0;
        while (true) {
            if (HAL_GetTick() - last > 10) {
                last = HAL_GetTick();
                bms.update();
            }
        }
    }
}
