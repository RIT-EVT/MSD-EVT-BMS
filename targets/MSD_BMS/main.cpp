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
    bms.init();
    bms.update();
    //
    // while (true) {
    //     RELAY_CTRL_PIN.writePin(core::io::GPIO::State::LOW); // enable relay
    //     core::time::wait(10000);
    //
    //     RELAY_CTRL_PIN.writePin(core::io::GPIO::State::HIGH); // disable relay
    //     core::time::wait(10000);
    // }

    // main loop
    while (true) {
        bms.update();
        core::time::wait(SLOW_UPDATE_MS); // wait 10 seconds for now
    }
    // load configuration data
    // thresholds, timing, cell count, calibration

    // check devices (handshake)

    // Slave BQ79616

    // Data Management control loop
    // create function

    // Protection and Command Logic state
    // create function

    // Data logging & Fault Recovery state
    // create function

    // Error state
    // create function

    // Warning state
}
