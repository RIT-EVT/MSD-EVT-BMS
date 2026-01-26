/**
    * @file main.cpp
 */

#include "BMS.hpp"
// #include "core/utils/time.hpp"

namespace IO = core::io;

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
