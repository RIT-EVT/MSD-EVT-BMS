//
// Created by Key'Mon Jenkins on 1/25/26.
//
#ifndef _BMS_THERMISTOR_HPP_
#define _BMS_THERMISTOR_HPP_

#include "core/dev/storage/M24C32.hpp"
#include <core/io/ADC.hpp>
#include <stdint.h>

namespace msd::bms {

struct Coefficients {
    float a;
    float b;
    float c;
};

class ThermistorArray {
public:
    static constexpr uint8_t SENSOR_COUNT = 2;

    enum class Fault : uint8_t {
        NONE,
        OPEN_CIRCUIT,
        SHORT_CIRCUIT,
        ADC_FAULT
    };

    struct Reading {
        uint32_t adc_raw;
        int16_t temperature_dC;   // 0.1 °C resolution
        Fault   fault;
    };

    /**
     * @brief Construct thermistor array
     *
     * @param[in] adcs Array of ADC references, one per thermistor
     */
    ThermistorArray(core::io::ADC* adcs[SENSOR_COUNT]);

    /**
     * @brief Sample all thermistors and update internal state
     */
    void update();

    /**
     * @brief Get temperature for a specific sensor
     *
     * @param[in] index Thermistor index [0..4]
     * @return Reading containing temperature and fault status
     */
    Reading getSensor(uint8_t index) const;

    /**
     * @brief Get average temperature of all valid sensors
     *
     * Sensors with faults are excluded.
     *
     * @return Average temperature (0.1 °C), or INT16_MIN if invalid
     */
    int16_t getAverage() const;

    void loadCoefficients(core::dev::M24C32& eeprom, uint16_t base_addr);

private:
    // --- Conversion helpers ---
    float thermistorBetaTemp(uint32_t adc,float vref, uint32_t adc_max,float r_pullup);
    Fault detectFault(uint32_t adc) const;

    Coefficients coeffs_[SENSOR_COUNT];

    core::io::ADC* adcs_[SENSOR_COUNT];
    Reading readings_[SENSOR_COUNT];

    // --- Hardware constants ---
    static constexpr float VREF        = 3.3f;
    static constexpr float PULLUP_OHMS = 10000.0f;
    static constexpr float ADC_MAX     = 4095.0f;

    // --- Steinhart–Hart coefficients (placeholder defaults) ---
    static constexpr float SH_A = 1.009249522e-03f;
    static constexpr float SH_B = 2.378405444e-04f;
    static constexpr float SH_C = 2.019202697e-07f;
};

} // namespace msd::bms

#endif
