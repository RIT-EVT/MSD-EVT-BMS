//
// Created by Key'Mon Jenkins on 2/16/26.
//

#ifndef _EVT_SPI3_F4XX_
#define _EVT_SPI3_F4XX_

#include <core/io/SPI.hpp>
#include "HALf4/stm32f4xx.h"


// Data Size bits (DS[3:0]) - bits 8-11
#define SPI_CR2_DS_Pos      (8U)
#define SPI_CR2_DS_0        (0x1U << 8)   // Bit 8
#define SPI_CR2_DS_1        (0x1U << 9)   // Bit 9
#define SPI_CR2_DS_2        (0x1U << 10)  // Bit 10
#define SPI_CR2_DS_3        (0x1U << 11)  // Bit 11

// FIFO Reception Threshold - bit 12
#define SPI_CR2_FRXTH_Pos   (12U)
#define SPI_CR2_FRXTH       (0x1U << 12)
#define SPI_MAX_BAUD 4000000


namespace core::io {

class SPI3f4xx : public SPI {
public:
    SPI3f4xx(GPIO* CSPins[], uint8_t pinLength,
             Pin sckPin, Pin mosiPin, Pin misoPin);

    SPI3f4xx(GPIO* CSPins[], uint8_t pinLength,
             Pin sckPin, Pin mosiPin);

    bool startTransmission(uint8_t device) override;
    bool endTransmission(uint8_t device) override;

    SPIStatus write(uint8_t byte) override;
    SPIStatus read(uint8_t* out) override;

    void configureSPI(uint32_t baudRate,
                      SPIMode mode,
                      bool order) override;

private:
    void enableClocks();
    void configurePins();
    static uint32_t baudToPrescalerHAL(uint32_t baud);
    SPI_HandleTypeDef halSPI3_ = {};

};

} // namespace core::io

#endif