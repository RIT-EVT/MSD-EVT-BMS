//
// Created by Key'Mon Jenkins on 2/16/26.
//
#include "dev/spi.hpp"

#include "core/utils/log.hpp"

namespace core::io {

// =====================================================
// Constructor
// =====================================================

SPI3f4xx::SPI3f4xx(GPIO* cs[], uint8_t len, Pin sck, Pin mosi, Pin miso)
    : SPI(cs, len, sck, mosi, miso)
{
    enableClocks();
    HAL_SPI_MspInit(&halSPI3_);
}

SPI3f4xx::SPI3f4xx(GPIO* cs[], uint8_t len, Pin sck, Pin mosi)
    : SPI(cs, len, sck, mosi)
{
    enableClocks();
    HAL_SPI_MspInit(&halSPI3_);
}

// =====================================================
// Clock Setup
// =====================================================

void SPI3f4xx::enableClocks()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
}

// =====================================================
// GPIO Configuration
// PC10  → SCK   (AF6)
// PC11  → MISO  (AF6)
// PC12  → MOSI  (AF6)
// PA15  → NCS   (GPIO output)
// =====================================================

void SPI3f4xx::configurePins()
{

    GPIOC->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11 | GPIO_PUPDR_PUPD12);
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11 | GPIO_OTYPER_OT12);

    // --- PC10,11,12 alternate function ---
    GPIOC->MODER &= ~(GPIO_MODER_MODE10 |
                      GPIO_MODER_MODE11 |
                      GPIO_MODER_MODE12);

    GPIOC->MODER |= (GPIO_MODER_MODE10_1 |
                     GPIO_MODER_MODE11_1 |
                     GPIO_MODER_MODE12_1);

    // Disable any pull-up/pull-down
    // GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD11;

    // AF6 = SPI3
    GPIOC->AFR[1] &= ~(
        (0xF << ((10-8)*4)) |
        (0xF << ((11-8)*4)) |
        (0xF << ((12-8)*4)));

    GPIOC->AFR[1] |= (
        (6 << ((10-8)*4)) |
        (6 << ((11-8)*4)) |
        (6 << ((12-8)*4)));

    // High speed
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR10 |
                       GPIO_OSPEEDER_OSPEEDR11 |
                       GPIO_OSPEEDER_OSPEEDR12);

    // --- PA15 as GPIO output (if used as CS externally) ---
    GPIOA->MODER &= ~GPIO_MODER_MODE15;
    GPIOA->MODER |= GPIO_MODER_MODE15_0;
}

// =====================================================
// Transmission Control
// =====================================================

bool SPI3f4xx::startTransmission(uint8_t device)
{
    if (device >= CSPinsLength)
        return false;

    CSPins[device]->writePin(static_cast<GPIO::State>(false)); // CS LOW
    return true;
}

bool SPI3f4xx::endTransmission(uint8_t device)
{
    if (device >= CSPinsLength)
        return false;

    while (SPI3->SR & SPI_SR_BSY) {}
    CSPins[device]->writePin(static_cast<GPIO::State>(true));  // CS HIGH
    return true;
}

// =====================================================
// Write Single Byte
// =====================================================

// SPI::SPIStatus SPI3f4xx::write(uint8_t byte)
// {
//     uint32_t timeout = EVT_SPI_TIMEOUT;
//
//     while (!(SPI3->SR & SPI_SR_TXE))
//         if (!timeout--) return SPIStatus::TIMEOUT;
//
//     SPI3->DR = byte;
//
//     timeout = EVT_SPI_TIMEOUT;
//     while (!(SPI3->SR & SPI_SR_RXNE))
//         if (!timeout--) return SPIStatus::TIMEOUT;
//
//     (void)SPI3->DR; // clear RX
//
//     return SPIStatus::OK;
// }

SPI::SPIStatus SPI3f4xx::write(uint8_t byte)
{
    if (HAL_SPI_Transmit(&halSPI3_, &byte, 1, HAL_MAX_DELAY) != HAL_OK) {
        return SPIStatus::ERROR;
    }
    return SPIStatus::OK;
}


// =====================================================
// Read Single Byte
// =====================================================

// SPI::SPIStatus SPI3f4xx::read(uint8_t* out)
// {
//     uint32_t timeout = EVT_SPI_TIMEOUT;
//
//     while (!(SPI3->SR & SPI_SR_TXE))
//         if (!timeout--) return SPIStatus::TIMEOUT;
//
//     SPI3->DR = 0xFF; // dummy byte
//
//     timeout = EVT_SPI_TIMEOUT;
//     while (!(SPI3->SR & SPI_SR_RXNE))
//         if (!timeout--) return SPIStatus::TIMEOUT;
//
//     *out = SPI3->DR;
//
//     return SPIStatus::OK;
// }

// Update read() to use HAL
SPI::SPIStatus SPI3f4xx::read(uint8_t* out)
{
    uint8_t dummy = 0xFF;
    if (HAL_SPI_Receive(&halSPI3_, out, 1, HAL_MAX_DELAY) != HAL_OK)
        return SPIStatus::ERROR;

    return SPIStatus::OK;
}

// =====================================================
// Baudrate Mapping
// =====================================================

// uint32_t SPI3f4xx::baudToPrescaler(uint32_t baud)
// {
//     // Assuming APB1 ≈ 45MHz (typical F446 config)
//     // Adjust if your clock differs
//
//     if (baud >= SPI_SPEED_4MHZ)   return SPI_CR1_BR_0;                       // /4
//     if (baud >= SPI_SPEED_2MHZ)   return SPI_CR1_BR_1;                       // /8
//     if (baud >= SPI_SPEED_1MHZ)   return SPI_CR1_BR_1 | SPI_CR1_BR_0;        // /16
//     if (baud >= SPI_SPEED_500KHZ) return SPI_CR1_BR_2;                       // /32
//     if (baud >= SPI_SPEED_250KHZ) return SPI_CR1_BR_2 | SPI_CR1_BR_0;        // /64
//     if (baud >= SPI_SPEED_125KHZ) return SPI_CR1_BR_2 | SPI_CR1_BR_1;        // /128
//
//     return SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;                       // /256
// }

// =====================================================
// SPI Configuration
// =====================================================

// void SPI3f4xx::configureSPI(uint32_t baudRate, SPIMode mode, bool order)
// {
//     // Disable SPI
//     SPI3->CR1 = 0;
//     while (SPI3->SR & SPI_SR_BSY){}
//
//     // CR2 configuration - only use valid bits for STM32F4
//     SPI3->CR2 = 0;  // Keep all reserved bits at 0
//     // Don't write to bits 15:8 or bit 3 - they're reserved!
//
//     // Configure CR1 for 8-bit mode
//     uint32_t cr1 = 0;
//
//     cr1 |= SPI_CR1_MSTR;   // Master mode (bit 2)
//     cr1 |= SPI_CR1_SSM;    // Software slave management (bit 9)
//     cr1 |= SPI_CR1_SSI;    // Internal slave select (bit 8)
//
//     // DFF = 0 for 8-bit (bit 11) - this is the default, but let's be explicit
//     // cr1 &= ~SPI_CR1_DFF;  // Not needed since we start with cr1=0
//
//     cr1 |= baudToPrescaler(baudRate);
//
//     // Bit order
//     if (!order)
//         cr1 |= SPI_CR1_LSBFIRST;  // LSB first (bit 7)
//
//     // Clock mode
//     switch (mode)
//     {
//     case SPIMode::SPI_MODE0:
//         // CPOL=0, CPHA=0 (default)
//         break;
//     case SPIMode::SPI_MODE1:
//         cr1 |= SPI_CR1_CPHA;  // Bit 0
//         break;
//     case SPIMode::SPI_MODE2:
//         cr1 |= SPI_CR1_CPOL;  // Bit 1
//         break;
//     case SPIMode::SPI_MODE3:
//         cr1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;
//         break;
//     }
//
//     // Write CR1 (with DFF=0 for 8-bit)
//     SPI3->CR1 = cr1;
//
//     // Enable SPI
//     SPI3->CR1 |= SPI_CR1_SPE;  // Bit 6
//
//     // Clear any garbage in DR
//     volatile uint32_t dummy;
//     while (SPI3->SR & SPI_SR_RXNE) {
//         dummy = SPI3->DR;
//     }
//     (void)dummy;
// }

void SPI3f4xx::configureSPI(uint32_t baudRate, SPIMode mode, bool firstBitMSB) {

    // Deinitialize if already configured
    if (halSPI3_.Instance != NULL) {
        HAL_SPI_DeInit(&halSPI3_);
    }

    // Configure SPI handle
    halSPI3_.Instance = SPI3;
    halSPI3_.Init.Mode = SPI_MODE_MASTER;
    halSPI3_.Init.Direction = SPI_DIRECTION_2LINES;  // Full duplex
    halSPI3_.Init.DataSize = SPI_DATASIZE_8BIT;      // 8-bit data
    halSPI3_.Init.NSS = SPI_NSS_SOFT;                // Software CS management
    halSPI3_.Init.FirstBit = firstBitMSB ? SPI_FIRSTBIT_MSB : SPI_FIRSTBIT_LSB;

    switch (mode) {
    case io::SPI::SPIMode::SPI_MODE0:
        halSPI3_.Init.CLKPolarity = SPI_POLARITY_LOW;
        halSPI3_.Init.CLKPhase    = SPI_PHASE_1EDGE;
        break;
    case io::SPI::SPIMode::SPI_MODE1:
        halSPI3_.Init.CLKPolarity = SPI_POLARITY_LOW;
        halSPI3_.Init.CLKPhase    = SPI_PHASE_2EDGE;
        break;
    case io::SPI::SPIMode::SPI_MODE2:
        halSPI3_.Init.CLKPolarity = SPI_POLARITY_HIGH;
        halSPI3_.Init.CLKPhase    = SPI_PHASE_1EDGE;
        break;
    case io::SPI::SPIMode::SPI_MODE3:
        halSPI3_.Init.CLKPolarity = SPI_POLARITY_HIGH;
        halSPI3_.Init.CLKPhase    = SPI_PHASE_2EDGE;
        break;
    default:
        log::LOGGER.log(log::Logger::LogLevel::ERROR, "Invalid SPI Mode");
        break;
    }

    // Set baud rate prescaler
    halSPI3_.Init.BaudRatePrescaler = baudToPrescalerHAL(baudRate);

    // Other settings
    halSPI3_.Init.TIMode = SPI_TIMODE_DISABLE;
    halSPI3_.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    halSPI3_.Init.CRCPolynomial = 10;

    // Initialize SPI
    if (HAL_SPI_Init(&halSPI3_) != HAL_OK) {
        // Initialization failed
        // Handle error (you might want to add error handling here)
    }
}

uint32_t SPI3f4xx::baudToPrescalerHAL(uint32_t baud)
{
    // Assuming APB1 ≈ 45MHz for STM32F446
    if (baud >= SPI_SPEED_4MHZ)   return SPI_BAUDRATEPRESCALER_4;   // 45/4  = 11.25 MHz
    if (baud >= SPI_SPEED_2MHZ)   return SPI_BAUDRATEPRESCALER_8;   // 45/8  = 5.625 MHz
    if (baud >= SPI_SPEED_1MHZ)   return SPI_BAUDRATEPRESCALER_16;  // 45/16 = 2.8 MHz
    if (baud >= SPI_SPEED_500KHZ) return SPI_BAUDRATEPRESCALER_32;  // 45/32 = 1.4 MHz
    if (baud >= SPI_SPEED_250KHZ) return SPI_BAUDRATEPRESCALER_64;  // 45/64 = 703 KHz
    if (baud >= SPI_SPEED_125KHZ) return SPI_BAUDRATEPRESCALER_128; // 45/128 = 352 KHz

    return SPI_BAUDRATEPRESCALER_256; // 45/256 = 176 KHz
}

// HAL callback for SPI initialization
// This gets called automatically by HAL_SPI_Init()
extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    if (hspi->Instance == SPI3)
    {
        // Enable clocks
        __HAL_RCC_SPI3_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        // Configure GPIO pins
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        // PC10 = SCK, PC11 = MISO, PC12 = MOSI
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // No pull-up/down
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;  // AF6 for SPI3

        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

// HAL callback for SPI deinitialization
extern "C" void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
    if (hspi->Instance == SPI3)
    {
        __HAL_RCC_SPI3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    }
}

} // namespace core::io