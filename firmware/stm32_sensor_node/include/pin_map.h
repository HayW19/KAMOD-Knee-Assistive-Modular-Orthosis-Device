#ifndef PIN_MAP_H
#define PIN_MAP_H

#include <Arduino.h>

static const int32_t KAMOD_PIN_DISABLED = -1;

// placeholder weact map
static const int32_t KAMOD_PIN_FDCAN1_RX = PB8;
static const int32_t KAMOD_PIN_FDCAN1_TX = PB9;

// shared spi bus
static const int32_t KAMOD_PIN_SPI_SCK = PA5;
static const int32_t KAMOD_PIN_SPI_MISO = PA6;
static const int32_t KAMOD_PIN_SPI_MOSI = PA7;

// digipot chip selects
static const int32_t KAMOD_PIN_DIGIPOT_0_CS = PA4;
static const int32_t KAMOD_PIN_DIGIPOT_1_CS = PB0;

// adc pins
static const int32_t KAMOD_PIN_ADC_0_CS = PA15;
static const int32_t KAMOD_PIN_ADC_DRDY = PB1;
static const int32_t KAMOD_PIN_ADC_RESET = PB2;
static const int32_t KAMOD_PIN_ADC_SYNC = PB10;

// imu placeholder pins
static const int32_t KAMOD_PIN_IMU_CS = PB12;
static const int32_t KAMOD_PIN_IMU_INT = PB13;
static const int32_t KAMOD_PIN_IMU_I2C_SCL = PB6;
static const int32_t KAMOD_PIN_IMU_I2C_SDA = PB7;

// debug
static const int32_t KAMOD_PIN_STATUS_LED = PC13;

#endif /* PIN_MAP_H */
