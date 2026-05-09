#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <Arduino.h>
#include <SPI.h>

typedef struct
{
    int32_t sck_pin;
    int32_t miso_pin;
    int32_t mosi_pin;
    uint32_t clock_hz;
    uint8_t data_mode;
} spi_bus_config_t;

typedef struct
{
    bool initialized;
    spi_bus_config_t config;
} spi_bus_state_t;

bool spi_bus_init(spi_bus_state_t *state, const spi_bus_config_t *config);
void spi_bus_select(int32_t cs_pin);
void spi_bus_deselect(int32_t cs_pin);
void spi_bus_begin(spi_bus_state_t *state);
void spi_bus_end(void);
uint8_t spi_bus_transfer8(uint8_t value);

#endif /* SPI_BUS_H */
