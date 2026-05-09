#ifndef DIGIPOT_MCP41XX_STYLE_H
#define DIGIPOT_MCP41XX_STYLE_H

#include <Arduino.h>

#include "spi_bus.h"

typedef enum
{
    DIGIPOT_CHANNEL_A = 0,
    DIGIPOT_CHANNEL_B = 1
} digipot_channel_t;

typedef struct
{
    int32_t cs_pin;
    uint32_t nominal_ohms;
} digipot_mcp41xx_style_config_t;

typedef struct
{
    bool initialized;
    digipot_mcp41xx_style_config_t config;
    uint8_t last_tap_a;
    uint8_t last_tap_b;
} digipot_mcp41xx_style_state_t;

bool digipot_mcp41xx_style_init(
    digipot_mcp41xx_style_state_t *state,
    spi_bus_state_t *spi_bus,
    const digipot_mcp41xx_style_config_t *config);
uint8_t digipot_mcp41xx_style_clamp_tap(uint16_t tap_value);
bool digipot_mcp41xx_style_write_tap(
    digipot_mcp41xx_style_state_t *state,
    spi_bus_state_t *spi_bus,
    digipot_channel_t channel,
    uint8_t tap_value);
uint8_t digipot_mcp41xx_style_get_tap(
    const digipot_mcp41xx_style_state_t *state,
    digipot_channel_t channel);

#endif /* DIGIPOT_MCP41XX_STYLE_H */
