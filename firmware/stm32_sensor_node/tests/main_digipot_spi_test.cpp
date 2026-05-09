#include <Arduino.h>

#include "../include/digipot_mcp41xx_style.h"
#include "../include/kamod_sensor_config.h"
#include "../include/pin_map.h"
#include "../include/sensor_logger.h"
#include "../include/spi_bus.h"

namespace
{
spi_bus_state_t g_spi_bus = {};
digipot_mcp41xx_style_state_t g_digipot = {};
uint8_t g_current_tap = KAMOD_DIGIPOT_MIN_TAP;
uint32_t g_last_step_ms = 0U;
}

void setup()
{
    sensor_logger_begin(115200UL);
    const spi_bus_config_t spi_config = {
        KAMOD_PIN_SPI_SCK,
        KAMOD_PIN_SPI_MISO,
        KAMOD_PIN_SPI_MOSI,
        1000000UL,
        SPI_MODE0,
    };
    const digipot_mcp41xx_style_config_t digipot_config = {
        KAMOD_PIN_DIGIPOT_0_CS,
        KAMOD_DIGIPOT_NOMINAL_OHMS,
    };
    spi_bus_init(&g_spi_bus, &spi_config);
    digipot_mcp41xx_style_init(&g_digipot, &g_spi_bus, &digipot_config);
    sensor_logger_log_text("warn", millis(), "bench test only");
}

void loop()
{
    if ((millis() - g_last_step_ms) < 200U)
    {
        return;
    }

    g_last_step_ms = millis();
    digipot_mcp41xx_style_write_tap(&g_digipot, &g_spi_bus, DIGIPOT_CHANNEL_A, g_current_tap);
    Serial.print("digipot_tap,");
    Serial.print(millis());
    Serial.print(",");
    Serial.println(g_current_tap);
    g_current_tap = static_cast<uint8_t>(g_current_tap + 32U);
}
