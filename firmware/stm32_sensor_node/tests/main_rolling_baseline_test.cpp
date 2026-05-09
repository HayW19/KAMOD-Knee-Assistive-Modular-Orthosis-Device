#include <Arduino.h>

#include "../include/rolling_baseline.h"
#include "../include/sensor_logger.h"

namespace
{
rolling_baseline_state_t g_state = {};
rolling_baseline_config_t g_config = {};
uint32_t g_index = 0U;
const float g_samples[] = {1200000.0f, 1200500.0f, 1200200.0f, 1200400.0f, 1245000.0f, 1200600.0f, 1200300.0f};
}

void setup()
{
    sensor_logger_begin(115200UL);
    g_config.alpha_slow = 0.0025f;
    g_config.alpha_fast = 0.01f;
    g_config.movement_threshold = 15000.0f;
    rolling_baseline_init(&g_state, &g_config, 1200000.0f);
    sensor_logger_log_text("warn", millis(), "bench test only");
}

void loop()
{
    if (g_index >= (sizeof(g_samples) / sizeof(g_samples[0])))
    {
        return;
    }

    rolling_baseline_step(&g_state, &g_config, g_samples[g_index], false, false);
    Serial.print("baseline,");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(g_samples[g_index]);
    Serial.print(",");
    Serial.print(g_state.baseline);
    Serial.print(",");
    Serial.println(g_state.corrected_value);
    g_index += 1U;
}
