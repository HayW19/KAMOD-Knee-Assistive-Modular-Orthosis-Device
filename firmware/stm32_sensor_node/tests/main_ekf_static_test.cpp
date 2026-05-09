#include <Arduino.h>

#include "../include/ekf_sensor_fusion.h"
#include "../include/sensor_logger.h"

namespace
{
ekf_sensor_fusion_state_t g_ekf = {};
uint32_t g_last_step_ms = 0U;
}

void setup()
{
    sensor_logger_begin(115200UL);
    const ekf_sensor_fusion_config_t config = {
        0.02f,
        0.15f,
        0.05f,
        0.08f,
        0.05f,
        0.20f,
        1.0f,
    };
    ekf_sensor_fusion_init(&g_ekf, &config);
    sensor_logger_log_text("warn", millis(), "bench test only");
}

void loop()
{
    if ((millis() - g_last_step_ms) < 20U)
    {
        return;
    }

    g_last_step_ms = millis();
    ekf_sensor_fusion_predict(&g_ekf, 0.02f, true, 0.0f);
    ekf_sensor_fusion_update_imu(&g_ekf, true, 0.15f, true, 0.0f);
    ekf_sensor_fusion_update_fsr(&g_ekf, true, 0.0f);

    ekf_sensor_fusion_output_t output = {};
    ekf_sensor_fusion_get_state(&g_ekf, &output);

    Serial.print("ekf_static,");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(output.knee_angle_rad, 6);
    Serial.print(",");
    Serial.print(output.knee_angular_velocity_rad_s, 6);
    Serial.print(",");
    Serial.println(output.fsr_pressure_bias, 6);
}
