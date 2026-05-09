#include "imu_interface.h"

#include "pin_map.h"

bool imu_interface_init(
    imu_interface_state_t *state,
    const imu_interface_config_t *config)
{
    if ((state == nullptr) || (config == nullptr))
    {
        return false;
    }

    if (config->int_pin != KAMOD_PIN_DISABLED)
    {
        pinMode(config->int_pin, INPUT_PULLUP);
    }

    if (config->cs_pin != KAMOD_PIN_DISABLED)
    {
        pinMode(config->cs_pin, OUTPUT);
        digitalWrite(config->cs_pin, HIGH);
    }

    state->initialized = true;
#if KAMOD_USE_MOCK_IMU
    state->mock_mode = true;
#else
    state->mock_mode = false;
#endif
    state->mock_sample = {};
    return true;
}

bool imu_interface_poll(
    imu_interface_state_t *state,
    imu_interface_sample_t *sample_out)
{
    if ((state == nullptr) || (sample_out == nullptr) || !state->initialized)
    {
        return false;
    }

    if (state->mock_mode)
    {
        *sample_out = state->mock_sample;
        return state->mock_sample.angle_valid || state->mock_sample.gyro_valid;
    }

    sample_out->angle_valid = false;
    sample_out->gyro_valid = false;
    sample_out->angle_rad = 0.0f;
    sample_out->gyro_rad_s = 0.0f;
    return false;
}

void imu_interface_set_mock_sample(
    imu_interface_state_t *state,
    float angle_rad,
    float gyro_rad_s,
    bool valid)
{
    if (state == nullptr)
    {
        return;
    }

    state->mock_sample.angle_valid = valid;
    state->mock_sample.gyro_valid = valid;
    state->mock_sample.angle_rad = angle_rad;
    state->mock_sample.gyro_rad_s = gyro_rad_s;
}
