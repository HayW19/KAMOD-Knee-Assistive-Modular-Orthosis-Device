#include "ekf_sensor_fusion.h"

#include <math.h>
#include <string.h>

namespace
{
static const float k_min_dt_s = 0.0005f;
static const float k_max_dt_s = 0.0500f;
static const float k_min_covariance = 1.0e-6f;
static const float k_max_covariance = 1.0e6f;

float clamp_finite(float value, float fallback)
{
    if (!isfinite(value))
    {
        return fallback;
    }
    return value;
}

void clamp_covariance(ekf_sensor_fusion_state_t *state)
{
    for (uint8_t row = 0U; row < 3U; ++row)
    {
        for (uint8_t col = 0U; col < 3U; ++col)
        {
            float value = clamp_finite(state->p[row][col], 0.0f);
            if (row == col)
            {
                if (value < k_min_covariance)
                {
                    value = k_min_covariance;
                }
                if (value > k_max_covariance)
                {
                    value = k_max_covariance;
                }
            }
            state->p[row][col] = value;
        }
    }
}

void scalar_update(
    ekf_sensor_fusion_state_t *state,
    float measurement,
    const float h[3],
    float measurement_noise)
{
    float ph[3] = {0.0f, 0.0f, 0.0f};
    float innovation_covariance = measurement_noise;

    for (uint8_t row = 0U; row < 3U; ++row)
    {
        for (uint8_t col = 0U; col < 3U; ++col)
        {
            ph[row] += state->p[row][col] * h[col];
        }
        innovation_covariance += h[row] * ph[row];
    }

    if (!isfinite(innovation_covariance) || (innovation_covariance <= k_min_covariance))
    {
        return;
    }

    float expected_measurement = 0.0f;
    for (uint8_t index = 0U; index < 3U; ++index)
    {
        expected_measurement += h[index] * state->x[index];
    }

    const float residual = measurement - expected_measurement;
    float kalman_gain[3] = {0.0f, 0.0f, 0.0f};

    for (uint8_t index = 0U; index < 3U; ++index)
    {
        kalman_gain[index] = ph[index] / innovation_covariance;
        state->x[index] = clamp_finite(state->x[index] + (kalman_gain[index] * residual), 0.0f);
    }

    float updated_covariance[3][3] = {};
    for (uint8_t row = 0U; row < 3U; ++row)
    {
        for (uint8_t col = 0U; col < 3U; ++col)
        {
            updated_covariance[row][col] = state->p[row][col] - (kalman_gain[row] * ph[col]);
        }
    }

    memcpy(state->p, updated_covariance, sizeof(updated_covariance));
    clamp_covariance(state);
}
}

bool ekf_sensor_fusion_init(
    ekf_sensor_fusion_state_t *state,
    const ekf_sensor_fusion_config_t *config)
{
    if ((state == nullptr) || (config == nullptr))
    {
        return false;
    }

    state->initialized = true;
    state->config = *config;
    ekf_sensor_fusion_reset(state);
    return true;
}

void ekf_sensor_fusion_reset(
    ekf_sensor_fusion_state_t *state)
{
    if ((state == nullptr) || !state->initialized)
    {
        return;
    }

    state->x[0] = 0.0f;
    state->x[1] = 0.0f;
    state->x[2] = 0.0f;

    for (uint8_t row = 0U; row < 3U; ++row)
    {
        for (uint8_t col = 0U; col < 3U; ++col)
        {
            state->p[row][col] = (row == col) ? state->config.covariance_seed : 0.0f;
        }
    }
}

void ekf_sensor_fusion_predict(
    ekf_sensor_fusion_state_t *state,
    float dt_s,
    bool gyro_available,
    float gyro_rad_s)
{
    if ((state == nullptr) || !state->initialized)
    {
        return;
    }

    dt_s = clamp_finite(dt_s, k_min_dt_s);
    if (dt_s < k_min_dt_s)
    {
        dt_s = k_min_dt_s;
    }
    if (dt_s > k_max_dt_s)
    {
        dt_s = k_max_dt_s;
    }

    const float old_p00 = state->p[0][0];
    const float old_p01 = state->p[0][1];
    const float old_p02 = state->p[0][2];
    const float old_p10 = state->p[1][0];
    const float old_p11 = state->p[1][1];
    const float old_p12 = state->p[1][2];
    const float old_p20 = state->p[2][0];
    const float old_p21 = state->p[2][1];
    const float old_p22 = state->p[2][2];

    state->x[0] = clamp_finite(state->x[0] + (state->x[1] * dt_s), 0.0f);
    if (gyro_available)
    {
        state->x[1] = clamp_finite(gyro_rad_s, state->x[1]);
    }

    state->p[0][0] = old_p00 + (dt_s * (old_p10 + old_p01)) + (dt_s * dt_s * old_p11) + state->config.process_angle;
    state->p[0][1] = old_p01 + (dt_s * old_p11);
    state->p[0][2] = old_p02 + (dt_s * old_p12);
    state->p[1][0] = old_p10 + (dt_s * old_p11);
    state->p[1][1] = old_p11 + state->config.process_velocity;
    state->p[1][2] = old_p12;
    state->p[2][0] = old_p20 + (dt_s * old_p21);
    state->p[2][1] = old_p21;
    state->p[2][2] = old_p22 + state->config.process_bias;

    clamp_covariance(state);
}

void ekf_sensor_fusion_update_imu(
    ekf_sensor_fusion_state_t *state,
    bool angle_valid,
    float angle_rad,
    bool gyro_valid,
    float gyro_rad_s)
{
    if ((state == nullptr) || !state->initialized)
    {
        return;
    }

    if (angle_valid)
    {
        const float h_angle[3] = {1.0f, 0.0f, 0.0f};
        scalar_update(state, angle_rad, h_angle, state->config.measurement_angle);
    }

    if (gyro_valid)
    {
        const float h_rate[3] = {0.0f, 1.0f, 0.0f};
        scalar_update(state, gyro_rad_s, h_rate, state->config.measurement_gyro);
    }
}

void ekf_sensor_fusion_update_fsr(
    ekf_sensor_fusion_state_t *state,
    bool feature_valid,
    float feature_value)
{
    if ((state == nullptr) || !state->initialized || !feature_valid)
    {
        return;
    }

    const float h_fsr[3] = {0.0f, 0.0f, 1.0f};
    scalar_update(state, feature_value, h_fsr, state->config.measurement_fsr);
}

void ekf_sensor_fusion_get_state(
    const ekf_sensor_fusion_state_t *state,
    ekf_sensor_fusion_output_t *output)
{
    if ((state == nullptr) || (output == nullptr))
    {
        return;
    }

    output->knee_angle_rad = state->x[0];
    output->knee_angular_velocity_rad_s = state->x[1];
    output->fsr_pressure_bias = state->x[2];
}

void ekf_sensor_fusion_get_covariance(
    const ekf_sensor_fusion_state_t *state,
    float covariance_out[3][3])
{
    if ((state == nullptr) || (covariance_out == nullptr))
    {
        return;
    }

    memcpy(covariance_out, state->p, sizeof(state->p));
}
