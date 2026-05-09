#ifndef EKF_SENSOR_FUSION_H
#define EKF_SENSOR_FUSION_H

#include <Arduino.h>

typedef struct
{
    float process_angle;
    float process_velocity;
    float process_bias;
    float measurement_angle;
    float measurement_gyro;
    float measurement_fsr;
    float covariance_seed;
} ekf_sensor_fusion_config_t;

typedef struct
{
    float knee_angle_rad;
    float knee_angular_velocity_rad_s;
    float fsr_pressure_bias;
} ekf_sensor_fusion_output_t;

typedef struct
{
    bool initialized;
    ekf_sensor_fusion_config_t config;
    float x[3];
    float p[3][3];
} ekf_sensor_fusion_state_t;

bool ekf_sensor_fusion_init(
    ekf_sensor_fusion_state_t *state,
    const ekf_sensor_fusion_config_t *config);
void ekf_sensor_fusion_reset(
    ekf_sensor_fusion_state_t *state);
void ekf_sensor_fusion_predict(
    ekf_sensor_fusion_state_t *state,
    float dt_s,
    bool gyro_available,
    float gyro_rad_s);
void ekf_sensor_fusion_update_imu(
    ekf_sensor_fusion_state_t *state,
    bool angle_valid,
    float angle_rad,
    bool gyro_valid,
    float gyro_rad_s);
void ekf_sensor_fusion_update_fsr(
    ekf_sensor_fusion_state_t *state,
    bool feature_valid,
    float feature_value);
void ekf_sensor_fusion_get_state(
    const ekf_sensor_fusion_state_t *state,
    ekf_sensor_fusion_output_t *output);
void ekf_sensor_fusion_get_covariance(
    const ekf_sensor_fusion_state_t *state,
    float covariance_out[3][3]);

#endif /* EKF_SENSOR_FUSION_H */
