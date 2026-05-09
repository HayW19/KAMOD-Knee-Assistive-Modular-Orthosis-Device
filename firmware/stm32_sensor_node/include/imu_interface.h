#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include <Arduino.h>

typedef struct
{
    bool angle_valid;
    bool gyro_valid;
    float angle_rad;
    float gyro_rad_s;
} imu_interface_sample_t;

typedef struct
{
    int32_t cs_pin;
    int32_t int_pin;
} imu_interface_config_t;

typedef struct
{
    bool initialized;
    bool mock_mode;
    imu_interface_sample_t mock_sample;
} imu_interface_state_t;

bool imu_interface_init(
    imu_interface_state_t *state,
    const imu_interface_config_t *config);
bool imu_interface_poll(
    imu_interface_state_t *state,
    imu_interface_sample_t *sample_out);
void imu_interface_set_mock_sample(
    imu_interface_state_t *state,
    float angle_rad,
    float gyro_rad_s,
    bool valid);

#endif /* IMU_INTERFACE_H */
