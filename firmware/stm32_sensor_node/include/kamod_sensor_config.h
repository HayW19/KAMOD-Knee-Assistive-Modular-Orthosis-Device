#ifndef KAMOD_SENSOR_CONFIG_H
#define KAMOD_SENSOR_CONFIG_H

#include <Arduino.h>

#include "../../../common/types/kamod_ids.h"

#ifndef KAMOD_USE_MOCK_ADC
#define KAMOD_USE_MOCK_ADC 1
#endif

#ifndef KAMOD_USE_MOCK_IMU
#define KAMOD_USE_MOCK_IMU 1
#endif

#ifndef KAMOD_USE_MOCK_CANFD
#define KAMOD_USE_MOCK_CANFD 1
#endif

static const uint32_t KAMOD_SENSOR_SERIAL_BAUD = 115200UL;
static const kamod_node_id_t KAMOD_NODE_ID = KAMOD_NODE_ID_SMART_NODE_1;
static const uint8_t KAMOD_PACKET_TYPE_SENSOR_NODE = 1U;

static const uint16_t KAMOD_FSR_ADC_SAMPLE_HZ = 250U;
static const uint16_t KAMOD_SENSOR_PACKET_RATE_HZ = 100U;
static const uint16_t KAMOD_EKF_UPDATE_HZ = 250U;
static const uint16_t KAMOD_SERIAL_LOG_RATE_HZ = 50U;

static const uint8_t KAMOD_FSR_CHANNEL_COUNT = 1U;
static const uint8_t KAMOD_FSR_CHANNEL_ID_0 = 0U;
static const uint8_t KAMOD_ADC_CHANNEL_0 = 0U;

static const uint8_t KAMOD_DIGIPOT_MIN_TAP = 0U;
static const uint8_t KAMOD_DIGIPOT_MAX_TAP = 255U;
static const uint32_t KAMOD_DIGIPOT_NOMINAL_OHMS = 100000UL;
static const uint8_t KAMOD_DIGIPOT_START_TAP = 128U;
static const uint8_t KAMOD_DIGIPOT_CALIBRATION_STEP = 4U;
static const uint32_t KAMOD_DIGIPOT_CALIBRATION_TIMEOUT_MS = 3000UL;

static const int32_t KAMOD_ADC_TARGET_MIN = 1200000L;
static const int32_t KAMOD_ADC_TARGET_MAX = 1800000L;
static const int32_t KAMOD_ADC_SATURATION_LOW = 100000L;
static const int32_t KAMOD_ADC_SATURATION_HIGH = 8200000L;
static const uint32_t KAMOD_STARTUP_SAMPLE_COUNT = 32UL;
static const uint32_t KAMOD_CALIBRATION_TIMEOUT_MS = 3000UL;
static const float KAMOD_ADC_REFERENCE_VOLTAGE = 2.500f;
static const int32_t KAMOD_MOCK_ADC_BASE_COUNTS = 900000L;
static const int32_t KAMOD_MOCK_ADC_TAP_GAIN_COUNTS = 6000L;

static const float KAMOD_FSR_FILTER_ALPHA = 0.15f;
static const bool KAMOD_ENABLE_MEDIAN_FILTER_PLACEHOLDER = false;

static const float KAMOD_BASELINE_ALPHA_SLOW = 0.0025f;
static const float KAMOD_BASELINE_ALPHA_FAST = 0.0100f;
static const float KAMOD_MOVEMENT_THRESHOLD = 15000.0f;
static const float KAMOD_FMG_ACTIVE_THRESHOLD = 20000.0f;

static const float KAMOD_EKF_PROCESS_ANGLE = 0.02f;
static const float KAMOD_EKF_PROCESS_VELOCITY = 0.15f;
static const float KAMOD_EKF_PROCESS_BIAS = 0.05f;
static const float KAMOD_EKF_MEASUREMENT_ANGLE = 0.08f;
static const float KAMOD_EKF_MEASUREMENT_GYRO = 0.05f;
static const float KAMOD_EKF_MEASUREMENT_FSR = 0.20f;
static const float KAMOD_EKF_COVARIANCE_SEED = 1.00f;

static const uint32_t KAMOD_CANFD_PACKET_ID = 0x301UL;
static const uint32_t KAMOD_CANFD_NOMINAL_BITRATE = 1000000UL;
static const uint32_t KAMOD_CANFD_DATA_BITRATE = 2000000UL;

#endif /* KAMOD_SENSOR_CONFIG_H */
