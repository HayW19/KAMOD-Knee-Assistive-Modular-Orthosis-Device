#include <Arduino.h>

#include <math.h>

#include "adc_24bit_interface.h"
#include "canfd_packets.h"
#include "digipot_mcp41xx_style.h"
#include "ekf_sensor_fusion.h"
#include "fmg_band.h"
#include "fsr_channel.h"
#include "imu_interface.h"
#include "kamod_sensor_config.h"
#include "pin_map.h"
#include "rolling_baseline.h"
#include "sensor_logger.h"
#include "spi_bus.h"

namespace
{
spi_bus_state_t g_spi_bus = {};
digipot_mcp41xx_style_state_t g_digipots[2] = {};
adc_24bit_interface_state_t g_adc = {};
rolling_baseline_config_t g_baseline_config = {};
fsr_channel_state_t g_fsr_channels[KAMOD_FSR_CHANNEL_COUNT] = {};
fmg_band_state_t g_fmg_band = {};
imu_interface_state_t g_imu = {};
ekf_sensor_fusion_state_t g_ekf = {};
canfd_packets_state_t g_packets = {};
bool g_runtime_ready = false;
uint32_t g_last_sample_us = 0U;
uint32_t g_last_ekf_us = 0U;
uint32_t g_last_packet_us = 0U;
uint32_t g_last_log_us = 0U;
uint32_t g_last_led_toggle_ms = 0U;

uint32_t rate_to_period_us(uint16_t rate_hz)
{
    if (rate_hz == 0U)
    {
        return 0U;
    }

    return 1000000UL / rate_hz;
}

void init_led(void)
{
    if (KAMOD_PIN_STATUS_LED != KAMOD_PIN_DISABLED)
    {
        pinMode(KAMOD_PIN_STATUS_LED, OUTPUT);
        digitalWrite(KAMOD_PIN_STATUS_LED, LOW);
    }
}

void toggle_led(uint32_t now_ms)
{
    if ((KAMOD_PIN_STATUS_LED == KAMOD_PIN_DISABLED) || ((now_ms - g_last_led_toggle_ms) < 250U))
    {
        return;
    }

    g_last_led_toggle_ms = now_ms;
    digitalWrite(KAMOD_PIN_STATUS_LED, !digitalRead(KAMOD_PIN_STATUS_LED));
}

void update_mock_sources(uint32_t now_ms)
{
#if KAMOD_USE_MOCK_ADC
    const uint32_t phase_ms = now_ms % 1000UL;
    int32_t pulse = 0L;
    if (phase_ms < 80UL)
    {
        pulse = 42000L;
    }
    else if (phase_ms < 140UL)
    {
        pulse = 24000L;
    }

    const int32_t slow_drift = static_cast<int32_t>(((now_ms / 1000UL) % 20UL) * 250L);
    const int32_t base_value = static_cast<int32_t>(g_fsr_channels[0].baseline);
    adc_24bit_interface_set_mock_raw(
        &g_adc,
        KAMOD_ADC_CHANNEL_0,
        base_value + slow_drift + pulse);
#endif

#if KAMOD_USE_MOCK_IMU
    const float phase = static_cast<float>(now_ms % 2000UL) / 2000.0f;
    const float angle_rad = 0.35f * sinf(phase * 6.2831853f);
    const float gyro_rad_s = 0.35f * 6.2831853f * cosf(phase * 6.2831853f) / 2.0f;
    imu_interface_set_mock_sample(&g_imu, angle_rad, gyro_rad_s, true);
#endif
}
}

void setup()
{
    sensor_logger_begin(KAMOD_SENSOR_SERIAL_BAUD);
    sensor_logger_log_header();
    sensor_logger_log_text("warn", millis(), "bench test only");
    sensor_logger_log_text("warn", millis(), "check weact pin map");
    sensor_logger_log_text("warn", millis(), "ekf rebuilt not validated");

    init_led();

    const spi_bus_config_t spi_config = {
        KAMOD_PIN_SPI_SCK,
        KAMOD_PIN_SPI_MISO,
        KAMOD_PIN_SPI_MOSI,
        1000000UL,
        SPI_MODE0,
    };
    spi_bus_init(&g_spi_bus, &spi_config);

    const digipot_mcp41xx_style_config_t digipot_0_config = {
        KAMOD_PIN_DIGIPOT_0_CS,
        KAMOD_DIGIPOT_NOMINAL_OHMS,
    };
    const digipot_mcp41xx_style_config_t digipot_1_config = {
        KAMOD_PIN_DIGIPOT_1_CS,
        KAMOD_DIGIPOT_NOMINAL_OHMS,
    };
    digipot_mcp41xx_style_init(&g_digipots[0], &g_spi_bus, &digipot_0_config);
    digipot_mcp41xx_style_init(&g_digipots[1], &g_spi_bus, &digipot_1_config);

    const adc_24bit_config_t adc_config = {
        KAMOD_PIN_ADC_0_CS,
        KAMOD_PIN_ADC_DRDY,
        KAMOD_PIN_ADC_RESET,
        KAMOD_PIN_ADC_SYNC,
        KAMOD_ADC_REFERENCE_VOLTAGE,
        KAMOD_FSR_CHANNEL_COUNT,
    };
    adc_24bit_interface_init(&g_adc, &g_spi_bus, &adc_config);
    adc_24bit_interface_set_mock_base(&g_adc, KAMOD_ADC_CHANNEL_0, KAMOD_MOCK_ADC_BASE_COUNTS);

    g_baseline_config.alpha_slow = KAMOD_BASELINE_ALPHA_SLOW;
    g_baseline_config.alpha_fast = KAMOD_BASELINE_ALPHA_FAST;
    g_baseline_config.movement_threshold = KAMOD_MOVEMENT_THRESHOLD;

    const fsr_channel_config_t fsr_0_config = {
        KAMOD_FSR_CHANNEL_ID_0,
        KAMOD_ADC_CHANNEL_0,
        0U,
        DIGIPOT_CHANNEL_A,
        static_cast<uint16_t>(KAMOD_STARTUP_SAMPLE_COUNT),
        KAMOD_ADC_TARGET_MIN,
        KAMOD_ADC_TARGET_MAX,
        KAMOD_ADC_SATURATION_LOW,
        KAMOD_ADC_SATURATION_HIGH,
        KAMOD_FSR_FILTER_ALPHA,
    };
    fsr_channel_init(&g_fsr_channels[0], &fsr_0_config, &g_baseline_config);

    const fmg_band_config_t fmg_config = {
        0U,
        KAMOD_FMG_ACTIVE_THRESHOLD,
    };
    fmg_band_init(&g_fmg_band, &fmg_config);

    const imu_interface_config_t imu_config = {
        KAMOD_PIN_IMU_CS,
        KAMOD_PIN_IMU_INT,
    };
    imu_interface_init(&g_imu, &imu_config);

    const ekf_sensor_fusion_config_t ekf_config = {
        KAMOD_EKF_PROCESS_ANGLE,
        KAMOD_EKF_PROCESS_VELOCITY,
        KAMOD_EKF_PROCESS_BIAS,
        KAMOD_EKF_MEASUREMENT_ANGLE,
        KAMOD_EKF_MEASUREMENT_GYRO,
        KAMOD_EKF_MEASUREMENT_FSR,
        KAMOD_EKF_COVARIANCE_SEED,
    };
    ekf_sensor_fusion_init(&g_ekf, &ekf_config);
    canfd_packets_init(&g_packets);

    const bool calibrated = fsr_channel_startup_calibrate(
        &g_fsr_channels[0],
        &g_spi_bus,
        g_digipots,
        2U,
        &g_adc,
        &g_baseline_config,
        millis(),
        KAMOD_CALIBRATION_TIMEOUT_MS,
        KAMOD_DIGIPOT_START_TAP,
        KAMOD_DIGIPOT_CALIBRATION_STEP);

    sensor_logger_log_calibration(
        millis(),
        KAMOD_FSR_CHANNEL_ID_0,
        g_fsr_channels[0].digipot_tap,
        g_fsr_channels[0].raw_adc,
        calibrated);

    g_runtime_ready = calibrated;
    if (!g_runtime_ready)
    {
        sensor_logger_log_text("fault", millis(), "fsr calibration failed");
    }
}

void loop()
{
    const uint32_t now_ms = millis();
    const uint32_t now_us = micros();

    toggle_led(now_ms);

    if (!g_runtime_ready)
    {
        return;
    }

    update_mock_sources(now_ms);

    const uint32_t sample_period_us = rate_to_period_us(KAMOD_FSR_ADC_SAMPLE_HZ);
    const uint32_t ekf_period_us = rate_to_period_us(KAMOD_EKF_UPDATE_HZ);
    const uint32_t packet_period_us = rate_to_period_us(KAMOD_SENSOR_PACKET_RATE_HZ);
    const uint32_t log_period_us = rate_to_period_us(KAMOD_SERIAL_LOG_RATE_HZ);

    static const fmg_band_config_t fmg_config = {
        0U,
        KAMOD_FMG_ACTIVE_THRESHOLD,
    };

    if ((now_us - g_last_sample_us) >= sample_period_us)
    {
        g_last_sample_us = now_us;
        fsr_channel_sample(
            &g_fsr_channels[0],
            &g_adc,
            &g_baseline_config,
            g_fmg_band.movement_active);
        fmg_band_update(&g_fmg_band, &fmg_config, g_fsr_channels, KAMOD_FSR_CHANNEL_COUNT);
    }

    if ((now_us - g_last_ekf_us) >= ekf_period_us)
    {
        const uint32_t elapsed_us = now_us - g_last_ekf_us;
        g_last_ekf_us = now_us;

        imu_interface_sample_t imu_sample = {};
        imu_interface_poll(&g_imu, &imu_sample);

        ekf_sensor_fusion_predict(
            &g_ekf,
            static_cast<float>(elapsed_us) / 1000000.0f,
            imu_sample.gyro_valid,
            imu_sample.gyro_rad_s);
        ekf_sensor_fusion_update_imu(
            &g_ekf,
            imu_sample.angle_valid,
            imu_sample.angle_rad,
            imu_sample.gyro_valid,
            imu_sample.gyro_rad_s);
        ekf_sensor_fusion_update_fsr(
            &g_ekf,
            true,
            g_fmg_band.feature_mean);
    }

    if ((now_us - g_last_packet_us) >= packet_period_us)
    {
        g_last_packet_us = now_us;
        ekf_sensor_fusion_output_t ekf_output = {};
        canfd_sensor_packet_t packet = {};

        ekf_sensor_fusion_get_state(&g_ekf, &ekf_output);
        if (canfd_packets_build(&g_packets, &packet, now_ms, &g_fsr_channels[0], &ekf_output))
        {
            canfd_packets_transmit(&g_packets, &packet, now_ms);
        }
    }

    if ((now_us - g_last_log_us) >= log_period_us)
    {
        g_last_log_us = now_us;
        ekf_sensor_fusion_output_t ekf_output = {};
        ekf_sensor_fusion_get_state(&g_ekf, &ekf_output);
        sensor_logger_log_row(now_ms, KAMOD_NODE_ID, &g_fsr_channels[0], &ekf_output);
    }
}
