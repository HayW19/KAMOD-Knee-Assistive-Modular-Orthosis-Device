#ifndef SENSOR_LOGGER_H
#define SENSOR_LOGGER_H

#include <Arduino.h>

#include "canfd_packets.h"
#include "ekf_sensor_fusion.h"
#include "fsr_channel.h"

void sensor_logger_begin(uint32_t baud_rate);
void sensor_logger_log_text(const char *tag, uint32_t now_ms, const char *message);
void sensor_logger_log_header(void);
void sensor_logger_log_calibration(
    uint32_t now_ms,
    uint8_t channel_id,
    uint8_t digipot_tap,
    int32_t averaged_adc,
    bool calibrated);
void sensor_logger_log_row(
    uint32_t now_ms,
    kamod_node_id_t node_id,
    const fsr_channel_state_t *fsr_channel,
    const ekf_sensor_fusion_output_t *ekf_output);
void sensor_logger_log_packet(const canfd_sensor_packet_t *packet);

#endif /* SENSOR_LOGGER_H */
