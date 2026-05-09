#ifndef CANFD_PACKETS_H
#define CANFD_PACKETS_H

#include <Arduino.h>
#include <stddef.h>

#include "../../../common/types/kamod_ids.h"

#include "ekf_sensor_fusion.h"
#include "fsr_channel.h"

typedef struct __attribute__((packed))
{
    uint8_t packet_type;
    uint8_t node_id;
    uint32_t timestamp_ms;
    uint16_t sequence_counter;
    uint8_t fsr_channel_id;
    int32_t raw_adc_24bit;
    uint8_t digipot_tap;
    int32_t baseline_value;
    int32_t corrected_value;
    int32_t filtered_value;
    int32_t ekf_angle_mrad;
    int32_t ekf_velocity_mrad_s;
    uint16_t status_flags;
    uint8_t crc8;
} canfd_sensor_packet_t;

static const size_t KAMOD_CANFD_SENSOR_PACKET_SIZE = sizeof(canfd_sensor_packet_t);

typedef struct
{
    bool initialized;
    bool backend_ready;
    uint16_t sequence_counter;
    canfd_sensor_packet_t last_packet;
} canfd_packets_state_t;

bool canfd_packets_init(canfd_packets_state_t *state);
uint8_t canfd_packets_crc8(const uint8_t *data, size_t length);
bool canfd_packets_build(
    canfd_packets_state_t *state,
    canfd_sensor_packet_t *packet,
    uint32_t timestamp_ms,
    const fsr_channel_state_t *fsr_channel,
    const ekf_sensor_fusion_output_t *ekf_output);
bool canfd_packets_transmit(
    canfd_packets_state_t *state,
    const canfd_sensor_packet_t *packet,
    uint32_t now_ms);

#endif /* CANFD_PACKETS_H */
