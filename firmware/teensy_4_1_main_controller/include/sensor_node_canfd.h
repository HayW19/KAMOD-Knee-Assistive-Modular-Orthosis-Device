#ifndef SENSOR_NODE_CANFD_H
#define SENSOR_NODE_CANFD_H

#include <Arduino.h>

#include <stddef.h>
#include "../../../common/types/kamod_ids.h"

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
} sensor_node_canfd_wire_packet_t;

static const size_t KAMOD_SENSOR_NODE_CANFD_PACKET_SIZE = sizeof(sensor_node_canfd_wire_packet_t);

typedef struct
{
    sensor_node_canfd_wire_packet_t fields;
    bool crc_ok;
    bool sequence_jump;
} sensor_node_canfd_packet_t;

typedef struct
{
    bool initialized;
    bool backend_ready;
    bool mock_ready;
    uint32_t rx_count;
    uint32_t size_error_count;
    uint32_t crc_error_count;
    uint32_t sequence_jump_count;
    bool last_sequence_valid;
    uint16_t last_sequence_counter;
    size_t pending_size;
    uint8_t pending_bytes[KAMOD_SENSOR_NODE_CANFD_PACKET_SIZE];
} sensor_node_canfd_state_t;

bool sensor_node_canfd_init(sensor_node_canfd_state_t *state);
uint8_t sensor_node_canfd_crc8(const uint8_t *data, size_t length);
bool sensor_node_canfd_poll(sensor_node_canfd_state_t *state, sensor_node_canfd_packet_t *packet_out);
bool sensor_node_canfd_load_mock(sensor_node_canfd_state_t *state, const uint8_t *packet_bytes, size_t packet_size);

#endif /* SENSOR_NODE_CANFD_H */
