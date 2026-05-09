#include "sensor_node_canfd.h"

#include <string.h>

bool sensor_node_canfd_init(sensor_node_canfd_state_t *state)
{
    if (state == nullptr)
    {
        return false;
    }

    state->initialized = true;
    state->backend_ready = false;
    state->mock_ready = false;
    state->rx_count = 0U;
    state->size_error_count = 0U;
    state->crc_error_count = 0U;
    state->sequence_jump_count = 0U;
    state->last_sequence_valid = false;
    state->last_sequence_counter = 0U;
    state->pending_size = 0U;
    memset(state->pending_bytes, 0, sizeof(state->pending_bytes));
    return true;
}

uint8_t sensor_node_canfd_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0x00U;

    if (data == nullptr)
    {
        return crc;
    }

    for (size_t index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = static_cast<uint8_t>((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

bool sensor_node_canfd_poll(sensor_node_canfd_state_t *state, sensor_node_canfd_packet_t *packet_out)
{
    if ((state == nullptr) || (packet_out == nullptr) || !state->initialized || !state->mock_ready)
    {
        return false;
    }

    packet_out->crc_ok = false;
    packet_out->sequence_jump = false;

    if (state->pending_size != KAMOD_SENSOR_NODE_CANFD_PACKET_SIZE)
    {
        state->size_error_count += 1U;
        state->mock_ready = false;
        state->pending_size = 0U;
        return false;
    }

    memcpy(&packet_out->fields, state->pending_bytes, KAMOD_SENSOR_NODE_CANFD_PACKET_SIZE);
    state->mock_ready = false;
    state->pending_size = 0U;
    state->rx_count += 1U;

    packet_out->crc_ok =
        (packet_out->fields.crc8 ==
         sensor_node_canfd_crc8(state->pending_bytes, KAMOD_SENSOR_NODE_CANFD_PACKET_SIZE - sizeof(packet_out->fields.crc8)));

    if (!packet_out->crc_ok)
    {
        state->crc_error_count += 1U;
        return true;
    }

    if (state->last_sequence_valid)
    {
        const uint16_t expected_sequence = static_cast<uint16_t>(state->last_sequence_counter + 1U);
        packet_out->sequence_jump = (packet_out->fields.sequence_counter != expected_sequence);
        if (packet_out->sequence_jump)
        {
            state->sequence_jump_count += 1U;
        }
    }

    state->last_sequence_valid = true;
    state->last_sequence_counter = packet_out->fields.sequence_counter;
    return true;
}

bool sensor_node_canfd_load_mock(sensor_node_canfd_state_t *state, const uint8_t *packet_bytes, size_t packet_size)
{
    if ((state == nullptr) || (packet_bytes == nullptr))
    {
        return false;
    }

    if (packet_size > sizeof(state->pending_bytes))
    {
        state->pending_size = packet_size;
        state->mock_ready = true;
        return true;
    }

    memcpy(state->pending_bytes, packet_bytes, packet_size);
    state->pending_size = packet_size;
    state->mock_ready = true;
    return true;
}
