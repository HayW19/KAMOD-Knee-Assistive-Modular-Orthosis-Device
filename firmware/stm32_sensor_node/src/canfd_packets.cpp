#include "canfd_packets.h"

#include <math.h>
#include <string.h>

#include "kamod_sensor_config.h"

bool canfd_packets_init(canfd_packets_state_t *state)
{
    if (state == nullptr)
    {
        return false;
    }

    state->initialized = true;
#if KAMOD_USE_MOCK_CANFD
    state->backend_ready = true;
#else
    state->backend_ready = false;
#endif
    state->sequence_counter = 0U;
    memset(&state->last_packet, 0, sizeof(state->last_packet));
    return true;
}

uint8_t canfd_packets_crc8(const uint8_t *data, size_t length)
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

bool canfd_packets_build(
    canfd_packets_state_t *state,
    canfd_sensor_packet_t *packet,
    uint32_t timestamp_ms,
    const fsr_channel_state_t *fsr_channel,
    const ekf_sensor_fusion_output_t *ekf_output)
{
    if ((state == nullptr) || (packet == nullptr) || (fsr_channel == nullptr) || (ekf_output == nullptr) || !state->initialized)
    {
        return false;
    }

    packet->packet_type = KAMOD_PACKET_TYPE_SENSOR_NODE;
    packet->node_id = static_cast<uint8_t>(KAMOD_NODE_ID);
    packet->timestamp_ms = timestamp_ms;
    packet->sequence_counter = state->sequence_counter++;
    packet->fsr_channel_id = static_cast<uint8_t>(fsr_channel->channel_id);
    packet->raw_adc_24bit = fsr_channel->raw_adc;
    packet->digipot_tap = fsr_channel->digipot_tap;
    packet->baseline_value = static_cast<int32_t>(lroundf(fsr_channel->baseline));
    packet->corrected_value = static_cast<int32_t>(lroundf(fsr_channel->corrected_value));
    packet->filtered_value = static_cast<int32_t>(lroundf(fsr_channel->filtered_value));
    packet->ekf_angle_mrad = static_cast<int32_t>(lroundf(ekf_output->knee_angle_rad * 1000.0f));
    packet->ekf_velocity_mrad_s = static_cast<int32_t>(lroundf(ekf_output->knee_angular_velocity_rad_s * 1000.0f));
    packet->status_flags = fsr_channel->status_flags;
    packet->crc8 = 0U;
    packet->crc8 = canfd_packets_crc8(reinterpret_cast<const uint8_t *>(packet), sizeof(canfd_sensor_packet_t) - sizeof(packet->crc8));
    state->last_packet = *packet;
    return true;
}

bool canfd_packets_transmit(
    canfd_packets_state_t *state,
    const canfd_sensor_packet_t *packet,
    uint32_t now_ms)
{
    (void) now_ms;

    if ((state == nullptr) || (packet == nullptr) || !state->initialized)
    {
        return false;
    }

    state->last_packet = *packet;
    return state->backend_ready;
}
