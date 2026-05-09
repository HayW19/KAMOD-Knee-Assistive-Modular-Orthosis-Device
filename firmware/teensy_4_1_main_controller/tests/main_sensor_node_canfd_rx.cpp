#include <Arduino.h>

#include "../include/logger.h"
#include "../include/sensor_node_canfd.h"

namespace
{
sensor_node_canfd_state_t g_state = {};
bool g_loaded = false;
}

void setup()
{
    logger_begin(115200UL);
    sensor_node_canfd_init(&g_state);
    logger_log_text("warn", millis(), "bench test only");
}

void loop()
{
    if (!g_loaded)
    {
        sensor_node_canfd_wire_packet_t packet = {};
        packet.packet_type = 1U;
        packet.node_id = static_cast<uint8_t>(KAMOD_NODE_ID_SMART_NODE_1);
        packet.timestamp_ms = 1000U;
        packet.sequence_counter = 7U;
        packet.fsr_channel_id = 0U;
        packet.raw_adc_24bit = 1450000L;
        packet.digipot_tap = 42U;
        packet.baseline_value = 1400000L;
        packet.corrected_value = 60000L;
        packet.filtered_value = 1460000L;
        packet.ekf_angle_mrad = 220;
        packet.ekf_velocity_mrad_s = 750;
        packet.status_flags = 3U;
        packet.crc8 =
            sensor_node_canfd_crc8(reinterpret_cast<const uint8_t *>(&packet), KAMOD_SENSOR_NODE_CANFD_PACKET_SIZE - sizeof(packet.crc8));
        sensor_node_canfd_load_mock(
            &g_state,
            reinterpret_cast<const uint8_t *>(&packet),
            KAMOD_SENSOR_NODE_CANFD_PACKET_SIZE);
        g_loaded = true;
    }

    sensor_node_canfd_packet_t rx = {};
    if (sensor_node_canfd_poll(&g_state, &rx))
    {
        logger_log_sensor_packet(&rx, millis());
    }
}
