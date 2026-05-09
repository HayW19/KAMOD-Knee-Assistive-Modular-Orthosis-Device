#include <Arduino.h>

#include "../include/canfd_packets.h"
#include "../include/ekf_sensor_fusion.h"
#include "../include/fsr_channel.h"
#include "../include/sensor_logger.h"

namespace
{
canfd_packets_state_t g_packets = {};
fsr_channel_state_t g_channel = {};
ekf_sensor_fusion_output_t g_ekf_output = {};
bool g_sent = false;
}

void setup()
{
    sensor_logger_begin(115200UL);
    canfd_packets_init(&g_packets);
    g_channel.channel_id = 0;
    g_channel.raw_adc = 1450000;
    g_channel.digipot_tap = 140U;
    g_channel.baseline = 1400000.0f;
    g_channel.filtered_value = 1460000.0f;
    g_channel.corrected_value = 60000.0f;
    g_channel.status_flags = FSR_CHANNEL_STATUS_PRESENT | FSR_CHANNEL_STATUS_CALIBRATED;
    g_ekf_output.knee_angle_rad = 0.22f;
    g_ekf_output.knee_angular_velocity_rad_s = 0.75f;
    g_ekf_output.fsr_pressure_bias = 0.05f;
    sensor_logger_log_text("warn", millis(), "bench test only");
}

void loop()
{
    if (g_sent)
    {
        return;
    }

    canfd_sensor_packet_t packet = {};
    canfd_packets_build(&g_packets, &packet, millis(), &g_channel, &g_ekf_output);
    canfd_packets_transmit(&g_packets, &packet, millis());
    sensor_logger_log_packet(&packet);
    g_sent = true;
}
