#include "sensor_logger.h"

#include <math.h>

void sensor_logger_begin(uint32_t baud_rate)
{
    Serial.begin(baud_rate);
}

void sensor_logger_log_text(const char *tag, uint32_t now_ms, const char *message)
{
    Serial.print(tag);
    Serial.print(",");
    Serial.print(now_ms);
    Serial.print(",");
    Serial.println(message);
}

void sensor_logger_log_header(void)
{
    Serial.println("sensor_time_ms,node_id,fsr_id,raw_adc,digipot_tap,baseline,filtered,corrected,ekf_angle_mrad,ekf_velocity_mrad_s,status");
}

void sensor_logger_log_calibration(
    uint32_t now_ms,
    uint8_t channel_id,
    uint8_t digipot_tap,
    int32_t averaged_adc,
    bool calibrated)
{
    Serial.print("calibration,");
    Serial.print(now_ms);
    Serial.print(",");
    Serial.print(channel_id);
    Serial.print(",");
    Serial.print(digipot_tap);
    Serial.print(",");
    Serial.print(averaged_adc);
    Serial.print(",");
    Serial.println(calibrated ? 1 : 0);
}

void sensor_logger_log_row(
    uint32_t now_ms,
    kamod_node_id_t node_id,
    const fsr_channel_state_t *fsr_channel,
    const ekf_sensor_fusion_output_t *ekf_output)
{
    if ((fsr_channel == nullptr) || (ekf_output == nullptr))
    {
        return;
    }

    Serial.print(now_ms);
    Serial.print(",");
    Serial.print(static_cast<uint32_t>(node_id));
    Serial.print(",");
    Serial.print(fsr_channel->channel_id);
    Serial.print(",");
    Serial.print(fsr_channel->raw_adc);
    Serial.print(",");
    Serial.print(fsr_channel->digipot_tap);
    Serial.print(",");
    Serial.print(static_cast<int32_t>(lroundf(fsr_channel->baseline)));
    Serial.print(",");
    Serial.print(static_cast<int32_t>(lroundf(fsr_channel->filtered_value)));
    Serial.print(",");
    Serial.print(static_cast<int32_t>(lroundf(fsr_channel->corrected_value)));
    Serial.print(",");
    Serial.print(static_cast<int32_t>(lroundf(ekf_output->knee_angle_rad * 1000.0f)));
    Serial.print(",");
    Serial.print(static_cast<int32_t>(lroundf(ekf_output->knee_angular_velocity_rad_s * 1000.0f)));
    Serial.print(",");
    Serial.println(fsr_channel->status_flags);
}

void sensor_logger_log_packet(const canfd_sensor_packet_t *packet)
{
    if (packet == nullptr)
    {
        return;
    }

    Serial.print("packet,");
    Serial.print(packet->timestamp_ms);
    Serial.print(",");
    Serial.print(static_cast<uint32_t>(packet->node_id));
    Serial.print(",");
    Serial.print(packet->sequence_counter);
    Serial.print(",");
    Serial.print(packet->fsr_channel_id);
    Serial.print(",");
    Serial.print(packet->raw_adc_24bit);
    Serial.print(",");
    Serial.print(packet->digipot_tap);
    Serial.print(",");
    Serial.print(packet->baseline_value);
    Serial.print(",");
    Serial.print(packet->filtered_value);
    Serial.print(",");
    Serial.print(packet->corrected_value);
    Serial.print(",");
    Serial.print(packet->ekf_angle_mrad);
    Serial.print(",");
    Serial.print(packet->ekf_velocity_mrad_s);
    Serial.print(",");
    Serial.print(packet->status_flags);
    Serial.print(",");
    Serial.println(packet->crc8);
}
