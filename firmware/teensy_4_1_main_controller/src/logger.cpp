#include "logger.h"

void logger_begin(uint32_t baud_rate)
{
    Serial.begin(baud_rate);
}

void logger_log_text(const char *tag, uint32_t now_ms, const char *message)
{
    Serial.print(tag);
    Serial.print(",");
    Serial.print(now_ms);
    Serial.print(",");
    Serial.println(message);
}

void logger_log_status(const safety_state_t *state, uint32_t now_ms)
{
    if (state == nullptr)
    {
        return;
    }

    Serial.print("status,");
    Serial.print(now_ms);
    Serial.print(",");
    Serial.print(safety_state_name(state->bench_state));
    Serial.print(",");
    Serial.print(safety_mode_name(state->system_mode));
    Serial.print(",");
    Serial.print(state->actuator_enabled ? 1 : 0);
    Serial.print(",");
    Serial.print(state->fault_latched ? 1 : 0);
    Serial.print(",");
    Serial.println(state->timeout_active ? 1 : 0);
}

void logger_log_command(uint32_t now_ms, const safety_state_t *state, int32_t command_q15_16, bool accepted)
{
    if (state == nullptr)
    {
        return;
    }

    Serial.print("command,");
    Serial.print(now_ms);
    Serial.print(",");
    Serial.print(safety_state_name(state->bench_state));
    Serial.print(",");
    Serial.print(state->actuator_enabled ? 1 : 0);
    Serial.print(",");
    Serial.print(accepted ? 1 : 0);
    Serial.print(",");
    Serial.println(command_q15_16);
}

void logger_log_feedback(const actuator_feedback_frame_t *frame)
{
    if (frame == nullptr)
    {
        return;
    }

    Serial.print("feedback,");
    Serial.print(frame->timestamp_ms);
    Serial.print(",");
    Serial.print(frame->frame_id, HEX);
    Serial.print(",");
    Serial.print(frame->position_q15_16);
    Serial.print(",");
    Serial.print(frame->velocity_q15_16);
    Serial.print(",");
    Serial.print(frame->torque_q15_16);
    Serial.print(",");
    Serial.print(frame->current_ma);
    Serial.print(",");
    Serial.print(frame->temperature_c);
    Serial.print(",");
    Serial.println(frame->status_flags);
}

void logger_log_sensor_packet(const sensor_node_canfd_packet_t *packet, uint32_t controller_time_ms)
{
    if (packet == nullptr)
    {
        return;
    }

    Serial.print("sensor,");
    Serial.print(controller_time_ms);
    Serial.print(",");
    Serial.print(packet->fields.timestamp_ms);
    Serial.print(",");
    Serial.print(static_cast<uint32_t>(packet->fields.node_id));
    Serial.print(",");
    Serial.print(packet->fields.sequence_counter);
    Serial.print(",");
    Serial.print(static_cast<uint32_t>(packet->fields.fsr_channel_id));
    Serial.print(",");
    Serial.print(packet->fields.raw_adc_24bit);
    Serial.print(",");
    Serial.print(static_cast<uint32_t>(packet->fields.digipot_tap));
    Serial.print(",");
    Serial.print(packet->fields.baseline_value);
    Serial.print(",");
    Serial.print(packet->fields.filtered_value);
    Serial.print(",");
    Serial.print(packet->fields.corrected_value);
    Serial.print(",");
    Serial.print(packet->fields.ekf_angle_mrad);
    Serial.print(",");
    Serial.print(packet->fields.ekf_velocity_mrad_s);
    Serial.print(",");
    Serial.print(static_cast<uint32_t>(packet->fields.status_flags));
    Serial.print(",");
    Serial.print(packet->crc_ok ? 1 : 0);
    Serial.print(",");
    Serial.println(packet->sequence_jump ? 1 : 0);
}
