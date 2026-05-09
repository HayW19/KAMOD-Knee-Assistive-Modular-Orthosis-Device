#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "actuator_feedback.h"
#include "safety_state.h"
#include "sensor_node_canfd.h"

void logger_begin(uint32_t baud_rate);
void logger_log_text(const char *tag, uint32_t now_ms, const char *message);
void logger_log_status(const safety_state_t *state, uint32_t now_ms);
void logger_log_command(uint32_t now_ms, const safety_state_t *state, int32_t command_q15_16, bool accepted);
void logger_log_feedback(const actuator_feedback_frame_t *frame);
void logger_log_sensor_packet(const sensor_node_canfd_packet_t *packet, uint32_t controller_time_ms);

#endif /* LOGGER_H */
