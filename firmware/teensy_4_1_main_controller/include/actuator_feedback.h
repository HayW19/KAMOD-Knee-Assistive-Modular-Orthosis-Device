#ifndef ACTUATOR_FEEDBACK_H
#define ACTUATOR_FEEDBACK_H

#include <Arduino.h>

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t frame_id;
    int32_t position_q15_16;
    int32_t velocity_q15_16;
    int32_t torque_q15_16;
    int16_t current_ma;
    int16_t temperature_c;
    uint16_t status_flags;
} actuator_feedback_frame_t;

typedef struct
{
    bool initialized;
    bool mock_ready;
    uint32_t rx_count;
    actuator_feedback_frame_t pending_frame;
} actuator_feedback_state_t;

bool actuator_feedback_init(actuator_feedback_state_t *state);
bool actuator_feedback_poll(actuator_feedback_state_t *state, actuator_feedback_frame_t *frame_out);
void actuator_feedback_load_mock(actuator_feedback_state_t *state, const actuator_feedback_frame_t *frame);

#endif /* ACTUATOR_FEEDBACK_H */
