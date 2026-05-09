#include "actuator_feedback.h"

bool actuator_feedback_init(actuator_feedback_state_t *state)
{
    if (state == nullptr)
    {
        return false;
    }

    state->initialized = true;
    state->mock_ready = false;
    state->rx_count = 0U;
    state->pending_frame = {};
    return true;
}

bool actuator_feedback_poll(actuator_feedback_state_t *state, actuator_feedback_frame_t *frame_out)
{
    if ((state == nullptr) || (frame_out == nullptr) || !state->initialized || !state->mock_ready)
    {
        return false;
    }

    *frame_out = state->pending_frame;
    state->mock_ready = false;
    state->rx_count += 1U;
    return true;
}

void actuator_feedback_load_mock(actuator_feedback_state_t *state, const actuator_feedback_frame_t *frame)
{
    if ((state == nullptr) || (frame == nullptr))
    {
        return;
    }

    state->pending_frame = *frame;
    state->mock_ready = true;
}
