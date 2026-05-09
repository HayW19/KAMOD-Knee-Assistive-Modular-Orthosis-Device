#include "actuator_can.h"

#include "kamod_config.h"

namespace
{
int32_t absolute_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}
}

bool actuator_can_init(actuator_can_state_t *state)
{
    if (state == nullptr)
    {
        return false;
    }

    state->initialized = true;
#if KAMOD_USE_MOCK_ACTUATOR_CAN
    state->mock_mode = true;
    state->backend_ready = true;
#else
    state->mock_mode = false;
    state->backend_ready = false;
#endif
    state->backend_not_ready = false;
    state->enabled = false;
    state->last_tx_ms = 0U;
    state->tx_count = 0U;
    state->last_command_q15_16 = 0;
    return true;
}

void actuator_can_reset(actuator_can_state_t *state)
{
    if (state == nullptr)
    {
        return;
    }

    state->enabled = false;
    state->backend_not_ready = false;
    state->last_tx_ms = 0U;
    state->last_command_q15_16 = 0;
}

bool actuator_can_send_enable(actuator_can_state_t *state, uint32_t now_ms)
{
    if ((state == nullptr) || !state->initialized)
    {
        return false;
    }

    if (!state->backend_ready)
    {
        state->backend_not_ready = true;
        return false;
    }

    state->enabled = true;
    state->backend_not_ready = false;
    state->last_tx_ms = now_ms;
    state->tx_count += 1U;
    return true;
}

bool actuator_can_send_disable(actuator_can_state_t *state, uint32_t now_ms)
{
    if ((state == nullptr) || !state->initialized)
    {
        return false;
    }

    if (!state->backend_ready)
    {
        state->backend_not_ready = true;
        return false;
    }

    state->enabled = false;
    state->backend_not_ready = false;
    state->last_command_q15_16 = 0;
    state->last_tx_ms = now_ms;
    state->tx_count += 1U;
    return true;
}

bool actuator_can_send_command(actuator_can_state_t *state, int32_t command_q15_16, uint32_t now_ms)
{
    if ((state == nullptr) || !state->initialized || !state->enabled)
    {
        return false;
    }

    if (!state->backend_ready)
    {
        state->backend_not_ready = true;
        return false;
    }

    state->backend_not_ready = false;
    state->last_command_q15_16 = actuator_can_clamp_command(command_q15_16);
    state->last_tx_ms = now_ms;
    state->tx_count += 1U;
    return true;
}

bool actuator_can_command_timed_out(const actuator_can_state_t *state, uint32_t now_ms, uint32_t timeout_ms)
{
    if ((state == nullptr) || !state->enabled)
    {
        return false;
    }

    return ((now_ms - state->last_tx_ms) > timeout_ms);
}

int32_t actuator_can_clamp_command(int32_t command_q15_16)
{
    if (absolute_i32(command_q15_16) > KAMOD_COMMAND_LIMIT_Q15_16)
    {
        if (command_q15_16 < 0)
        {
            return -KAMOD_COMMAND_LIMIT_Q15_16;
        }
        return KAMOD_COMMAND_LIMIT_Q15_16;
    }

    return command_q15_16;
}
