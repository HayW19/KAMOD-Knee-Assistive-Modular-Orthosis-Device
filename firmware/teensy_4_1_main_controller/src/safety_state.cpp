#include "safety_state.h"

#include "kamod_config.h"

namespace
{
int32_t absolute_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

void clear_outputs(safety_outputs_t *outputs)
{
    outputs->send_enable = false;
    outputs->send_disable = false;
    outputs->command_valid = false;
    outputs->unsafe_command_rejected = false;
    outputs->safe_command_q15_16 = 0;
}
}

void safety_state_init(safety_state_t *state, uint32_t now_ms)
{
    if (state == nullptr)
    {
        return;
    }

    state->bench_state = KAMOD_BENCH_STATE_BOOT;
    state->system_mode = KAMOD_MODE_BOOT;
    state->actuator_enabled = false;
    state->fault_latched = false;
    state->timeout_active = false;
    state->state_enter_ms = now_ms;
    state->last_safe_command_ms = now_ms;
    state->last_safe_command_q15_16 = 0;
}

void safety_state_reset(safety_state_t *state, uint32_t now_ms)
{
    safety_state_init(state, now_ms);
}

void safety_state_step(safety_state_t *state, const safety_inputs_t *inputs, safety_outputs_t *outputs)
{
    if ((state == nullptr) || (inputs == nullptr) || (outputs == nullptr))
    {
        return;
    }

    clear_outputs(outputs);

    if (state->bench_state == KAMOD_BENCH_STATE_BOOT)
    {
        state->bench_state = KAMOD_BENCH_STATE_IDLE;
        state->system_mode = KAMOD_MODE_IDLE_SAFE;
        state->state_enter_ms = inputs->now_ms;
    }

    if (state->fault_latched)
    {
        state->bench_state = KAMOD_BENCH_STATE_FAULT;
        state->system_mode = KAMOD_MODE_FAULT_LATCHED;
        state->actuator_enabled = false;
        outputs->send_disable = true;
        return;
    }

    if (state->actuator_enabled && ((inputs->now_ms - state->last_safe_command_ms) > KAMOD_COMMAND_TIMEOUT_MS))
    {
        state->timeout_active = true;
        state->bench_state = KAMOD_BENCH_STATE_SAFE_DISABLE;
        state->system_mode = KAMOD_MODE_IDLE_SAFE;
        state->actuator_enabled = false;
        state->state_enter_ms = inputs->now_ms;
        outputs->send_disable = true;
        return;
    }

    if (inputs->fault_request)
    {
        state->fault_latched = true;
        state->bench_state = KAMOD_BENCH_STATE_FAULT;
        state->system_mode = KAMOD_MODE_FAULT_LATCHED;
        state->actuator_enabled = false;
        state->state_enter_ms = inputs->now_ms;
        outputs->send_disable = true;
        return;
    }

    if (inputs->disable_request)
    {
        state->bench_state = KAMOD_BENCH_STATE_SAFE_DISABLE;
        state->system_mode = KAMOD_MODE_IDLE_SAFE;
        state->actuator_enabled = false;
        state->state_enter_ms = inputs->now_ms;
        outputs->send_disable = true;
        return;
    }

    if ((state->bench_state == KAMOD_BENCH_STATE_SAFE_DISABLE) && !state->actuator_enabled)
    {
        state->bench_state = KAMOD_BENCH_STATE_IDLE;
        state->system_mode = KAMOD_MODE_IDLE_SAFE;
        state->state_enter_ms = inputs->now_ms;
    }

    if ((state->bench_state == KAMOD_BENCH_STATE_IDLE) && inputs->enable_request)
    {
        state->bench_state = KAMOD_BENCH_STATE_ACTUATOR_ENABLED;
        state->system_mode = KAMOD_MODE_ARMED;
        state->actuator_enabled = true;
        state->timeout_active = false;
        state->state_enter_ms = inputs->now_ms;
        state->last_safe_command_ms = inputs->now_ms;
        outputs->send_enable = true;
    }

    if (inputs->test_command_request)
    {
        if (!state->actuator_enabled)
        {
            state->fault_latched = true;
            state->bench_state = KAMOD_BENCH_STATE_FAULT;
            state->system_mode = KAMOD_MODE_FAULT_LATCHED;
            outputs->send_disable = true;
            return;
        }

        if (absolute_i32(inputs->requested_command_q15_16) > KAMOD_COMMAND_LIMIT_Q15_16)
        {
            state->fault_latched = true;
            state->bench_state = KAMOD_BENCH_STATE_FAULT;
            state->system_mode = KAMOD_MODE_FAULT_LATCHED;
            state->actuator_enabled = false;
            outputs->unsafe_command_rejected = true;
            outputs->send_disable = true;
            return;
        }

        state->bench_state = KAMOD_BENCH_STATE_TEST_COMMAND;
        state->system_mode = KAMOD_MODE_ACTIVE_ASSIST;
        state->last_safe_command_ms = inputs->now_ms;
        state->last_safe_command_q15_16 = inputs->requested_command_q15_16;
        outputs->command_valid = true;
        outputs->safe_command_q15_16 = inputs->requested_command_q15_16;
    }
}

const char *safety_state_name(kamod_bench_state_t state)
{
    switch (state)
    {
        case KAMOD_BENCH_STATE_BOOT:
            return "boot";
        case KAMOD_BENCH_STATE_IDLE:
            return "idle";
        case KAMOD_BENCH_STATE_ACTUATOR_ENABLED:
            return "actuator_enabled";
        case KAMOD_BENCH_STATE_TEST_COMMAND:
            return "test_command";
        case KAMOD_BENCH_STATE_FAULT:
            return "fault";
        case KAMOD_BENCH_STATE_SAFE_DISABLE:
            return "safe_disable";
        default:
            return "unknown";
    }
}

const char *safety_mode_name(kamod_system_mode_t mode)
{
    switch (mode)
    {
        case KAMOD_MODE_BOOT:
            return "BOOT";
        case KAMOD_MODE_SELF_TEST:
            return "SELF_TEST";
        case KAMOD_MODE_IDLE_SAFE:
            return "IDLE_SAFE";
        case KAMOD_MODE_CALIBRATION:
            return "CALIBRATION";
        case KAMOD_MODE_READY:
            return "READY";
        case KAMOD_MODE_ARMED:
            return "ARMED";
        case KAMOD_MODE_ACTIVE_ASSIST:
            return "ACTIVE_ASSIST";
        case KAMOD_MODE_DEGRADED:
            return "DEGRADED";
        case KAMOD_MODE_FAULT_LATCHED:
            return "FAULT_LATCHED";
        default:
            return "UNKNOWN";
    }
}
