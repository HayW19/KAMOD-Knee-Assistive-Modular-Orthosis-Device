#ifndef SAFETY_STATE_H
#define SAFETY_STATE_H

#include <Arduino.h>
#include "../../../common/state_machine/kamod_modes.h"
#include "kamod_config.h"

typedef struct
{
    bool enable_request;
    bool test_command_request;
    bool disable_request;
    bool fault_request;
    uint32_t now_ms;
    int32_t requested_command_q15_16;
} safety_inputs_t;

typedef struct
{
    kamod_bench_state_t bench_state;
    kamod_system_mode_t system_mode;
    bool actuator_enabled;
    bool fault_latched;
    bool timeout_active;
    uint32_t state_enter_ms;
    uint32_t last_safe_command_ms;
    int32_t last_safe_command_q15_16;
} safety_state_t;

typedef struct
{
    bool send_enable;
    bool send_disable;
    bool command_valid;
    bool unsafe_command_rejected;
    int32_t safe_command_q15_16;
} safety_outputs_t;

void safety_state_init(safety_state_t *state, uint32_t now_ms);
void safety_state_reset(safety_state_t *state, uint32_t now_ms);
void safety_state_step(safety_state_t *state, const safety_inputs_t *inputs, safety_outputs_t *outputs);
const char *safety_state_name(kamod_bench_state_t state);
const char *safety_mode_name(kamod_system_mode_t mode);

#endif /* SAFETY_STATE_H */
