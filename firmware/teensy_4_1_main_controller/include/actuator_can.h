#ifndef ACTUATOR_CAN_H
#define ACTUATOR_CAN_H

#include <Arduino.h>

typedef struct
{
    bool initialized;
    bool mock_mode;
    bool backend_ready;
    bool backend_not_ready;
    bool enabled;
    uint32_t last_tx_ms;
    uint32_t tx_count;
    int32_t last_command_q15_16;
} actuator_can_state_t;

typedef struct
{
    bool enable;
    int32_t command_q15_16;
    uint32_t request_ms;
} actuator_command_request_t;

bool actuator_can_init(actuator_can_state_t *state);
void actuator_can_reset(actuator_can_state_t *state);
bool actuator_can_send_enable(actuator_can_state_t *state, uint32_t now_ms);
bool actuator_can_send_disable(actuator_can_state_t *state, uint32_t now_ms);
bool actuator_can_send_command(actuator_can_state_t *state, int32_t command_q15_16, uint32_t now_ms);
bool actuator_can_command_timed_out(const actuator_can_state_t *state, uint32_t now_ms, uint32_t timeout_ms);
int32_t actuator_can_clamp_command(int32_t command_q15_16);

#endif /* ACTUATOR_CAN_H */
