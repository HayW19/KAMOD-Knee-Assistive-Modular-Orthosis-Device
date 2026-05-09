#include <Arduino.h>

#include "../include/actuator_can.h"
#include "../include/logger.h"
#include "../include/safety_state.h"

namespace
{
actuator_can_state_t g_actuator = {};
safety_state_t g_safety = {};
uint32_t g_start_ms = 0U;
bool g_enabled = false;
}

void setup()
{
    logger_begin(115200UL);
    actuator_can_init(&g_actuator);
    safety_state_init(&g_safety, millis());
    g_start_ms = millis();
    logger_log_text("warn", millis(), "bench test only");
}

void loop()
{
    safety_inputs_t inputs = {};
    safety_outputs_t outputs = {};
    inputs.now_ms = millis();

    if (!g_enabled)
    {
        inputs.enable_request = true;
        g_enabled = true;
    }

    if ((inputs.now_ms - g_start_ms) < 100U)
    {
        inputs.test_command_request = true;
        inputs.requested_command_q15_16 = 1024;
    }

    safety_state_step(&g_safety, &inputs, &outputs);

    if (outputs.send_enable)
    {
        actuator_can_send_enable(&g_actuator, inputs.now_ms);
    }

    if (outputs.command_valid)
    {
        actuator_can_send_command(&g_actuator, outputs.safe_command_q15_16, inputs.now_ms);
    }

    if (outputs.send_disable || actuator_can_command_timed_out(&g_actuator, inputs.now_ms, 500U))
    {
        actuator_can_send_disable(&g_actuator, inputs.now_ms);
        logger_log_text("test", inputs.now_ms, "safe disable hit");
    }
}
