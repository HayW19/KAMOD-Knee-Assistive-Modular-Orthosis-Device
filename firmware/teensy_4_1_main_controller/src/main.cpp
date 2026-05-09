#include <Arduino.h>

#include "actuator_can.h"
#include "actuator_feedback.h"
#include "kamod_config.h"
#include "logger.h"
#include "safety_state.h"
#include "sensor_node_canfd.h"

namespace
{
actuator_can_state_t g_actuator = {};
actuator_feedback_state_t g_feedback = {};
sensor_node_canfd_state_t g_sensor_node = {};
safety_state_t g_safety = {};
uint32_t g_last_status_ms = 0U;
bool g_auto_enable_sent = false;
bool g_auto_disable_sent = false;

void run_bench_schedule(uint32_t now_ms, safety_inputs_t *inputs)
{
    if (inputs == nullptr)
    {
        return;
    }

    inputs->enable_request = false;
    inputs->test_command_request = false;
    inputs->disable_request = false;

    if (!KAMOD_ENABLE_STARTUP_BENCH_COMMAND)
    {
        return;
    }

    if ((now_ms >= KAMOD_AUTO_ENABLE_DELAY_MS) && !g_auto_enable_sent)
    {
        inputs->enable_request = true;
        g_auto_enable_sent = true;
    }

    if ((now_ms >= KAMOD_AUTO_COMMAND_START_MS) && (now_ms < KAMOD_AUTO_COMMAND_STOP_MS))
    {
        inputs->test_command_request = true;
        inputs->requested_command_q15_16 = KAMOD_TEST_COMMAND_Q15_16;
    }

    if ((now_ms >= KAMOD_AUTO_COMMAND_STOP_MS) && !g_auto_disable_sent)
    {
        inputs->disable_request = true;
        g_auto_disable_sent = true;
    }
}
}

void setup()
{
    logger_begin(KAMOD_SERIAL_BAUD);
    safety_state_init(&g_safety, millis());
    actuator_can_init(&g_actuator);
    actuator_feedback_init(&g_feedback);
    sensor_node_canfd_init(&g_sensor_node);

    logger_log_text("warn", millis(), "bench test only");
    logger_log_text("warn", millis(), "actuator output disabled on boot");
    if (g_actuator.mock_mode)
    {
        logger_log_text("warn", millis(), "actuator can mock backend active");
    }
    else
    {
        logger_log_text("warn", millis(), "actuator can backend not ready");
    }
    logger_log_text("warn", millis(), "sensor packet path is logging only");

    // todo: add emergency stop input
    // todo: add watchdog
    // todo: validate brake chopper
    // todo: validate regen clamp
    // todo: verify torque mode
    // todo: verify impedance control
    // todo: finish bench-rig test
    // todo: add human-in-the-loop lockout
}

void loop()
{
    const uint32_t now_ms = millis();
    safety_inputs_t safety_inputs = {};
    safety_outputs_t safety_outputs = {};
    actuator_feedback_frame_t feedback_frame = {};
    sensor_node_canfd_packet_t sensor_packet = {};

    safety_inputs.now_ms = now_ms;
    run_bench_schedule(now_ms, &safety_inputs);
    safety_state_step(&g_safety, &safety_inputs, &safety_outputs);

    if (safety_outputs.send_enable)
    {
        if (actuator_can_send_enable(&g_actuator, now_ms))
        {
            logger_log_text("warn", now_ms, "actuator enable path active");
        }
        else if (g_actuator.backend_not_ready)
        {
            logger_log_text("fault", now_ms, "actuator backend not ready");
        }
    }

    if (safety_outputs.command_valid)
    {
        const bool accepted = actuator_can_send_command(&g_actuator, safety_outputs.safe_command_q15_16, now_ms);
        logger_log_command(now_ms, &g_safety, safety_outputs.safe_command_q15_16, accepted);
        if (!accepted && g_actuator.backend_not_ready)
        {
            logger_log_text("fault", now_ms, "actuator backend not ready");
        }
    }

    if (safety_outputs.send_disable || actuator_can_command_timed_out(&g_actuator, now_ms, KAMOD_COMMAND_TIMEOUT_MS))
    {
        if (actuator_can_send_disable(&g_actuator, now_ms))
        {
            logger_log_text("warn", now_ms, "safe disable active");
        }
        else if (g_actuator.backend_not_ready)
        {
            logger_log_text("fault", now_ms, "actuator backend not ready");
        }
    }

    if (safety_outputs.unsafe_command_rejected)
    {
        logger_log_text("fault", now_ms, "reject unsafe command");
    }

    if (actuator_feedback_poll(&g_feedback, &feedback_frame))
    {
        logger_log_feedback(&feedback_frame);
    }

    if (sensor_node_canfd_poll(&g_sensor_node, &sensor_packet))
    {
        logger_log_sensor_packet(&sensor_packet, now_ms);
    }

    if ((now_ms - g_last_status_ms) >= KAMOD_STATUS_LOG_PERIOD_MS)
    {
        g_last_status_ms = now_ms;
        logger_log_status(&g_safety, now_ms);
    }
}
