#include <Arduino.h>

#include "../include/actuator_feedback.h"
#include "../include/logger.h"

namespace
{
actuator_feedback_state_t g_state = {};
bool g_loaded = false;
}

void setup()
{
    logger_begin(115200UL);
    actuator_feedback_init(&g_state);
    logger_log_text("warn", millis(), "bench test only");
}

void loop()
{
    if (!g_loaded)
    {
        actuator_feedback_frame_t frame = {};
        frame.timestamp_ms = millis();
        frame.frame_id = 0x181UL;
        frame.position_q15_16 = 1024;
        frame.velocity_q15_16 = 64;
        frame.torque_q15_16 = 32;
        frame.current_ma = 250;
        frame.temperature_c = 31;
        frame.status_flags = 1U;
        actuator_feedback_load_mock(&g_state, &frame);
        g_loaded = true;
    }

    actuator_feedback_frame_t rx = {};
    if (actuator_feedback_poll(&g_state, &rx))
    {
        logger_log_feedback(&rx);
    }
}
