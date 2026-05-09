#include <Arduino.h>

#include "../include/actuator_can.h"
#include "../include/logger.h"

namespace
{
actuator_can_state_t g_state = {};
bool g_ran = false;
}

void setup()
{
    logger_begin(115200UL);
    actuator_can_init(&g_state);
    logger_log_text("warn", millis(), "bench test only");
}

void loop()
{
    if (g_ran)
    {
        return;
    }

    g_ran = true;
    actuator_can_send_enable(&g_state, millis());
    actuator_can_send_command(&g_state, 2048, millis());
    actuator_can_send_disable(&g_state, millis());
    logger_log_text("test", millis(), "enable command disable");
}
