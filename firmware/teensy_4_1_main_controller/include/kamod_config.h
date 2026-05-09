#ifndef KAMOD_CONFIG_H
#define KAMOD_CONFIG_H

#include <Arduino.h>
#include "../../../common/state_machine/kamod_modes.h"
#include "../../../common/types/kamod_ids.h"

static const uint32_t KAMOD_SERIAL_BAUD = 115200UL;
static const uint32_t KAMOD_STATUS_LOG_PERIOD_MS = 500UL;
static const uint32_t KAMOD_COMMAND_TIMEOUT_MS = 500UL;
static const uint32_t KAMOD_AUTO_ENABLE_DELAY_MS = 2000UL;
static const uint32_t KAMOD_AUTO_COMMAND_START_MS = 2200UL;
static const uint32_t KAMOD_AUTO_COMMAND_STOP_MS = 2600UL;

static const uint32_t KAMOD_ACTUATOR_BUS_BITRATE = 1000000UL;
static const uint32_t KAMOD_SENSOR_NODE_BUS_BITRATE = 2000000UL;

static const uint32_t KAMOD_ACTUATOR_COMMAND_CAN_ID = 0x201UL;
static const uint32_t KAMOD_ACTUATOR_FEEDBACK_CAN_ID = 0x181UL;
static const uint32_t KAMOD_SENSOR_NODE_CANFD_ID = 0x301UL;

static const int32_t KAMOD_COMMAND_LIMIT_Q15_16 = 6553;
static const int32_t KAMOD_TEST_COMMAND_Q15_16 = 3276;

#ifndef KAMOD_USE_MOCK_ACTUATOR_CAN
#define KAMOD_USE_MOCK_ACTUATOR_CAN 1
#endif

static const bool KAMOD_ENABLE_STARTUP_BENCH_COMMAND = false;

typedef enum
{
    KAMOD_BENCH_STATE_BOOT = 0,
    KAMOD_BENCH_STATE_IDLE = 1,
    KAMOD_BENCH_STATE_ACTUATOR_ENABLED = 2,
    KAMOD_BENCH_STATE_TEST_COMMAND = 3,
    KAMOD_BENCH_STATE_FAULT = 4,
    KAMOD_BENCH_STATE_SAFE_DISABLE = 5
} kamod_bench_state_t;

#endif /* KAMOD_CONFIG_H */
