#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

#define FLAGS_POWER_ON_REQD          0x01
#define FLAGS_POWER_OFF_REQD         0x02
#define FLAGS_RESET_REQD             0x04
#define FLAGS_HELP_REQD              0x08
extern uint8_t reqd_action_flags;

#define STATE_SYSTEM_OFF             0x00
#define STATE_SYSTEM_ON              0x01
#define STATE_POWERING_ON            0x02
#define STATE_POWERING_OFF           0x03
extern uint8_t system_state;

const char* get_system_state_str(char* s);

bool mobo_hdd_active();

#define WAIT_STATUS_MSG_LEN           32
extern char wait_status_msg[WAIT_STATUS_MSG_LEN];

#endif // GLOBALS_H
