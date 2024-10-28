#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "protocol_examples_utils.h"
#include "esp_tls_crypto.h"
#include <esp_http_server.h>
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "freertos/timers.h"

#include "globals.h"
#include "gpios.h"

char wait_status_msg[WAIT_STATUS_MSG_LEN] = {0};

const char* get_system_state_str(char* s) {
    switch(system_state) {
        case STATE_SYSTEM_ON:
            sprintf(s, "STATE_SYSTEM_ON");
            break;
        case STATE_SYSTEM_OFF:
            sprintf(s, "STATE_SYSTEM_OFF");
            break;
        case STATE_POWERING_ON:
            sprintf(s, "STATE_POWERING_ON");
            break;
        case STATE_POWERING_OFF:
            sprintf(s, "STATE_POWERING_OFF");
            break;
        default:
            sprintf(s, "UNKNOWN_STATE");
    }
    return s;
}

bool mobo_hdd_active() {
    return gpio_get_level(GPIO_INPUT_HDD_ACTIVE);
}