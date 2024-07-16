/* Simple HTTP Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

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
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "freertos/timers.h"

#if !CONFIG_IDF_TARGET_LINUX
#include <esp_wifi.h>
#include <esp_system.h>
#include "nvs_flash.h"
#include "esp_eth.h"
#endif  // !CONFIG_IDF_TARGET_LINUX

///////////////////////////////////////////////////////////////////////////////
///
/// System config specific #define's
///
///////////////////////////////////////////////////////////////////////////////

// definte if pressing the power button requires HID kybd input to confirm logout
// ref:  https://askubuntu.com/a/1272316
//#define SYSTEM_PRESENTS_LOGOUT_DIALOG
//#define SYSTEM_PRESENTS_LOGIN_DIALOG


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///
///  GPIOs
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//
// Output pins
//

// Signals to Teensy
// Default:  pin 21 sends user/password HID keyboard request (Teensy B0)
//           pin 22 sends arrow/enter HID keyboard request (Teensy B1)
#define GPIO_OUTPUT_TEENSY_B0         CONFIG_GPIO_OUTPUT_TEENSY_B0
#define GPIO_OUTPUT_TEENSY_B1         CONFIG_GPIO_OUTPUT_TEENSY_B1
#define GPIO_OUTPUT_PIN_SEL_TEENSY    ((1ULL<<GPIO_OUTPUT_TEENSY_B0) | (1ULL<<GPIO_OUTPUT_TEENSY_B1))

// Signals to Motherboard
// Defualts:  pin 26 output active low => power on/off
//            pin 27 output active low => reset
#define GPIO_OUTPUT_MOBO_POWER       CONFIG_GPIO_OUTPUT_MOBO_POWER
#define GPIO_OUTPUT_MOBO_RESET       CONFIG_GPIO_OUTPUT_MOBO_RESET
#define GPIO_OUTPUT_PIN_SEL_MOBO     ((1ULL<<GPIO_OUTPUT_MOBO_POWER) | (1ULL<<GPIO_OUTPUT_MOBO_RESET))

// Signals to status LEDs:
//   RED = mobo reset (pin 0)
//   GREEN = system ready (pin 2)
//   BLUE = mobo power off/on (pin 4)
#define GPIO_OUTPUT_RED               CONFIG_GPIO_OUTPUT_RGB_RED
#define GPIO_OUTPUT_GREEN             CONFIG_GPIO_OUTPUT_RGB_GREEN
#define GPIO_OUTPUT_BLUE              CONFIG_GPIO_OUTPUT_RGB_BLUE
#define GPIO_OUTPUT_PIN_SEL_LEDS      ((1ULL<<GPIO_OUTPUT_RED) | (1ULL<<GPIO_OUTPUT_GREEN) | (1ULL<<GPIO_OUTPUT_BLUE))

//
// Input pins
//

// Input read off of motherboard
// Default:  pin 32 input active high => HDD activity on/off
//           (HDD activity is used to detect if the system is running)
#define GPIO_INPUT_HDD_ACTIVE                   CONFIG_GPIO_INPUT_HDD_ACTIVE
#define GPIO_INPUT_SENSING_HDD_ACTIVE_PIN_SEL   (1ULL<<GPIO_INPUT_HDD_ACTIVE)

// Buttons for manual triggering of Teensy
// Default:  pin 18 - left button (power cycle)
//           pin 19 - right button (reset)
#define GPIO_INPUT_TEST_BTN_LEFT     CONFIG_TEST_GPIO_INPUT_BTN_LEFT
#define GPIO_INPUT_TEST_BTN_RIGHT    CONFIG_TEST_GPIO_INPUT_BTN_RIGHT
#define GPIO_INPUT_PIN_SEL           ((1ULL<<GPIO_INPUT_TEST_BTN_LEFT) | (1ULL<<GPIO_INPUT_TEST_BTN_RIGHT))

#define ESP_INTR_FLAG_DEFAULT 0

// static QueueHandle_t gpio_evt_queue = NULL;

#define FLAGS_POWER_ON_REQD          0x01
#define FLAGS_POWER_OFF_REQD         0x02
#define FLAGS_RESET_REQD             0x04
#define FLAGS_HELP_REQD              0x08
static uint8_t reqd_action_flags = 0b00000000;

#define STATE_SYSTEM_OFF             0x00
#define STATE_SYSTEM_ON              0x01
#define STATE_POWERING_ON            0x02
#define STATE_POWERING_OFF           0x03
static uint8_t system_state = STATE_SYSTEM_OFF;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    // xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

    if (gpio_num==GPIO_INPUT_TEST_BTN_LEFT) {
        reqd_action_flags |= FLAGS_POWER_ON_REQD;
    } else if (gpio_num==GPIO_INPUT_TEST_BTN_RIGHT) {
        reqd_action_flags |= FLAGS_POWER_OFF_REQD;
    }

}

// static void gpio_task_example(void* arg)
// {
//     uint32_t io_num;
//     for(;;) {
//         if(xQueueReceive(gpio_evt_queue, (void*)&io_num, portMAX_DELAY)) {
//             printf("ISR> GPIO[%"PRIu32"] interrupted, val: %d\n", io_num, gpio_get_level(io_num));
//         }
// 
//         if (io_num==GPIO_INPUT_TEST_BTN_LEFT) {
//             btn_presses |= BTN_LEFT_PRESS_MASK;
//         } else if (io_num==GPIO_INPUT_TEST_BTN_RIGHT) {
//             btn_presses |= BTN_RIGHT_PRESS_MASK;
//         }
//     }
// }

#define POWER_ON_KEYSTROKE_WAIT_SEC       30 // delay until system is ready for user/pass
#define POWER_OFF_PRE_KEYSTROKE_WAIT_SEC  5  // seconds to wait before sending logout keystrokes
#define POWER_OFF_MAX_SEC_WAIT_HDD_ACTIVE 30 // max seconds to wait for HDD activity to go low
#define RESET_KEYSTROKE_WAIT_SEC          62 // delay for bootup after a reset
char wait_status_msg[32] = {0};

///////////////////////////////////////////////////////////////////////////////
///
/// Actions
///
///////////////////////////////////////////////////////////////////////////////

bool mobo_hdd_active() {
    return gpio_get_level(GPIO_INPUT_HDD_ACTIVE);
}

void update_system_state() {
    if (system_state != STATE_POWERING_OFF && system_state != STATE_POWERING_ON) {
        system_state = mobo_hdd_active() ? STATE_SYSTEM_ON : STATE_SYSTEM_OFF;
    }
}

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

void do_power_on() {
    char system_state_str[64] = {0};

    // bail if we're already in a "powering" state
    if (system_state==STATE_POWERING_ON || system_state==STATE_POWERING_OFF) {
        printf("do_power_on() [%s]> ERR: should not reach here b/c we're already powering on or off\n", get_system_state_str(system_state_str));
        vTaskDelete(NULL);
        return;
    }

    // update to powering on state since that's what we're now doing
    printf("do_power_on() [%s]> going to state STATE_POWERING_ON\n", get_system_state_str(system_state_str));
    system_state = STATE_POWERING_ON;
    printf("do_power_off() [%s]> state updated\n", get_system_state_str(system_state_str));

    // notify power on starting
    printf("do_power_on() [%s]> powering on...\n", get_system_state_str(system_state_str));

    // Do power on; after POWER_ON_KEYSTROKE_WAIT_SEC seconds, 
    // send login keystrokes with teensy D0
    gpio_set_level(GPIO_OUTPUT_MOBO_POWER, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_OUTPUT_MOBO_POWER, 1);

#ifdef SYSTEM_PRESENTS_LOGIN_DIALOG
    for (int i=0; i<=POWER_ON_KEYSTROKE_WAIT_SEC; i++) {
        sprintf(wait_status_msg, "(%d of %d sec elapsed)", i, POWER_ON_KEYSTROKE_WAIT_SEC);
        if (!(i%10)) printf("do_power_on() [%s]> delay before sending user/pass %s\n", get_system_state_str(system_state_str), wait_status_msg);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    gpio_set_level(GPIO_OUTPUT_TEENSY_B0, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_OUTPUT_TEENSY_B0, 1);
#endif

    printf("do_power_on() [%s]> power on complete (system state=%d)\n", get_system_state_str(system_state_str), gpio_get_level(GPIO_INPUT_HDD_ACTIVE));
    reqd_action_flags = 0;

    // change state to SYSTEM_ON and notify
    if (mobo_hdd_active()) {
        printf("do_power_on() [%s]> going to state STATE_SYSTEM_ON\n", get_system_state_str(system_state_str));
        system_state = STATE_SYSTEM_ON;
        printf("do_power_on() [%s]> state updated\n", get_system_state_str(system_state_str));
    } else {
        system_state = STATE_SYSTEM_OFF;
        printf("do_power_on() [%s]> ERR: system is off after power on\n", get_system_state_str(system_state_str));
    }

    vTaskDelete(NULL);
}

void do_power_off() {
    char system_state_str[64] = {0};

    // bail if we're already in a "powering" state
    if (system_state==STATE_POWERING_ON || system_state==STATE_POWERING_OFF) {
        printf("do_power_off() [%s]> ERR: should not reach here b/c we're already powering on or off\n", get_system_state_str(system_state_str));
        vTaskDelete(NULL);
        return;
    }

    // update to powering off state since that's what we're now doing
    printf("do_power_off() [%s]> going to state STATE_POWERING_OFF\n", get_system_state_str(system_state_str));
    system_state = STATE_POWERING_OFF;
    printf("do_power_off() [%s]> state updated\n", get_system_state_str(system_state_str));

    // notify powering off starting
    printf("do_power_off() [%s]> powering off...\n", get_system_state_str(system_state_str));

    // - Trigger power cycle pin on mobo
    // - wait 5 seconds
    // - send logout keystrokes
    // - wait max(fixed delay, hdd activity pin goes low (meaning system is off))
    gpio_set_level(GPIO_OUTPUT_MOBO_POWER, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_OUTPUT_MOBO_POWER, 1);
    
    // the obligatory gnome logout dialog can be disabled on 22.04 like this:
    //   `gsettings set org.gnome.SessionManager logout-prompt false`
    // (see readme)
#ifdef SYSTEM_PRESENTS_LOGOUT_DIALOG

    for (int i=0; i<=POWER_OFF_PRE_KEYSTROKE_WAIT_SEC; i++) {
        sprintf(wait_status_msg, "(%d of %d sec elapsed)", i, POWER_OFF_PRE_KEYSTROKE_WAIT_SEC);
        if (!(i%2)) printf("do_power_off() [%s]> delay before sending logout keystrokes %s\n", get_system_state_str(system_state_str), wait_status_msg);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    gpio_set_level(GPIO_OUTPUT_TEENSY_B1, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_OUTPUT_TEENSY_B1, 1);
#endif

    for (int i=0; i<=POWER_OFF_MAX_SEC_WAIT_HDD_ACTIVE; i++) {
        if (gpio_get_level(GPIO_INPUT_HDD_ACTIVE)==0) {
            printf("do_power_off() [%s]> power off complete\n", get_system_state_str(system_state_str));
            break;
        }
        sprintf(wait_status_msg, "(%d of %d max wait sec elapsed)", i, POWER_OFF_MAX_SEC_WAIT_HDD_ACTIVE);
        if (!(i%2)) printf("do_power_off() [%s]> waiting for board to power down %s\n", get_system_state_str(system_state_str), wait_status_msg);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    printf("do_power_off() [%s]> power state: %d\n", get_system_state_str(system_state_str), gpio_get_level(GPIO_INPUT_HDD_ACTIVE));
    reqd_action_flags = 0;

    // change state to SYSTEM_OFF and notify
    if (!mobo_hdd_active()) {
        printf("do_power_off() [%s]> going to state STATE_SYSTEM_OFF\n", get_system_state_str(system_state_str));
        system_state = STATE_SYSTEM_OFF;
        printf("do_power_off() [%s]> state updated\n", get_system_state_str(system_state_str));
    } else {
        system_state = STATE_SYSTEM_ON;
        printf("do_power_off() [%s]> ERR: system is still on after power off\n", get_system_state_str(system_state_str));
    }

    vTaskDelete(NULL);
}

void do_reset_task(void *pvParameters) {
    char system_state_str[64] = {0};

    // bail if we're already in a "powering" state
    if (system_state==STATE_POWERING_ON || system_state==STATE_POWERING_OFF) {
        printf("do_reset_task [%s]> ERR: should not reach here b/c we're already powering on or off\n", get_system_state_str(system_state_str));
        vTaskDelete(NULL);
        return;
    }

    // notify reset starting
    printf("do_reset_task() [%s]> reseting...\n", get_system_state_str(system_state_str));

    // trigger reset pin
    gpio_set_level(GPIO_OUTPUT_MOBO_RESET, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_OUTPUT_MOBO_RESET, 1);

    // update to powering on state since that's what we're now doing
    printf("do_reset_task() [%s]> going to state STATE_POWERING_ON\n", get_system_state_str(system_state_str));
    system_state = STATE_POWERING_ON;
    printf("do_reset_task() [%s]> state updated\n", get_system_state_str(system_state_str));

    // skip this if system is automatic login + blank password keychain
#ifdef SYSTEM_PRESENTS_LOGIN_DIALOG
    // after RESET_KEYSTROKE_WAIT_SEC seconds, send login keystrokes with teensy D0
    for (int i=0; i<=RESET_KEYSTROKE_WAIT_SEC; i++) {
        sprintf(wait_status_msg, "(%d of %d sec elapsed)", i, RESET_KEYSTROKE_WAIT_SEC);
        if (!(i%10)) printf("do_reset_task() [%s]> waiting for boot up %s\n", get_system_state_str(system_state_str), wait_status_msg);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    // send login keystrokes with teensy D0
    gpio_set_level(GPIO_OUTPUT_TEENSY_B0, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_OUTPUT_TEENSY_B0, 1);
#endif

    // notify done
    printf("do_reset_task() [%s]> reset actions completed\n", get_system_state_str(system_state_str));
    reqd_action_flags = 0;

    // change state to SYSTEM_ON and notify
    if (mobo_hdd_active()) {
        printf("do_reset_task() [%s]> going to state STATE_SYSTEM_ON\n", get_system_state_str(system_state_str));
        system_state = STATE_SYSTEM_ON;
        printf("do_reset_task() [%s]> state updated\n", get_system_state_str(system_state_str));
    } else {
        system_state = STATE_SYSTEM_OFF;
        printf("do_reset_task() [%s]> ERR: system is still off after reset\n", get_system_state_str(system_state_str));
    }

    vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
///
/// GPIO timer
///
///////////////////////////////////////////////////////////////////////////////

TimerHandle_t gpioTimer;
void gpioTimerCallback(TimerHandle_t xTimer) {
    char system_state_str[64] = {0}; // buffer for get_system_state_str(char*)
    update_system_state();

    static int startup_delay_sec = 2;
    static bool startup_complete = false;
    if (!startup_complete && startup_delay_sec-- > 0) {
        gpio_set_level(GPIO_OUTPUT_BLUE, 1); // blue = startup delay
        printf("timer [%s]> starting up\n", get_system_state_str(system_state_str));
        return;
    } else {
        if (!startup_complete) {
            startup_complete = true;
            printf("timer [%s]> startup complete\n", get_system_state_str(system_state_str));
            reqd_action_flags = 0;
        }
        gpio_set_level(GPIO_OUTPUT_BLUE, 0); // blue off for remainder of operation
    }

    // once we're past startup delay...
    
    // set LEDs per current system state:
    //  - powered off:  solid red
    //  - powered on:  solid green
    //  - powering up:  flashing green
    //  - powering down: flashing red
    static int flasher_cnt = 0;
    switch (system_state) {
        case STATE_SYSTEM_OFF:
            gpio_set_level(GPIO_OUTPUT_RED, 1);
            gpio_set_level(GPIO_OUTPUT_GREEN, 0);
            break;
        case STATE_SYSTEM_ON:
            gpio_set_level(GPIO_OUTPUT_RED, 0);
            gpio_set_level(GPIO_OUTPUT_GREEN, 1);
            break;
        case STATE_POWERING_ON:
            gpio_set_level(GPIO_OUTPUT_RED, 0);
            gpio_set_level(GPIO_OUTPUT_GREEN, (flasher_cnt++ % 2));
            break;
        case STATE_POWERING_OFF:
            gpio_set_level(GPIO_OUTPUT_RED, (flasher_cnt++ % 2));
            gpio_set_level(GPIO_OUTPUT_GREEN, 0);
            break;
    }
    static int cnt = 0;
    if (cnt++ % 15 == 0) printf("timer [%s]> tick\n", get_system_state_str(system_state_str));
    
    // handle requests if we're not in a powering state
    if (reqd_action_flags!=0) {
        if (system_state != STATE_POWERING_OFF && system_state != STATE_POWERING_ON) {
            if (reqd_action_flags & FLAGS_RESET_REQD) {
                printf("timer [%s]> reset requested\n", get_system_state_str(system_state_str));
                xTaskCreate(do_reset_task, "ResetTask", 2048, NULL, 10, NULL);
            } else if (reqd_action_flags & FLAGS_POWER_ON_REQD) {
                if (system_state==STATE_SYSTEM_OFF) {
                    printf("timer [%s]> power on requested\n", get_system_state_str(system_state_str));
                    xTaskCreate(do_power_on, "PowerOnTask", 2048, NULL, 10, NULL);
                } else {
                    printf("timer [%s]> power on requested but system already on (try /reset to recover?)\n", get_system_state_str(system_state_str));
                    reqd_action_flags &= ~FLAGS_POWER_ON_REQD;
                }
            } else if (reqd_action_flags & FLAGS_POWER_OFF_REQD) {
                if (system_state==STATE_SYSTEM_ON) {
                    printf("timer [%s]> power off requested\n", get_system_state_str(system_state_str));
                    xTaskCreate(do_power_off, "PowerOffTask", 2048, NULL, 10, NULL);
                } else {
                    printf("timer [%s]> power off requested but system already off (try /reset to recover?)\n", get_system_state_str(system_state_str));
                    reqd_action_flags &= ~FLAGS_POWER_OFF_REQD;
                }
            }
        } else {
            //printf("timer [%s]> ignoring request because we're powering on or off\n", get_system_state_str(system_state_str));
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///
///  Wifi / HTTP Server
///
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define HTTPD_LISTEN_PORT               CONFIG_HTTPD_LISTEN_PORT

#define EXAMPLE_HTTP_QUERY_KEY_MAX_LEN  (64)

/* A simple example that demonstrates how to create GET and POST
 * handlers for the web server.
 */

static const char *TAG = "esp32-mobo";

/*
// An HTTP GET handler
static esp_err_t hello_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    // Get header value string length and allocate memory for length + 1,
    // extra byte for null termination
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        // Copy null terminated value string into buffer
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    // Read URL query string length and allocate memory for length + 1,
    // extra byte for null termination
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[EXAMPLE_HTTP_QUERY_KEY_MAX_LEN], dec_param[EXAMPLE_HTTP_QUERY_KEY_MAX_LEN] = {0};
            // Get value of expected key from query string
            if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
                example_uri_decode(dec_param, param, strnlen(param, EXAMPLE_HTTP_QUERY_KEY_MAX_LEN));
                ESP_LOGI(TAG, "Decoded query parameter => %s", dec_param);
            }
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
                example_uri_decode(dec_param, param, strnlen(param, EXAMPLE_HTTP_QUERY_KEY_MAX_LEN));
                ESP_LOGI(TAG, "Decoded query parameter => %s", dec_param);
            }
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
                example_uri_decode(dec_param, param, strnlen(param, EXAMPLE_HTTP_QUERY_KEY_MAX_LEN));
                ESP_LOGI(TAG, "Decoded query parameter => %s", dec_param);
            }
        }
        free(buf);
    }

    // Set some custom headers
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    // Send response with custom headers and body set as the
    // string passed in user context
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    // After sending the HTTP response the old HTTP request
    // headers are lost. Check if HTTP request headers can be read now.
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "(request headers no longer available)");
    }
    return ESP_OK;
}

static const httpd_uri_t hello = {
    .uri       = "/hello",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    // Let's pass response string in user
    // context to demonstrate it's usage
    .user_ctx  = "Hello World!"
};

// An HTTP POST handler
static esp_err_t echo_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        // Read the data for the request
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                // Retry receiving if timeout occurred
                continue;
            }
            return ESP_FAIL;
        }

        // Send back the same data
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        // Log data received
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t echo = {
    .uri       = "/echo",
    .method    = HTTP_POST,
    .handler   = echo_post_handler,
    .user_ctx  = NULL
};

// This handler allows the custom error handling functionality to be
// tested from client side. For that, when a PUT request 0 is sent to
// URI /ctrl, the /hello and /echo URIs are unregistered and following
// custom error handler http_404_error_handler() is registered.
// Afterwards, when /hello or /echo is requested, this custom error
// handler is invoked which, after sending an error message to client,
// either closes the underlying socket (when requested URI is /echo)
// or keeps it open (when requested URI is /hello). This allows the
// client to infer if the custom error handler is functioning as expected
// by observing the socket state.
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/hello", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        // Return ESP_OK to keep underlying socket open
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        // Return ESP_FAIL to close underlying socket
        return ESP_FAIL;
    }
    // For any other URI send 404 and close socket
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

// An HTTP PUT handler. This demonstrates realtime
// registration and deregistration of URI handlers
static esp_err_t ctrl_put_handler(httpd_req_t *req)
{
    char buf;
    int ret;

    if ((ret = httpd_req_recv(req, &buf, 1)) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    if (buf == '0') {
        // URI handlers can be unregistered using the uri string
        ESP_LOGI(TAG, "Unregistering /hello and /echo URIs");
        httpd_unregister_uri(req->handle, "/hello");
        httpd_unregister_uri(req->handle, "/echo");
        // Register the custom error handler
        httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    else {
        ESP_LOGI(TAG, "Registering /hello and /echo URIs");
        httpd_register_uri_handler(req->handle, &hello);
        httpd_register_uri_handler(req->handle, &echo);
        // Unregister custom error handler
        httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, NULL);
    }

    // Respond with empty body
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t ctrl = {
    .uri       = "/ctrl",
    .method    = HTTP_PUT,
    .handler   = ctrl_put_handler,
    .user_ctx  = NULL
};
*/



// Handles all requests
#define HTTP_REQ_POWER_CYCLE  0b00000001
#define HTTP_REQ_POWER_ON     0b00000010
#define HTTP_REQ_POWER_OFF    0b00000100
#define HTTP_REQ_RESET        0b00001000
#define HTTP_REQ_STATUS       0b00010000
#define HTTP_REQ_HELP         0b00100000
#define HTTP_REQ_RESEND_LOGIN 0b01000000
static esp_err_t req_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    // Get the host header and print it out
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        // Copy null terminated value string into buffer
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "http> Remote host: %s", buf);
        }
        free(buf);
    }

    // Read the value of the input pin GPIO_INPUT_HDD_ACTIVE
    char system_state_str[32] = {0};
    char status_str[384];

    // Set a flag or provide status based on the request type
    int req_type = (int) req->user_ctx;
    if (req_type == HTTP_REQ_HELP) {
        sprintf(status_str, "http [%s]> \n  *** HELP ***\n  /help       (this help)\n  /power      (cycles power regardless of current state)\n  /on         (turns on)\n  /off        (turns off)\n  /reset      (performs a hard reset)\n  /status     (shows current status)\n  /send-login (send u/p again)\n", get_system_state_str(system_state_str));
        httpd_resp_send(req, status_str, HTTPD_RESP_USE_STRLEN);
    } else if (req_type == HTTP_REQ_STATUS) {
        if (reqd_action_flags & FLAGS_RESET_REQD) {
            sprintf(status_str, "http [%s]> reset in progress %s (hdd=%d)", get_system_state_str(system_state_str), wait_status_msg, mobo_hdd_active());
        } else if (reqd_action_flags & FLAGS_POWER_ON_REQD) {
            sprintf(status_str, "http [%s]> power on in progress %s (hdd=%d)", get_system_state_str(system_state_str), wait_status_msg, mobo_hdd_active());
        } else if (reqd_action_flags & FLAGS_POWER_OFF_REQD) {
            sprintf(status_str, "http [%s]> power off in progress %s (hdd=%d)", get_system_state_str(system_state_str), wait_status_msg, mobo_hdd_active());
        } else {
            sprintf(status_str, "http [%s]> idle (hdd=%d)", get_system_state_str(system_state_str), mobo_hdd_active());
        }
        httpd_resp_send(req, status_str, HTTPD_RESP_USE_STRLEN);
    } else if (req_type == HTTP_REQ_RESEND_LOGIN) {
        // send login keystrokes with teensy D0
        gpio_set_level(GPIO_OUTPUT_TEENSY_B0, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_TEENSY_B0, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        sprintf(status_str, "http [%s]> resent u/p keystrokes\n", get_system_state_str(system_state_str));
        httpd_resp_send(req, status_str, HTTPD_RESP_USE_STRLEN);
    } else if (reqd_action_flags != 0) {
        sprintf(status_str, "http [%s]> another flag already set; doing nothing (flags=%x)", get_system_state_str(system_state_str), reqd_action_flags);
        httpd_resp_send(req, status_str, HTTPD_RESP_USE_STRLEN);
    } else {
        if (req_type == HTTP_REQ_POWER_ON) {
            if (system_state == STATE_SYSTEM_ON) {
                sprintf(status_str, "http [%s]> system already on; doing nothing (hdd=%d)", get_system_state_str(system_state_str), mobo_hdd_active());
            } else {
                reqd_action_flags |= FLAGS_POWER_ON_REQD;
                sprintf(status_str, "http [%s]> power on flag set (hdd=%d)", get_system_state_str(system_state_str), mobo_hdd_active());
            }
            httpd_resp_send(req, status_str, HTTPD_RESP_USE_STRLEN);
        } else if (req_type == HTTP_REQ_POWER_OFF) {
            if (system_state == STATE_SYSTEM_OFF) {
                sprintf(status_str, "http [%s]> system already off; doing nothing (hdd=%d)", get_system_state_str(system_state_str), mobo_hdd_active());
            } else {
                reqd_action_flags |= FLAGS_POWER_OFF_REQD;
                sprintf(status_str, "http [%s]> power off flag set (hdd=%d)", get_system_state_str(system_state_str), mobo_hdd_active());
            }
            httpd_resp_send(req, status_str, HTTPD_RESP_USE_STRLEN);
        } else if (req_type == HTTP_REQ_POWER_CYCLE) {
            reqd_action_flags |= mobo_hdd_active() ? FLAGS_POWER_OFF_REQD : FLAGS_POWER_ON_REQD;
            sprintf(status_str, "http [%s]> board %s; power %s flag is now set (hdd=%d)", mobo_hdd_active() ? "on" : "off", get_system_state_str(system_state_str), mobo_hdd_active() ? "off" : "on", mobo_hdd_active());
            httpd_resp_send(req, status_str, HTTPD_RESP_USE_STRLEN);
        } else if (req_type == HTTP_REQ_RESET) {
            if (system_state == STATE_SYSTEM_OFF) {
                reqd_action_flags |= FLAGS_POWER_ON_REQD;
                sprintf(status_str, "http [%s]> got reset, but power off ==> power on flag set instead (hdd=%d)", get_system_state_str(system_state_str), mobo_hdd_active());
            } else {
                reqd_action_flags |= FLAGS_RESET_REQD;
                sprintf(status_str, "http [%s]> reset flag set (hdd=%d)", get_system_state_str(system_state_str), mobo_hdd_active());
            }
            httpd_resp_send(req, status_str, HTTPD_RESP_USE_STRLEN);
        }
    }

    return ESP_OK;
}

static const httpd_uri_t uri_help = {
    .uri       = "/help",
    .method    = HTTP_GET,
    .handler   = req_handler,
    // the user context tells what operation is to be performed
    .user_ctx  = (void*) HTTP_REQ_HELP
};
static const httpd_uri_t uri_power_cycle = {
    .uri       = "/power",
    .method    = HTTP_GET,
    .handler   = req_handler,
    // the user context tells what operation is to be performed
    .user_ctx  = (void*) HTTP_REQ_POWER_CYCLE
};
static const httpd_uri_t uri_power_on = {
    .uri       = "/on",
    .method    = HTTP_GET,
    .handler   = req_handler,
    // the user context tells what operation is to be performed
    .user_ctx  = (void*) HTTP_REQ_POWER_ON
};
static const httpd_uri_t uri_power_off = {
    .uri       = "/off",
    .method    = HTTP_GET,
    .handler   = req_handler,
    // the user context tells what operation is to be performed
    .user_ctx  = (void*) HTTP_REQ_POWER_OFF
};
static const httpd_uri_t uri_reset = {
    .uri       = "/reset",
    .method    = HTTP_GET,
    .handler   = req_handler,
    .user_ctx  = (void*) HTTP_REQ_RESET
};
static const httpd_uri_t uri_status = {
    .uri       = "/status",
    .method    = HTTP_GET,
    .handler   = req_handler,
    .user_ctx  = (void*) HTTP_REQ_STATUS
};
static const httpd_uri_t uri_resend_login = {
    .uri       = "/send-login",
    .method    = HTTP_GET,
    .handler   = req_handler,
    .user_ctx  = (void*) HTTP_REQ_RESEND_LOGIN
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = HTTPD_LISTEN_PORT;
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &uri_help);
        httpd_register_uri_handler(server, &uri_power_cycle);
        httpd_register_uri_handler(server, &uri_power_on);
        httpd_register_uri_handler(server, &uri_power_off);
        httpd_register_uri_handler(server, &uri_reset);
        httpd_register_uri_handler(server, &uri_status);
        httpd_register_uri_handler(server, &uri_resend_login);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

#if !CONFIG_IDF_TARGET_LINUX
static esp_err_t stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK) {
            *server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}
#endif // !CONFIG_IDF_TARGET_LINUX

void app_main(void)
{
    ///////////////////////////////////////////////////////////////////////////////
    ///
    ///  Http Server
    ///
    ///////////////////////////////////////////////////////////////////////////////
 
    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    // Read "Establishing Wi-Fi or Ethernet Connection" section in
    // examples/protocols/README.md for more information about this function.
    //
    ESP_ERROR_CHECK(example_connect());

    // Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
    // and re-start it upon connection.
    //
#if !CONFIG_IDF_TARGET_LINUX
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET
#endif // !CONFIG_IDF_TARGET_LINUX

    // Start the server for the first time
    server = start_webserver();

    ///////////////////////////////////////////////////////////////////////////////
    ///
    ///  GPIOs
    ///
    ///////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////
    //
    // outputs setup
    //
    ///////////////////////////////////

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};

    //
    // outputs for LEDs, mobo pins (active low) and teensy pins (active low)
    //

    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO21/22
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL_LEDS | GPIO_OUTPUT_PIN_SEL_TEENSY | GPIO_OUTPUT_PIN_SEL_MOBO;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable/enable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // these are active low; bring them high as early as possible
    gpio_set_level(GPIO_OUTPUT_MOBO_RESET, 1);
    gpio_set_level(GPIO_OUTPUT_MOBO_POWER, 1);
    gpio_set_level(GPIO_OUTPUT_TEENSY_B0,  1);
    gpio_set_level(GPIO_OUTPUT_TEENSY_B1,  1);


    ///////////////////////////////////////////////
    //
    // inputs setup: btns trigger on rising edges
    //
    ///////////////////////////////////////////////

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    ///////////////////////////////////////////////
    //
    // PLED+ sensing input
    //
    ///////////////////////////////////////////////

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO32 here
    io_conf.pin_bit_mask = GPIO_INPUT_SENSING_HDD_ACTIVE_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    // //create a queue to handle gpio event from isr
    // gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // //start gpio task
    // xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_TEST_BTN_LEFT, gpio_isr_handler, (void*) GPIO_INPUT_TEST_BTN_LEFT);
    gpio_isr_handler_add(GPIO_INPUT_TEST_BTN_RIGHT, gpio_isr_handler, (void*) GPIO_INPUT_TEST_BTN_RIGHT);    
    // printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    // Create a software timer
    gpioTimer = xTimerCreate("GPIOTimer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, gpioTimerCallback);
    if (gpioTimer == NULL) {
        printf("Timer create failed\n");
    } else {
        if (xTimerStart(gpioTimer, 0) != pdPASS) {
            printf("Timer start failed\n");
        }
    }

    // int cnt = 0;
    while (server) {

        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        // gpio_set_level(GPIO_OUTPUT_BLUE, (++cnt % 2)); // exp

        // if (btn_presses & BTN_LEFT_PRESS_MASK) {
        //     printf("Main loop: left button pressed; clearing it\n");
        //
        //     // B0: send user/password HID keyboard request
        //     gpio_set_level(GPIO_OUTPUT_BLUE, 1); // temp
        //     gpio_set_level(GPIO_OUTPUT_TEENSY_B0, 0);
        //     vTaskDelay(4000 / portTICK_PERIOD_MS);
        //     gpio_set_level(GPIO_OUTPUT_TEENSY_B0, 1);
        //     gpio_set_level(GPIO_OUTPUT_BLUE, 0); // temp
        //
        //     btn_presses &= ~BTN_LEFT_PRESS_MASK;
        // }
        // else if (btn_presses & BTN_RIGHT_PRESS_MASK) {
        //     printf("Main loop: right button pressed; clearing it\n");
        //
        //     // B1: send arrow+enter HID keyboard request
        //     gpio_set_level(GPIO_OUTPUT_RED, 1); // temp
        //     gpio_set_level(GPIO_OUTPUT_TEENSY_B1, 0);
        //     vTaskDelay(4000 / portTICK_PERIOD_MS);
        //     gpio_set_level(GPIO_OUTPUT_TEENSY_B1, 1);
        //     gpio_set_level(GPIO_OUTPUT_RED, 0); // temp
        //
        //     btn_presses &= ~BTN_RIGHT_PRESS_MASK;
        // } else {
        //     gpio_set_level(GPIO_OUTPUT_TEENSY_B0, 1);
        //     gpio_set_level(GPIO_OUTPUT_TEENSY_B1, 1);
        // }

       vTaskDelay(portMAX_DELAY); // Sleep indefinitely
    }
}
