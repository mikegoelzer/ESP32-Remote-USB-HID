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

#include "globals.h"
#include "gpios.h"
#include "httpd.h"

#if !CONFIG_IDF_TARGET_LINUX
#include <esp_wifi.h>
#include <esp_system.h>
#include "nvs_flash.h"
#include "esp_eth.h"
#endif  // !CONFIG_IDF_TARGET_LINUX

// tinyusb managed component
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "esp_private/usb_phy.h"

// tinyusb hid device functions
#include "tinyusb_hid.h"
//TEMP:
static void send_key_x();

///////////////////////////////////////////////////////////////////////////////
///
/// System config specific #define's
///
///////////////////////////////////////////////////////////////////////////////

// definte if pressing the power button requires HID kybd input to confirm logout
// ref:  https://askubuntu.com/a/1272316
//#define SYSTEM_PRESENTS_LOGOUT_DIALOG
//#define SYSTEM_PRESENTS_LOGIN_DIALOG

#define ESP_INTR_FLAG_DEFAULT 0

// static QueueHandle_t gpio_evt_queue = NULL;

uint8_t reqd_action_flags = 0b00000000;
uint8_t system_state = STATE_SYSTEM_OFF;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    // xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

    if (gpio_num==GPIO_INPUT_BTN_POWER) {
        reqd_action_flags |= FLAGS_POWER_ON_REQD;
    } else if (gpio_num==GPIO_INPUT_BTN_RESET) {
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

///////////////////////////////////////////////////////////////////////////////
///
/// Actions
///
///////////////////////////////////////////////////////////////////////////////

void update_system_state() {
    if (system_state != STATE_POWERING_OFF && system_state != STATE_POWERING_ON) {
        system_state = mobo_hdd_active() ? STATE_SYSTEM_ON : STATE_SYSTEM_OFF;
    }
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
        gpio_set_level(GPIO_OUTPUT_AMBER, 1); // amber = startup delay
        printf("timer [%s]> starting up\n", get_system_state_str(system_state_str));
        return;
    } else {
        if (!startup_complete) {
            startup_complete = true;
            printf("timer [%s]> startup complete\n", get_system_state_str(system_state_str));
            reqd_action_flags = 0;
        }
        gpio_set_level(GPIO_OUTPUT_AMBER, 0); // amber off for remainder of operation
    }
    send_key_x();
    // once we're past startup delay...

    // set LEDs per current system state:
    //  - powered off:  solid amber
    //  - powered on:  solid green
    //  - powering up:  flashing green
    //  - powering down: flashing amber
    static int flasher_cnt = 0;
    switch (system_state) {
        case STATE_SYSTEM_OFF:
            gpio_set_level(GPIO_OUTPUT_AMBER, 1);
            gpio_set_level(GPIO_OUTPUT_GREEN, 0);
            break;
        case STATE_SYSTEM_ON:
            gpio_set_level(GPIO_OUTPUT_AMBER, 0);
            gpio_set_level(GPIO_OUTPUT_GREEN, 1);
            break;
        case STATE_POWERING_ON:
            gpio_set_level(GPIO_OUTPUT_AMBER, 0);
            gpio_set_level(GPIO_OUTPUT_GREEN, (flasher_cnt++ % 2));
            break;
        case STATE_POWERING_OFF:
            gpio_set_level(GPIO_OUTPUT_AMBER, (flasher_cnt++ % 2));
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
///
/// USB HID keyboard
///
///////////////////////////////////////////////////////////////////////////////
static uint8_t keycode[15] = {0};
static void send_key_x()
{
    uint8_t key = HID_KEY_X;
    keycode[0] = key;
    ESP_LOGI("USB Kybd", "Keyboard callback: %c", key - HID_KEY_A + 'a');
    tinyusb_hid_keyboard_report(0, keycode);
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

static void tusb_device_task(void *arg)
{
    while (1) {
        tud_task();
    }
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
    ESP_LOGI("USB Kybd", "USB Mount");
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    ESP_LOGI("USB Kybd", "USB Un-Mount");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    ESP_LOGI("USB Kybd", "USB Suspend");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    ESP_LOGI("USB Kybd", "USB Resume");
}

//--------------------------------------------------------------------+
// USB PHY config
//--------------------------------------------------------------------+
static void usb_phy_init(void)
{
    usb_phy_handle_t phy_hdl;
    // Configure USB PHY
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT,
    };
    usb_new_phy(&phy_conf, &phy_hdl);
}

void app_main(void)
{
    ///////////////////////////////////////////////////////////////////////////////
    //
    // usb init
    //
    ///////////////////////////////////////////////////////////////////////////////
    ESP_LOGI("XXXXXXXX", "testing");
    usb_phy_init();
#define BOARD_TUD_RHPORT 0
    tud_init(BOARD_TUD_RHPORT);
    xTaskCreate(tusb_device_task, "TinyUSB", 4096, NULL, 5, NULL);
    tinyusb_hid_init();
    send_key_x();

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
    // outputs for LEDs and mobo pins (active low)
    //

    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO21/22
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL_LEDS | GPIO_OUTPUT_PIN_SEL_MOBO;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable/enable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // these are active low; bring them high as early as possible
    gpio_set_level(GPIO_OUTPUT_MOBO_RESET, 1);
    gpio_set_level(GPIO_OUTPUT_MOBO_POWER, 1);

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
    gpio_isr_handler_add(GPIO_INPUT_BTN_POWER, gpio_isr_handler, (void*) GPIO_INPUT_BTN_POWER);
    gpio_isr_handler_add(GPIO_INPUT_BTN_RESET, gpio_isr_handler, (void*) GPIO_INPUT_BTN_RESET);
    gpio_isr_handler_add(GPIO_INPUT_BTN_OK, gpio_isr_handler, (void*) GPIO_INPUT_BTN_OK);
    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

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
