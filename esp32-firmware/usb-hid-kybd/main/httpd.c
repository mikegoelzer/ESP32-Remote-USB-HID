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

#include "globals.h"
#include "gpios.h"
#include "httpd.h"

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
        // gpio_set_level(GPIO_OUTPUT_TEENSY_B0, 0);
        // vTaskDelay(100 / portTICK_PERIOD_MS);
        // gpio_set_level(GPIO_OUTPUT_TEENSY_B0, 1);
        // vTaskDelay(100 / portTICK_PERIOD_MS);

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

httpd_handle_t start_webserver(void)
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
esp_err_t stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_stop(server);
}

void disconnect_handler(void* arg, esp_event_base_t event_base,
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

void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}
#endif // !CONFIG_IDF_TARGET_LINUX
