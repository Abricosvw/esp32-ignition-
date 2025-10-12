#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_err.h"
#include <inttypes.h>
#include <stdbool.h>

static const char *TAG = "can_ignition";

// Handle for the alert task so we can suspend/resume it during driver restart
static TaskHandle_t g_twai_alert_handle = NULL;
// Counters and state
static int g_twai_baud_kbps = 500; // current baud in kbps (500 by default)
// Bus error state — when true, error LEDs blink until cleared
volatile bool g_twai_bus_error = false;
static int g_twai_bus_error_clear_count = 0;
// Scheduled restart tick (used with backoff)
// static TickType_t g_twai_restart_scheduled_tick = 0; // No longer needed
// Exponential backoff for restart attempts (ms)
// volatile uint32_t g_twai_backoff_ms = 500; // No longer needed
// static const uint32_t BACKOFF_INITIAL_MS = 500; // No longer needed
// (ERROR_POLL_THRESHOLD removed — policy now ignores bus errors by default)
static int g_twai_error_poll_count = 0;
// Track whether TX was running before an error so we resume automatically
// volatile bool g_twai_tx_was_running = false; // No longer needed
// Continuous TX control
volatile bool g_twai_tx_running = false;
volatile int g_ignition_state = 0; // 0=OFF,1=Key,2=Ignition
#define TWAI_TX_PERIOD_MS 100
// Restart cooldown to avoid restart storms (ms)
// #define RESTART_COOLDOWN_MS 2000 // No longer needed
// static TickType_t g_twai_last_restart_tick = 0; // No longer needed

// Pins (match original sketch)
// NOTE: some boards/wiring expect RX=GPIO4 and TX=GPIO5. Set accordingly below.
#define CAN_TX_GPIO GPIO_NUM_5
#define CAN_RX_GPIO GPIO_NUM_4
// Button moved from GPIO18 to GPIO25 as requested
#define BUTTON_GPIO GPIO_NUM_25

// Activity LEDs (D26, D27)
#define LED_ACTIVITY_H GPIO_NUM_26
#define LED_ACTIVITY_L GPIO_NUM_27

// CAN message parameters
#define CAN_ID 0x3C0
#define CAN_DLC 4

// Debounce / timing
#define DEBOUNCE_MS 50
#define BUTTON_DELAY_MS 500

static void twai_init_500kb(void)
{
    // General config: tx, rx, normal mode
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    // Increase queue sizes to match Arduino example and avoid dropped frames under load
    g_config.rx_queue_len = 32;
    // increase tx queue to allow short bursts without immediate blocking
    g_config.tx_queue_len = 4;

    // Timing for 500 kbps
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    
    // Accept all IDs
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret;
    ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(ret));
        twai_driver_uninstall();
        return;
    }

    // Configure important alerts we want to monitor (including Bus-Off)
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF | TWAI_ALERT_RECOVERED;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK) {
        ESP_LOGW(TAG, "twai_reconfigure_alerts failed");
    }

    ESP_LOGI(TAG, "TWAI driver started on TX=%d RX=%d @500kbps", CAN_TX_GPIO, CAN_RX_GPIO);
}

static void twai_alert_task(void *arg)
{
    (void)arg;
    const TickType_t poll_ticks = pdMS_TO_TICKS(100);
    for (;;) {
        uint32_t alerts = 0;
        esp_err_t r = twai_read_alerts(&alerts, poll_ticks);
        if (r == ESP_OK && alerts) {
            if (alerts & TWAI_ALERT_BUS_OFF) {
                ESP_LOGE(TAG, "TWAI alert: BUS_OFF, initiating recovery...");
                // This will automatically trigger recovery
                if (twai_initiate_recovery() == ESP_OK) {
                    ESP_LOGI(TAG, "TWAI recovery initiated.");
                } else {
                    ESP_LOGE(TAG, "TWAI recovery initiation failed.");
                }
            }
            if (alerts & TWAI_ALERT_RECOVERED) {
                ESP_LOGI(TAG, "TWAI alert: Bus has recovered.");
            }
            // Any error alerts: Option 4 chosen — ignore bus errors and do not schedule restarts.
            // Reduce logging and do not change TX/UI state. This avoids restart storms but
            // also means the controller may remain in error-passive/bus-off until hardware is fixed.
            if (alerts & TWAI_ALERT_RX_DATA) {
                // receiving a frame counts as a clean sign of activity — clear error poll counter
                g_twai_error_poll_count = 0;
                twai_message_t msg;
                while (twai_receive(&msg, 0) == ESP_OK) {
                    // Log received frame
                    if (msg.data_length_code > 8) msg.data_length_code = 8; // sanity
                    if (msg.extd) {
                        ESP_LOGI(TAG, "RX EXT: ID=0x%08" PRIX32 " DLC=%d", msg.identifier, msg.data_length_code);
                    } else {
                        ESP_LOGI(TAG, "RX: ID=0x%03" PRIX32 " DLC=%d", msg.identifier, msg.data_length_code);
                    }
                    char buf[64];
                    int off = 0;
                    off += snprintf(buf + off, sizeof(buf) - off, "<");
                    for (int i = 0; i < msg.data_length_code && off < (int)sizeof(buf) - 4; ++i) {
                        if (i) off += snprintf(buf + off, sizeof(buf) - off, ":");
                        off += snprintf(buf + off, sizeof(buf) - off, "%02X", msg.data[i]);
                    }
                    off += snprintf(buf + off, sizeof(buf) - off, ">");
                    ESP_LOGI(TAG, "%s", buf);
                    // brief LED flash to indicate received CAN activity (skip while in bus-error mode)
                    if (!g_twai_bus_error) {
                        gpio_set_level(LED_ACTIVITY_H, 1);
                        gpio_set_level(LED_ACTIVITY_L, 1);
                        vTaskDelay(pdMS_TO_TICKS(50));
                        gpio_set_level(LED_ACTIVITY_H, 0);
                        gpio_set_level(LED_ACTIVITY_L, 0);
                    }
                }
            }
            if (alerts & TWAI_ALERT_ERR_PASS) {
                ESP_LOGD(TAG, "TWAI alert: ERR_PASSIVE (ignored)");
                // Read status info if available (struct fields vary between IDF versions)
                twai_status_info_t status_info;
                if (twai_get_status_info(&status_info) == ESP_OK) {
                    ESP_LOGD(TAG, "TWAI status info read on ERR_PASSIVE");
                } else {
                    ESP_LOGD(TAG, "Couldn't read TWAI status info on ERR_PASSIVE");
                }
            }
            if (alerts & TWAI_ALERT_BUS_ERROR) {
                ESP_LOGD(TAG, "TWAI alert: BUS_ERROR (ignored)");
                // Read status info if available (struct fields vary between IDF versions)
                twai_status_info_t status_info;
                if (twai_get_status_info(&status_info) == ESP_OK) {
                    ESP_LOGD(TAG, "TWAI status info read on BUS_ERROR");
                } else {
                    ESP_LOGD(TAG, "Couldn't read TWAI status info on BUS_ERROR");
                }
            }
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
                ESP_LOGW(TAG, "TWAI alert: RX_QUEUE_FULL");
            }
        } else {
            // no alerts on this poll
            // clear error poll counter on clean poll
            g_twai_error_poll_count = 0;
            if (!g_twai_restarting && g_twai_bus_error) {
                // count consecutive clean polls; require 3 to clear
                g_twai_bus_error_clear_count++;
                if (g_twai_bus_error_clear_count >= 3) {
                    g_twai_bus_error = false;
                    g_twai_bus_error_clear_count = 0;
                    ESP_LOGW(TAG, "TWAI bus-error cleared after consecutive clean polls");
                    // reset backoff
                    g_twai_backoff_ms = BACKOFF_INITIAL_MS;
                    // restore TX if it was running before the error
                    if (g_twai_tx_was_running) {
                        g_twai_tx_running = true;
                        g_twai_tx_was_running = false;
                        ESP_LOGI(TAG, "TWAI TX resumed after bus cleared");
                    }
                }
            }
        }
        // small delay to yield
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void twai_error_led_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (g_twai_bus_error) {
            // blink error LED (long on, long off)
            gpio_set_level(LED_ACTIVITY_H, 1);
            vTaskDelay(pdMS_TO_TICKS(300));
            gpio_set_level(LED_ACTIVITY_H, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            // sleep while no error
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

/* This function is no longer needed, recovery is handled by twai_initiate_recovery() */
// static void twai_restart(void) { ... }

/* This function is no longer needed */
// static bool twai_probe_bus(void) { ... }


/* forward-declare TX task defined below */
static void twai_tx_task(void *arg);

void app_main(void)
{
    ESP_LOGI(TAG, "App started (ESP-IDF TWAI version)");


    // Configure button pin as input with internal pull-up
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << BUTTON_GPIO;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // Configure activity LEDs as outputs (active HIGH)
    gpio_config_t led_conf = {};
    led_conf.intr_type = GPIO_INTR_DISABLE;
    led_conf.mode = GPIO_MODE_OUTPUT;
    led_conf.pin_bit_mask = (1ULL << LED_ACTIVITY_H) | (1ULL << LED_ACTIVITY_L);
    led_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    led_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&led_conf);
    // start with LEDs off
    gpio_set_level(LED_ACTIVITY_H, 0);
    gpio_set_level(LED_ACTIVITY_L, 0);

    // Initialize TWAI (CAN) at fixed 500 kbps
    twai_init_500kb();

    // Start a background task to monitor TWAI alerts and incoming frames
    BaseType_t xret = xTaskCreate(twai_alert_task, "twai_alert", 4096, NULL, 5, &g_twai_alert_handle);
    if (xret != pdPASS || g_twai_alert_handle == NULL) {
        ESP_LOGW(TAG, "Failed to create twai_alert_task");
        g_twai_alert_handle = NULL;
    }
        // create error LED task
        xTaskCreatePinnedToCore(twai_error_led_task, "twai_error_led_task", 2048, NULL, 2, NULL, tskNO_AFFINITY);

    // create periodic TX task
    xTaskCreatePinnedToCore(twai_tx_task, "twai_tx", 4096, NULL, 4, NULL, tskNO_AFFINITY);

    while (1) {
        // The complex restart logic is no longer needed here.
        // Bus-off recovery is handled automatically in twai_alert_task.

        int level = gpio_get_level(BUTTON_GPIO);

        if (level == 0) {
            // simple debounce: wait a bit and recheck
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            if (gpio_get_level(BUTTON_GPIO) != 0) {
                // false trigger
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            // On first press, start continuous TX; subsequent presses just advance the ignition state
            if (!g_twai_tx_running) {
                g_twai_tx_running = true;
            }
            g_ignition_state++;
            if (g_ignition_state > 2) g_ignition_state = 0;

            switch (g_ignition_state) {
                case 0:
                    ESP_LOGI(TAG, "Button pressed: State: OFF (0x00)");
                    break;
                case 1:
                    ESP_LOGI(TAG, "Button pressed: State: Key in (0x01)");
                    break;
                case 2:
                    ESP_LOGI(TAG, "Button pressed: State: Ignition (0x03)");
                    break;
            }

            // anti-bounce delay
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DELAY_MS));
        }

        // periodic small delay
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void twai_tx_task(void *arg)
{
    (void)arg;
    int tx_timeout_count = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize for vTaskDelayUntil
    const TickType_t xFrequency = pdMS_TO_TICKS(TWAI_TX_PERIOD_MS);

    for (;;) {
        // Wait for the next cycle for precise timing.
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (!g_twai_tx_running) {
            continue;
        }
        // No longer need to check for g_twai_restarting
        /*
        if (g_twai_restarting) {
            // wait until restart finished
            vTaskDelay(pdMS_TO_TICKS(100));
            xLastWakeTime = xTaskGetTickCount(); // Re-initialize after unexpected delay
            continue;
        }
        */

        twai_message_t msg;
        memset(&msg, 0, sizeof(msg));
        msg.identifier = CAN_ID;
        msg.data_length_code = CAN_DLC;
        msg.extd = 0;
        for (int i = 0; i < CAN_DLC; ++i) msg.data[i] = 0x00;
        switch (g_ignition_state) {
            case 0: msg.data[2] = 0x00; break;
            case 1: msg.data[2] = 0x01; break;
            case 2: msg.data[2] = 0x03; break;
            default: msg.data[2] = 0x00; break;
        }

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(20));
        if (err == ESP_OK) {
            tx_timeout_count = 0;
            // flash LEDs to indicate TX activity (brief)
            gpio_set_level(LED_ACTIVITY_H, 1);
            gpio_set_level(LED_ACTIVITY_L, 1);
            vTaskDelay(pdMS_TO_TICKS(20));
            gpio_set_level(LED_ACTIVITY_H, 0);
            gpio_set_level(LED_ACTIVITY_L, 0);
        } else if (err == ESP_ERR_INVALID_STATE) {
            // This can happen during recovery, just log it.
            ESP_LOGW(TAG, "TX task: driver invalid state (likely recovering)");
        } else if (err == ESP_ERR_TIMEOUT) {
            tx_timeout_count++;
            ESP_LOGW(TAG, "TX task: transmit timeout (count=%d)", tx_timeout_count);
            // The bus-off recovery mechanism will handle this if it gets too bad.
        } else {
            ESP_LOGW(TAG, "TX task: twai_transmit error: %s", esp_err_to_name(err));
        }

        // The main delay is now at the top of the loop with vTaskDelayUntil
    }
}

