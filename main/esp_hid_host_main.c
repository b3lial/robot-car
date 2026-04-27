/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#else
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#define ESP_BD_ADDR_STR         "%02x:%02x:%02x:%02x:%02x:%02x"
#define ESP_BD_ADDR_HEX(addr)   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#else
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif

#include "esp_hidh.h"
#include "esp_hid_gap.h"

static const char *TAG = "ESP_HIDH_DEMO";

// --- Stored peer address for direct reconnect ---
#define PEER_BDA_NVS_KEY  "peer_bda"
#define PEER_ADDRTYPE_NVS_KEY "peer_atype"
#define PEER_NVS_NS "hidh"

static void save_peer(const uint8_t *bda, esp_ble_addr_type_t addr_type)
{
    nvs_handle_t h;
    if (nvs_open(PEER_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, PEER_BDA_NVS_KEY, bda, 6);
    nvs_set_u8(h, PEER_ADDRTYPE_NVS_KEY, (uint8_t)addr_type);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Saved peer address to NVS");
}

static bool load_peer(uint8_t *bda, esp_ble_addr_type_t *addr_type)
{
    nvs_handle_t h;
    if (nvs_open(PEER_NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = 6;
    uint8_t atype = 0;
    bool ok = (nvs_get_blob(h, PEER_BDA_NVS_KEY, bda, &len) == ESP_OK) &&
              (nvs_get_u8(h, PEER_ADDRTYPE_NVS_KEY, &atype) == ESP_OK);
    nvs_close(h);
    if (ok) *addr_type = (esp_ble_addr_type_t)atype;
    return ok;
}

#define SCAN_DURATION_SECONDS      5
#define SCAN_DURATION_RECONNECT    2

// --- Connection state ---
static EventGroupHandle_t s_hid_event_group;
#define HID_CONNECTED_BIT    BIT0
#define HID_DISCONNECTED_BIT BIT1

#if CONFIG_BT_HID_HOST_ENABLED
static const char *remote_device_name = CONFIG_EXAMPLE_PEER_DEVICE_NAME;
#endif

static void reconnect_task(void *pvParameters)
{
    uint8_t peer_bda[6];
    esp_ble_addr_type_t peer_addr_type;

    // First-time pairing: scan until a HID device is found and its BDA saved
    if (!load_peer(peer_bda, &peer_addr_type)) {
        for (;;) {
            size_t results_len = 0;
            esp_hid_scan_result_t *results = NULL;
            ESP_LOGI(TAG, "SCAN...");
            esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
            ESP_LOGI(TAG, "SCAN: %u results", results_len);
            if (results_len) {
                esp_hid_scan_result_t *r = results;
                esp_hid_scan_result_t *cr = NULL;
                while (r) {
#if CONFIG_BT_BLE_ENABLED
                    if (r->transport == ESP_HID_TRANSPORT_BLE) {
                        cr = r;
                    }
#endif
#if CONFIG_BT_HID_HOST_ENABLED
                    if (r->transport == ESP_HID_TRANSPORT_BT) {
                        cr = r;
                        if (r->name && strncmp(r->name, remote_device_name, strlen(remote_device_name)) == 0) {
                            break;
                        }
                    }
#endif
                    r = r->next;
                }
#if CONFIG_BT_HID_HOST_ENABLED
                if (cr && cr->name && strncmp(cr->name, remote_device_name, strlen(remote_device_name)) == 0) {
#else
                if (cr) {
#endif
                    save_peer(cr->bda, cr->ble.addr_type);
                    memcpy(peer_bda, cr->bda, 6);
                    peer_addr_type = cr->ble.addr_type;
                    esp_hid_scan_results_free(results);
                    break;
                }
                esp_hid_scan_results_free(results);
            }
            ESP_LOGI(TAG, "Device not found, retrying...");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    // Reconnect loop: connect directly without scanning.
    // The HCI layer retries internally for ~30s and catches the advertising window on its own.
    for (;;) {
        xEventGroupClearBits(s_hid_event_group, HID_CONNECTED_BIT | HID_DISCONNECTED_BIT);
        esp_ble_gattc_cache_clean(peer_bda);
        ESP_LOGI(TAG, "Connecting...");
        esp_hidh_dev_open(peer_bda, ESP_HID_TRANSPORT_BLE, peer_addr_type);

        // Wait indefinitely for a definitive signal. Every failure path eventually sets
        // HID_DISCONNECTED_BIT: GATT failure fires CLOSE_EVENT after ~40s; HCI timeout
        // fires OPEN_EVENT with status!=OK which our callback converts to DISCONNECTED_BIT.
        EventBits_t bits = xEventGroupWaitBits(s_hid_event_group,
                                               HID_CONNECTED_BIT | HID_DISCONNECTED_BIT,
                                               pdFALSE, pdFALSE,
                                               portMAX_DELAY);

        if (bits & HID_CONNECTED_BIT) {
            // OPEN_EVENT fired with status==OK: GATT succeeded, genuine connection.
            ESP_LOGI(TAG, "Connected! Waiting for disconnect...");
            xEventGroupWaitBits(s_hid_event_group, HID_DISCONNECTED_BIT,
                                pdFALSE, pdFALSE, portMAX_DELAY);
            ESP_LOGI(TAG, "Disconnected, retrying in 3s...");
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else {
            // DISCONNECTED_BIT set without CONNECTED_BIT: GATT failure (CLOSE after ~40s)
            // or HCI timeout. Retry quickly to catch the next advertising window.
            ESP_LOGW(TAG, "Connection failed, retrying...");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

// --- Motor control ---

#include "hal/gpio_types.h"
#include "driver/ledc.h"

#define MOTOR1_PIN_A  GPIO_NUM_12
#define MOTOR1_PIN_B  GPIO_NUM_14
#define MOTOR2_PIN_A  GPIO_NUM_27
#define MOTOR2_PIN_B  GPIO_NUM_26

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT   // duty range 0–1023
#define LEDC_FREQ_HZ    1000

#define LEDC_CH_M1A     LEDC_CHANNEL_0
#define LEDC_CH_M1B     LEDC_CHANNEL_1
#define LEDC_CH_M2A     LEDC_CHANNEL_2
#define LEDC_CH_M2B     LEDC_CHANNEL_3

static void ledc_set(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_MODE, ch, duty);
    ledc_update_duty(LEDC_MODE, ch);
}

// speed: 0–100 (percent). Returns the corresponding LEDC duty value.
static uint32_t speed_to_duty(uint8_t speed)
{
    uint32_t max = (1u << LEDC_DUTY_RES) - 1;
    if (speed >= 100) return max;
    return (uint32_t)speed * max / 100;
}

static void set_motors(int m1a, int m1b, int m2a, int m2b, uint8_t speed)
{
    uint32_t duty = speed_to_duty(speed);
    ledc_set(LEDC_CH_M1A, m1a ? duty : 0);
    ledc_set(LEDC_CH_M1B, m1b ? duty : 0);
    ledc_set(LEDC_CH_M2A, m2a ? duty : 0);
    ledc_set(LEDC_CH_M2B, m2b ? duty : 0);
}

static void motors_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channels[4] = {
        { .channel = LEDC_CH_M1A, .gpio_num = MOTOR1_PIN_A, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 },
        { .channel = LEDC_CH_M1B, .gpio_num = MOTOR1_PIN_B, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 },
        { .channel = LEDC_CH_M2A, .gpio_num = MOTOR2_PIN_A, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 },
        { .channel = LEDC_CH_M2B, .gpio_num = MOTOR2_PIN_B, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 },
    };
    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&channels[i]);
    }
}

static void moveForward(uint8_t speed)  { printf("CAR: FORWARD\n");  set_motors(0, 1, 0, 1, speed); }
static void moveBackward(uint8_t speed) { printf("CAR: BACKWARD\n"); set_motors(1, 0, 1, 0, speed); }
static void moveLeft(uint8_t speed)     { printf("CAR: LEFT\n");     set_motors(1, 0, 0, 1, speed); }
static void moveRight(uint8_t speed)    { printf("CAR: RIGHT\n");    set_motors(0, 1, 1, 0, speed); }
static void stop(void)                  { printf("CAR: STOP\n");     set_motors(0, 0, 0, 0, 0); }

#define DPAD_UP         0x00
#define DPAD_UP_RIGHT   0x01
#define DPAD_RIGHT      0x02
#define DPAD_DOWN_RIGHT 0x03
#define DPAD_DOWN       0x04
#define DPAD_DOWN_LEFT  0x05
#define DPAD_LEFT       0x06
#define DPAD_UP_LEFT    0x07
#define DPAD_NEUTRAL    0x08

// Diagonal helpers: one side at full speed, the other at half.
static void moveForwardLeft(void) {
    printf("CAR: FORWARD-LEFT\n");
    ledc_set(LEDC_CH_M1A, 0);                     // left:  50% forward
    ledc_set(LEDC_CH_M1B, speed_to_duty(40));
    ledc_set(LEDC_CH_M2A, 0);                     // right: 100% forward
    ledc_set(LEDC_CH_M2B, speed_to_duty(100));
}

static void moveForwardRight(void) {
    printf("CAR: FORWARD-RIGHT\n");
    ledc_set(LEDC_CH_M1A, 0);                     // left:  100% forward
    ledc_set(LEDC_CH_M1B, speed_to_duty(100));
    ledc_set(LEDC_CH_M2A, 0);                     // right: 50% forward
    ledc_set(LEDC_CH_M2B, speed_to_duty(40));
}

static void moveBackwardLeft(void) {
    printf("CAR: BACKWARD-LEFT\n");
    ledc_set(LEDC_CH_M1A, speed_to_duty(40));     // left:  50% backward
    ledc_set(LEDC_CH_M1B, 0);
    ledc_set(LEDC_CH_M2A, speed_to_duty(100));    // right: 100% backward
    ledc_set(LEDC_CH_M2B, 0);
}

static void moveBackwardRight(void) {
    printf("CAR: BACKWARD-RIGHT\n");
    ledc_set(LEDC_CH_M1A, speed_to_duty(100));    // left:  100% backward
    ledc_set(LEDC_CH_M1B, 0);
    ledc_set(LEDC_CH_M2A, speed_to_duty(40));     // right: 50% backward
    ledc_set(LEDC_CH_M2B, 0);
}

static void handle_gamepad_input(const uint8_t *data, uint16_t len)
{
    if (len < 1) return;
    switch (data[0] & 0x0F) {
        case DPAD_UP:           moveForward(100);       break;
        case DPAD_UP_LEFT:      moveForwardLeft();      break;
        case DPAD_UP_RIGHT:     moveForwardRight();     break;
        case DPAD_DOWN:         moveBackward(100);      break;
        case DPAD_DOWN_LEFT:    moveBackwardLeft();     break;
        case DPAD_DOWN_RIGHT:   moveBackwardRight();    break;
        case DPAD_LEFT:         moveLeft(100);           break;
        case DPAD_RIGHT:        moveRight(100);          break;
        case DPAD_NEUTRAL:      stop();                 break;
        default: break;
    }
}

#if !CONFIG_BT_NIMBLE_ENABLED
static char *bda2str(uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}
#endif

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            if (bda) {
                ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
                esp_hidh_dev_dump(param->open.dev, stdout);
            }
            xEventGroupSetBits(s_hid_event_group, HID_CONNECTED_BIT);
        } else {
            ESP_LOGE(TAG, "OPEN failed!");
            xEventGroupSetBits(s_hid_event_group, HID_DISCONNECTED_BIT);
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        }
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
            ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
            if (param->input.report_id == 3) {
                handle_gamepad_input(param->input.data, param->input.length);
            }
        }
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                    esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                    param->feature.length);
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        }
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        }
        stop();
        xEventGroupSetBits(s_hid_event_group, HID_DISCONNECTED_BIT);
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

#if CONFIG_BT_NIMBLE_ENABLED
void ble_hid_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}
void ble_store_config_init(void);
#endif
void app_main(void)
{
    esp_err_t ret;
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK( esp_hidh_init(&config) );

#if !CONFIG_BT_NIMBLE_ENABLED
    char bda_str[18] = {0};
    ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
#endif


#if CONFIG_BT_NIMBLE_ENABLED
    /* XXX Need to have template for store */
    ble_store_config_init();

    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	/* Starting nimble task after gatts is initialized*/
    ret = esp_nimble_enable(ble_hid_host_task);
    if (ret) {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
    }

    vTaskDelay(200);

    uint8_t own_addr_type = 0;
    int rc;
    uint8_t addr_val[6] = {0};

    rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, NULL, NULL);

    rc = ble_hs_id_infer_auto(0, &own_addr_type);

    if (rc != 0) {
        ESP_LOGI(TAG, "error determining address type; rc=%d\n", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    ESP_LOGI(TAG, "Device Address: ");
    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x \n", addr_val[5], addr_val[4], addr_val[3],
		                                      addr_val[2], addr_val[1], addr_val[0]);

#endif
    motors_init();
    s_hid_event_group = xEventGroupCreate();
    xTaskCreate(reconnect_task, "reconnect_task", 6 * 1024, NULL, 2, NULL);
}
