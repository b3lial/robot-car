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

// --- Reconnect task ---
static TaskHandle_t s_reconnect_task = NULL;

static void reconnect_task(void *pvParameters)
{
    uint8_t peer_bda[6];
    esp_ble_addr_type_t peer_addr_type;
    ESP_LOGI(TAG, "Reconnect task started");
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(6000));

        // Attempt connection, retry until connected
        for (;;) {
            if (!load_peer(peer_bda, &peer_addr_type)) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            // Scan first — only connect once we see the device advertising.
            // This ensures its GATT stack is fully up before we knock.
            ESP_LOGI(TAG, "Scanning for known peer...");
            size_t results_len = 0;
            esp_hid_scan_result_t *results = NULL;
            esp_hid_ble_scan(SCAN_DURATION_RECONNECT, peer_bda, &results_len, &results);
            bool found = false;
            esp_hid_scan_result_t *r = results;
            while (r) {
                if (memcmp(r->bda, peer_bda, 6) == 0) {
                    found = true;
                    break;
                }
                r = r->next;
            }
            esp_hid_scan_results_free(results);

            if (!found) {
                ESP_LOGI(TAG, "Peer not advertising yet, scanning again...");
                continue;
            }

            ESP_LOGI(TAG, "Peer found, connecting...");
            esp_ble_gattc_cache_clean(peer_bda);
            xEventGroupClearBits(s_hid_event_group, HID_CONNECTED_BIT | HID_DISCONNECTED_BIT);
            esp_hidh_dev_open(peer_bda, ESP_HID_TRANSPORT_BLE, peer_addr_type);
            // Wait for success or failure — 35s covers HCI timeout
            EventBits_t bits = xEventGroupWaitBits(s_hid_event_group,
                                                   HID_CONNECTED_BIT | HID_DISCONNECTED_BIT,
                                                   pdFALSE, pdFALSE, pdMS_TO_TICKS(35000));
            if (bits & HID_CONNECTED_BIT) {
                ESP_LOGI(TAG, "Connected successfully, waiting for disconnect...");
                xEventGroupWaitBits(s_hid_event_group, HID_DISCONNECTED_BIT,
                                    pdFALSE, pdFALSE, portMAX_DELAY);
                ESP_LOGI(TAG, "Disconnected, will retry...");
            } else {
                ESP_LOGW(TAG, "Connection failed, retrying...");
            }
            vTaskDelay(pdMS_TO_TICKS(3000));
            // continue inner retry loop
        }
    }
}

// --- Motor control ---

#include "driver/gpio.h"

#define MOTOR1_PIN_A  GPIO_NUM_12
#define MOTOR1_PIN_B  GPIO_NUM_14
#define MOTOR2_PIN_A  GPIO_NUM_27
#define MOTOR2_PIN_B  GPIO_NUM_26

static void set_motors(int m1a, int m1b, int m2a, int m2b)
{
    gpio_set_level(MOTOR1_PIN_A, m1a);
    gpio_set_level(MOTOR1_PIN_B, m1b);
    gpio_set_level(MOTOR2_PIN_A, m2a);
    gpio_set_level(MOTOR2_PIN_B, m2b);
}

static void motors_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR1_PIN_A) | (1ULL << MOTOR1_PIN_B) |
                        (1ULL << MOTOR2_PIN_A) | (1ULL << MOTOR2_PIN_B),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    set_motors(0, 0, 0, 0);
}

static void moveForward(void)  { printf("CAR: FORWARD\n");  set_motors(0, 1, 0, 1); }
static void moveBackward(void) { printf("CAR: BACKWARD\n"); set_motors(1, 0, 1, 0); }
static void moveLeft(void)     { printf("CAR: LEFT\n");     set_motors(1, 0, 0, 1); }
static void moveRight(void)    { printf("CAR: RIGHT\n");    set_motors(0, 1, 1, 0); }
static void stop(void)         { printf("CAR: STOP\n");     set_motors(0, 0, 0, 0); }

#define DPAD_UP     0x00
#define DPAD_RIGHT  0x02
#define DPAD_DOWN   0x04
#define DPAD_LEFT   0x06
#define DPAD_NEUTRAL 0x08

static void handle_gamepad_input(const uint8_t *data, uint16_t len)
{
    if (len < 1) return;
    switch (data[0] & 0x0F) {
        case DPAD_UP:      moveForward();  break;
        case DPAD_DOWN:    moveBackward(); break;
        case DPAD_LEFT:    moveLeft();     break;
        case DPAD_RIGHT:   moveRight();    break;
        case DPAD_NEUTRAL: stop();         break;
        default: break;
    }
}

#if CONFIG_BT_HID_HOST_ENABLED
static const char * remote_device_name = CONFIG_EXAMPLE_PEER_DEVICE_NAME;
#endif // CONFIG_BT_HID_HOST_ENABLED

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
            xEventGroupClearBits(s_hid_event_group, HID_CONNECTED_BIT);
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

void hid_demo_task(void *pvParameters)
{
    // If we have a stored peer address, hand off to reconnect_task directly
    uint8_t peer_bda[6];
    esp_ble_addr_type_t peer_addr_type;
    if (load_peer(peer_bda, &peer_addr_type)) {
        ESP_LOGI(TAG, "Known peer found in NVS, connecting directly...");
        xTaskNotifyGive(s_reconnect_task);
        vTaskDelete(NULL);
        return;
    }

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
                printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
                printf("RSSI: %d, ", r->rssi);
                printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
                if (r->transport == ESP_HID_TRANSPORT_BLE) {
                    cr = r;
                    printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                    printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
                }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_NIMBLE_ENABLED
                if (r->transport == ESP_HID_TRANSPORT_BLE) {
                    cr = r;
                    printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                    printf("ADDR_TYPE: '%d', ", r->ble.addr_type);
                }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
                if (r->transport == ESP_HID_TRANSPORT_BT) {
                    cr = r;
                    printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                    esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                    printf("] srv 0x%03x, ", r->bt.cod.service);
                    print_uuid(&r->bt.uuid);
                    printf(", ");
                    if (r->name && strncmp(r->name, remote_device_name, strlen(remote_device_name)) == 0) {
                        break;
                    }
                }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
                printf("NAME: %s ", r->name ? r->name : "");
                printf("\n");
                r = r->next;
            }

#if CONFIG_BT_HID_HOST_ENABLED
            if (cr && cr->name && strncmp(cr->name, remote_device_name, strlen(remote_device_name)) == 0) {
                save_peer(cr->bda, cr->ble.addr_type);
                esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
            }
#else
            if (cr) {
                save_peer(cr->bda, cr->ble.addr_type);
                esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
            }
#endif // CONFIG_BT_HID_HOST_ENABLED
            esp_hid_scan_results_free(results);
            vTaskDelete(NULL);
        }
        ESP_LOGI(TAG, "Device not found, retrying...");
        vTaskDelay(pdMS_TO_TICKS(2000));
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
    xTaskCreate(reconnect_task, "reconnect_task", 4 * 1024, NULL, 2, &s_reconnect_task);
    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);
}
