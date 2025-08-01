/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

// --- Configuration Constants ---
#define JOYSTICK_MAX_RAW_VALUE 17.0f // The max absolute value from your joystick (e.g., 'ef' is -17)
#define JOYSTICK_DEADZONE 3          // Raw joystick values from -3 to 3 will be ignored
#define ANGLE_DECREMENT_C 15.0f      // Amount to decrease anglular velocity by when C button is pressed
#define ANGLE_INCREMENT_D 15.0f      // Amount to increase anglular velocity by when D button is pressed

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
#define ESP_BD_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define ESP_BD_ADDR_HEX(addr) addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
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

typedef struct
{
    float x;     // Vehicle Forward/Backward motion (-1.0 to 1.0)
    float y;     // Vehicle Strafe Left/Right motion (-1.0 to 1.0)
    float theta; // Vehicle Rotation Left/Right (-1.0 to 1.0)
} motion_command_t;

// A global variable to store the current motion vector of the controller.
motion_command_t current_motion = {0.0f, 0.0f, 0.0f};

void parse_and_map_hid_report(uint8_t report_id, const uint8_t *data, int length);

static const char *TAG = "ESP_HIDH_DEMO";

#if CONFIG_BT_HID_HOST_ENABLED
static const char *remote_device_name = CONFIG_EXAMPLE_PEER_DEVICE_NAME;
#endif // CONFIG_BT_HID_HOST_ENABLED

#if !CONFIG_BT_NIMBLE_ENABLED
static char *bda2str(uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18)
    {
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

    switch (event)
    {
    case ESP_HIDH_OPEN_EVENT:
    {
        if (param->open.status == ESP_OK)
        {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            if (bda)
            {
                ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
                esp_hidh_dev_dump(param->open.dev, stdout);
            }
        }
        else
        {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        if (bda)
        {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        }
        break;
    }
        // case ESP_HIDH_INPUT_EVENT:
        // {
        //     const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        //     if (bda)
        //     {
        //         ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        //         ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
        //     }
        //     break;
        // }

    case ESP_HIDH_INPUT_EVENT:
    {
        // parse data from controller input
        parse_and_map_hid_report(param->input.report_id, param->input.data, param->input.length);

        // print to monitor
        ESP_LOGI("MIXER_INPUT", "x: %.2f, y: %.2f, theta: %.2f", current_motion.x, current_motion.y, current_motion.theta);
        break;
    }

    case ESP_HIDH_FEATURE_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        if (bda)
        {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                     esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                     param->feature.length);
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        }
        break;
    }
    case ESP_HIDH_CLOSE_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        if (bda)
        {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        }
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

#define SCAN_DURATION_SECONDS 5

void hid_demo_task(void *pvParameters)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");
    // start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len)
    {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        while (r)
        {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE)
            {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_NIMBLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE)
            {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%d', ", r->ble.addr_type);
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT)
            {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
                if (strncmp(r->name, remote_device_name, strlen(remote_device_name)) == 0)
                {
                    break;
                }
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }

#if CONFIG_BT_HID_HOST_ENABLED
        if (cr && strncmp(cr->name, remote_device_name, strlen(remote_device_name)) == 0)
        {
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }
#else
        if (cr)
        {
            // open the last result
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }
#endif // CONFIG_BT_HID_HOST_ENABLED
       // free the results
        esp_hid_scan_results_free(results);
    }
    vTaskDelete(NULL);
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

/**
 * @brief Parses raw HID reports from controller and maps them to vehicle motion commands.
 *
 * @param report_id The ID of the incoming HID report.
 * @param data A pointer to the raw data buffer.
 * @param length The length of the data buffer.
 */
void parse_and_map_hid_report(uint8_t report_id, const uint8_t *data, int length)
{
    // NEW: Static variable to track button state (for C and D)
    static bool button_already_processed = false;

    // ---Joystick Movement ---
    if (report_id == 2 && length >= 3)
    {
        // The remote sends joystick data as signed 8-bit integers.
        int8_t raw_phys_y = (int8_t)data[1]; // Physical Up/Down axis
        int8_t raw_phys_x = (int8_t)data[2]; // Physical Left/Right axis

        // Apply Deadzone
        if (abs(raw_phys_y) < JOYSTICK_DEADZONE)
        {
            raw_phys_y = 0;
        }
        if (abs(raw_phys_x) < JOYSTICK_DEADZONE)
        {
            raw_phys_x = 0;
        }

        // === Sideways Orientation Mapping ===
        // We map the physical axes to the vehicle's logical axes.
        // Physical "Up" (positive Y) becomes vehicle "Forward" (positive X value).
        // Physical "Right" (positive X) becomes vehicle "Strafe Right" (positive Y).
        // We normalize the raw value to a float between -1.0 and 1.0.
        current_motion.x = (float)raw_phys_y / JOYSTICK_MAX_RAW_VALUE;
        current_motion.y = (float)raw_phys_x / JOYSTICK_MAX_RAW_VALUE;

        // Clamp values to ensure they are within [-1.0, 1.0]
        if (current_motion.x > 1.0f)
            current_motion.x = 1.0f;
        if (current_motion.x < -1.0f)
            current_motion.x = -1.0f;
        if (current_motion.y > 1.0f)
            current_motion.y = 1.0f;
        if (current_motion.y < -1.0f)
            current_motion.y = -1.0f;
    }

    // --- Report ID 3 is for the C and D buttons ---
    if (report_id == 3 && length >= 1)
    {
        uint8_t button_code = data[0];

        switch (button_code)
        {
        case 0xe9: // Button C (leftmost) - Decrease Angle
            if (!button_already_processed)
            {
                current_motion.theta -= ANGLE_DECREMENT_C;
                // Wrap angle if it goes below -180
                if (current_motion.theta < -180.0f)
                {
                    current_motion.theta += 360.0f;
                }
                button_already_processed = true;
            }
            break;
        case 0xea: // Button D (rightmost) - Increase Angle
            if (!button_already_processed)
            {
                current_motion.theta += ANGLE_INCREMENT_D;
                // Wrap angle if it goes above 180
                if (current_motion.theta > 180.0f)
                {
                    current_motion.theta -= 360.0f;
                }
                button_already_processed = true;
            }
            break;
        case 0x00: // All buttons released
            // Reset the flag so the next press can be registered.
            button_already_processed = false;
            break;
        default:
            // Handle other buttons (A, B) here if needed later
            break;
        }
    }
}

void app_main(void)
{
    esp_err_t ret;
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(esp_hidh_init(&config));

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
    if (ret)
    {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
    }

    vTaskDelay(200);

    uint8_t own_addr_type = 0;
    int rc;
    uint8_t addr_val[6] = {0};

    rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, NULL, NULL);

    rc = ble_hs_id_infer_auto(0, &own_addr_type);

    if (rc != 0)
    {
        ESP_LOGI(TAG, "error determining address type; rc=%d\n", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    ESP_LOGI(TAG, "Device Address: ");
    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x \n", addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0]);

#endif
    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);
}