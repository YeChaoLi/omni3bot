
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>
#include <string.h>
#include "qmi8658.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"
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
#include "driver/ledc.h"
#include <math.h>
#include <float.h>

#define USE_IMU
#define USE_RGB
#define USE_REMOTE
#define RED (1)
#define BLACK (2)

#define USE_OMNI3
#define TARGET (BLACK)

static const char *TAG = "O3";

enum Mode
{
    MODE_FPS = 0,
    MODE_TPS,
};

struct Status
{
    // Position data
    struct
    {
        float x, y, roll, pitch, yaw;
    } target_position;

    struct
    {
        float x, y, roll, pitch, yaw;
    } current_position;

    // Speed data
    struct
    {
        float x, y, v, roll, pitch, yaw;
    } target_speed;

    struct
    {
        float x, y, v, roll, pitch, yaw;
    } current_speed;

    struct
    {
        float x, y, v, yaw;
    } set_speed;

    // Remote control data
    struct
    {
        float x, y, theta;
        bool button_a, button_b, button_c, button_d;
    } remote;

    enum Mode mode;

} status;

#ifdef USE_IMU

#define I2C_MASTER_SDA_IO 11
#define I2C_MASTER_SCL_IO 12
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

static esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true};

    return i2c_new_master_bus(&bus_config, bus_handle);
}

static void sensor_task(void *pvParameters)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing I2C...");
    i2c_master_bus_handle_t bus_handle;
    ret = i2c_master_init(&bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C (error: %d)", ret);
        return;
    }

    qmi8658_dev_t dev;
    qmi8658_data_t data;
    ESP_LOGI(TAG, "Initializing QMI8658...");
    ret = qmi8658_init(&dev, bus_handle, QMI8658_ADDRESS_HIGH);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize QMI8658 (error: %d)", ret);
        vTaskDelete(NULL);
    }
    qmi8658_set_accel_range(&dev, QMI8658_ACCEL_RANGE_8G);
    qmi8658_set_accel_odr(&dev, QMI8658_ACCEL_ODR_1000HZ);
    qmi8658_set_gyro_range(&dev, QMI8658_GYRO_RANGE_512DPS);
    qmi8658_set_gyro_odr(&dev, QMI8658_GYRO_ODR_1000HZ);
    qmi8658_set_accel_unit_mps2(&dev, true);
    qmi8658_set_gyro_unit_rads(&dev, true);
    qmi8658_set_display_precision(&dev, 4);

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000; // ms
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        bool ready;
        ret = qmi8658_is_data_ready(&dev, &ready);
        if (ret == ESP_OK && ready)
        {
            ret = qmi8658_read_sensor_data(&dev, &data);
            if (ret == ESP_OK)
            {
                ESP_LOGI(TAG, "Accel: X=%.4f m/s², Y=%.4f m/s², Z=%.4f m/s²",
                         data.accelX, data.accelY, data.accelZ);
                ESP_LOGI(TAG, "Gyro:  X=%.4f rad/s, Y=%.4f rad/s, Z=%.4f rad/s",
                         data.gyroX, data.gyroY, data.gyroZ);
                ESP_LOGI(TAG, "Temp:  %.2f °C, Timestamp: %lu",
                         data.temperature, data.timestamp);
                ESP_LOGI(TAG, "----------------------------------------");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to read sensor data (error: %d)", ret);
            }
        }
        else
        {
            ESP_LOGE(TAG, "Data not ready or error reading status (error: %d)", ret);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#endif

#ifdef USE_RGB

// Set to 1 to use DMA for driving the LED strip, 0 otherwise
// Please note the RMT DMA feature is only available on chips e.g. ESP32-S3/P4
#define LED_STRIP_USE_DMA 0

#if LED_STRIP_USE_DMA
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 256
#define LED_STRIP_MEMORY_BLOCK_WORDS 1024 // this determines the DMA block size
#else
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 64
#define LED_STRIP_MEMORY_BLOCK_WORDS 0 // let the driver choose a proper memory block size automatically
#endif                                 // LED_STRIP_USE_DMA

// GPIO assignment
#define LED_STRIP_GPIO_PIN 14

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

static led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,                        // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,                             // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,                               // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }};

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,                    // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ,             // RMT counter clock frequency
        .mem_block_symbols = LED_STRIP_MEMORY_BLOCK_WORDS, // the memory block size used by the RMT channel
        .flags = {
            .with_dma = LED_STRIP_USE_DMA, // Using DMA can improve performance when driving more LEDs
        }};

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

static void pixel_task(void *pvParameters)
{
    led_strip_handle_t led_strip = configure_led();
    bool led_on_off = false;

    ESP_LOGI(TAG, "Start blinking LED strip");
    int j = 0;

    while (1)
    {
        j = (j + 1) % 10;

        if (led_on_off)
        {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            for (int i = 0; i < LED_STRIP_LED_COUNT; i++)
            {
                if (i % 3 == 0)
                {
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, j, 0, 0));
                }
                else if (i % 3 == 1)
                {
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, j, 0));
                }
                else
                {
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, j));
                }
            }
            /* Refresh the strip to send data */
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            // ESP_LOGI(TAG, "LED ON!");
        }
        else
        {
            /* Set all LED off to clear all pixels */
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
            // ESP_LOGI(TAG, "LED OFF!");
        }

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif

#ifdef USE_REMOTE

// --- Configuration Constants ---
#define JOYSTICK_MAX_RAW_VALUE 15.0f // The max absolute value from your joystick (e.g., 'ef' is -17)
#define JOYSTICK_DEADZONE 3          // Raw joystick values from -3 to 3 will be ignored
#define ANGLE_DECREMENT_C 15.0f      // Amount to decrease anglular velocity by when C button is pressed
#define ANGLE_INCREMENT_D 15.0f      // Amount to increase anglular velocity by when  button is pressed

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

// typedef struct
// {
//     float x;     // Vehicle Forward/Backward motion (-1.0 to 1.0)
//     float y;     // Vehicle Strafe Left/Right motion (-1.0 to 1.0)
//     float theta; // Vehicle Rotation counter and clockwise (-180 to 180)
// } motion_command_t;

// // A global variable to store the current motion vector of the controller.
// motion_command_t current_motion = {0.0f, 0.0f, 0.0f};

void parse_and_map_hid_report(uint8_t report_id, const uint8_t *data, int length)
{
    // ---Joystick Movement (Report ID 2) ---
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

        // --- New Mapping ---
        // Physical Up/Down (Y-axis) controls forward/backward speed (status.remote.x)
        // Physical Left/Right (X-axis) controls spin speed (status.remote.y)

        // Normalize the raw values to a float between -1.0 and 1.0.
        status.remote.x = (float)raw_phys_y / JOYSTICK_MAX_RAW_VALUE;
        status.remote.y = (float)raw_phys_x / JOYSTICK_MAX_RAW_VALUE;

        // Clamp values to ensure they are within [-1.0, 1.0]
        if (status.remote.x > 1.0f)
            status.remote.x = 1.0f;
        if (status.remote.x < -1.0f)
            status.remote.x = -1.0f;
        if (status.remote.y > 1.0f)
            status.remote.y = 1.0f;
        if (status.remote.y < -1.0f)
            status.remote.y = -1.0f;
    }
}

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

static void remote_task(void *pvParameters)
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
    // while(1){
    hid_demo_task(NULL);
    // }
}

#endif

// Motion control for a abstract vehicle robot
// input/read:
// mode: FPS or TPS
// output/write:
// FPS: target linear speed (x), target angular speed (θ') on body frame
// TPS: target linear speed (x, y), target angular speed (θ')
static void motion_task(void *pvParameters)
{
    status.mode = MODE_FPS;
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1)
    {
        if (status.mode == MODE_FPS)
        {
            // --- New Control Logic ---
            status.target_speed.v = status.remote.x * 0.70;
            status.target_speed.yaw = -status.remote.y * 0.60;

            ESP_LOGI(TAG, "target speed: %f", status.target_speed.v);
            ESP_LOGI(TAG, "target yaw: %f", status.target_speed.yaw);

            // Set the final speed for the mixer task
            status.set_speed.v = status.target_speed.v;
            status.set_speed.yaw = status.target_speed.yaw;

            // ADD THIS "IF" STATEMENT
            if (status.set_speed.v != 0.0f || status.set_speed.yaw != 0.0f)
            {

                // ESP_LOGI(TAG, "status.set_speed.v: %f", status.set_speed.v);
                // ESP_LOGI(TAG, "status.set_speed.yaw: %f", status.set_speed.yaw);
            }
        }
        else
        {
            /* code */
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#ifdef USE_OMNI3

static void pwm1_init(void)
{
    ledc_timer_config_t pwm1_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&pwm1_timer));
    ledc_channel_config_t pwm1_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = 4,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&pwm1_channel));
}

static void pwm2_init(void)
{
    ledc_timer_config_t pwm2_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&pwm2_timer));
    ledc_channel_config_t pwm2_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = 5,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&pwm2_channel));
}

static void pwm3_init(void)
{
    ledc_timer_config_t pwm3_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&pwm3_timer));
    ledc_channel_config_t pwm3_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = 6,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&pwm3_channel));
}

static void pwm1_set(float percent)
{
    // map duty from -1.0~1.0 to 0.05~0.1. 0.15 is the middle.
    float duty = 0.05 + (percent + 1.0) * 0.025;
    // ESP_LOGI(TAG, "duty 1 =  %f", duty);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty * 8192));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void pwm2_set(float percent)
{
    float duty = 0.05 + (percent + 1.0) * 0.025;
    // ESP_LOGI(TAG, "duty 2 =  %f", duty);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty * 8192));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

static void pwm3_set(float percent)
{
    float duty = 0.05 + (percent + 1.0) * 0.025;
    // ESP_LOGI(TAG, "duty 3 =  %f", duty);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty * 8192));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
}

static void motor_setA(float throttle)
{
#if (TARGET == RED)
    pwm1_set(-throttle);
#else
    pwm1_set(throttle);
#endif
}

static void motor_setB(float throttle)
{
#if (TARGET == RED)
    pwm2_set(throttle);
#else
    pwm3_set(-throttle);
#endif
}

static void motor_setC(float throttle)
{
#if (TARGET == RED)
    pwm3_set(-throttle);
#else
    pwm2_set(throttle);
#endif
}

// Geometry constant: robot radius (wheel distance to center) scaled
// so that ω_max * L ≈ 1.0.  You must tune this for your chassis.
#define L 50 // mm

// Wheel “angles” βi (location) relative to robot-forward (0 rad):
//   A at -60°, B at 60°, C at 180°.
#define BETA_A (-M_PI / 3.0f)
#define BETA_B (M_PI / 3.0f)
#define BETA_C (M_PI)

void omni_drive_fps(float v, float rate_yaw)
{
    // PROJECT body motion into each wheel's drive direction.
    // v is forward velocity in body frame (x-axis)
    // rate_yaw is angular velocity around z-axis

    // Project (v, 0) + rate_yaw*L into each wheel's tangent axis:
    // For forward motion: v in x-direction of body frame
    // For yaw motion: rate_yaw * L contributes to each wheel
    // Wheel tangent vectors: ti = [ -sin βi,  cos βi ]
    float mA = -sinf(BETA_A) * v + cosf(BETA_A) * 0.0f - rate_yaw * L / 100;
    float mB = -sinf(BETA_B) * v + cosf(BETA_B) * 0.0f - rate_yaw * L / 100;
    float mC = -sinf(BETA_C) * v + cosf(BETA_C) * 0.0f - rate_yaw * L / 100;

    // ESP_LOGI(TAG, "mA: %f", mA);
    // ESP_LOGI(TAG, "mB: %f", mB);
    // ESP_LOGI(TAG, "mC: %f", mC);

    // Normalize if any |m| exceeds 1.0
    float maxm = fmaxf(fabsf(mA), fmaxf(fabsf(mB), fabsf(mC)));
    if (maxm > 1.0f)
    {
        mA /= maxm;
        mB /= maxm;
        mC /= maxm;
    }

    // Finally, send to your motor driver (throttle in [–1…1])
    motor_setA(mA);
    motor_setB(mB);
    motor_setC(mC);
}

void omni_drive_tps(float rate_x, float rate_y, float omega, float yaw)
{
    // 1) Rotate world→body:
    //    [vxb]   [ cos yaw   sin yaw ] [rate_x]
    //    [vyb] = [ -sin yaw  cos yaw ] [rate_y]
    float vxb = rate_x * cosf(yaw) + rate_y * sinf(yaw);
    float vyb = -rate_x * sinf(yaw) + rate_y * cosf(yaw);

    // 2) Project (vxb, vyb) + ω·L into each wheel’s tangent axis:
    //    ti = [ -sin βi,  cos βi ]
    float mA = -sinf(BETA_A) * vxb + cosf(BETA_A) * vyb + omega * L;
    float mB = -sinf(BETA_B) * vxb + cosf(BETA_B) * vyb + omega * L;
    float mC = -sinf(BETA_C) * vxb + cosf(BETA_C) * vyb + omega * L;

    // 3) Normalize if any |m| > 1 so we stay in the [–1…+1] throttle range
    float maxm = fmaxf(fabsf(mA), fmaxf(fabsf(mB), fabsf(mC)));
    if (maxm > 1.0f)
    {
        mA /= maxm;
        mB /= maxm;
        mC /= maxm;
    }

    // 4) Send to your motor drivers:
    // motor_setA(mA);
    // motor_setB(mB);
    // motor_setC(mC);
}

// An omni3 mixer (120 degrees apart in 3 directions) we don’t have any torque control or speed feedback, so just control speed with pwm pulse width
// input/read:
// mode: FPS(only body frame to wheel frame)
// mode: TPS(world frame to body frame to wheel frame)
// target linear speed (x, y), target angular speed (θ'),
// current linear speed (x, y), current angular speed (θ'), current yaw (θ)
// output/write:
// motor A speed(actually throttle because no encoder feedback), motor B speed, motor C speed
static void mixer_task(void *pvParameters)
{
    pwm1_init();
    pwm1_set(0);
    pwm2_init();
    pwm2_set(0);
    pwm3_init();
    pwm3_set(0);
    vTaskDelay(pdMS_TO_TICKS(500));

    // motor_setC(0.5);
    // vTaskDelay(pdMS_TO_TICKS(5000000));

    motor_setA(0.5);
    motor_setB(0.5);
    motor_setC(0.5);
    vTaskDelay(pdMS_TO_TICKS(100));
    motor_setA(-0.5);
    motor_setB(-0.5);
    motor_setC(-0.5);
    vTaskDelay(pdMS_TO_TICKS(120));
    motor_setA(0);
    motor_setB(0);
    motor_setC(0);

    while (1)
    {
        if (status.mode == MODE_FPS)
        {
            omni_drive_fps(status.set_speed.v, status.set_speed.yaw);
        }
        else
        {
            // omni_drive_tps(status.target_speed.x, status.target_speed.y, status.target_speed.yaw, status.target_position.yaw);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#else
// bb8
#endif

// some exception like picked up, battery low, just set a error flag
static void monitor_task(void *pvParameters)
{
    while (1)
    {
        // if (pitch > 30 || roll > 30)
        // {
        //     ESP_LOGI(TAG, "Picked up!");
        //     pickup = true;
        // }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));

    // while (1)
    // {
    //     ESP_LOGI(TAG, "duty1 ");
    //     vTaskDelay(pdMS_TO_TICKS(500));
    // }

    // xTaskCreate(remote_task, "remote_task", 2 * 4096, NULL, 2, NULL);
    xTaskCreate(sensor_task, "sensor_task", 2 * 4096, NULL, 1, NULL);
    // xTaskCreate(pixel_task, "pixel_task", 2 * 4096, NULL, 3, NULL);
    // xTaskCreate(motion_task, "motion_task", 1 * 4096, NULL, 1, NULL);
    // xTaskCreate(mixer_task, "mixer_task", 1 * 4096, NULL, 1, NULL);
    // xTaskCreate(monitor_task, "monitor_task", 1 * 4096, NULL, 1, NULL);
}