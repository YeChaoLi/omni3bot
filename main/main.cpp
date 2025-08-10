#include "robot_controller.hpp"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>
#include <string.h>
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
#include <math.h>
#include <float.h>
#include <memory>
#include <vector>
#include <functional>
extern "C"
{
#include "ESP_CRSF.h"
}

#ifdef TAG
#undef TAG
#endif
#define TAG "omni3"

// Global status instance
RobotStatus status;

#ifdef USE_IMU

IMUManager::IMUManager()
{
    // Initialize accelerometer filters with 10Hz cutoff
    lpf2_init(accel_lpf_x_, 10.0f, SAMPLING_RATE);
    lpf2_init(accel_lpf_y_, 10.0f, SAMPLING_RATE);
    lpf2_init(accel_lpf_z_, 10.0f, SAMPLING_RATE);

    // Initialize gyroscope filters with 50Hz cutoff
    lpf2_init(gyro_lpf_x_, 50.0f, SAMPLING_RATE);
    lpf2_init(gyro_lpf_y_, 50.0f, SAMPLING_RATE);
    lpf2_init(gyro_lpf_z_, 50.0f, SAMPLING_RATE);
}

void IMUManager::lpf2_init(LPF2State &filter, float cutoff_freq, float sampling_rate)
{
    // Second-order Butterworth low-pass filter design
    // Calculate normalized frequency (0 to 1, where 1 = Nyquist frequency)
    float omega = 2.0f * M_PI * cutoff_freq / sampling_rate;
    float sin_omega = sinf(omega);
    float cos_omega = cosf(omega);
    float alpha = sin_omega / (2.0f * 0.707f); // Q = 0.707 for Butterworth

    // Calculate filter coefficients
    float a0 = 1.0f + alpha;
    filter.a1 = -2.0f * cos_omega / a0;
    filter.a2 = (1.0f - alpha) / a0;

    filter.b0 = (1.0f - cos_omega) / (2.0f * a0);
    filter.b1 = (1.0f - cos_omega) / a0;
    filter.b2 = (1.0f - cos_omega) / (2.0f * a0);

    // Initialize state variables to zero
    filter.x1 = filter.x2 = 0.0f;
    filter.y1 = filter.y2 = 0.0f;
}

float IMUManager::lpf2_apply(LPF2State &filter, float input)
{
    // Apply second-order IIR filter: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float output = filter.b0 * input + filter.b1 * filter.x1 + filter.b2 * filter.x2 - filter.a1 * filter.y1 - filter.a2 * filter.y2;

    // Update state variables
    filter.x2 = filter.x1;
    filter.x1 = input;
    filter.y2 = filter.y1;
    filter.y1 = output;

    return output;
}

esp_err_t IMUManager::initialize()
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing I2C...");

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true}};

    ret = i2c_new_master_bus(&bus_config, &bus_handle_);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C (error: %d)", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Initializing QMI8658...");
    ret = qmi8658_init(&dev_, bus_handle_, QMI8658_ADDRESS_HIGH);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize QMI8658 (error: %d)", ret);
        return ret;
    }

    // Configure sensor
    // you can set 1kHz ODR if use interrupt io
    qmi8658_set_accel_range(&dev_, QMI8658_ACCEL_RANGE_8G);
    qmi8658_set_accel_odr(&dev_, QMI8658_ACCEL_ODR_2000HZ);
    qmi8658_set_gyro_range(&dev_, QMI8658_GYRO_RANGE_1024DPS);
    qmi8658_set_gyro_odr(&dev_, QMI8658_GYRO_ODR_2000HZ);
    qmi8658_set_accel_unit_mps2(&dev_, true);
    qmi8658_set_gyro_unit_rads(&dev_, true);
    qmi8658_set_display_precision(&dev_, 4);

    initialized_ = true;
    return ESP_OK;
}

void IMUManager::update()
{
    if (!initialized_)
        return;

    bool ready;
    esp_err_t ret = qmi8658_is_data_ready(&dev_, &ready);
    if (ret == ESP_OK && ready)
    {
        ret = qmi8658_read_sensor_data(&dev_, &raw_data_);
        if (ret == ESP_OK)
        {
            // ESP_LOGI(TAG, "Accel: %f %f %f", raw_data_.accelX, raw_data_.accelY, raw_data_.accelZ);
            // ESP_LOGI(TAG, "Gyro: %f %f %f", raw_data_.gyroX, raw_data_.gyroY, raw_data_.gyroZ);
            status.sensor_data.timestamp = raw_data_.timestamp;
            status.sensor_data.sensor_updated++;

            // minus bias & scale factor
            float accelX = (raw_data_.accelX - accel_bias_x_) * accel_scale_x_;
            float accelY = (raw_data_.accelY - accel_bias_y_) * accel_scale_y_;
            float accelZ = (raw_data_.accelZ - accel_bias_z_) * accel_scale_z_;
            float gyroX = (raw_data_.gyroX - gyro_bias_x_) * gyro_scale_x_;
            float gyroY = (raw_data_.gyroY - gyro_bias_y_) * gyro_scale_y_;
            float gyroZ = (raw_data_.gyroZ - gyro_bias_z_) * gyro_scale_z_;

            // 2nd order low-pass filter
            // Gyro: 50Hz. Accel: 10Hz
            // status.sensor_data.acc_x = lpf2_apply(accel_lpf_x_, accelX);
            // status.sensor_data.acc_y = lpf2_apply(accel_lpf_y_, accelY);
            // status.sensor_data.acc_z = lpf2_apply(accel_lpf_z_, accelZ);
            // status.sensor_data.gyro_x = lpf2_apply(gyro_lpf_x_, gyroX);
            // status.sensor_data.gyro_y = lpf2_apply(gyro_lpf_y_, gyroY);
            // status.sensor_data.gyro_z = lpf2_apply(gyro_lpf_z_, gyroZ);
            status.sensor_data.acc_x = -accelX;
            status.sensor_data.acc_y = -accelY;
            status.sensor_data.acc_z = -accelZ;
            status.sensor_data.gyro_x = -gyroX;
            status.sensor_data.gyro_y = -gyroY;
            status.sensor_data.gyro_z = -gyroZ;

            // status.sensor_data.yaw += status.sensor_data.gyro_z * 180.0f / M_PI * 0.001f;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read sensor data (error: %d)", ret);
        }
    }
}

void sensor_task(void *pvParameters)
{
    auto imu = std::make_unique<IMUManager>();
    esp_err_t ret = imu->initialize();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize IMU");
        vTaskDelete(nullptr);
        return;
    }

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1; // ms
    xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        imu->update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#endif

#ifdef USE_RGB

esp_err_t LEDStripManager::initialize()
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,
        .max_leds = LED_STRIP_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        }};

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .mem_block_symbols = LED_STRIP_MEMORY_BLOCK_WORDS,
        .flags = {
            .with_dma = LED_STRIP_USE_DMA,
        }};

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Created LED strip object with RMT backend");
        initialized_ = true;
    }
    return ret;
}

void LEDStripManager::set_pattern(int j)
{
    if (!initialized_)
        return;

    for (int i = 0; i < LED_STRIP_LED_COUNT; i++)
    {
        if (i % 3 == 0)
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip_, i, j, 0, 0));
        }
        else if (i % 3 == 1)
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip_, i, 0, j, 0));
        }
        else
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip_, i, 0, 0, j));
        }
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strip_));
}

void LEDStripManager::clear()
{
    if (initialized_)
    {
        ESP_ERROR_CHECK(led_strip_clear(led_strip_));
    }
}

void pixel_task(void *pvParameters)
{
    auto led_strip = std::make_unique<LEDStripManager>();
    esp_err_t ret = led_strip->initialize();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize LED strip");
        vTaskDelete(nullptr);
        return;
    }

    bool led_on_off = false;
    ESP_LOGI(TAG, "Start blinking LED strip");
    int j = 0;

    while (true)
    {
        j = (j + 1) % 10;

        if (led_on_off)
        {
            led_strip->set_pattern(j);
        }
        else
        {
            led_strip->clear();
        }

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif

#ifdef USE_REMOTE

void RemoteController::parse_and_map_hid_report(uint8_t report_id, const uint8_t *data, int length)
{
    if (report_id == 2 && length >= 3)
    {
        int8_t raw_phys_y = static_cast<int8_t>(data[1]);
        int8_t raw_phys_x = static_cast<int8_t>(data[2]);

        // Apply Deadzone
        if (abs(raw_phys_y) < JOYSTICK_DEADZONE)
        {
            raw_phys_y = 0;
        }
        if (abs(raw_phys_x) < JOYSTICK_DEADZONE)
        {
            raw_phys_x = 0;
        }

        // Normalize the raw values to a float between -1.0 and 1.0
        status.remote.x = static_cast<float>(raw_phys_y) / JOYSTICK_MAX_RAW_VALUE;
        status.remote.y = static_cast<float>(raw_phys_x) / JOYSTICK_MAX_RAW_VALUE;

        // Clamp values to ensure they are within [-1.0, 1.0]
        status.remote.x = std::clamp(status.remote.x, -1.0f, 1.0f);
        status.remote.y = std::clamp(status.remote.y, -1.0f, 1.0f);
    }
}

void RemoteController::hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = static_cast<esp_hidh_event_t>(id);
    esp_hidh_event_data_t *param = static_cast<esp_hidh_event_data_t *>(event_data);

    switch (event)
    {
    case ESP_HIDH_OPEN_EVENT:
        if (param->open.status == ESP_OK)
        {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            if (bda)
            {
                ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x OPEN: %s",
                         bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
                         esp_hidh_dev_name_get(param->open.dev));
                esp_hidh_dev_dump(param->open.dev, stdout);
            }
        }
        else
        {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;

    case ESP_HIDH_BATTERY_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        if (bda)
        {
            ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x BATTERY: %d%%",
                     bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
                     param->battery.level);
        }
        break;
    }

    case ESP_HIDH_INPUT_EVENT:
        parse_and_map_hid_report(param->input.report_id, param->input.data, param->input.length);
        break;

    case ESP_HIDH_FEATURE_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        if (bda)
        {
            ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d",
                     bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
                     esp_hid_usage_str(param->feature.usage), param->feature.map_index,
                     param->feature.report_id, param->feature.length);
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        }
        break;
    }

    case ESP_HIDH_CLOSE_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        if (bda)
        {
            ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x CLOSE: %s",
                     bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
                     esp_hidh_dev_name_get(param->close.dev));
        }
        break;
    }

    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

void RemoteController::scan_and_connect()
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = nullptr;
    ESP_LOGI(TAG, "SCAN...");

    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);

    if (results_len)
    {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = nullptr;

        while (r)
        {
            printf("  %s: %02x:%02x:%02x:%02x:%02x:%02x, ",
                   (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ",
                   r->bda[0], r->bda[1], r->bda[2], r->bda[3], r->bda[4], r->bda[5]);
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));

            if (r->transport == ESP_HID_TRANSPORT_BLE)
            {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%d', ", r->ble.addr_type);
            }

            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }

        if (cr)
        {
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }

        esp_hid_scan_results_free(results);
    }
}

esp_err_t RemoteController::initialize()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));

    esp_hidh_config_t config = {
        .callback = [](void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
        {
            static_cast<RemoteController *>(handler_args)->hidh_callback(handler_args, base, id, event_data);
        },
        .event_stack_size = 4096,
        .callback_arg = this,
    };
    ESP_ERROR_CHECK(esp_hidh_init(&config));

    return ESP_OK;
}

void ble_task(void *pvParameters)
{
    auto remote = std::make_unique<RemoteController>();
    esp_err_t ret = remote->initialize();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize remote controller");
        vTaskDelete(nullptr);
        return;
    }

    remote->scan_and_connect();
    vTaskDelete(nullptr);
}

#endif

void MotionController::update()
{
    if (status.mode == Mode::FPS)
    {
        // New Control Logic
        status.target_attitude.speed.v = status.remote.x * 0.70f;
        status.target_attitude.speed.yaw = -status.remote.y * 0.60f;
    }
    else if (status.mode == Mode::TPS)
    {
        if (abs(status.remote.yaw_pos) < 0.9f)
        {
            status.target_attitude.position.yaw = -status.remote.yaw_pos * 3.14f;
        }
        // if you wanna adjust the heading, just put your yaw stick in extream edge
        else
        {
            status.target_attitude.speed.yaw = -status.remote.yaw_pos * 0.1f;
        }

        status.target_attitude.speed.x = status.remote.roll_rate;
        status.target_attitude.speed.y = -status.remote.pitch_rate;
        status.throttle = status.remote.throttle;
    }
}

void OmniDriveMixer::update()
{
    if (status.mode == Mode::FPS)
    {
        omni_drive_fps(status.target_attitude.speed.v, status.target_attitude.speed.yaw);
    }
    else
    {
        omni_drive_tps(status.target_attitude.speed.x, status.target_attitude.speed.y,
                       status.target_attitude.speed.yaw, status.current_attitude.position.yaw);
    }
}

void motion_task(void *pvParameters)
{
    status.mode = Mode::TPS;

    auto motion_controller = std::make_unique<MotionController>();
    auto omnidrive_mixer = std::make_unique<OmniDriveMixer>();

    omnidrive_mixer->initialize();

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10; // ms
    xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        // Update motion controller (processes remote input and sets target speeds)
        motion_controller->update();

        // run the speed or throttle to each motor
        // change this to adjust multi type (bb8)
        omnidrive_mixer->update();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#ifdef USE_OMNI3

void PWMController::initialize_channel(ledc_channel_t channel, int gpio_pin)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = static_cast<ledc_timer_bit_t>(PWM_RESOLUTION),
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {
        .gpio_num = gpio_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

void PWMController::set_duty(ledc_channel_t channel, float percent)
{
    // Map duty from -1.0~1.0 to 0.05~0.1. 0.15 is the middle.
    float duty = 0.05f + (percent + 1.0f) * 0.025f;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty * PWM_DUTY_MAX));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

void OmniDriveMixer::initialize()
{
    pwm_controller_.initialize_channel(LEDC_CHANNEL_0, 4);
    pwm_controller_.initialize_channel(LEDC_CHANNEL_1, 5);
    pwm_controller_.initialize_channel(LEDC_CHANNEL_2, 6);

    // Pre-compute wheel kinematics matrix for efficiency
    wheel_kinematics_matrix_ = MatrixUtils::create_wheel_kinematics_matrix(BETA_A, BETA_B, BETA_C);

    // Initialize all motors to zero
    pwm_controller_.set_duty(LEDC_CHANNEL_0, 0.0f);
    pwm_controller_.set_duty(LEDC_CHANNEL_1, 0.0f);
    pwm_controller_.set_duty(LEDC_CHANNEL_2, 0.0f);

    vTaskDelay(pdMS_TO_TICKS(500));

    // Test sequence
    pwm_controller_.set_duty(LEDC_CHANNEL_0, 0.3f);
    pwm_controller_.set_duty(LEDC_CHANNEL_1, 0.3f);
    pwm_controller_.set_duty(LEDC_CHANNEL_2, 0.3f);
    vTaskDelay(pdMS_TO_TICKS(200));

    pwm_controller_.set_duty(LEDC_CHANNEL_0, -0.3f);
    pwm_controller_.set_duty(LEDC_CHANNEL_1, -0.3f);
    pwm_controller_.set_duty(LEDC_CHANNEL_2, -0.3f);
    vTaskDelay(pdMS_TO_TICKS(200));

    pwm_controller_.set_duty(LEDC_CHANNEL_0, 0.0f);
    pwm_controller_.set_duty(LEDC_CHANNEL_1, 0.0f);
    pwm_controller_.set_duty(LEDC_CHANNEL_2, 0.0f);
}

void OmniDriveMixer::motor_setA(float throttle)
{
#if (TARGET == RED)
    pwm_controller_.set_duty(LEDC_CHANNEL_0, -throttle);
#else
    pwm_controller_.set_duty(LEDC_CHANNEL_0, throttle);
#endif
}

void OmniDriveMixer::motor_setB(float throttle)
{
#if (TARGET == RED)
    pwm_controller_.set_duty(LEDC_CHANNEL_1, throttle);
#else
    pwm_controller_.set_duty(LEDC_CHANNEL_2, -throttle);
#endif
}

void OmniDriveMixer::motor_setC(float throttle)
{
#if (TARGET == RED)
    pwm_controller_.set_duty(LEDC_CHANNEL_2, -throttle);
#else
    pwm_controller_.set_duty(LEDC_CHANNEL_1, throttle);
#endif
}

void OmniDriveMixer::omni_drive_fps(float v, float rate_yaw)
{
    // Create body velocity vector (vx, vy, omega)
    Eigen::Vector3f body_velocities(v, 0.0f, rate_yaw);

    // Compute wheel speeds using pre-computed kinematics matrix
    Eigen::Vector3f wheel_speeds = MatrixUtils::compute_wheel_speeds(body_velocities, wheel_kinematics_matrix_, L);

    // Extract individual wheel speeds
    float mA = wheel_speeds(0);
    float mB = wheel_speeds(1);
    float mC = wheel_speeds(2);

    // Normalize if any |m| exceeds 1.0
    float maxm = std::max(std::abs(mA), std::max(std::abs(mB), std::abs(mC)));
    if (maxm > 1.0f)
    {
        mA /= maxm;
        mB /= maxm;
        mC /= maxm;
    }

    motor_setA(mA);
    motor_setB(mB);
    motor_setC(mC);
}

void OmniDriveMixer::omni_drive_tps(float rate_x, float rate_y, float rate_yaw, float yaw)
{
    // Create world frame velocity vector
    Eigen::Vector2f velocity_world = MatrixUtils::translation_vector(rate_x, rate_y);

    // Transform world→body using rotation matrix
    Eigen::Vector2f velocity_body = MatrixUtils::world_to_body(velocity_world, yaw);

    // Create body velocity vector (vx, vy, omega)
    Eigen::Vector3f body_velocities(velocity_body(0), velocity_body(1), rate_yaw);

    // Compute wheel speeds using pre-computed kinematics matrix
    Eigen::Vector3f wheel_speeds = MatrixUtils::compute_wheel_speeds(body_velocities, wheel_kinematics_matrix_, L);

    // Extract individual wheel speeds
    float mA = wheel_speeds(0);
    float mB = wheel_speeds(1);
    float mC = wheel_speeds(2);

    // Normalize if any |m| > 1
    float maxm = std::max(std::abs(mA), std::max(std::abs(mB), std::abs(mC)));
    if (maxm > 1.0f)
    {
        mA /= maxm;
        mB /= maxm;
        mC /= maxm;
    }

    motor_setA(mA);
    motor_setB(mB);
    motor_setC(mC);
}

#else
// bb8 implementation would go here
#endif

void SystemMonitor::update()
{
    // Monitor system status, battery, etc.
}

void monitor_task(void *pvParameters)
{
    auto monitor = std::make_unique<SystemMonitor>();

    while (true)
    {
        monitor->update();
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust frequency as needed
    }
}

void DebugLogger::log_status()
{
    ESP_LOGI("Sensor", "Gyro: %f, %f, %f. Accel: %f, %f, %f.", status.sensor_data.gyro_x, status.sensor_data.gyro_y, status.sensor_data.gyro_z, status.sensor_data.acc_x, status.sensor_data.acc_y, status.sensor_data.acc_z);
}

void DebugLogger::test_eigen_implementation()
{
    // Test rotation matrix creation
    Eigen::Matrix2f rot_90 = MatrixUtils::rotation_matrix(M_PI / 2.0f);
    ESP_LOGI(TAG, "Rotation 90° matrix: [[%f, %f], [%f, %f]]",
             rot_90(0, 0), rot_90(0, 1), rot_90(1, 0), rot_90(1, 1));

    // Test vector rotation
    Eigen::Vector2f vec(1.0f, 0.0f);
    Eigen::Vector2f rotated = MatrixUtils::rotate_vector(vec, M_PI / 2.0f);
    ESP_LOGI(TAG, "Vector (1,0) rotated 90°: (%f, %f)", rotated(0), rotated(1));

    // Test world to body transformation
    Eigen::Vector2f world_vel(1.0f, 1.0f);
    Eigen::Vector2f body_vel = MatrixUtils::world_to_body(world_vel, M_PI / 4.0f);
    ESP_LOGI(TAG, "World velocity (1,1) to body (45°): (%f, %f)", body_vel(0), body_vel(1));

    // Test wheel kinematics matrix
    Eigen::Matrix3f wheel_matrix = MatrixUtils::create_wheel_kinematics_matrix(-M_PI / 3, M_PI / 3, M_PI);
    ESP_LOGI(TAG, "Wheel kinematics matrix created successfully");
    ESP_LOGI(TAG, "Matrix determinant: %f", wheel_matrix.determinant());
}

void debug_task(void *pvParameters)
{
    auto logger = std::make_unique<DebugLogger>();
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 200; // ms
    xLastWakeTime = xTaskGetTickCount();

    // Run Eigen test once at startup
    logger->test_eigen_implementation();

    while (true)
    {
        logger->log_status();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#ifdef USE_CRSF
void radio_task(void *pvParameters)
{
    ESP_LOGI(TAG, "########### Starting radio task ###########");

    // CRSF configuration
    crsf_config_t crsf_config = {
        .uart_num = CONFIG_CRSF_UART_NUM,
        .tx_pin = CONFIG_CRSF_TX_PIN,
        .rx_pin = CONFIG_CRSF_RX_PIN};

    // Initialize CRSF
    CRSF_init(&crsf_config);

    // Channel data structure
    crsf_channels_t channels = {0};

    // Battery telemetry data
    crsf_battery_t battery = {0};

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10; // ms
    xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        // Receive channel data
        CRSF_receive_channels(&channels);

        // constrain channel values to [174, 1811]
        const float min_channel = 174.0f, mid_channel = 992.0f, max_channel = 1811.0f;
        float ch1_constrained = std::min(std::max(static_cast<float>(channels.ch1), min_channel), max_channel);
        float ch2_constrained = std::min(std::max(static_cast<float>(channels.ch2), min_channel), max_channel);
        float ch3_constrained = std::min(std::max(static_cast<float>(channels.ch3), min_channel), max_channel);
        float ch4_constrained = std::min(std::max(static_cast<float>(channels.ch4), min_channel), max_channel);

        // Map CRSF channels to robot control
        // Assuming standard mapping:
        // ch1: roll (left/right)
        // ch2: pitch (forward/backward)
        // ch3: throttle (power)
        // ch4: yaw (rotation)

        // Convert to normalized values (-1.0 to 1.0)
        float roll = (ch1_constrained - mid_channel) / (max_channel - mid_channel);
        float pitch = (ch2_constrained - mid_channel) / (max_channel - mid_channel);
        float throttle = (ch3_constrained - min_channel) / (max_channel - min_channel);
        float yaw = (ch4_constrained - mid_channel) / (max_channel - mid_channel);

        // Apply deadzone
        const float deadzone = 0.1f;
        if (std::abs(roll) < deadzone)
            roll = 0.0f;
        if (std::abs(pitch) < deadzone)
            pitch = 0.0f;
        if (std::abs(throttle) < deadzone)
            throttle = 0.0f;
        if (std::abs(yaw) < deadzone)
            yaw = 0.0f;

        // Update robot status with remote control data
        status.remote.roll_rate = roll;
        status.remote.pitch_rate = pitch;
        status.remote.throttle = throttle;
        status.remote.yaw_pos = yaw;

        // Send battery telemetry back to transmitter
        battery.voltage = 58;    // 12.0V * 10
        battery.current = 50;    // 5.0A * 10
        battery.capacity = 1000; // 1000mAh
        battery.remaining = 30;  // 80%

        CRSF_send_battery_data(CRSF_DEST_FC, &battery);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
#endif

extern "C" void app_main(void)
{
    // xTaskCreate(ble_task, "ble_task", 2 * 4096, nullptr, 1, nullptr);
    // xTaskCreate(radio_task, "radio_task", 1 * 4096, nullptr, 1, nullptr);
    xTaskCreate(sensor_task, "sensor_task", 2 * 4096, nullptr, 1, nullptr);
    // xTaskCreate(pixel_task, "pixel_task", 2 * 4096, nullptr, 3, nullptr);
    // xTaskCreate(motion_task, "motion_task", 2 * 4096, nullptr, 1, nullptr);
    // xTaskCreate(monitor_task, "monitor_task", 1 * 4096, nullptr, 1, nullptr);
    xTaskCreate(debug_task, "debug_task", 1 * 4096, nullptr, 1, nullptr);
}