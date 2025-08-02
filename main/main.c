
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>
#include <string.h>
#include <qmi8658c.h>
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

    enum Mode mode;
    
} status;

#ifdef USE_IMU

static void sensor_task(void *pvParameters)
{
    i2c_dev_t dev = {0};

    ESP_ERROR_CHECK(i2cdev_init());

    ESP_ERROR_CHECK(qmi8658c_init_desc(&dev, 0x6B, 0, 11, 12));

    qmi8658c_config_t config = {
        .mode = QMI8658C_MODE_DUAL,
        .acc_scale = QMI8658C_ACC_SCALE_8G,
        .acc_odr = QMI8658C_ACC_ODR_1000,
        .gyro_scale = QMI8658C_GYRO_SCALE_512DPS,
        .gyro_odr = QMI8658C_GYRO_ODR_1000,
    };

    ESP_ERROR_CHECK(qmi8658c_setup(&dev, &config));
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1)
    {
        qmi8658c_data_t data;
        esp_err_t res = qmi8658c_read_data(&dev, &data);

        if (res == ESP_OK)
        {
            // ESP_LOGI(TAG, "Acc: x=%.3f y=%.3f z=%.3f | Gyro: x=%.3f y=%.3f z=%.3f | Temp: %.2f",
            //          data.acc.x, data.acc.y, data.acc.z,
            //          data.gyro.x, data.gyro.y, data.gyro.z,
            //          data.temperature);
        }
        else
        {
            ESP_LOGE(TAG, "Sensor read error: %s", esp_err_to_name(res));
        }
        vTaskDelay(pdMS_TO_TICKS(1));
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
            ESP_LOGI(TAG, "LED ON!");
        }
        else
        {
            /* Set all LED off to clear all pixels */
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
            ESP_LOGI(TAG, "LED OFF!");
        }

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif

#ifdef USE_REMOTE

static void remote_task(void *pvParameters)
{
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
    while (1)
    {
        if (status.mode == MODE_FPS)
        {
            /* code */
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
    ESP_LOGI(TAG, "duty 1 =  %f", duty);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty * 8192));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void pwm2_set(float percent)
{
    float duty = 0.05 + (percent + 1.0) * 0.025;
    ESP_LOGI(TAG, "duty 2 =  %f", duty);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty * 8192));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

static void pwm3_set(float percent)
{
    float duty = 0.05 + (percent + 1.0) * 0.025;
    ESP_LOGI(TAG, "duty 3 =  %f", duty);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty * 8192));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
}

static void motor_setA(float throttle)
{
#if (TARGET == 1)
    pwm1_set(throttle);
#else
    pwm1_set(-throttle);
#endif
}

static void motor_setB(float throttle)
{
#if (TARGET == 1)
    pwm2_set(-throttle);
#else
    pwm3_set(throttle);
#endif
}

static void motor_setC(float throttle)
{
#if (TARGET == 1)
    pwm3_set(throttle);
#else
    pwm2_set(-throttle);
#endif
}

// Geometry constant: robot radius (wheel distance to center) scaled
// so that ω_max * L ≈ 1.0.  You must tune this for your chassis.
#define L 50 // mm

// Wheel “angles” βi (location) relative to robot-forward (0 rad):
//   A at +60°, B at –60°, C at 180°.
#define BETA_A (M_PI / 3.0f)
#define BETA_B (-M_PI / 3.0f)
#define BETA_C (M_PI)

void omni_drive_fps(float v, float omega)
{
    // PROJECT body motion into each wheel’s drive direction.
    // For an omniwheel at βi, its drive axis is tangent at αi = βi + 90°.
    // Wheel speed =  (–sin αi)*v  +  ω * L
    //   = (–sin(βi + π/2))*v  +  ω * L
    //   = (–cos βi)*v         +  ω * L
    float mA = (-cosf(BETA_A)) * v + omega * L;
    float mB = (-cosf(BETA_B)) * v + omega * L;
    float mC = (-cosf(BETA_C)) * v + omega * L;

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
    motor_setA(mA);
    motor_setB(mB);
    motor_setC(mC);
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

    motor_setA(0.5);
    motor_setB(0.5);
    motor_setC(0.5);
    vTaskDelay(pdMS_TO_TICKS(30));
    motor_setA(-0.5);
    motor_setB(-0.5);
    motor_setC(-0.5);
    vTaskDelay(pdMS_TO_TICKS(30));
    motor_setA(0);
    motor_setB(0);
    motor_setC(0);

    while (1)
    {
        if (status.mode == MODE_FPS)
        {
            // omni_drive_fps(speed.v, speed.yaw);
        }
        else
        {
            // omni_drive_tps(speed.x, speed.y, speed.yaw, position.yaw);
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
    xTaskCreate(remote_task, "remote_task", 2 * 4096, NULL, 2, NULL);
    // xTaskCreate(sensor_task, "sensor_task", 1 * 4096, NULL, 1, NULL);
    // xTaskCreate(pixel_task, "pixel_task", 2 * 4096, NULL, 3, NULL);
    // xTaskCreate(motion_task, "motion_task", 1 * 4096, NULL, 1, NULL);
    // xTaskCreate(mixer_task, "mixer_task", 1 * 4096, NULL, 1, NULL);
    // xTaskCreate(monitor_task, "monitor_task", 1 * 4096, NULL, 1, NULL);
}