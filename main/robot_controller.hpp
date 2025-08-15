#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "madgwick_filter.hpp"  // Include Madgwick filter
#include "pid_ctrl.h"           // Include ESP-IDF PID controller

// Matrix transformation utilities for 2D coordinate manipulation
// Using Eigen library for efficient matrix operations instead of manual sin/cos calculations
// Benefits:
// - More readable and maintainable code
// - Optimized matrix operations
// - Better numerical stability
// - Easier to extend for more complex transformations
extern "C"
{
#include "qmi8658.h"
#include "led_strip.h"
#include "esp_hidh.h"
#include "esp_hid_gap.h"
#include "driver/ledc.h"
}

// Matrix transformation utilities for 2D coordinate manipulation
namespace MatrixUtils
{
    // Create 2D rotation matrix
    inline Eigen::Matrix2f rotation_matrix(float angle)
    {
        return Eigen::Rotation2Df(angle).toRotationMatrix();
    }

    // Create 2D translation vector
    inline Eigen::Vector2f translation_vector(float x, float y)
    {
        return Eigen::Vector2f(x, y);
    }

    // Apply rotation to a 2D vector
    inline Eigen::Vector2f rotate_vector(const Eigen::Vector2f &vec, float angle)
    {
        return rotation_matrix(angle) * vec;
    }

    // Transform from world coordinates to body coordinates
    inline Eigen::Vector2f world_to_body(const Eigen::Vector2f &world_vec, float yaw)
    {
        return rotation_matrix(yaw) * world_vec;
    }

    // Transform from body coordinates to world coordinates
    inline Eigen::Vector2f body_to_world(const Eigen::Vector2f &body_vec, float yaw)
    {
        return rotation_matrix(-yaw) * body_vec;
    }

    // Create wheel kinematics matrix for omni drive
    inline Eigen::Matrix3f create_wheel_kinematics_matrix(float beta_a, float beta_b, float beta_c)
    {
        Eigen::Matrix3f H;
        // Each row represents the contribution of (vx, vy, omega) to each wheel
        // Swapped X and Y components so rate_x > 0 moves right, rate_y > 0 moves forward
        H.row(0) = Eigen::Vector3f(cosf(beta_a), -sinf(beta_a), -1.0f); // Wheel A
        H.row(1) = Eigen::Vector3f(cosf(beta_b), -sinf(beta_b), -1.0f); // Wheel B
        H.row(2) = Eigen::Vector3f(cosf(beta_c), -sinf(beta_c), -1.0f); // Wheel C
        return H;
    }

    // Compute wheel speeds from body velocities using matrix operations
    inline Eigen::Vector3f compute_wheel_speeds(const Eigen::Vector3f &body_velocities,
                                                const Eigen::Matrix3f &wheel_matrix,
                                                float wheel_distance)
    {
        Eigen::Vector3f scaled_velocities = body_velocities;
        scaled_velocities(2) *= wheel_distance / 100.0f; // Scale yaw rate by wheel distance
        return wheel_matrix * scaled_velocities;
    }
}

#define USE_IMU
#define USE_RGB
#define USE_REMOTE
#define USE_CRSF
#define RED (1)
#define BLACK (2)
#define USE_OMNI3
#define TARGET (BLACK)

static const char *TAG = "O3";

enum class Mode
{
    FPS = 0,
    TPS,
};

// Position data structure
struct Position
{
    float x = 0.0f, y = 0.0f, z = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
};

// Speed data structure
struct Speed
{
    float v = 0.0f; // fps
    float x = 0.0f, y = 0.0f, z = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
};

struct Accelleration
{
    float v = 0.0f; // fps
    float x = 0.0f, y = 0.0f, z = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
};

struct Attitude
{
    Position position;
    Speed speed;
    Accelleration accel;
};

// Remote control data structure
struct RemoteControl
{
    float x = 0.0f, y = 0.0f, theta = 0.0f;
    bool button_a = false, button_b = false, button_c = false, button_d = false;
    float roll_rate, pitch_rate, throttle, yaw_pos;
};

// Sensor data structure (processed from raw)
struct SensorData
{
    time_t timestamp = 0;
    float acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
    uint32_t sensor_updated = 0;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;  // Euler angles in radians
};

// Main status class
class RobotStatus
{
public:
    Mode mode = Mode::FPS;

    RemoteControl remote;
    float throttle = 0.0f;

    SensorData sensor_data;
    Attitude current_attitude;
    Attitude target_attitude;   // mixer will pick members as needed
    bool imu_ready = false;
};

// Global status instance
extern RobotStatus status;

#ifdef USE_IMU
// Second-order low-pass filter structure
struct LPF2State
{
    float x1 = 0.0f, x2 = 0.0f;  // Input history
    float y1 = 0.0f, y2 = 0.0f;  // Output history
    float a1 = 0.0f, a2 = 0.0f;  // Denominator coefficients
    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;  // Numerator coefficients
};

class IMUManager
{
private:
    static constexpr gpio_num_t I2C_MASTER_SDA_IO = GPIO_NUM_11;
    static constexpr gpio_num_t I2C_MASTER_SCL_IO = GPIO_NUM_12;
    static constexpr int I2C_MASTER_NUM = I2C_NUM_0;
    static constexpr int I2C_MASTER_FREQ_HZ = 400000;
    static constexpr float SAMPLING_RATE = 1000.0f; // Hz, based on ODR setting

    i2c_master_bus_handle_t bus_handle_;
    qmi8658_dev_t dev_;
    qmi8658_data_t raw_data_;
    bool initialized_ = false;
    
    float accel_bias_x_ = 0.0f, accel_bias_y_ = 0.0f, accel_bias_z_ = 0.0f;
    float gyro_bias_x_ = -0.002182f, gyro_bias_y_ = 0.047997f, gyro_bias_z_ = -0.019090f;

    float accel_scale_x_ = 1.0f, accel_scale_y_ = 1.0f, accel_scale_z_ = 1.0f;
    // float gyro_scale_x_ = 1.0f, gyro_scale_y_ = 1.0f, gyro_scale_z_ = 1.0f;
    float gyro_scale_x_ = 0.501945f, gyro_scale_y_ = 0.501945f, gyro_scale_z_ = 0.501945f;

    // Low-pass filter states for each sensor axis
    LPF2State accel_lpf_x_, accel_lpf_y_, accel_lpf_z_;
    LPF2State gyro_lpf_x_, gyro_lpf_y_, gyro_lpf_z_;

    // Madgwick filter for attitude estimation
    espp::MadgwickFilter madgwick_filter_;
    
    // Timing for Madgwick filter
    uint32_t last_update_time_ms_ = 0;

    // Private helper functions
    void lpf2_init(LPF2State& filter, float cutoff_freq, float sampling_rate);
    float lpf2_apply(LPF2State& filter, float input);
    void update_attitude(float ax, float ay, float az, float gx, float gy, float gz);

public:
    IMUManager();
    void calibrate();
    esp_err_t initialize();
    void update();
    bool is_initialized() const { return initialized_; }
    
    // Get current attitude from Madgwick filter
    void get_attitude(float& roll, float& pitch, float& yaw) const;
};
#endif

#ifdef USE_RGB
class LEDStripManager
{
private:
    static constexpr bool LED_STRIP_USE_DMA = false;
    static constexpr int LED_STRIP_LED_COUNT = 64;
    static constexpr int LED_STRIP_MEMORY_BLOCK_WORDS = 0;
    static constexpr int LED_STRIP_GPIO_PIN = 14;
    static constexpr int LED_STRIP_RMT_RES_HZ = 10 * 1000 * 1000;

    led_strip_handle_t led_strip_;
    bool initialized_ = false;

public:
    esp_err_t initialize();
    void set_pattern(int j);
    void clear();
    bool is_initialized() const { return initialized_; }
};
#endif

#ifdef USE_REMOTE
class RemoteController
{
private:
    static constexpr float JOYSTICK_MAX_RAW_VALUE = 15.0f;
    static constexpr int JOYSTICK_DEADZONE = 3;
    static constexpr float ANGLE_DECREMENT_C = 15.0f;
    static constexpr float ANGLE_INCREMENT_D = 15.0f;
    static constexpr int SCAN_DURATION_SECONDS = 5;

public:
    void parse_and_map_hid_report(uint8_t report_id, const uint8_t *data, int length);
    void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
    void scan_and_connect();
    esp_err_t initialize();
};
#endif

// PID Controller is now defined in pid_controller.hpp

class MotionController
{
private:
    pid_ctrl_block_handle_t yaw_pid_controller_;
    bool pid_initialized_;

public:
    MotionController() : yaw_pid_controller_(nullptr), pid_initialized_(false) {
        // PID controller will be initialized in initialize() method
    }

    ~MotionController() {
        if (yaw_pid_controller_) {
            pid_del_control_block(yaw_pid_controller_);
        }
    }

    esp_err_t initialize();
    void update();
    
    void set_yaw_pid_gains(float kp, float ki, float kd);
    
    // Get access to PID controller for external use
    pid_ctrl_block_handle_t get_yaw_pid_controller() const { return yaw_pid_controller_; }
};

#ifdef USE_OMNI3
class PWMController
{
private:
    static constexpr int PWM_FREQ_HZ = 50;
    static constexpr int PWM_RESOLUTION = LEDC_TIMER_13_BIT;
    static constexpr int PWM_DUTY_MAX = 8192;

public:
    void initialize_channel(ledc_channel_t channel, int gpio_pin);
    void set_duty(ledc_channel_t channel, float percent);
};

class OmniDriveMixer
{
private:
    PWMController pwm_controller_;
    static constexpr float L = 50.0f; // mm
    static constexpr float BETA_A = -M_PI / 3.0f;  // -60° (left front)
    static constexpr float BETA_B = -M_PI;          // -180° (backward)
    static constexpr float BETA_C = M_PI / 3.0f;    // +60° (right front)

    // Pre-computed wheel kinematics matrix for efficiency
    Eigen::Matrix3f wheel_kinematics_matrix_;

public:
    void initialize();
    void motor_setA(float throttle);
    void motor_setB(float throttle);
    void motor_setC(float throttle);
    void omni_drive_fps(float v, float rate_yaw);
    void omni_drive_tps(float rate_x, float rate_y, float rate_yaw, float yaw);
    void update();
};
#endif

class SystemMonitor
{
public:
    void update();
};

class DebugLogger
{
public:
    void log_status();
    void test_eigen_implementation();
};

// Task function declarations
void sensor_task(void *pvParameters);
void pixel_task(void *pvParameters);
void motion_task(void *pvParameters);
void mixer_task(void *pvParameters);
void monitor_task(void *pvParameters);
void debug_task(void *pvParameters);
void ble_task(void *pvParameters);
void radio_task(void *pvParameters);