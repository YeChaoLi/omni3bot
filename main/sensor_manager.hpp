#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <memory>
#include <string>

// Forward declaration
class RobotStatus;

// Second-order low-pass filter structure
struct LPF2State
{
    float x1 = 0.0f, x2 = 0.0f;  // Input history
    float y1 = 0.0f, y2 = 0.0f;  // Output history
    float a1 = 0.0f, a2 = 0.0f;  // Denominator coefficients
    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;  // Numerator coefficients
};

// IMU Sensor Manager
class SensorManager
{
public:
    // Configuration constants
    static constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_11;
    static constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_12;
    static constexpr int I2C_PORT = I2C_NUM_0;
    static constexpr int I2C_FREQ_HZ = 400000;
    static constexpr int TASK_STACK_SIZE = 4096;
    static constexpr int TASK_PRIORITY = 5;
    static constexpr int TASK_UPDATE_RATE_MS = 1;  // 1ms = 1000Hz
    
    // Sensor configuration
    static constexpr float ACCEL_RANGE_G = 8.0f;
    static constexpr float GYRO_RANGE_DPS = 1024.0f;
    static constexpr float ACCEL_ODR_HZ = 2000.0f;
    static constexpr float GYRO_ODR_HZ = 2000.0f;
    static constexpr float SAMPLING_RATE_HZ = 1000.0f;
    
    // Filter configuration
    static constexpr float ACCEL_LPF_CUTOFF_HZ = 10.0f;
    static constexpr float GYRO_LPF_CUTOFF_HZ = 50.0f;
    static constexpr float MADGWICK_BETA = 0.1f;
    
    // Constructor
    SensorManager(RobotStatus& robot_status);
    
    // Destructor
    ~SensorManager();
    
    // Initialize sensor subsystem
    esp_err_t initialize();
    
    // Start sensor task
    esp_err_t start();
    
    // Stop sensor task
    void stop();
    
    // Check if initialized
    bool is_initialized() const { return initialized_; }
    
    // Get sensor status string
    std::string get_sensor_status() const;
    
    // Manual sensor update (for testing)
    esp_err_t update_sensors();
    
    // Calibration functions
    esp_err_t calibrate_accelerometer();
    esp_err_t calibrate_gyroscope();
    
    // Get raw sensor data
    struct RawSensorData {
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        uint64_t timestamp;
    };
    
    RawSensorData get_raw_data() const { return raw_data_; }

private:
    // Robot status reference
    RobotStatus& robot_status_;
    
    // Sensor state
    bool initialized_ = false;
    bool task_running_ = false;
    
    // I2C handles
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    
    // Sensor device handle (QMI8658)
    void* sensor_device_ = nullptr;  // Will be cast to appropriate type
    
    // Raw sensor data
    RawSensorData raw_data_;
    
    // Task handle
    TaskHandle_t task_handle_ = nullptr;
    
    // Filter states
    LPF2State accel_lpf_x_, accel_lpf_y_, accel_lpf_z_;
    LPF2State gyro_lpf_x_, gyro_lpf_y_, gyro_lpf_z_;
    
    // Calibration data
    struct CalibrationData {
        float accel_bias_x = 0.0f, accel_bias_y = 0.0f, accel_bias_z = 0.0f;
        float gyro_bias_x = -0.002182f, gyro_bias_y = 0.047997f, gyro_bias_z = -0.019090f;
        float accel_scale_x = 1.0f, accel_scale_y = 1.0f, accel_scale_z = 1.0f;
        float gyro_scale_x = 0.5f, gyro_scale_y = 0.5f, gyro_scale_z = 0.5f;
    };
    
    CalibrationData calibration_;
    
    // Private methods
    static void sensor_task_wrapper(void* param);
    void sensor_task_main();
    
    // I2C initialization
    esp_err_t init_i2c();
    void deinit_i2c();
    
    // Sensor initialization
    esp_err_t init_qmi8658();
    void deinit_qmi8658();
    
    // Filter functions
    void init_lpf2_filters();
    void lpf2_init(LPF2State& filter, float cutoff_freq, float sampling_rate);
    float lpf2_apply(LPF2State& filter, float input);
    
    // Data processing
    void process_sensor_data();
    void apply_calibration();
    void apply_filters();
    void update_attitude();
    
    // Madgwick filter integration
    void init_madgwick_filter();
    void update_madgwick_filter(float dt, float ax, float ay, float az, float gx, float gy, float gz);
    void get_madgwick_attitude(float& roll, float& pitch, float& yaw);
    
    // Utility functions
    float get_time_delta_ms();
    void update_timestamp();
};
