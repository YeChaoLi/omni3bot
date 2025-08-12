#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Eigen/Dense>
#include <memory>
#include <string>

// Forward declaration
class RobotStatus;

// PWM Controller for motor control
class PWMController
{
public:
    // Configuration constants
    static constexpr int PWM_FREQ_HZ = 50;
    static constexpr int PWM_RESOLUTION = LEDC_TIMER_13_BIT;
    static constexpr int PWM_DUTY_MAX = 8192;
    static constexpr int PWM_MIN_DUTY_PERCENT = 5;   // 5% minimum duty cycle
    static constexpr int PWM_MAX_DUTY_PERCENT = 10;  // 10% maximum duty cycle
    static constexpr int PWM_MID_DUTY_PERCENT = 7;   // 7.5% middle duty cycle
    
    // Constructor
    PWMController();
    
    // Destructor
    ~PWMController();
    
    // Initialize PWM channel
    esp_err_t initialize_channel(ledc_channel_t channel, int gpio_pin);
    
    // Set duty cycle (-1.0 to 1.0, where 0.0 is neutral)
    esp_err_t set_duty(ledc_channel_t channel, float percent);
    
    // Emergency stop (set all channels to neutral)
    esp_err_t emergency_stop();
    
    // Test sequence
    esp_err_t run_test_sequence();

private:
    // PWM state
    bool initialized_ = false;
    std::vector<ledc_channel_t> active_channels_;
    
    // Convert normalized value to duty cycle
    float normalize_to_duty(float percent) const;
    
    // Validate channel
    bool is_valid_channel(ledc_channel_t channel) const;
};

// Matrix utilities for motion calculations
namespace MatrixUtils
{
    // Create 2D rotation matrix
    Eigen::Matrix2f rotation_matrix(float angle);
    
    // Create 2D translation vector
    Eigen::Vector2f translation_vector(float x, float y);
    
    // Apply rotation to a 2D vector
    Eigen::Vector2f rotate_vector(const Eigen::Vector2f& vec, float angle);
    
    // Transform from world coordinates to body coordinates
    Eigen::Vector2f world_to_body(const Eigen::Vector2f& world_vec, float yaw);
    
    // Transform from body coordinates to world coordinates
    Eigen::Vector2f body_to_world(const Eigen::Vector2f& body_vec, float yaw);
    
    // Create wheel kinematics matrix for omni drive
    Eigen::Matrix3f create_wheel_kinematics_matrix(float beta_a, float beta_b, float beta_c);
    
    // Compute wheel speeds from body velocities
    Eigen::Vector3f compute_wheel_speeds(const Eigen::Vector3f& body_velocities,
                                        const Eigen::Matrix3f& wheel_matrix,
                                        float wheel_distance);
}

// Omni Drive Mixer
class OmniDriveMixer
{
public:
    // Configuration constants
    static constexpr float WHEEL_DISTANCE_MM = 50.0f;
    static constexpr float BETA_A = -M_PI / 3.0f;  // Wheel A angle
    static constexpr float BETA_B = M_PI / 3.0f;   // Wheel B angle
    static constexpr float BETA_C = M_PI;           // Wheel C angle
    static constexpr int TARGET_ROBOT = 2;          // 1: RED, 2: BLACK
    
    // Constructor
    OmniDriveMixer();
    
    // Destructor
    ~OmniDriveMixer();
    
    // Initialize omni drive system
    esp_err_t initialize();
    
    // Check if initialized
    bool is_initialized() const { return initialized_; }
    
    // Update motion based on current status
    void update();
    
    // FPS mode control (First Person Shooter)
    void omni_drive_fps(float v, float rate_yaw);
    
    // TPS mode control (Third Person Shooter)
    void omni_drive_tps(float rate_x, float rate_y, float rate_yaw, float yaw);
    
    // Manual motor control
    void motor_setA(float throttle);
    void motor_setB(float throttle);
    void motor_setC(float throttle);
    
    // Get current motor values
    struct MotorValues {
        float motor_a = 0.0f;
        float motor_b = 0.0f;
        float motor_c = 0.0f;
    };
    
    MotorValues get_motor_values() const { return current_motor_values_; }

private:
    // Mixer state
    bool initialized_ = false;
    
    // PWM controller
    PWMController pwm_controller_;
    
    // Current motor values
    MotorValues current_motor_values_;
    
    // Pre-computed wheel kinematics matrix
    Eigen::Matrix3f wheel_kinematics_matrix_;
    
    // Motor mapping based on target robot
    void motor_setA_impl(float throttle);
    void motor_setB_impl(float throttle);
    void motor_setC_impl(float throttle);
    
    // Normalize wheel speeds to prevent saturation
    void normalize_wheel_speeds(float& mA, float& mB, float& mC);
    
    // Safety checks
    bool is_safe_motor_value(float value) const;
    void apply_motor_limits(float& value) const;
};

// Motion Controller
class MotionController
{
public:
    // Constructor
    MotionController(RobotStatus& robot_status);
    
    // Destructor
    ~MotionController();
    
    // Update motion control based on current status
    void update();
    
    // Set control mode
    void set_mode(RobotMode mode);
    
    // Get current mode
    RobotMode get_mode() const { return current_mode_; }
    
    // Emergency stop
    void emergency_stop();
    
    // Get motion status string
    std::string get_motion_status() const;

private:
    // Robot status reference
    RobotStatus& robot_status_;
    
    // Motion state
    RobotMode current_mode_ = RobotMode::TPS;
    
    // Omni drive mixer
    OmniDriveMixer omni_drive_mixer_;
    
    // Control logic for different modes
    void update_fps_mode();
    void update_tps_mode();
    void update_manual_mode();
    void update_auto_mode();
    
    // Safety functions
    void check_safety_limits();
    bool is_safe_motion() const;
    
    // Motion validation
    void validate_target_attitude();
    void limit_acceleration();
};

// Motion Manager (main interface)
class MotionManager
{
public:
    // Configuration constants
    static constexpr int TASK_STACK_SIZE = 4096;
    static constexpr int TASK_PRIORITY = 4;
    static constexpr int TASK_UPDATE_RATE_MS = 10;  // 10ms = 100Hz
    
    // Constructor
    MotionManager(RobotStatus& robot_status);
    
    // Destructor
    ~MotionManager();
    
    // Initialize motion subsystem
    esp_err_t initialize();
    
    // Start motion task
    esp_err_t start();
    
    // Stop motion task
    void stop();
    
    // Check if initialized
    bool is_initialized() const { return initialized_; }
    
    // Get motion status string
    std::string get_motion_status() const;
    
    // Emergency stop
    void emergency_stop();

private:
    // Robot status reference
    RobotStatus& robot_status_;
    
    // Motion state
    bool initialized_ = false;
    bool task_running_ = false;
    
    // Motion components
    MotionController motion_controller_;
    
    // Task handle
    TaskHandle_t task_handle_ = nullptr;
    
    // Private methods
    static void motion_task_wrapper(void* param);
    void motion_task_main();
    
    // Task management
    esp_err_t create_motion_task();
    void delete_motion_task();
};
