#pragma once

#include <cstdint>
#include <cstddef>

// Robot operation modes
enum class RobotMode
{
    FPS = 0,  // First Person Shooter mode
    TPS,      // Third Person Shooter mode
    MANUAL,   // Manual control mode
    AUTO      // Autonomous mode
};

// Position data structure
struct Position
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

// Velocity data structure
struct Velocity
{
    float v = 0.0f;        // Forward velocity
    float x = 0.0f;        // X-axis velocity
    float y = 0.0f;        // Y-axis velocity
    float z = 0.0f;        // Z-axis velocity
    float roll = 0.0f;     // Roll rate
    float pitch = 0.0f;    // Pitch rate
    float yaw = 0.0f;      // Yaw rate
};

// Acceleration data structure
struct Acceleration
{
    float v = 0.0f;        // Forward acceleration
    float x = 0.0f;        // X-axis acceleration
    float y = 0.0f;        // Y-axis acceleration
    float z = 0.0f;        // Z-axis acceleration
    float roll = 0.0f;     // Roll acceleration
    float pitch = 0.0f;    // Pitch acceleration
    float yaw = 0.0f;      // Yaw acceleration
};

// Attitude data structure
struct Attitude
{
    Position position;
    Velocity velocity;
    Acceleration acceleration;
};

// Remote control data structure
struct RemoteControl
{
    // Joystick values (-1.0 to 1.0)
    float x = 0.0f;        // Left-right
    float y = 0.0f;        // Forward-backward
    float theta = 0.0f;    // Rotation
    
    // Button states
    bool button_a = false;
    bool button_b = false;
    bool button_c = false;
    bool button_d = false;
    
    // Control values
    float roll_rate = 0.0f;
    float pitch_rate = 0.0f;
    float throttle = 0.0f;
    float yaw_pos = 0.0f;
};

// Sensor data structure
struct SensorData
{
    uint64_t timestamp = 0;
    
    // Accelerometer data (m/s²)
    float acc_x = 0.0f;
    float acc_y = 0.0f;
    float acc_z = 0.0f;
    
    // Gyroscope data (rad/s)
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    
    // Euler angles (rad)
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    
    // Update counter
    uint32_t update_count = 0;
};

// System status structure
struct SystemStatus
{
    bool ble_connected = false;
    bool radio_connected = false;
    bool sensors_ok = false;
    bool motors_ok = false;
    bool battery_low = false;
    
    float battery_voltage = 0.0f;
    float battery_current = 0.0f;
    uint8_t battery_percentage = 100;
    
    float cpu_usage = 0.0f;
    uint32_t free_heap = 0;
    uint32_t uptime_seconds = 0;
};

// Main robot status class
class RobotStatus
{
public:
    // Current operation mode
    RobotMode mode = RobotMode::TPS;
    
    // Control data
    RemoteControl remote;
    float throttle = 0.0f;
    
    // Sensor and attitude data
    SensorData sensor_data;
    Attitude current_attitude;
    Attitude target_attitude;
    
    // System status
    SystemStatus system_status;
    
    // Constructor
    RobotStatus() = default;
    
    // Update system status
    void update_system_status();
    
    // Get formatted status string
    std::string get_status_string() const;
};
