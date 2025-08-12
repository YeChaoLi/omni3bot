#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string>
#include <memory>

// Forward declaration
class RobotStatus;

// CRSF Radio Manager
class RadioManager
{
public:
    // Configuration constants
    static constexpr int UART_NUM = 1;  // UART port number
    static constexpr int TX_PIN = 17;   // Transmit pin
    static constexpr int RX_PIN = 16;   // Receive pin
    static constexpr int BAUD_RATE = 420000;  // CRSF baud rate
    static constexpr int TASK_STACK_SIZE = 4096;
    static constexpr int TASK_PRIORITY = 5;
    
    // Channel constraints
    static constexpr float MIN_CHANNEL_VALUE = 174.0f;
    static constexpr float MID_CHANNEL_VALUE = 992.0f;
    static constexpr float MAX_CHANNEL_VALUE = 1811.0f;
    static constexpr float DEADZONE = 0.1f;
    
    // Constructor
    RadioManager(RobotStatus& robot_status);
    
    // Destructor
    ~RadioManager();
    
    // Initialize radio subsystem
    esp_err_t initialize();
    
    // Start radio task
    esp_err_t start();
    
    // Stop radio task
    void stop();
    
    // Check if connected
    bool is_connected() const { return connected_; }
    
    // Get connection status string
    std::string get_connection_status() const;
    
    // Send telemetry data
    esp_err_t send_telemetry();
    
    // Get last received channel data
    struct ChannelData {
        float roll = 0.0f;
        float pitch = 0.0f;
        float throttle = 0.0f;
        float yaw = 0.0f;
        uint64_t timestamp = 0;
    };
    
    ChannelData get_last_channels() const { return last_channels_; }

private:
    // Robot status reference
    RobotStatus& robot_status_;
    
    // Radio state
    bool connected_ = false;
    bool initialized_ = false;
    bool task_running_ = false;
    
    // Channel data
    ChannelData last_channels_;
    
    // Task handle
    TaskHandle_t task_handle_ = nullptr;
    
    // Private methods
    static void radio_task_wrapper(void* param);
    void radio_task_main();
    
    // CRSF protocol handling
    esp_err_t init_crsf();
    esp_err_t receive_channels();
    esp_err_t send_battery_telemetry();
    
    // Channel processing
    void process_channels(const uint8_t* data, size_t length);
    float normalize_channel_value(uint16_t raw_value) const;
    void apply_channel_deadzone(float& value) const;
    
    // Telemetry data
    struct TelemetryData {
        uint16_t voltage = 0;      // Battery voltage * 10
        uint16_t current = 0;      // Current * 10
        uint16_t capacity = 0;     // Capacity in mAh
        uint8_t remaining = 0;     // Remaining percentage
    };
    
    TelemetryData telemetry_data_;
    void update_telemetry_data();
};
