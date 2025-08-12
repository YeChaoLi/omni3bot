#pragma once

#include <esp_err.h>
#include <esp_hidh.h>
#include <esp_hid_gap.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <memory>
#include <string>

// Forward declaration
class RobotStatus;

// BLE HID Host Manager
class BLEManager
{
public:
    // Configuration constants
    static constexpr float JOYSTICK_MAX_RAW_VALUE = 15.0f;
    static constexpr int JOYSTICK_DEADZONE = 3;
    static constexpr int SCAN_DURATION_SECONDS = 5;
    static constexpr int HID_HOST_MODE = 0;  // 0: BLE, 1: BT Classic
    
    // Constructor
    BLEManager(RobotStatus& robot_status);
    
    // Destructor
    ~BLEManager();
    
    // Initialize BLE subsystem
    esp_err_t initialize();
    
    // Start scanning and connection process
    esp_err_t start_scan_and_connect();
    
    // Check if connected
    bool is_connected() const { return connected_; }
    
    // Get connection status string
    std::string get_connection_status() const;
    
    // Disconnect current device
    esp_err_t disconnect();

private:
    // Robot status reference
    RobotStatus& robot_status_;
    
    // Connection state
    bool connected_ = false;
    bool initialized_ = false;
    
    // HID device handle
    esp_hidh_dev_handle_t current_device_ = nullptr;
    
    // Private methods
    void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
    void parse_and_map_hid_report(uint8_t report_id, const uint8_t *data, int length);
    void scan_and_connect();
    
    // Event handler registration
    esp_err_t register_event_handlers();
    void unregister_event_handlers();
    
    // HID report processing
    void process_joystick_data(const uint8_t *data, int length);
    void process_button_data(const uint8_t *data, int length);
    
    // Utility functions
    float normalize_joystick_value(int8_t raw_value) const;
    void apply_deadzone(float& value) const;
};
