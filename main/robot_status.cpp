#include "robot_status.hpp"
#include <esp_log.h>
#include <esp_timer.h>
#include <sstream>
#include <iomanip>

static const char* TAG = "ROBOT_STATUS";

// Constructor
RobotStatus::RobotStatus() = default;

// Update system status
void RobotStatus::update_system_status()
{
    // Update uptime
    system_status.uptime_seconds = esp_timer_get_time() / 1000000;  // Convert microseconds to seconds
    
    // Update free heap memory
    system_status.free_heap = esp_get_free_heap_size();
    
    // Update battery status (placeholder - will be implemented by BatteryMonitor)
    if (system_status.battery_voltage < 11.0f) {
        system_status.battery_low = true;
    } else {
        system_status.battery_low = false;
    }
}

// Get formatted status string
std::string RobotStatus::get_system_status_string() const
{
    std::ostringstream oss;
    
    oss << "Mode: ";
    switch (mode) {
        case RobotMode::FPS: oss << "FPS"; break;
        case RobotMode::TPS: oss << "TPS"; break;
        case RobotMode::MANUAL: oss << "MANUAL"; break;
        case RobotMode::AUTO: oss << "AUTO"; break;
        default: oss << "UNKNOWN"; break;
    }
    
    oss << " | BLE: " << (system_status.ble_connected ? "ON" : "OFF");
    oss << " | Radio: " << (system_status.radio_connected ? "ON" : "OFF");
    oss << " | Sensors: " << (system_status.sensors_ok ? "OK" : "ERR");
    oss << " | Motors: " << (system_status.motors_ok ? "OK" : "ERR");
    
    oss << " | Battery: " << std::fixed << std::setprecision(1) 
        << system_status.battery_voltage << "V (" 
        << static_cast<int>(system_status.battery_percentage) << "%)";
    
    oss << " | Heap: " << system_status.free_heap << " bytes";
    oss << " | Uptime: " << system_status.uptime_seconds << "s";
    
    return oss.str();
}


