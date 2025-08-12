#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <memory>
#include <string>
#include <vector>

// Forward declaration
class RobotStatus;

// System event types
enum class SystemEventType
{
    NONE = 0,
    BLE_CONNECTED,
    BLE_DISCONNECTED,
    RADIO_CONNECTED,
    RADIO_DISCONNECTED,
    SENSOR_ERROR,
    MOTOR_ERROR,
    BATTERY_LOW,
    BATTERY_CRITICAL,
    SYSTEM_ERROR,
    WARNING,
    INFO
};

// System event structure
struct SystemEvent
{
    SystemEventType type;
    uint64_t timestamp;
    std::string message;
    int priority;  // 0: Low, 1: Normal, 2: High, 3: Critical
    
    SystemEvent(SystemEventType t, const std::string& msg, int prio = 1)
        : type(t), timestamp(0), message(msg), priority(prio) {}
};

// Battery monitoring
class BatteryMonitor
{
public:
    // Configuration constants
    static constexpr float VOLTAGE_DIVIDER_RATIO = 2.0f;  // Voltage divider ratio
    static constexpr float BATTERY_FULL_VOLTAGE = 12.6f;  // Full battery voltage
    static constexpr float BATTERY_EMPTY_VOLTAGE = 10.0f; // Empty battery voltage
    static constexpr float BATTERY_LOW_THRESHOLD = 11.0f; // Low battery warning
    static constexpr float BATTERY_CRITICAL_THRESHOLD = 10.5f; // Critical battery level
    
    // Constructor
    BatteryMonitor();
    
    // Initialize battery monitoring
    esp_err_t initialize();
    
    // Update battery status
    void update();
    
    // Get battery voltage
    float get_voltage() const { return voltage_; }
    
    // Get battery percentage
    uint8_t get_percentage() const { return percentage_; }
    
    // Get battery current
    float get_current() const { return current_; }
    
    // Check if battery is low
    bool is_low() const { return voltage_ < BATTERY_LOW_THRESHOLD; }
    
    // Check if battery is critical
    bool is_critical() const { return voltage_ < BATTERY_CRITICAL_THRESHOLD; }
    
    // Get battery status string
    std::string get_status_string() const;

private:
    // Battery state
    float voltage_ = 0.0f;
    float current_ = 0.0f;
    uint8_t percentage_ = 100;
    bool initialized_ = false;
    
    // ADC configuration
    void* adc_handle_ = nullptr;
    
    // Calculate battery percentage from voltage
    uint8_t calculate_percentage(float voltage) const;
    
    // Read ADC values
    esp_err_t read_voltage();
    esp_err_t read_current();
};

// Performance monitoring
class PerformanceMonitor
{
public:
    // Constructor
    PerformanceMonitor();
    
    // Update performance metrics
    void update();
    
    // Get CPU usage percentage
    float get_cpu_usage() const { return cpu_usage_; }
    
    // Get free heap memory
    uint32_t get_free_heap() const { return free_heap_; }
    
    // Get minimum free heap
    uint32_t get_min_free_heap() const { return min_free_heap_; }
    
    // Get uptime in seconds
    uint32_t get_uptime_seconds() const { return uptime_seconds_; }
    
    // Get performance status string
    std::string get_status_string() const;

private:
    // Performance metrics
    float cpu_usage_ = 0.0f;
    uint32_t free_heap_ = 0;
    uint32_t min_free_heap_ = UINT32_MAX;
    uint32_t uptime_seconds_ = 0;
    
    // Timing for CPU usage calculation
    uint64_t last_cpu_check_ = 0;
    uint32_t last_idle_time_ = 0;
    
    // Calculate CPU usage
    void calculate_cpu_usage();
    
    // Update memory statistics
    void update_memory_stats();
    
    // Update uptime
    void update_uptime();
};

// System Monitor (main interface)
class SystemMonitor
{
public:
    // Configuration constants
    static constexpr int TASK_STACK_SIZE = 2048;
    static constexpr int TASK_PRIORITY = 2;
    static constexpr int TASK_UPDATE_RATE_MS = 100;  // 100ms = 10Hz
    static constexpr int MAX_EVENTS = 50;            // Maximum events to store
    
    // Constructor
    SystemMonitor(RobotStatus& robot_status);
    
    // Destructor
    ~SystemMonitor();
    
    // Initialize system monitoring
    esp_err_t initialize();
    
    // Start monitoring task
    esp_err_t start();
    
    // Stop monitoring task
    void stop();
    
    // Check if initialized
    bool is_initialized() const { return initialized_; }
    
    // Get system status string
    std::string get_system_status() const;
    
    // Log system event
    void log_event(SystemEventType type, const std::string& message, int priority = 1);
    
    // Get recent events
    std::vector<SystemEvent> get_recent_events(int count = 10) const;
    
    // Clear old events
    void clear_old_events(uint64_t older_than_timestamp);
    
    // Emergency shutdown
    void emergency_shutdown(const std::string& reason);

private:
    // Robot status reference
    RobotStatus& robot_status_;
    
    // Monitor state
    bool initialized_ = false;
    bool task_running_ = false;
    
    // Monitoring components
    BatteryMonitor battery_monitor_;
    PerformanceMonitor performance_monitor_;
    
    // Event storage
    std::vector<SystemEvent> system_events_;
    mutable std::mutex events_mutex_;
    
    // Task handle
    TaskHandle_t task_handle_ = nullptr;
    
    // Private methods
    static void monitor_task_wrapper(void* param);
    void monitor_task_main();
    
    // Task management
    esp_err_t create_monitor_task();
    void delete_monitor_task();
    
    // System health checks
    void check_system_health();
    void check_battery_status();
    void check_memory_status();
    void check_connection_status();
    
    // Event management
    void add_event(const SystemEvent& event);
    void process_events();
    void cleanup_old_events();
    
    // Status updates
    void update_robot_status();
    void update_system_metrics();
    
    // Error handling
    void handle_system_error(const std::string& error_message);
    void handle_warning(const std::string& warning_message);
};
