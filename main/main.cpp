#include "main.hpp"
#include "robot_status.hpp"
#include "ble_manager.hpp"
#include "radio_manager.hpp"
#include "sensor_manager.hpp"
#include "display_manager.hpp"
#include "motion_manager.hpp"
#include "system_monitor.hpp"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <nvs_flash.h>
#include <memory>
#include <string>

// Tag for logging
static const char* TAG = "OMNI3BOT_MAIN";

// Global robot status instance
RobotStatus g_robot_status;

// Global manager instances
static std::unique_ptr<BLEManager> g_ble_manager;
static std::unique_ptr<RadioManager> g_radio_manager;
static std::unique_ptr<SensorManager> g_sensor_manager;
static std::unique_ptr<DisplayManager> g_display_manager;
static std::unique_ptr<MotionManager> g_motion_manager;
static std::unique_ptr<SystemMonitor> g_system_monitor;

// Event group for system initialization
static EventGroupHandle_t g_system_event_group;
static const EventBits_t BLE_INIT_BIT = BIT0;
static const EventBits_t RADIO_INIT_BIT = BIT1;
static const EventBits_t SENSOR_INIT_BIT = BIT2;
static const EventBits_t DISPLAY_INIT_BIT = BIT3;
static const EventBits_t MOTION_INIT_BIT = BIT4;
static const EventBits_t MONITOR_INIT_BIT = BIT5;
static const EventBits_t ALL_INIT_BITS = BLE_INIT_BIT | RADIO_INIT_BIT | SENSOR_INIT_BIT | 
                                        DISPLAY_INIT_BIT | MOTION_INIT_BIT | MONITOR_INIT_BIT;

// System initialization
esp_err_t system_init()
{
    ESP_LOGI(TAG, "Starting system initialization...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS flash erased, reinitializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create system event group
    g_system_event_group = xEventGroupCreate();
    if (g_system_event_group == nullptr) {
        ESP_LOGE(TAG, "Failed to create system event group");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize managers
    ESP_LOGI(TAG, "Initializing managers...");
    
    g_ble_manager = std::make_unique<BLEManager>(g_robot_status);
    g_radio_manager = std::make_unique<RadioManager>(g_robot_status);
    g_sensor_manager = std::make_unique<SensorManager>(g_robot_status);
    g_display_manager = std::make_unique<DisplayManager>(g_robot_status);
    g_motion_manager = std::make_unique<MotionManager>(g_robot_status);
    g_system_monitor = std::make_unique<SystemMonitor>(g_robot_status);
    
    if (!g_ble_manager || !g_radio_manager || !g_sensor_manager || 
        !g_display_manager || !g_motion_manager || !g_system_monitor) {
        ESP_LOGE(TAG, "Failed to create manager instances");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize each manager
    ESP_LOGI(TAG, "Initializing BLE manager...");
    ret = g_ble_manager->initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE manager initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    xEventGroupSetBits(g_system_event_group, BLE_INIT_BIT);
    
    ESP_LOGI(TAG, "Initializing radio manager...");
    ret = g_radio_manager->initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Radio manager initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    xEventGroupSetBits(g_system_event_group, RADIO_INIT_BIT);
    
    ESP_LOGI(TAG, "Initializing sensor manager...");
    ret = g_sensor_manager->initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor manager initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    xEventGroupSetBits(g_system_event_group, SENSOR_INIT_BIT);
    
    ESP_LOGI(TAG, "Initializing display manager...");
    ret = g_display_manager->initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Display manager initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    xEventGroupSetBits(g_system_event_group, DISPLAY_INIT_BIT);
    
    ESP_LOGI(TAG, "Initializing motion manager...");
    ret = g_motion_manager->initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motion manager initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    xEventGroupSetBits(g_system_event_group, MOTION_INIT_BIT);
    
    ESP_LOGI(TAG, "Initializing system monitor...");
    ret = g_system_monitor->initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "System monitor initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    xEventGroupSetBits(g_system_event_group, MONITOR_INIT_BIT);
    
    // Wait for all managers to initialize
    EventBits_t bits = xEventGroupWaitBits(g_system_event_group, ALL_INIT_BITS, 
                                          pdFALSE, pdTRUE, pdMS_TO_TICKS(10000));
    if ((bits & ALL_INIT_BITS) != ALL_INIT_BITS) {
        ESP_LOGE(TAG, "System initialization timeout");
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGI(TAG, "System initialization completed successfully");
    return ESP_OK;
}

// Create and start all tasks
void create_tasks()
{
    ESP_LOGI(TAG, "Creating and starting tasks...");
    
    // Start BLE manager
    esp_err_t ret = g_ble_manager->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BLE manager: %s", esp_err_to_name(ret));
    }
    
    // Start radio manager
    ret = g_radio_manager->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start radio manager: %s", esp_err_to_name(ret));
    }
    
    // Start sensor manager
    ret = g_sensor_manager->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start sensor manager: %s", esp_err_to_name(ret));
    }
    
    // Start display manager
    ret = g_display_manager->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start display manager: %s", esp_err_to_name(ret));
    }
    
    // Start motion manager
    ret = g_motion_manager->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start motion manager: %s", esp_err_to_name(ret));
    }
    
    // Start system monitor
    ret = g_system_monitor->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start system monitor: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "All tasks created and started");
}

// Task wrapper functions
void ble_task(void *pvParameters)
{
    ESP_LOGI(TAG, "BLE task started");
    
    // Start BLE scanning and connection
    if (g_ble_manager) {
        esp_err_t ret = g_ble_manager->start_scan_and_connect();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BLE scan and connect failed: %s", esp_err_to_name(ret));
        }
    }
    
    // Keep task alive for event handling
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Check connection status
        if (g_ble_manager && g_ble_manager->is_connected()) {
            g_robot_status.system_status.ble_connected = true;
        } else {
            g_robot_status.system_status.ble_connected = false;
        }
    }
}

void radio_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Radio task started");
    
    // Keep task alive for CRSF handling
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Check connection status
        if (g_radio_manager && g_radio_manager->is_connected()) {
            g_robot_status.system_status.radio_connected = true;
        } else {
            g_robot_status.system_status.radio_connected = false;
        }
    }
}

void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor task started");
    
    // Keep task alive for sensor updates
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Check sensor status
        if (g_sensor_manager && g_sensor_manager->is_initialized()) {
            g_robot_status.system_status.sensors_ok = true;
        } else {
            g_robot_status.system_status.sensors_ok = false;
        }
    }
}

void display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Display task started");
    
    // Keep task alive for LED updates
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Update display based on system status
        if (g_display_manager && g_display_manager->is_initialized()) {
            g_display_manager->show_system_status();
        }
    }
}

void motion_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Motion task started");
    
    // Keep task alive for motion updates
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Check motion status
        if (g_motion_manager && g_motion_manager->is_initialized()) {
            g_robot_status.system_status.motors_ok = true;
        } else {
            g_robot_status.system_status.motors_ok = false;
        }
    }
}

void monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Monitor task started");
    
    // Keep task alive for system monitoring
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Update system status
        g_robot_status.update_system_status();
    }
}

// Main application entry point
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "OMNI3BOT starting up...");
    ESP_LOGI(TAG, "Version: 1.0.0");
    ESP_LOGI(TAG, "Build: %s %s", __DATE__, __TIME__);
    
    // Initialize system
    esp_err_t ret = system_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "System initialization failed: %s", esp_err_to_name(ret));
        esp_restart();
    }
    
    // Create and start tasks
    create_tasks();
    
    ESP_LOGI(TAG, "OMNI3BOT startup completed successfully");
    
    // Main loop - just keep the system running
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // 10 second delay
        
        // Log system status periodically
        ESP_LOGI(TAG, "System status: %s", g_robot_status.get_system_status_string().c_str());
        
        // Check for critical errors
        if (g_robot_status.system_status.battery_low) {
            ESP_LOGW(TAG, "Battery low warning");
        }
        
        if (!g_robot_status.system_status.sensors_ok) {
            ESP_LOGW(TAG, "Sensor system warning");
        }
        
        if (!g_robot_status.system_status.motors_ok) {
            ESP_LOGW(TAG, "Motor system warning");
        }
    }
}