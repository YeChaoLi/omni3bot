#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <memory>
#include <string>
#include <vector>

// Forward declaration
class RobotStatus;

// LED Strip Manager
class DisplayManager
{
public:
    // Configuration constants
    static constexpr bool USE_DMA = false;
    static constexpr int LED_COUNT = 64;
    static constexpr int MEMORY_BLOCK_WORDS = 0;
    static constexpr int GPIO_PIN = 14;
    static constexpr int RMT_RESOLUTION_HZ = 10 * 1000 * 1000;  // 10MHz
    static constexpr int TASK_STACK_SIZE = 2048;
    static constexpr int TASK_PRIORITY = 3;
    static constexpr int TASK_UPDATE_RATE_MS = 1000;  // 1 second
    
    // Color definitions
    struct RGBColor {
        uint8_t r, g, b;
        RGBColor(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0) 
            : r(red), g(green), b(blue) {}
    };
    
    // Predefined colors
    static const RGBColor RED;
    static const RGBColor GREEN;
    static const RGBColor BLUE;
    static const RGBColor YELLOW;
    static const RGBColor CYAN;
    static const RGBColor MAGENTA;
    static const RGBColor WHITE;
    static const RGBColor BLACK;
    
    // Constructor
    DisplayManager(RobotStatus& robot_status);
    
    // Destructor
    ~DisplayManager();
    
    // Initialize display subsystem
    esp_err_t initialize();
    
    // Start display task
    esp_err_t start();
    
    // Stop display task
    void stop();
    
    // Check if initialized
    bool is_initialized() const { return initialized_; }
    
    // Get display status string
    std::string get_display_status() const;
    
    // Basic LED control
    esp_err_t set_pixel(int pixel_index, const RGBColor& color);
    esp_err_t set_all_pixels(const RGBColor& color);
    esp_err_t clear();
    esp_err_t refresh();
    
    // Pattern functions
    esp_err_t set_pattern(const std::vector<RGBColor>& pattern);
    esp_err_t set_rainbow_pattern();
    esp_err_t set_breathing_pattern(const RGBColor& color);
    esp_err_t set_blinking_pattern(const RGBColor& color, int blink_rate_ms = 500);
    
    // Status display functions
    esp_err_t show_connection_status(bool ble_connected, bool radio_connected);
    esp_err_t show_battery_status(float battery_percentage);
    esp_err_t show_error_status(const std::string& error_message);
    esp_err_t show_system_status();
    
    // Animation functions
    esp_err_t start_animation();
    esp_err_t stop_animation();
    bool is_animation_running() const { return animation_running_; }
    
    // Custom patterns
    esp_err_t set_custom_pattern(const std::string& pattern_name);
    esp_err_t register_custom_pattern(const std::string& name, const std::vector<RGBColor>& pattern);

private:
    // Robot status reference
    RobotStatus& robot_status_;
    
    // Display state
    bool initialized_ = false;
    bool task_running_ = false;
    bool animation_running_ = false;
    
    // LED strip handle
    void* led_strip_ = nullptr;  // Will be cast to appropriate type
    
    // Task handle
    TaskHandle_t task_handle_ = nullptr;
    
    // Animation state
    int animation_frame_ = 0;
    uint64_t last_animation_update_ = 0;
    
    // Custom patterns storage
    std::map<std::string, std::vector<RGBColor>> custom_patterns_;
    
    // Private methods
    static void display_task_wrapper(void* param);
    void display_task_main();
    
    // LED strip initialization
    esp_err_t init_led_strip();
    void deinit_led_strip();
    
    // Pattern generation
    std::vector<RGBColor> generate_rainbow_pattern();
    std::vector<RGBColor> generate_breathing_pattern(const RGBColor& color);
    std::vector<RGBColor> generate_status_pattern();
    
    // Animation functions
    void update_animation();
    void draw_animation_frame();
    
    // Utility functions
    RGBColor interpolate_color(const RGBColor& color1, const RGBColor& color2, float factor);
    uint32_t rgb_to_grb(const RGBColor& color);
    void apply_brightness(RGBColor& color, float brightness);
    
    // Error handling
    esp_err_t handle_led_error(esp_err_t error, const char* operation);
};
