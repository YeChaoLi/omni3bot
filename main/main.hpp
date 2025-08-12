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

// Forward declarations
class RobotStatus;
class BLEManager;
class RadioManager;
class SensorManager;
class DisplayManager;
class MotionManager;
class SystemMonitor;

// Global status instance
extern RobotStatus g_robot_status;

// Task function declarations
void ble_task(void *pvParameters);
void radio_task(void *pvParameters);
void sensor_task(void *pvParameters);
void display_task(void *pvParameters);
void motion_task(void *pvParameters);
void monitor_task(void *pvParameters);

// System initialization
esp_err_t system_init();
void create_tasks();
