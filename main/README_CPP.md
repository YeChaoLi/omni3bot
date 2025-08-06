# C++ Robot Controller

This directory contains the C++ version of the Omni3Bot robot controller, converted from the original C implementation.

## File Structure

- `main.cpp` - Main implementation file containing all the C++ classes and task functions
- `robot_controller.hpp` - Header file with class declarations and data structures
- `esp_hid_gap.c` - Original C file for HID gap functionality (unchanged)
- `CMakeLists.txt` - Updated build configuration for C++ compilation

## Key Changes from C to C++

### 1. Class-Based Architecture

The original C code has been restructured into proper C++ classes:

- **RobotStatus** - Central data structure for robot state
- **IMUManager** - Handles QMI8658 sensor initialization and data reading
- **LEDStripManager** - Manages WS2812 LED strip operations
- **RemoteController** - Handles Bluetooth HID device communication
- **MotionController** - Processes motion control logic
- **PWMController** - Low-level PWM channel management
- **OmniDriveController** - High-level omni-directional drive control
- **SystemMonitor** - System monitoring and safety checks
- **DebugLogger** - Status logging and debugging

### 2. Modern C++ Features

- **Smart Pointers**: Uses `std::unique_ptr` for automatic memory management
- **Strong Typing**: Enum classes and proper type safety
- **Const Correctness**: Proper use of const qualifiers
- **RAII**: Resource Acquisition Is Initialization pattern
- **Standard Library**: Uses `std::clamp`, `std::max`, `std::abs` etc.

### 3. Data Structures

```cpp
// Position and orientation data
struct Position {
    float x = 0.0f, y = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
};

// Speed and angular velocity data
struct Speed {
    float x = 0.0f, y = 0.0f, v = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
};

// Remote control input data
struct RemoteControl {
    float x = 0.0f, y = 0.0f, theta = 0.0f;
    bool button_a = false, button_b = false, button_c = false, button_d = false;
};
```

### 4. Configuration

The code uses the same preprocessor definitions as the original C version:

```cpp
#define USE_IMU      // Enable IMU sensor support
#define USE_RGB      // Enable LED strip support
#define USE_REMOTE   // Enable remote control support
#define USE_OMNI3    // Enable omni-directional drive
#define TARGET (BLACK) // Robot color target
```

## Building

The project uses ESP-IDF with C++ support. The CMakeLists.txt has been updated to:

- Use C++17 standard
- Disable RTTI and exceptions for embedded use
- Include the new C++ source files

## Task Structure

The main application creates several FreeRTOS tasks:

1. **sensor_task** - IMU data acquisition (1ms period)
2. **pixel_task** - LED strip control (1s period)
3. **motion_task** - Motion control processing (10ms period)
4. **mixer_task** - Motor control output (10ms period)
5. **monitor_task** - System monitoring (100ms period)
6. **debug_task** - Status logging (1000ms period)

## Key Improvements

1. **Better Encapsulation**: Each subsystem is properly encapsulated in its own class
2. **Resource Management**: Automatic cleanup with smart pointers
3. **Type Safety**: Strong typing prevents many runtime errors
4. **Maintainability**: Clear separation of concerns and modular design
5. **Extensibility**: Easy to add new features or modify existing ones

## Usage

The C++ version maintains the same functionality as the original C code while providing better structure and maintainability. All the original features are preserved:

- IMU sensor reading and processing
- LED strip pattern control
- Bluetooth remote control
- Omni-directional motion control
- System monitoring and debugging

## Compatibility

The C++ version is fully compatible with the ESP-IDF framework and maintains the same hardware interface as the original C implementation. 