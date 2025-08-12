# OMNI3BOT Modular Project Structure

This document describes the new modular architecture for the OMNI3BOT project.

## Project Overview

The OMNI3BOT project has been restructured into a modular, maintainable architecture with clear separation of concerns. The main application (`app_main()`) now only handles system initialization and task creation, while all functionality is organized into specialized manager classes.

## Architecture

### Core Components

1. **Main Application** (`main.cpp`)
   - System initialization
   - Task creation and management
   - High-level system coordination

2. **Robot Status** (`robot_status.hpp/cpp`)
   - Central data structure for robot state
   - System status tracking
   - Data validation and formatting

3. **BLE Manager** (`ble_manager.hpp/cpp`)
   - Bluetooth Low Energy communication
   - HID device management
   - Remote control input processing

4. **Radio Manager** (`radio_manager.hpp/cpp`)
   - CRSF radio communication
   - Channel data processing
   - Telemetry transmission

5. **Sensor Manager** (`sensor_manager.hpp/cpp`)
   - IMU sensor management (QMI8658)
   - Data filtering and processing
   - Attitude estimation (Madgwick filter)

6. **Display Manager** (`display_manager.hpp/cpp`)
   - LED strip control (WS2812)
   - Pattern generation and animation
   - Status visualization

7. **Motion Manager** (`motion_manager.hpp/cpp`)
   - Motor control and PWM management
   - Omni drive kinematics
   - Motion control algorithms

8. **System Monitor** (`system_monitor.hpp/cpp`)
   - System health monitoring
   - Battery monitoring
   - Performance metrics
   - Event logging

## File Structure

```
main/
├── main.hpp              # Main header with forward declarations
├── main.cpp              # Main application entry point
├── robot_status.hpp      # Robot status and data structures
├── robot_status.cpp      # Robot status implementation
├── ble_manager.hpp       # BLE management interface
├── ble_manager.cpp       # BLE management implementation
├── radio_manager.hpp     # Radio management interface
├── radio_manager.cpp     # Radio management implementation
├── sensor_manager.hpp    # Sensor management interface
├── sensor_manager.cpp    # Sensor management implementation
├── display_manager.hpp   # Display management interface
├── display_manager.cpp   # Display management implementation
├── motion_manager.hpp    # Motion management interface
├── motion_manager.cpp    # Motion management implementation
├── system_monitor.hpp    # System monitoring interface
├── system_monitor.cpp    # System monitoring implementation
└── README_MODULAR.md     # This documentation
```

## Key Features

### Modular Design
- Each manager handles a specific subsystem
- Clear interfaces between components
- Easy to test and maintain individual modules
- Simple to add new features or modify existing ones

### Task-Based Architecture
- Each manager runs in its own FreeRTOS task
- Proper task priorities and stack sizes
- Event-driven communication between modules
- Non-blocking operations

### Error Handling
- Comprehensive error checking and reporting
- Graceful degradation on subsystem failures
- System-wide error logging and monitoring
- Emergency shutdown capabilities

### Configuration Management
- Centralized configuration constants
- Easy to modify for different robot variants
- Runtime configuration validation
- Environment-specific settings

## Usage

### Building the Project
```bash
# Build the project
idf.py build

# Flash to device
idf.py flash

# Monitor serial output
idf.py monitor
```

### Adding New Features
1. Create a new manager class following the existing pattern
2. Add the manager to the main initialization sequence
3. Implement the required interface methods
4. Add appropriate error handling and logging

### Modifying Existing Features
1. Locate the relevant manager class
2. Modify the implementation while keeping the interface stable
3. Update error handling and logging as needed
4. Test the changes thoroughly

## Development Guidelines

### Code Style
- Use consistent naming conventions
- Include comprehensive error handling
- Add detailed logging for debugging
- Follow ESP-IDF coding standards

### Error Handling
- Always check return values from ESP-IDF functions
- Use appropriate error codes and messages
- Implement graceful fallbacks where possible
- Log errors with sufficient context

### Memory Management
- Use smart pointers where appropriate
- Avoid dynamic memory allocation in critical paths
- Monitor heap usage and fragmentation
- Implement proper cleanup in destructors

### Task Management
- Use appropriate task priorities
- Avoid blocking operations in task loops
- Implement proper task synchronization
- Monitor task stack usage

## Future Enhancements

### Planned Features
- Configuration file support
- Remote firmware updates
- Advanced diagnostics and telemetry
- Machine learning integration
- Multi-robot coordination

### Performance Optimizations
- DMA-based sensor data processing
- Optimized matrix operations
- Reduced memory allocations
- Improved task scheduling

### Safety Features
- Enhanced emergency stop systems
- Collision detection and avoidance
- Battery management improvements
- System health monitoring

## Troubleshooting

### Common Issues
1. **Compilation Errors**: Check that all header files are properly included
2. **Runtime Errors**: Verify that all managers are properly initialized
3. **Memory Issues**: Monitor heap usage and task stack sizes
4. **Performance Issues**: Check task priorities and update frequencies

### Debug Tools
- Serial monitor for logging output
- ESP-IDF heap monitoring
- Task status monitoring
- Performance profiling tools

## Contributing

When contributing to this project:
1. Follow the established modular architecture
2. Maintain consistent code style
3. Add appropriate error handling and logging
4. Update documentation as needed
5. Test changes thoroughly before submitting

## License

This project follows the same license as the original OMNI3BOT project.
