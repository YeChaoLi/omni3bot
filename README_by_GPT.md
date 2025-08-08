### Omni3Bot

Three-wheeled omni drive robot firmware for ESP32‑S3 (ESP‑IDF 5.x). It supports two control inputs out‑of‑the‑box:

- Bluetooth LE/BT HID Host (e.g., gamepad/joystick)
- Crossfire (CRSF) / ELRS over UART with telemetry (battery, GPS example)

It also integrates an IMU (QMI8658), an addressable RGB LED strip, and PWM motor control using LEDC.

### Features

- **Omni3 drive**: Three-wheel kinematics using Eigen, FPS/TPS control modes
- **BLE/BT HID host**: Scans, connects, parses joystick reports, maps to velocity and yaw
- **CRSF/ELRS**: Reads 16 channels, sends battery telemetry back to transmitter
- **IMU (QMI8658)**: I2C sampling with simple yaw integration
- **RGB LED strip**: Basic pattern/clear demo
- **Status & debugging**: Periodic logs and math self-test

### Hardware (defaults)

- **Target**: `ESP32-S3` (`CONFIG_IDF_TARGET="esp32s3"` in `sdkconfig.defaults`)
- **IMU (QMI8658)**: I2C SDA `GPIO 11`, SCL `GPIO 12` (address high)
- **RGB LED strip**: Data `GPIO 14`
- **Motors (LEDC PWM)**: Channels 0/1/2 on `GPIO 4/5/6`
- **CRSF/ELRS UART**: Defaults `UART1`, TX `GPIO 17`, RX `GPIO 16` (configurable)

Adjust pins as needed for your board and power electronics. Provide proper power and ground for motors/ESCs and LED strip.

### Repo layout

- `main/`
  - `main.cpp`: FreeRTOS tasks (BLE, CRSF, IMU, LEDs, motion, monitor, debug) and high-level glue
  - `robot_controller.hpp`: Core data structures, control logic, kinematics, and compile-time feature flags
  - `esp_hid_gap.[ch]`: HID host GAP helpers (Espressif example, adapted)
  - `idf_component.yml`: External components (Eigen, QMI8658, LED strip)
- `components/ESP_CRSF/`: CRSF protocol decoder and telemetry helpers
- `sdkconfig.defaults`: Minimal build and target defaults

### Build prerequisites

- ESP‑IDF 5.5.0 (or compatible 5.x). Follow Espressif setup for your OS
- Python and required ESP‑IDF Python env (handled by `idf.py`)

### Quick start (Windows shown; adapt serial port/baud for your host)

1) Open ESP‑IDF environment (PowerShell/CMD with ESP‑IDF exported)
2) Set target and fetch deps:
   - `idf.py set-target esp32s3`
   - `idf.py reconfigure`
3) Build, flash, monitor (replace `COMx`):
   - `idf.py -p COMx -b 460800 build flash monitor`

On first configure, the component manager will download dependencies declared in `main/idf_component.yml`:

- `espressif/led_strip`
- `waveshare/qmi8658`
- `eigen`

### Configuration

- Run `idf.py menuconfig` to adjust options.
- CRSF pins and UART are configurable under: `CRSF Configuration`
  - `CONFIG_CRSF_UART_NUM` (default 1)
  - `CONFIG_CRSF_TX_PIN` (default 17)
  - `CONFIG_CRSF_RX_PIN` (default 16)
- Bluetooth/NimBLE/HID options come from ESP‑IDF and the included HID helper. Defaults in `sdkconfig.defaults` enable BLE/BT for HID host.

Compile‑time feature toggles live in `main/robot_controller.hpp` (enabled by default):

- `USE_IMU`, `USE_RGB`, `USE_REMOTE`, `USE_CRSF`, `USE_OMNI3`

Disable features by commenting out the corresponding define if you want to trim tasks or reduce resource usage.

### Runtime overview

`app_main` starts these FreeRTOS tasks (see `main.cpp`):

- `ble_task`: Initialize HID host and scan/connect to a HID device, map joystick to `status.remote`
- `radio_task`: Initialize CRSF, read channels at ~20 Hz, map to `status.remote`, send battery telemetry
- `sensor_task`: Initialize QMI8658 and update IMU readings
- `pixel_task`: LED strip demo pattern/clear
- `motion_task`: Convert input to motor setpoints (FPS mode by default) and drive motors via LEDC
- `monitor_task`: Placeholder for system health
- `debug_task`: Periodic status logging and Eigen math checks

Control mapping (high level):

- **HID**: Report id 2 → bytes `[1]` and `[2]` → normalized to `[-1,1]` with deadzone, mapped to linear velocity and yaw
- **CRSF**: Channels 1–4 (roll, pitch, throttle, yaw) normalized to `[-1,1]` with deadzone; pitch/roll/yaw used for motion

### Notes & tips

- BLE HID host will scan for a few seconds and connect to the first suitable result. Ensure your controller is advertising and in pairing mode.
- For CRSF/ELRS, wire RX module to the configured UART and set the same baud/CRSF settings on your radio.
- The PWM outputs assume typical RC ESC expectations at 50 Hz. Verify direction and scaling for your drive.

### Troubleshooting

- If build fails on missing components, run `idf.py reconfigure` or clear the build: `idf.py fullclean`
- If BLE scan shows no devices, enable controller advertising, check BLE enable flags in `sdkconfig`, and keep device close.
- If motors don’t move as expected, verify `GPIO 4/5/6` wiring and that your ESCs are armed and powered.

### Credits

- HID GAP helper based on Espressif examples (included source)
- CRSF component adapted from/credited to AlfredoSystems — see `components/ESP_CRSF/README.md`
