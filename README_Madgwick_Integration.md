# Madgwick滤波器集成说明

## 概述

本项目已成功集成了Madgwick滤波器，用于IMU数据的姿态融合。Madgwick滤波器能够将加速度计、陀螺仪和磁力计的数据融合，提供稳定的姿态估计。

## 设计决策

### 架构选择：组合而非继承

**选择将MadgwickFilter作为IMUManager的成员变量，原因如下：**

1. **单一职责原则**: 
   - IMUManager负责传感器硬件管理和数据采集
   - MadgwickFilter负责姿态融合算法

2. **灵活性**: 
   - 可以轻松替换不同的姿态融合算法（如Mahony、EKF等）
   - 便于测试和调试

3. **维护性**: 
   - 避免复杂的继承层次
   - 代码结构更清晰

## 集成细节

### 1. 头文件包含

```cpp
#include "madgwick_filter.hpp"  // Madgwick滤波器
```

### 2. 类成员变量

```cpp
class IMUManager {
private:
    // Madgwick filter for attitude estimation
    espp::MadgwickFilter madgwick_filter_;
    
    // Timing for Madgwick filter
    uint32_t last_update_time_ms_ = 0;
    
    // ... 其他成员
};
```

### 3. 姿态更新方法

```cpp
void IMUManager::update_attitude(float ax, float ay, float az, float gx, float gy, float gz)
{
    // 计算时间间隔
    uint32_t current_time_ms = esp_timer_get_time() / 1000;
    float dt = (current_time_ms - last_update_time_ms_) / 1000.0f;
    
    // 更新Madgwick滤波器
    madgwick_filter_.update(dt, ax, ay, az, gx, gy, gz);
    
    // 更新时间戳
    last_update_time_ms_ = current_time_ms;
}
```

### 4. 姿态获取方法

```cpp
void IMUManager::get_attitude(float& roll, float& pitch, float& yaw) const
{
    // 从Madgwick滤波器获取欧拉角（度）
    madgwick_filter_.get_euler(pitch, roll, yaw);
    
    // 转换为弧度以匹配现有代码
    roll *= M_PI / 180.0f;
    pitch *= M_PI / 180.0f;
    yaw *= M_PI / 180.0f;
}
```

## 数据格式要求

### 输入数据单位

- **加速度计 (ax, ay, az)**: **重力加速度 g** (1g = 9.81 m/s²)
- **陀螺仪 (gx, gy, gz)**: **弧度/秒 (rad/s)**
- **时间间隔 (dt)**: **秒 (s)**

### 坐标系定义

使用**右手坐标系**：
- **X轴**: 向前（Roll轴）
- **Y轴**: 向左（Pitch轴）
- **Z轴**: 向上（Yaw轴）

### 旋转方向约定

遵循**右手定则**：
- 正值表示**逆时针**旋转（从轴的正方向看）
- 负值表示**顺时针**旋转

## 使用方法

### 1. 初始化

```cpp
// 在IMUManager构造函数中
IMUManager::IMUManager()
    : madgwick_filter_(0.1f),  // beta = 0.1
      last_update_time_ms_(0)
{
    // ... 其他初始化
}
```

### 2. 数据更新

```cpp
void IMUManager::update()
{
    // ... 读取传感器数据
    
    // 更新Madgwick滤波器
    update_attitude(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
    
    // 获取滤波后的姿态
    float roll, pitch, yaw;
    get_attitude(roll, pitch, yaw);
    
    // 更新状态
    status.sensor_data.roll = roll;
    status.sensor_data.pitch = pitch;
    status.sensor_data.yaw = yaw;
}
```

### 3. 状态监控

```cpp
// 在debug_task中
ESP_LOGI("Attitude", "Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", 
          status.sensor_data.roll * 180.0f / M_PI, 
          status.sensor_data.pitch * 180.0f / M_PI, 
          status.sensor_data.yaw * 180.0f / M_PI);
```

## 参数调优

### Beta参数

- **默认值**: 0.1
- **作用**: 控制加速度计反馈的权重
- **调优建议**:
  - 值越小，陀螺仪权重越大，响应越快但噪声也越大
  - 值越大，加速度计权重越大，更稳定但响应较慢
  - 推荐范围: 0.01 - 0.5

### 采样频率

- **当前设置**: 1000 Hz
- **建议**: 保持稳定的采样频率，避免时间间隔过大

## 测试验证

### 1. 静态测试
- 将IMU平放，检查roll和pitch是否接近0°
- 检查yaw是否稳定（无漂移）

### 2. 动态测试
- 绕各轴旋转，检查角度变化是否正确
- 验证旋转方向是否符合右手定则

### 3. 日志输出

```
I (1234) Sensor: Gyro: 0.001, -0.002, 0.003. Accel: 0.12, -0.08, 1.02.
I (1234) Attitude: Roll: 0.15°, Pitch: -0.08°, Yaw: 45.23°
I (1234) Madgwick: Filter active - Sensor updates: 156
```

## 故障排除

### 常见问题

1. **姿态角度异常**:
   - 检查传感器数据单位是否正确
   - 验证坐标系定义是否匹配

2. **滤波器不收敛**:
   - 检查时间间隔dt是否合理
   - 调整beta参数

3. **编译错误**:
   - 确保包含正确的头文件
   - 检查MadgwickFilter的路径

### 调试建议

1. 启用详细日志输出
2. 监控传感器原始数据
3. 验证时间间隔计算
4. 检查姿态角度变化趋势

## 扩展功能

### 未来改进

1. **自适应参数**: 根据运动状态动态调整beta
2. **多传感器融合**: 集成磁力计数据
3. **卡尔曼滤波**: 作为Madgwick的替代方案
4. **姿态预测**: 基于历史数据预测未来姿态

## 总结

Madgwick滤波器的集成为项目提供了稳定的姿态估计能力。通过组合设计模式，保持了代码的清晰性和可维护性。正确的数据格式和坐标系定义是成功运行的关键。
