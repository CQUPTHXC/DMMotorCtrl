# DMMotorCtrl
重庆邮电大学HXC战队开源
readme持续更新中
# Contributors
![Contributors](https://contrib.rocks/image?repo=CQUPTHXC/DMMotorCtrl)
# V1.0.1
# 概述

该c++模块可用于esp32芯片的达妙电机控制。也可轻易移植到其他平台
esp32系列只需要外挂一颗CAN收发器，即可实现电机控制。

本文档提供了用于控制达妙电机的DMMotorCtrl模块的详细API说明。

## 类图
### 全局类图
![达妙电机库类图-类图 drawio](https://github.com/user-attachments/assets/3d313aa6-f2fe-4880-a2fc-1a077bce5193)
### 简化层级类图
![达妙电机库类图-层级图 drawio](https://github.com/user-attachments/assets/4baa4e1a-9c21-4fa1-b428-23044a3cf777)


## 控制算法
本电机库在使用如下图的控制方式
![达妙电机库类图-控制框图 drawio](https://github.com/user-attachments/assets/7da03e5d-c6b5-4a23-85b4-fa25aca78073)


# API 说明 (API Reference)

## `DMMotor` 类 (电机基类)

`DMMotor` 类是达妙电机控制的基础类，提供了电机控制的基本操作，如启动、停止、获取位置、速度等。

### 继承接口

`DMMotorMIT` 和其他继承类继承自 `DMMotor`，并基于此类提供更多的功能。以下是 `DMMotor` 类的接口：

* **构造函数**

  * `DMMotor(HXC_CAN* can, int MST_ID, int CAN_ID)`

* **电机控制**

  * `enable()`: 启动电机。
  * `disable()`: 停止电机。
  * `save_zero()`: 保存电机位置零点。
  * `clear_error()`: 清除电机错误。

* **状态检查**

  * `get_MST_ID()`: 获取电机的 MST ID。
  * `get_CAN_ID()`: 获取电机的 CAN ID。
  * `is_online()`: 检查电机是否在线。

* **位置和速度获取**

  * `get_location()`: 获取电机的多圈位置。
  * `get_pos_rad(bool is_multi_circle = false)`: 获取电机角度（弧度）。
  * `get_pos_deg(bool is_multi_circle = false)`: 获取电机角度（度）。
  * `get_speed_rad()`: 获取电机的速度（弧度/秒）。
  * `get_speed_rpm()`: 获取电机的速度（转/分钟）。

* **参数设置**

  * `set_param_max(float T_max, float V_max, float P_max)`: 设置最大参数映射值。
  * `set_Tmax(float Tmax)`: 设置扭矩最大值。
  * `set_Vmax(float Vmax)`: 设置速度最大值。
  * `set_Pmax(float Pmax)`: 设置位置最大值。

* **电机数据**

  * `get_pos_raw()`: 获取电机的原始位置数据。
  * `get_speed_raw()`: 获取电机的原始速度数据。
  * `get_torque_raw()`: 获取电机的原始扭矩数据。
  * `get_error()`: 获取电机的错误代码。
  * `get_controller_temperature()`: 获取电机控制器温度。
  * `get_motor_temperature()`: 获取电机转子温度。

---

## 控制方式封装
### `DMMotorMIT` 类 (MIT控制)

`DMMotorMIT` 类继承自 `DMMotor` 类，基于 MIT 控制方式扩展了电机控制功能，包括力矩控制、位置控制和速度控制。

#### 新增接口

* **构造函数**

  * `DMMotorMIT(HXC_CAN* can, int MST_ID, int CAN_ID)`
  * `DMMotorMIT(HXC_CAN* can, int MST_ID, int CAN_ID, uint8_t _kd, uint8_t _kp)`

* **控制参数设置**

  * `setDataRate(uint16_t value)`: 设置数据交互频率（1-1000Hz）。
  * `set_tff(uint16_t __tff)`: 设置力矩控制参数。
  * `set_pdes(uint16_t _p_des)`: 设置位置模式控制参数。
  * `set_vdes(uint16_t _v_des)`: 设置速度模式控制参数。

* **电机初始化**

  * `setup(bool isEnable = true)`: 初始化电机控制（是否使能电机）。

* **继承自 `DMMotor` 的接口**

  * `enable()`, `disable()`, `save_zero()`, `clear_error()`, `get_MST_ID()`, `get_CAN_ID()`, `is_online()`, `get_location()`, `reset_location()`, `get_pos_raw()`, `get_speed_raw()`, `get_pos_rad()`, `get_pos_deg()`, `get_speed_rad()`, `get_speed_rpm()`, `get_torque_raw()`, `get_error()`, `get_controller_temperature()`, `get_motor_temperature()`。

### `DMMotorSpeed` 类 （速度模式控制）

#### 新增接口

* **构造函数**

  * `DMMotorSpeed(HXC_CAN* can, int MST_ID, int CAN_ID)`：构造函数，初始化电机对象，并注册CAN接收回调。

* **控制方法**

  * `set_Speed(float _speed)`：设置电机的目标速度，单位为 rad/s。
  * `setDataRate(uint16_t value)`：设置数据交互频率，频率范围为 1-1000Hz。
  * `setup(bool isEnable = true)`：初始化速度控制，默认使能电机。

#### 控制流程

`DMMotorSpeed` 类通过继承 `DMMotor` 类，提供了速度控制的功能。用户可以通过 `set_Speed()` 设置目标速度，通过 `setDataRate()` 设置数据交互频率。`setup()` 方法用于初始化电机并启动控制任务，定期发送控制数据包。

---

### `DMMotorSpeedPos` 类（速度位置模式控制）

#### 新增接口

* **构造函数**

  * `DMMotorSpeedPos(HXC_CAN* can, int MST_ID, int CAN_ID)`：构造函数，初始化电机对象，并注册CAN接收回调。

* **控制方法**

  * `set_SpeedPos(float _speed, float _pos)`：设置电机的目标速度（rad/s）和目标位置（rad）。
  * `setDataRate(uint16_t value)`：设置数据交互频率，频率范围为 1-1000Hz。
  * `setup(bool isEnable = true)`：初始化位置速度控制，默认使能电机。

#### 控制流程

`DMMotorSpeedPos` 类继承自 `DMMotor` 类，扩展了位置和速度控制模式。用户可以通过 `set_SpeedPos()` 设置目标速度和目标位置。`setup()` 方法用于初始化电机并启动控制任务，定期发送包含速度和位置的控制数据包。
