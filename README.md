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
![达妙电机库类图-类图](https://github.com/user-attachments/assets/22f23d19-e116-4250-8f18-71f02bf52dc2)

### 简化层级类图
![达妙电机库类图-层级图 drawio](https://github.com/user-attachments/assets/4baa4e1a-9c21-4fa1-b428-23044a3cf777)


## 控制算法
本电机库在使用如下图的控制方式
![达妙电机库类图-控制框图 drawio](https://github.com/user-attachments/assets/7da03e5d-c6b5-4a23-85b4-fa25aca78073)

---

## API 说明 (API Reference)

### `DMMotor` 类 (电机基类)

`DMMotor` 类是达妙电机控制的基础类，提供了电机控制的基本操作，如启动、停止、获取位置、速度等。

**继承接口**

`DMMotorMIT` 和其他继承类继承自 `DMMotor`，并基于此类提供更多的功能。

**构造函数**

```cpp
DMMotor(HXC_CAN* can, int MST_ID, int CAN_ID);
```

* **参数**：

  * `can`: 指向 `HXC_CAN` 类型的指针，表示CAN通信接口。
  * `MST_ID`: 电机控制器的MST ID。
  * `CAN_ID`: 电机的CAN ID。

**电机控制**

```cpp
void enable();
```

* **返回值**：无
* **功能**：使能电机，电机开始运行。

```cpp
void disable();
```

* **返回值**：无
* **功能**：失能电机，电机停止运行。

```cpp
void save_zero();
```

* **返回值**：无
* **功能**：保存电机的当前位置作为零点。

```cpp
void clear_error();
```

* **返回值**：无
* **功能**：清除电机的错误状态。

**状态检查**

```cpp
int get_MST_ID();
```

* **返回值**：`int`：返回电机的MST ID。

```cpp
int get_CAN_ID();
```

* **返回值**：`int`：返回电机的CAN ID。

```cpp
bool is_online();
```

* **返回值**：`bool`：如果电机在线，返回 `true`；如果电机掉线，返回 `false`。

**位置和速度获取**

```cpp
int get_location();
```

* **返回值**：`int`：返回电机的多圈编码器位置（0-65535）映射到 \[-PMAX, PMAX]。

```cpp
float get_pos_rad(bool is_multi_circle = false);
```

* **参数**：

  * `is_multi_circle`: 默认为 `false`，指示是否获取多圈位置。
* **返回值**：`float`：电机角度，单位为弧度。

```cpp
float get_pos_deg(bool is_multi_circle = false);
```

* **参数**：

  * `is_multi_circle`: 默认为 `false`，指示是否获取多圈位置。
* **返回值**：`float`：电机角度，单位为度。

```cpp
float get_speed_rad();
```

* **返回值**：`float`：电机的速度，单位为弧度/秒。

```cpp
float get_speed_rpm();
```

* **返回值**：`float`：电机的速度，单位为转/分钟。

**参数设置**

```cpp
void set_param_max(float T_max, float V_max, float P_max);
```

* **参数**：

  * `T_max`: 最大扭矩值。
  * `V_max`: 最大速度值。
  * `P_max`: 最大位置值。
* **返回值**：无
* **功能**：设置电机的最大参数映射值。

```cpp
void set_Tmax(float Tmax);
```

* **参数**：

  * `Tmax`: 最大扭矩。
* **返回值**：无
* **功能**：设置最大扭矩值。

```cpp
void set_Vmax(float Vmax);
```

* **参数**：

  * `Vmax`: 最大速度。
* **返回值**：无
* **功能**：设置最大速度值。

```cpp
void set_Pmax(float Pmax);
```

* **参数**：

  * `Pmax`: 最大位置。
* **返回值**：无
* **功能**：设置最大位置值。

**电机数据**

```cpp
int get_pos_raw();
```

* **返回值**：`int`：返回电机的单圈编码器位置数据。

```cpp
int get_speed_raw();
```

* **返回值**：`int`：返回电机的原始速度数据。

```cpp
int get_torque_raw();
```

* **返回值**：`int`：返回电机的原始扭矩数据。

```cpp
int get_error();
```

* **返回值**：`int`：返回电机的错误代码。

```cpp
float get_controller_temperature();
```

* **返回值**：`float`：返回电机控制器的温度，单位为摄氏度。

```cpp
float get_motor_temperature();
```

* **返回值**：`float`：返回电机转子的温度，单位为摄氏度。

---

### `DMMotorMIT` 类（MIT控制）

`DMMotorMIT` 类继承自 `DMMotor` 类，扩展了基于MIT控制的电机控制功能，包括力矩控制、位置控制和速度控制。

**构造函数**

```cpp
DMMotorMIT(HXC_CAN* can, int MST_ID, int CAN_ID);
```

* **参数**：

  * `can`: 指向 `HXC_CAN` 类型的指针，表示CAN通信接口。
  * `MST_ID`: 电机控制器的MST ID。
  * `CAN_ID`: 电机的CAN ID。

```cpp
DMMotorMIT(HXC_CAN* can, int MST_ID, int CAN_ID, uint8_t _kd, uint8_t _kp);
```

* **参数**：

  * `can`: 指向 `HXC_CAN` 类型的指针，表示CAN通信接口。
  * `MST_ID`: 电机控制器的MST ID。
  * `CAN_ID`: 电机的CAN ID。
  * `_kd`: 速度控制中的微分增益。
  * `_kp`: 速度控制中的比例增益。

**控制参数设置**

```cpp
void setDataRate(uint16_t value);
```

* **参数**：

  * `value`: 设置数据交互频率，范围是1-1000Hz。
* **返回值**：无
* **功能**：设置数据交互频率。

```cpp
void set_tff(uint16_t __tff);
```

* **参数**：

  * `__tff`: 设置力矩控制的参数。
* **返回值**：无
* **功能**：设置力矩控制参数。

```cpp
void set_pdes(uint16_t _p_des);
```

* **参数**：

  * `_p_des`: 设置位置模式控制的目标位置。
* **返回值**：无
* **功能**：设置位置模式控制的目标位置。

```cpp
void set_vdes(uint16_t _v_des);
```

* **参数**：

  * `_v_des`: 设置速度模式控制的目标速度。
* **返回值**：无
* **功能**：设置速度模式控制的目标速度。

**电机初始化**

```cpp
void setup(bool isEnable = true);
```

* **参数**：

  * `isEnable`: 默认为 `true`，指示是否使能电机。
* **返回值**：无
* **功能**：初始化电机并根据需要使能或禁用电机。

---

### `DMMotorSpeed` 类（速度模式控制）

**构造函数**

```cpp
DMMotorSpeed(HXC_CAN* can, int MST_ID, int CAN_ID);
```

* **参数**：

  * `can`: 指向 `HXC_CAN` 类型的指针，表示CAN通信接口。
  * `MST_ID`: 电机控制器的MST ID。
  * `CAN_ID`: 电机的CAN ID。

**控制方法**

```cpp
void set_Speed(float _speed);
```

* **参数**：

  * `_speed`: 电机目标速度，单位为弧度/秒。
* **返回值**：无
* **功能**：设置电机的目标速度。

```cpp
void setDataRate(uint16_t value);
```

* **参数**：

  * `value`: 设置数据交互频率，频率范围为 1-1000Hz。
* **返回值**：无
* **功能**：设置数据交互频率。

```cpp
void setup(bool isEnable = true);
```

* **参数**：

  * `isEnable`: 默认为 `true`，指示是否使能电机。
* **返回值**：无
* **功能**：初始化速度控制并使能电机。

---

### `DMMotorSpeedPos` 类（速度位置模式控制）

**构造函数**

```cpp
DMMotorSpeedPos(HXC_CAN* can, int MST_ID, int CAN_ID);
```

* **参数**：

  * `can`: 指向 `HXC_CAN` 类型的指针，表示CAN通信接口。
  * `MST_ID`: 电机控制器的MST ID。
  * `CAN_ID`: 电机的CAN ID。

**控制方法**


```cpp
void set_SpeedPos(float _speed, float _pos);
```

* **参数**：

  * `_speed`: 电机目标速度，单位为弧度/秒。
  * `_pos`: 电机目标位置，单位为弧度。
* **返回值**：无
* **功能**：设置电机的目标速度和目标位置。

```cpp
void setDataRate(uint16_t value);
```

* **参数**：

  * `value`: 设置数据交互频率，频率范围为 1-1000Hz。
* **返回值**：无
* **功能**：设置数据交互频率。

```cpp
void setup(bool isEnable = true);
```

* **参数**：

  * `isEnable`: 默认为 `true`，指示是否使能电机。
* **返回值**：无
* **功能**：初始化速度位置控制并使能电机。
Got it! Here's the API description for the `HXC_DMCtrl` class in the same format as the previous one:

---

### `HXC_DMCtrl` 类（继承自 `DMMotorMIT`）

`HXC_DMCtrl` 类扩展了基于 MIT 控制的电机控制功能，并提供了 PID 控制以实现更精确的电机控制，支持速度、位置控制以及力矩控制。

**构造函数**

```cpp
HXC_DMCtrl(HXC_CAN* can, int MST_ID, int CAN_ID);
```

* **参数**：

  * `can`: 指向 `HXC_CAN` 类型的指针，表示CAN通信接口。
  * `MST_ID`: 电机控制器的 MST ID。
  * `CAN_ID`: 电机的 CAN ID。

```cpp
HXC_DMCtrl(HXC_CAN* can, int MST_ID, int CAN_ID, pid_param speed_pid, pid_param location_pid);
```

* **参数**：

  * `can`: 指向 `HXC_CAN` 类型的指针，表示 CAN 通信接口。
  * `MST_ID`: 电机控制器的 MST ID。
  * `CAN_ID`: 电机的 CAN ID。
  * `speed_pid`: 速度控制的 PID 参数。
  * `location_pid`: 位置控制的 PID 参数。

---

**新增方法说明**

`setup`

```cpp
void setup(bool is_enable = true);
```

* **参数**：

  * `is_enable`: 默认为 `true`，指示是否使能电机。
* **返回值**：无
* **功能**：初始化电机，进行位置闭环控制，并根据 `is_enable` 参数决定是否启用电机。

---

`stop`

```cpp
void stop(bool need_unload = true);
```

* **参数**：

  * `need_unload`: 默认为 `true`，表示是否卸载电机的使能。
* **返回值**：无
* **功能**：停止电机的运转，并根据 `need_unload` 参数决定是否卸载使能。

---

`set_location_pid`

```cpp
void set_location_pid(float _location_Kp, float _location_Ki, float _location_Kd, float __dead_zone, float _max_speed);
```

* **参数**：

  * `_location_Kp`: 位置控制的比例增益。
  * `_location_Ki`: 位置控制的积分增益。
  * `_location_Kd`: 位置控制的微分增益。
  * `__dead_zone`: 位置误差的死区值。
  * `_max_speed`: 最大速度限制。
* **返回值**：无
* **功能**：设置位置闭环控制的 PID 参数。

---

`set_speed_pid`

```cpp
void set_speed_pid(float _speed_Kp, float _speed_Ki, float _speed_Kd, float __dead_zone, float _max_curunt);
```

* **参数**：

  * `_speed_Kp`: 速度控制的比例增益。
  * `_speed_Ki`: 速度控制的积分增益。
  * `_speed_Kd`: 速度控制的微分增益。
  * `__dead_zone`: 速度误差的死区值。
  * `_max_curunt`: 最大电流限制。
* **返回值**：无
* **功能**：设置速度闭环控制的 PID 参数。

---

`set_target_pos_location`

```cpp
void set_target_pos_location(int64_t _location);
```

* **参数**：

  * `_location`: 目标位置，单位为 16 位编码器值（0-65535 映射为 -PMAX 到 PMAX）。
* **返回值**：无
* **功能**：设置目标位置。

---

`set_target_pos_rad`

```cpp
void set_target_pos_rad(float rad);
```

* **参数**：

  * `rad`: 目标位置，单位为弧度。
* **返回值**：无
* **功能**：设置目标位置，单位为弧度。

---

`set_target_pos_deg`

```cpp
void set_target_pos_deg(float deg);
```

* **参数**：

  * `deg`: 目标位置，单位为度。
* **返回值**：无
* **功能**：设置目标位置，单位为度。

---

`set_max_Torque`

```cpp
void set_max_Torque(float _max_Torque);
```

* **参数**：

  * `_max_Torque`: 最大扭矩，范围为 0 到 1。
* **返回值**：无
* **功能**：设置最大扭矩。

---

`unload`

```cpp
void unload();
```

* **返回值**：无
* **功能**：卸载电机的使能，使电机停止工作。

---

`load`

```cpp
void load();
```

* **返回值**：无
* **功能**：使能电机，使电机开始工作。

---

`get_is_load`

```cpp
bool get_is_load();
```

* **返回值**：`bool`

  * `true`: 电机已启用。
  * `false`: 电机未启用。
* **功能**：获取电机的使能状态。

---

`set_speed`

```cpp
void set_speed(float speed, float acce = 0);
```

* **参数**：

  * `speed`: 目标速度，单位为 RPM。
  * `acce`: 电机的加速度，单位为 RPM/s，默认为 0。
* **返回值**：无
* **功能**：设置目标速度，并支持加速度控制。

---

`get_location_target`

```cpp
int64_t get_location_target();
```

* **返回值**：`int64_t`

  * 目标位置，单位为编码器值。
* **功能**：获取当前的目标位置。

---

`get_target_speed`

```cpp
float get_target_speed();
```

* **返回值**：`float`

  * 目标速度，单位为 RPM。
* **功能**：获取当前的目标速度。

---

`set_acceleration`

```cpp
void set_acceleration(float acce = 0);
```

* **参数**：

  * `acce`: 电机的加速度，单位为 RPM/s，默认为 0。
* **返回值**：无
* **功能**：设置电机的加速度。

---

`get_reduction_ratio`

```cpp
float get_reduction_ratio();
```

* **返回值**：`float`

  * 减速比。
* **功能**：获取减速比，用于计算电机的实际输出速度。

---

`set_speed_location_K`

```cpp
void set_speed_location_K(float _K = 1000);
```

* **参数**：

  * `_K`: 速度环位置误差系数，默认为 1000。
* **返回值**：无
* **功能**：设置速度环的误差补偿系数。

---

`get_dynamic_send_frequency`

```cpp
int get_dynamic_send_frequency();
```

* **返回值**：`int`

  * 动态发送频率，单位为 Hz。
* **功能**：获取当前的动态发送频率。

---

`set_control_frequency`

```cpp
void set_control_frequency(uint16_t min = 100, uint16_t max = 1000);
```

* **参数**：

  * `min`: 控制频率的最小值，范围为 1 到 1000Hz，默认为 100Hz。
  * `max`: 控制频率的最大值，范围为 1 到 1000Hz，默认为 1000Hz。
* **返回值**：无
* **功能**：设置闭环控制频率的最小值和最大值。

---

`add_location_to_Torque_func`

```cpp
void add_location_to_Torque_func(std::function<int(int64_t)> func);
```

* **参数**：

  * `func`: 用于位置到扭矩的映射函数。
* **返回值**：无
* **功能**：设置一个位置到电流的映射函数。适用于需要根据位置来映射电流的场景，例如摇臂控制等。

---

### 具体型号封装类
在 `HXC_DMMotors.hpp`封装好了部分电机，内置了默认参数方便调用

## 使用实例

### 示例 1：初始化电机并控制其启动与停止

```cpp
#include "HXC_DMCtrl.hpp"

// 创建CAN通信接口
HXC_CAN can(1, 1);  // 参数为CAN接口的编号和通信波特率

// 创建电机对象
HXC_DMCtrl motor(&can, 1, 1);  // 参数为CAN接口、MST_ID和CAN_ID

// 初始化电机并启用
motor.setup();

// 启动电机
motor.load();

// 停止电机
motor.stop();
```

### 示例 2：设置PID控制参数并执行位置控制

```cpp
#include "HXC_DMCtrl.hpp"

// 创建CAN通信接口
HXC_CAN can(1, 1);

// 创建电机对象
HXC_DMCtrl motor(&can, 1, 1);

// 设置PID控制参数
motor.set_location_pid(50.0, 0.1, 5.0, 0.01, 10.0);  // 设置位置PID的Kp, Ki, Kd，死区值和最大速度

// 设置目标位置
motor.set_target_pos_deg(90.0);  // 设置目标位置为90度

// 启动电机并执行控制
motor.load();
```

### 示例 3：设置速度控制并获取电机状态

```cpp
#include "HXC_DMCtrl.hpp"

// 创建CAN通信接口
HXC_CAN can(1, 1);

// 创建电机对象
HXC_DMCtrl motor(&can, 1, 1);

// 设置PID控制参数
motor.set_speed_pid(30.0, 0.05, 1.0, 0.01, 2.0);  // 设置速度PID的Kp, Ki, Kd，死区值和最大电流

// 设置目标速度
motor.set_speed(120.0);  // 设置目标速度为120 RPM

// 获取当前目标位置和速度
int64_t target_location = motor.get_location_target();
float target_speed = motor.get_target_speed();

// 输出目标位置和速度
Serial.println("目标位置: " + String(target_location));
Serial.println("目标速度: " + String(target_speed));
```

### 示例 4：设置加速度并动态控制电机

```cpp
#include "HXC_DMCtrl.hpp"

// 创建CAN通信接口
HXC_CAN can(1, 1);

// 创建电机对象
HXC_DMCtrl motor(&can, 1, 1);

// 设置加速度
motor.set_acceleration(10.0);  // 设置加速度为10 RPM/s

// 启动电机并设置目标速度
motor.set_speed(200.0, 10.0);  // 设置目标速度为200 RPM，并设置加速度为10 RPM/s

// 获取电机的减速比
float reduction_ratio = motor.get_reduction_ratio();

// 输出减速比
Serial.println("减速比: " + String(reduction_ratio));
```

### 示例 5：根据位置控制电流的映射函数

```cpp
#include "HXC_DMCtrl.hpp"

// 创建CAN通信接口
HXC_CAN can(1, 1);

// 创建电机对象
HXC_DMCtrl motor(&can, 1, 1);

// 设置位置到扭矩的映射函数
motor.add_location_to_Torque_func([](int64_t location) {
    // 根据位置映射到扭矩（示例：位置越大，扭矩越大）
    return static_cast<int>(location / 100);
});

// 设置目标位置并启动电机
motor.set_target_pos_deg(45.0);
motor.load();
```

这些实例展示了如何使用 `HXC_DMCtrl` 类进行电机的初始化、启动、停止、PID控制、速度控制、加速度控制以及自定义的扭矩控制。
