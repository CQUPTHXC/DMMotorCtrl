/*
 * @LastEditors: qingmeijiupiao
 * @Description: 达妙电机控制
 * @Author: qingmeijiupiao
 * @LastEditTime: 2025-01-09 00:25:40
 */
#ifndef DMCtrlESP_HPP
#define DMCtrlESP_HPP
#include <Arduino.h>
#include "HXC_CAN.hpp"
#include <map>
#include "DMRegister.hpp"
#include <math.h>

// 默认控制任务栈大小
constexpr uint32_t DM_default_ctrl_task_stack_size = 4096;
// 默认控制任务优先级
constexpr uint8_t DM_default_ctrl_task_priority = 5;
// 默认控制任务名
constexpr char* DM_default_ctrl_task_name = "DMmotortask";

// 达妙电机基类
class DMMotor {
public:
    // 构造函数，初始化电机对象，注册CAN接收回调
    DMMotor(HXC_CAN* can, int MST_ID, int CAN_ID);

    // 使能电机 对应0xFC命令
    esp_err_t enable();

    // 失能电机 对应0xFD命令
    esp_err_t disable();

    // 保存电机位置零点
    esp_err_t save_zero();

    // 清除电机错误
    esp_err_t clear_error();

    // 获取电机的MST ID
    int get_MST_ID();

    // 获取电机的CAN ID
    int get_CAN_ID();

    // 检查电机是否在线（判断条件：20ms内未收到数据认为掉线）
    bool is_online();

    // 获取电机的多圈位置
    int64_t get_location();

    // 重置电机的多圈位置（默认为0）
    void reset_location(int l = 0);

    // 获取电机的原始位置数据（0-65535映射到 -Pmax~Pmax）
    uint16_t get_pos_raw();

    // 获取电机的原始速度数据（0-4095映射到 -Vmax~Vmax）
    uint16_t get_vel_raw();

    // 获取电机的原始扭矩数据（0-4095映射到 -Tmax~Tmax）
    uint16_t get_torque_raw();

    // 获取电机的错误代码
    uint8_t get_error();

    // 获取电机的控制器温度（单位：摄氏度）
    uint8_t get_controller_temperature();

    // 获取电机的转子温度（单位：摄氏度）
    uint8_t get_motor_temperature();

protected:
    //can消息回调函数
    void can_message_callback(HXC_CAN_message_t* can_message);

    // 电机数据的回调函数
    void update_date_callback(uint8_t* arr);

    /**
     * @brief : 读取电机的寄存器数据
     * @return  {esp_err_t} 成功返回ESP_OK，超时返回ESP_ERR_TIMEOUT
     * @Author : qingmeijiupiao
     * @param {DMRegisterAddress} addr 寄存器地址
     * @param {void*} value 读取到的数据，根据addr的类型有float或uint32_t两种类型
     */
    esp_err_t read_register(DMRegisterAddress addr,void* value);

    // 写入电机的寄存器数据（支持写入32位值）
    esp_err_t write_register(DMRegisterAddress addr, uint32_t value);

    // 重载写寄存器函数，支持写入浮点型数据
    esp_err_t write_register(DMRegisterAddress addr, float value);

    //写寄存器数据立即生效，但无法进行存储，掉电后丢失，需要发送存储参数的命令，将修改的参数全部写入片内
    esp_err_t save_register(DMRegisterAddress addr);
    
    HXC_CAN* can_bus;         // CAN总线对象

private:

    int MST_ID;               // 电机反馈数据ID
    int CAN_ID;               // 电机控制数据ID

    uint16_t POS_raw;         // 电机位置的原始数据
    uint16_t VEL_raw;         // 电机速度的原始数据
    uint16_t TORQUE_raw;      // 电机扭矩的原始数据
    uint8_t MOS_temp;         // 电机控制器MOS管温度
    uint8_t T_Rotor;          // 电机转子温度
    uint8_t ERR;              // 电机错误代码

    uint32_t last_can_message_update_time = 0;   // 上次数据更新时间（单位：毫秒）

    int64_t location = 0;    // 多圈位置（单位：脉冲数）
    bool can_register_flag=false;//是否在等待寄存器数据
    uint8_t register_buffer_addr=0;//接收到的寄存器数据地址
    uint8_t register_buffer[4]={0,0,0,0};//寄存器数据缓冲区
    constexpr static const int READ_REGISTER_TIMEOUT=100;//读取寄存器超时时间 毫秒
};


// 构造函数：初始化电机对象，设置CAN总线和电机ID，注册回调函数
DMMotor::DMMotor(HXC_CAN* can, int MST_ID, int CAN_ID) : can_bus(can), MST_ID(MST_ID), CAN_ID(CAN_ID) {

    //设置CAN接收回调
    std::function<void(HXC_CAN_message_t*)> motor_can_message_call_back=std::bind(&DMMotor::can_message_callback,this,std::placeholders::_1);

    can->add_can_receive_callback_func(MST_ID, motor_can_message_call_back);  // 注册CAN接收回调
}

int DMMotor::get_MST_ID() {
    return MST_ID;
}

int DMMotor::get_CAN_ID() {
    return CAN_ID;
}

// 使能电机
esp_err_t DMMotor::enable() {
    HXC_CAN_message_t can_message;
    can_message.identifier = CAN_ID;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    memset(can_message.data, 0xFF, 7);  // 数据填充0xFF
    can_message.data[7] = 0xFC;         // 设置使能命令
    return can_bus->send(&can_message); // 发送CAN命令
}

// 失能电机
esp_err_t DMMotor::disable() {
    HXC_CAN_message_t can_message;
    can_message.identifier = CAN_ID;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    memset(can_message.data, 0xFF, 7);  // 数据填充0xFF
    can_message.data[7] = 0xFD;         // 设置失能命令
    return can_bus->send(&can_message); // 发送CAN命令
}

// 保存电机位置零点
esp_err_t DMMotor::save_zero() {
    HXC_CAN_message_t can_message;
    can_message.identifier = CAN_ID;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    memset(can_message.data, 0xFF, 7);  // 数据填充0xFF
    can_message.data[7] = 0xFE;         // 设置零点保存命令
    return can_bus->send(&can_message); // 发送CAN命令
}

// 清除电机错误
esp_err_t DMMotor::clear_error() {
    HXC_CAN_message_t can_message;
    can_message.identifier = CAN_ID;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    memset(can_message.data, 0xFF, 7);  // 数据填充0xFF
    can_message.data[7] = 0xFB;         // 设置清除错误命令
    return can_bus->send(&can_message); // 发送CAN命令
}

// 检查电机是否在线（判断条件：20ms内未收到数据认为掉线）
bool DMMotor::is_online() {
    if (millis() - last_can_message_update_time > 20 && last_can_message_update_time != 0) {
        return false; // 20ms内未更新数据，认为电机掉线
    }
    return true; // 电机在线
}

// 获取电机的多圈位置
int64_t DMMotor::get_location() {
    return location;
}

// 重置电机的多圈位置（默认为0）
void DMMotor::reset_location(int l) {
    location = l;
}

// 获取电机的原始位置数据（0-65535映射到 -Pmax~Pmax）
uint16_t DMMotor::get_pos_raw() {
    return POS_raw;
}

// 获取电机的原始速度数据（0-4095映射到 -Vmax~Vmax）
uint16_t DMMotor::get_vel_raw() {
    return VEL_raw;
}

// 获取电机的原始扭矩数据（0-4095映射到 -Tmax~Tmax）
uint16_t DMMotor::get_torque_raw() {
    return TORQUE_raw;
}

// 获取电机的错误代码
uint8_t DMMotor::get_error() {
    return ERR;
}

// 获取电机的控制器温度（单位：摄氏度）
uint8_t DMMotor::get_controller_temperature() {
    return MOS_temp;
}

// 获取电机的转子温度（单位：摄氏度）
uint8_t DMMotor::get_motor_temperature() {
    return T_Rotor;
}

void DMMotor::can_message_callback(HXC_CAN_message_t* can_message){

    if(can_register_flag){//如果是寄存器数据
        uint8_t* arr=can_message->data;
        uint16_t recived_MOTOR_CANID=arr[0]|(arr[1]<<8);
        if(recived_MOTOR_CANID==CAN_ID){//如果是本电机的数据
            can_register_flag=false;
            register_buffer_addr=arr[3];//获取寄存器地址
            memcpy(register_buffer,arr+4,4);//复制数据
        }
    }else{
        //如果是普通数据
        update_date_callback(can_message->data);
    }
}

// 更新电机数据的回调函数
void DMMotor::update_date_callback(uint8_t* arr) {
    ERR = arr[0] >> 4;              // 获取错误代码
    uint16_t POS = (arr[1] << 8) | arr[2];  // 获取位置数据
    VEL_raw = (arr[3] << 4) | (arr[4] >> 4);  // 获取速度数据
    TORQUE_raw = ((arr[4] & 0x0F) << 8) | arr[5];  // 获取扭矩数据
    MOS_temp = arr[6];              // 获取MOS管温度
    T_Rotor = arr[7];               // 获取电机转子温度

    constexpr int MAX_POSITION=65535;//最大位置
    constexpr int HALF_POSITION=32768;//半最大位置

    // 更新电机的多圈位置
    int delta = 0;
    if ((POS + MAX_POSITION - POS_raw) % MAX_POSITION < HALF_POSITION) {  // 正转
        delta = POS - POS_raw;
        if (delta < 0) {
            delta += MAX_POSITION;
        }
    } else {  // 反转
        delta = POS - POS_raw;
        if (delta > 0) {
            delta -= MAX_POSITION;
        }
    }

    // 更新电机的多圈位置
    location += delta;
    POS_raw = POS;
    last_can_message_update_time = millis();  // 更新最后更新时间
}

// 读取电机寄存器数据
esp_err_t DMMotor::read_register(DMRegisterAddress addr,void* value) {
    HXC_CAN_message_t can_message;
    can_message.identifier = 0x7FF;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    can_message.data[0] = CAN_ID & 0xFF;
    can_message.data[1] = CAN_ID >> 8;
    can_message.data[2] = 0x33;
    can_message.data[3] = addr;
    esp_err_t status = can_bus->send(&can_message); // 发送读取寄存器的CAN命令
    if(status!=ESP_OK){//发送失败
        return status;
    }
    can_register_flag=true;//等待寄存器数据
    int start_time=millis();
    while(millis()-start_time<READ_REGISTER_TIMEOUT){//等待超时
        if(register_buffer_addr==addr){
            //复制数据
            memcpy(value,register_buffer,4);
            //清除标志位
            can_register_flag=false;
            //清除数据
            register_buffer_addr=0xFF;
            memset(register_buffer,0,4);
            //返回
            return ESP_OK;
        }
        delay(1);
    }
    can_register_flag=false;
    //超时
    return ESP_ERR_TIMEOUT;
}

// 写入电机寄存器数据（支持写入32位值）
esp_err_t DMMotor::write_register(DMRegisterAddress addr, uint32_t value) {
    HXC_CAN_message_t can_message;
    can_message.identifier = 0x7FF;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    can_message.data[0] = CAN_ID & 0xFF;
    can_message.data[1] = CAN_ID >> 8;
    can_message.data[2] = 0x55;  // 根据store参数设置命令
    can_message.data[3] = addr;
    memcpy(can_message.data + 4, &value, 4);  // 将值写入数据中
    esp_err_t status = can_bus->send(&can_message); // 发送写入寄存器的CAN命令
    if(status!=ESP_OK){//发送失败
        return status;
    }
    can_register_flag=true;//等待寄存器数据
    int start_time=millis();
    while(millis()-start_time<READ_REGISTER_TIMEOUT){//等待超时
        if(register_buffer_addr==addr){
            //比较数据
            int temp = memcmp(register_buffer,&value,4);
            
            //清除标志位
            can_register_flag=false;
            //清除数据
            register_buffer_addr=0xFF;
            memset(register_buffer,0,4);
            //返回
            if(temp==0){
                return ESP_OK;
            }else{
                return ESP_FAIL;
            }
            
        }
        delay(1);
    }
    can_register_flag=false;
    //超时
    return ESP_ERR_TIMEOUT;
}

// 重载写寄存器函数，支持写入浮点型数据
esp_err_t DMMotor::write_register(DMRegisterAddress addr, float value) {
    HXC_CAN_message_t can_message;
    can_message.identifier = 0x7FF;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    can_message.data[0] = CAN_ID & 0xFF;
    can_message.data[1] = CAN_ID >> 8;
    can_message.data[2] = 0x55;
    can_message.data[3] = addr;
    memcpy(can_message.data + 4, &value, 4);  // 将值写入数据中
    esp_err_t status = can_bus->send(&can_message); // 发送写入寄存器的CAN命令
    if(status!=ESP_OK){//发送失败
        return status;
    }
    can_register_flag=true;//等待寄存器数据
    int start_time=millis();
    while(millis()-start_time<READ_REGISTER_TIMEOUT){//等待超时
        if(register_buffer_addr==addr){
            //比较数据
            int temp = memcmp(register_buffer,&value,4);
            
            //清除标志位
            can_register_flag=false;
            //清除数据
            register_buffer_addr=0xFF;
            memset(register_buffer,0,4);
            //返回
            if(temp==0){
                return ESP_OK;
            }else{
                return ESP_FAIL;
            }
            
        }
        delay(1);
    }
    can_register_flag=false;
    //超时
    return ESP_ERR_TIMEOUT;
}

esp_err_t DMMotor::save_register(DMRegisterAddress addr){
    HXC_CAN_message_t can_message;
    can_message.identifier = 0x7FF;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    can_message.data[0] = CAN_ID & 0xFF;
    can_message.data[1] = CAN_ID >> 8;
    can_message.data[2] = 0xAA;
    can_message.data[3] = addr;
    return can_bus->send(&can_message); // 发送写入寄存器的CAN命令
}
#endif
