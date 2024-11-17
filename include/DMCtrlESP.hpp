/*
 * @LastEditors: qingmeijiupiao
 * @Description: 达妙电机控制
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-11-18 01:03:22
 */
#ifndef DMCtrlESP_HPP
#define DMCtrlESP_HPP
#include <Arduino.h>
#include "ESP_CAN.hpp"
#include "PID_CONTROL.hpp"
#include <map>
enum DMRegisterAddress;
struct DMRegisterData_t;
class DMMotor{
public:
    DMMotor(int _MST_ID,int _CAN_ID):MST_ID(_MST_ID),CAN_ID(_CAN_ID){
        motor_map[_MST_ID]=this;
        //添加回调函数,当收到对应地址CAN数据时，调用callback
        add_user_can_func(_MST_ID,callback);
    }
    //使能
    esp_err_t enable(){
        twai_message_t can_message;
        can_message.identifier=CAN_ID;
        can_message.data_length_code=8;
        can_message.extd=0;
        can_message.self=0;
        for (size_t i = 0; i < 7; i++)
        {
        can_message.data[i]=0xff;
        }
        can_message.data[7]=0xfc;
        return twai_transmit(&can_message,portMAX_DELAY);
    }
    //失能
    esp_err_t disable(){
        twai_message_t can_message;
        can_message.identifier=CAN_ID;
        can_message.data_length_code=8;
        can_message.extd=0;
        can_message.self=0;
        for (size_t i = 0; i < 7; i++)
        {
        can_message.data[i]=0xff;
        }
        can_message.data[7]=0xfd;
        return twai_transmit(&can_message,portMAX_DELAY);
    }
    //保存位置零点
    esp_err_t save_zero(){
        twai_message_t can_message;
        can_message.identifier=CAN_ID;
        can_message.data_length_code=8;
        can_message.extd=0;
        can_message.self=0;
        for (size_t i = 0; i < 7; i++)
        {
        can_message.data[i]=0xff;
        }
        can_message.data[7]=0xfe;
        return twai_transmit(&can_message,portMAX_DELAY);
    }
    esp_err_t clear_error(){
        twai_message_t can_message;
        can_message.identifier=CAN_ID;
        can_message.data_length_code=8;
        can_message.extd=0;
        can_message.self=0;
        for (size_t i = 0; i < 7; i++)
        {
        can_message.data[i]=0xff;
        }
        can_message.data[7]=0xfb;
        return twai_transmit(&can_message,portMAX_DELAY);
    }
protected:
    //can总线ID到电机的映射
    static std::map<int, DMMotor*> motor_map;
    //回调函数,更新电机数据
    static void callback(twai_message_t* can_message){
        if(DMMotor::motor_map.find(can_message->identifier)==DMMotor::motor_map.end()){
            return;
        }
        motor_map[can_message->identifier]->update_date_callback(can_message->data);
    }
    void update_date_callback(uint8_t* arr){
        ERR=arr[0]>>4;
        POS_raw=(arr[1]<<8)|arr[2];
        VEL_raw=(arr[3]<<4)|(arr[4]>>4);
        TORQUE_raw=((arr[4]&0x0F)<<8)|arr[5];
        MOS_temp=arr[6];
        T_Rotor=arr[7];
    }
    //读取寄存器,寄存器表：https://gitee.com/kit-miao/damiao/raw/master/DM%20%E5%88%86%E7%AB%8B%E7%B3%BB%E5%88%97/DM-S3519-1EC/DM-S3519-1EC%E5%87%8F%E9%80%9F%E7%94%B5%E6%9C%BA%EF%BC%88%E5%90%ABDM3520-1EC%E9%A9%B1%E5%8A%A8%E5%99%A8%EF%BC%89%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6V1.0.pdf
    esp_err_t read_register(DMRegisterAddress addr){
        twai_message_t can_message;
        can_message.identifier=0x7FF;
        can_message.data_length_code=8;
        can_message.extd=0;
        can_message.self=0;
        can_message.data[0]=CAN_ID&0xFF;
        can_message.data[1]=CAN_ID>>8;
        can_message.data[2]=0x33;
        can_message.data[3]=addr;
        return twai_transmit(&can_message,portMAX_DELAY);   
    }

    esp_err_t write_register(DMRegisterAddress addr,uint32_t value,bool store=false){
        twai_message_t can_message;
        can_message.identifier=0x7FF;
        can_message.data_length_code=8;
        can_message.extd=0;
        can_message.self=0;
        can_message.data[0]=CAN_ID&0xFF;
        can_message.data[1]=CAN_ID>>8;
        if(store){
            can_message.data[2]=0xAA;
        }else{
            can_message.data[2]=0x55;
        }

        can_message.data[3]=addr;
        memcpy(can_message.data+4,&value,4);
        return twai_transmit(&can_message,portMAX_DELAY);
    }
    esp_err_t write_register(DMRegisterAddress addr,float value,bool store=false){
        twai_message_t can_message;
        can_message.identifier=0x7FF;
        can_message.data_length_code=8;
        can_message.extd=0;
        can_message.self=0;
        can_message.data[0]=CAN_ID&0xFF;
        can_message.data[1]=CAN_ID>>8;
        if(store){
            can_message.data[2]=0xAA;
        }else{
            can_message.data[2]=0x55;
        }
        can_message.data[3]=addr;
        memcpy(can_message.data+4,&value,4);
        return twai_transmit(&can_message,portMAX_DELAY);  
    }
    
    int MST_ID;//电机反馈数据ID
    int CAN_ID;//电机控制数据ID
    //原始数据
    uint16_t POS_raw;//位置
    uint16_t VEL_raw;//速度
    uint16_t TORQUE_raw;//扭矩
    uint8_t MOS_temp;//MOS温度
    uint8_t T_Rotor;//电机温度
    uint8_t ERR;//错误信息
    DMRegisterData_t register_data;//寄存器数据
};
//类静态成员要在类外定义
std::map<int, DMMotor*> DMMotor::motor_map;

//DM寄存器
enum DMRegisterAddress {
    UV_Value = 0x00,  // 低压保护值
    KT_Value = 0x01,  // 扭矩系数
    OT_Value = 0x02,  // 过温保护值
    OC_Value = 0x03,  // 过流保护值
    ACCE = 0x04,       // 加速度
    DECE = 0x05,       // 减速度
    MAX_SPD = 0x06,   // 最大速度
    MST_ID = 0x07,    // 反馈 ID
    ESC_ID = 0x08,    // 接收 ID
    TIMEOUT = 0x09,   // 超时警报时间
    CTRL_MODE = 0x0A, // 控制模式
    Damp = 0x0B,      // 电机粘滞系数
    Inertia = 0x0C,   // 电机转动惯量
    hw_ver = 0x0D,    // 硬件版本号
    sw_ver = 0x0E,    // 软件版本号
    SN = 0x0F,        // 序列号
    NPP = 0x10,       // 电机极对数
    Rs = 0x11,        // 电机相电阻
    Ls = 0x12,        // 电机相电感
    Flux = 0x13,      // 电机磁链值
    Gr = 0x14,        // 齿轮减速比
    PMAX = 0x15,      // 位置映射范围
    VMAX = 0x16,      // 速度映射范围
    TMAX = 0x17,      // 扭矩映射范围
    I_BW = 0x18,      // 电流环控制带宽
    KP_ASR = 0x19,    // 速度环 Kp
    KI_ASR = 0x1A,    // 速度环 Ki
    OV_Value = 0x1D,  // 过压保护值
    GREF = 0x1E,      // 齿轮力矩效率
    Deta = 0x1F,      // 速度环阻尼系数
    V_BW = 0x20,      // 速度环滤波带宽
    IQ_c1 = 0x21,     // 电流环增强系数
    VL_c1 = 0x22,     // 速度环增强系数
    can_br = 0x23,    // CAN 波特率代码
    sub_ver = 0x24,   // 子版本号
    u_off = 0x32,     // u 相偏置
    v_off = 0x33,     // v 相偏置
    k1 = 0x34,        // 补偿因子 1
    k2 = 0x35,        // 补偿因子 2
    m_off = 0x36,     // 角度偏移
    dir = 0x37,       // 方向
    p_m = 0x50,       // 电机当前位置
    xout = 0x51,      // 输出轴位置
};
struct DMRegisterData_t {
    float UV_Value;      // 低压保护值
    float KT_Value;      // 扭矩系数
    float OT_Value;      // 过温保护值
    float OC_Value;      // 过流保护值
    float Acceleration;  // 加速度
    float Deceleration;  // 减速度
    float MaxSpeed;      // 最大速度
    uint32_t MST_ID;     // 反馈 ID
    uint32_t ESC_ID;     // 接收 ID
    uint32_t Timeout;    // 超时警报时间
    uint32_t ControlMode; // 控制模式
    float DampingCoefficient; // 电机粘滞系数
    float MomentOfInertia;   // 电机转动惯量
    uint32_t HardwareVersion; // 硬件版本号
    uint32_t SoftwareVersion; // 软件版本号
    uint32_t SerialNumber;     // 序列号
    uint32_t NumberPolePairs; // 电机极对数
    float PhaseResistance;    // 电机相电阻
    float PhaseInductance;    // 电机相电感
    float FluxLinkage;        // 电机磁链值
    float GearReduction;      // 齿轮减速比
    float PositionMapRange;   // 位置映射范围
    float SpeedMapRange;      // 速度映射范围
    float TorqueMapRange;     // 扭矩映射范围
    float CurrentLoopBandwidth;// 电流环控制带宽
    float SpeedLoopProportionalGain; // 速度环 Kp
    float SpeedLoopIntegralGain;    // 速度环 Ki
    float OV_Value;            // 过压保护值
    float GearTorqueEfficiency;    // 齿轮力矩效率
    float SpeedLoopDampingCoefficient; // 速度环阻尼系数
    float SpeedLoopFilterBandwidth; // 速度环滤波带宽
    float CurrentLoopCompensationFactor; // 电流环增强系数
    float SpeedLoopCompensationFactor;  // 速度环增强系数
    uint32_t CAN_BaudrateCode; // CAN 波特率代码
    uint32_t SubVersion;       // 子版本号
    float UPhaseOffset;        // U 相偏置
    float VPhaseOffset;        // V 相偏置
    float CompensationFactor1; // 补偿因子 1
    float CompensationFactor2; // 补偿因子 2
    float AngleOffset;         // 角度偏移
    float Direction;           // 方向
    float MotorCurrentPosition; // 电机当前位置
    float OutputShaftPosition;  // 输出轴位置
};
#endif
