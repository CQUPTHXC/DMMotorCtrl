/*
 * @LastEditors: qingmeijiupiao
 * @Description: 达妙电机控制
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-11-22 11:32:55
 */
#ifndef DMCtrlESP_HPP
#define DMCtrlESP_HPP
#include <Arduino.h>
#include "../ESP_CAN.hpp"
#include <map>
#include "DMRegister.hpp"




// 达妙电机基类
class DMMotor{
public:
    DMMotor(int MST_ID,int CAN_ID):MST_ID(MST_ID),CAN_ID(CAN_ID){
        motor_map[MST_ID]=this;
        //添加回调函数,当收到对应地址CAN数据时，调用callback
        add_user_can_func(MST_ID,callback);
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

class DMMotorMIT : public DMMotor{
public:

DMMotorMIT(int MST_ID,int CAN_ID):DMMotor(MST_ID,CAN_ID){};

DMMotorMIT(int MST_ID,int CAN_ID,uint8_t _kd,uint8_t _kp):DMMotor(MST_ID,CAN_ID){
    Kd=_kd;
    Kp=_kp;
};
//设置数据交互频率参数
void setDataRate(uint16_t value){
    value=value>1000?1000:value;
    value=value<1?1:value;
    DataRateHz=value;
}


/**
 * @description: 力矩控制
 * @return {*}
 * @Author: qingmeijiupiao
 * @param {float} __tff 取值范围为[-1,1]
 */
void set_tff(float __tff){  
    __tff=__tff>1?1:__tff;
    __tff=__tff<-1?-1:__tff;
    Kp=0;
    Kd=0;
    t_ff=__tff*(1<<12);
}

/**
 * @description: 位置模式控制
 * @return {*}
 * @Author: qingmeijiupiao
 * @param {uint16_t} _p_des 16位整数 0-65535 映射到[-P_max,P_max]
 */
void set_pdes(uint16_t _p_des){
    p_des= _p_des>((1<<16)-1)?((1<<16)-1): _p_des;
}


/**
 * @description: 速度模式控制
 * @return {*}
 * @Author: qingmeijiupiao
 * @param {uint16_t} _v_des 12位整数 -4096-4095 映射到[-V_max,V_max]
 */
void set_vdes(uint16_t _v_des){
    v_des= _v_des>((1<<12)-1)?((1<<12)-1): _v_des;
}

/**
 * @description: 初始化MIT电机控制
 * @param {bool} isEnable 是否使能
 * @return {*}
 * @Author: qingmeijiupiao
 */
void setup(bool isEnable=true){
    if(isEnable)
    {
        enable();
    }
    if(ctrl_task_handle==nullptr){
        xTaskCreate(DMmotortask,"DMmotortask",4096,this,5,&ctrl_task_handle
);
    }
}

private:

    void sendMITpakage(){
        uint8_t arr[8];
        arr[0]=p_des>>8;
        arr[1]=p_des&0xFF;
        arr[2]=v_des>>4;
        arr[3]=((v_des&0x0F)<<4)|((Kp>>8)&0x0F);
        arr[4]=Kp&0xFF;
        arr[5]=Kd>>4;
        arr[6]=((Kd&0x0F)<<4)|((t_ff>>8)&0x0F);
        arr[7]=t_ff&0xFF;
        twai_message_t can_message;//定义CAN数据对象
        can_message.identifier=CAN_ID;//电机ID就是CAN地址
        can_message.data_length_code=8;//数据长度为8字节
        can_message.self=false;
        memcpy(can_message.data,arr,8);
        //发送CAN数据包
        twai_transmit(&can_message,portMAX_DELAY);
    }

    static void DMmotortask(void* _motor){
        DMMotorMIT* motor=(DMMotorMIT*)_motor;
        while(1){
            motor->sendMITpakage();
            delay(1000/motor->DataRateHz);
        }
    };

    uint8_t Kp=0;//位置控制系数，使用速度控制时置为零
    uint8_t Kd=0;//速度控制系数,当使用位置控制时，此参数！不！能！为！零！
    uint16_t DataRateHz=1000;//数据交互频率
    uint16_t p_des=0; //位置串级控制参数
    uint16_t v_des=0.5*(1<<12);//速度串级控制参数
    uint16_t t_ff=0.5*(1<<12);//力矩串级控制参数

    TaskHandle_t ctrl_task_handle=nullptr;
};




#endif
