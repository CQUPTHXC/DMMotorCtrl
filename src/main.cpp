/*
 * @LastEditors: qingmeijiupiao
 * @Description: 
 * @Author: qingmeijiupiao
 * @Date: 2024-10-17 22:00:39
 */
#include <Arduino.h>
//#include "ESP_CAN.hpp"
#include "CONTROLLER.hpp"
#include "DJIMotorCtrlESP.hpp"
#include <map>
M3508_P19 M3508(2);
GM6020 GM6020_1(6);
M2006_P36 M2006_1(1);


class DM_MIT_MOTOR{
  public:
    DM_MIT_MOTOR(int _MST_ID,int _CAN_ID):MST_ID(_MST_ID),CAN_ID(_CAN_ID){
      motor_map[_MST_ID]=this;
      //添加回调函数,当收到对应地址CAN数据时，调用callback
      add_user_can_func(_MST_ID,callback);
    }

  //protected:
  //地址到电机的映射
  static std::map<int,DM_MIT_MOTOR*> motor_map;
  //回调函数,更新电机数据
  static void callback(twai_message_t* can_message){
    if(motor_map.find(can_message->identifier)==motor_map.end()){
      return;
    }
    motor_map[can_message->identifier]->update_date_callback(can_message->data);
  }


  void send_MIT_pakage(float _p_des,float _v_des,float _Kp,float _Kd,float _t_ff){

    //定义打包函数
    auto get_MIT_pakage=[&](uint8_t* arr,uint16_t p_des,uint16_t v_des,uint16_t Kp,uint16_t Kd,uint16_t t_ff){
      arr[0]=p_des>>8;
      arr[1]=p_des&0xFF;
      arr[2]=v_des>>4;
      arr[3]=((v_des&0x0F)<<4)|((Kp>>8)&0x0F);
      arr[4]=Kp&0xFF;
      arr[5]=Kd>>4;
      arr[6]=((Kd&0x0F)<<4)|((t_ff>>8)&0x0F);
      arr[7]=t_ff&0xFF;
    };
    
    twai_message_t can_message;//定义CAN数据对象
    can_message.identifier=CAN_ID;//电机ID就是CAN地址
    can_message.data_length_code=8;//数据长度为8字节
    //打包
    get_MIT_pakage(can_message.data,_p_des*(1<<16),_v_des*(1<<12),_Kp*(1<<8),_Kd*(1<<8),_t_ff*(1<<12));
    
    //发送CAN数据包
    twai_transmit(&can_message,portMAX_DELAY);
  }

  //使能，达妙MIT模式必须先使能才能控制
  void enable(){
    twai_message_t can_message;
    can_message.identifier=CAN_ID;
    can_message.data_length_code=8;
    for (size_t i = 0; i < 7; i++)
    {
      can_message.data[i]=0xff;
    }
    can_message.data[7]=0xfc;
    
    twai_transmit(&can_message,portMAX_DELAY);
  }
  //更新数据的回调
  //uint8_t* arr :CAN数据包
  void update_date_callback(uint8_t* arr){
    ERR=arr[0]>>4;
    POS_raw=(arr[1]<<8)|arr[2];
    VEL_raw=(arr[3]<<4)|(arr[4]>>4);
    TORQUE_raw=((arr[4]&0x0F)<<8)|arr[5];
    MOS_temp=arr[6];
    T_Rotor=arr[7];
  }
  //控制任务,没写完
  static void control_task(void *p){
    DM_MIT_MOTOR* motor=(DM_MIT_MOTOR*)p;

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

};
DM_MIT_MOTOR gm6220(0,1);

void setup() {
  Serial.begin(115200);
  ESPNOW::esp_now_setup();
  ESPNOW::add_callback_func("controller_data",test_callback);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);
  can_setup();
  delay(2000);//等待CAN初始化和电机启动
  //使能
  gm6220.enable();

}

void loop() {
  gm6220.send_MIT_pakage(0,0.2,0,0.5,0);
  delay(1);
}

