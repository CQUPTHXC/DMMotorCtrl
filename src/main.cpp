/*
 * @LastEditors: qingmeijiupiao
 * @Description: 主程序测试用
 * @Author: qingmeijiupiao
 * @Date: 2024-10-17 22:00:39
 */
#include <Arduino.h>

#include "CONTROLLER.hpp"
#include "DJIMotorCtrlESP.hpp"
#include "./DMMOTOR/DMCtrlESP.hpp"
#include <map>

M3508_P19 M3508(1);

// //达妙电机MIT控制类

DMMotorMIT gm6220(0,1,0.1*255,0.4*255);

//二维向量,极坐标表示
struct dir_and_value
{
  float dir=0; //方向,单位弧度
  float value=0;//值

  void xy_to_polar(float x,float y){//xy转极坐标
    if(x==0&&y==0){
      dir=0;
      value=0;
      return;
    }
    value=sqrt(x*x+y*y);//值
    dir=atan2(y,x);//方向
  }

};




void setup() {
  Serial.begin(115200);

  //遥控器
  ESPNOW::esp_now_setup();
  ESPNOW::add_callback_func("controller_data",test_callback);
  
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);
  can_init();
  gm6220.setup();
  M3508.set_max_curunt(16384);
  M3508.setup();

}
dir_and_value temp;
void loop() {
  temp.xy_to_polar(remote_data.ly,remote_data.lx);

  M3508.set_speed(temp.value*600);

  uint16_t DIR=65535/2;
  DIR+=temp.dir*(65535/25);
  Serial.print(temp.value);
  Serial.print(",");
  Serial.println(gm6220.get_pos());
  // gm6220.set_pdes(DIR);
  delay(20);
}



