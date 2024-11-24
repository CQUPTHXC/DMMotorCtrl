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
#include <cmath>

M3508_P19 M3508(1);
/**
 * @description: 浮点型映射函数，将浮点型变量映射为16位的原始数据
 * @param {float}  VectorX  X方向向量
 *        {float}  VectorY  Y方向向量
 * @return {float} magnitude  X,Y向量合成后的最终标量
 * @Author: riley
 */
uint16_t map_float(float x, float in_min, float in_max, uint16_t out_min, uint16_t out_max) {
    const float run = in_max - in_min;
    if(run == 0){
        log_e("map(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }
    const uint16_t rise = out_max - out_min;
    const float delta = x - in_min;
    uint16_t up=(uint16_t)(delta * rise) / run + out_min;
    // Serial.println(up);
    return up;
}
// //达妙电机MIT控制类

DMMotorMIT gm6220(0,1,0.4*255,0.3*255);

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
    if ((remote_data.lx<0&&remote_data.ly<0)||(remote_data.lx>0&&remote_data.ly<0)||remote_data.lx<0||remote_data.ly<0)
    {
      value=-value;
    }
    float dir=atan2(y,x);//方向
  }

//角度劣弧转换
  float angelchange(){
      float _dir=dir-gm6220.get_pos();
      float mod=fmod(_dir,PI/2);
      if ((0<=mod&&mod<=PI/2)||(-PI/2<=_dir&&_dir<=0))
      {
        dir=_dir;
        return dir;
      }
      else if ((PI/2<=mod&&mod<=PI)||-PI<=mod&&mod<=-PI/2)
      {
        dir=-(PI-_dir);
        Serial.println(dir);
        return dir;
      }
      
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
  delay(3000);
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
  temp.angelchange();
  // Serial.print(temp.value);
  gm6220.set_pdes(DIR);
  delay(20);
}



