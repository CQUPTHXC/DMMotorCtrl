/*
 * @LastEditors: qingmeijiupiao
 * @Description: 
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

/**
 * @description: 向量合成函数
 * @param {float}  VectorX  X方向向量
 *        {float}  VectorY  Y方向向量
 * @return {float} magnitude  X,Y向量合成后的最终标量
 * @Author: riley
 */
float vectorSynthesis(float VectorX,float VectorY){
    float magnitude =sqrt(VectorX*VectorX+VectorY*VectorY);
    return magnitude;
}


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
    return up;
}

/**
 * @description: 角度计算传递函数
 * @param {float}  VectorX  X方向向量
 *        {float}  VectorY  Y方向向量
 * @return {float} magnitude  X,Y向量合成后的最终标量
 * @Author: riley
 */
uint16_t anglecal(float VectorX,float VectorY){
  float angle=0;
//角度判别有问题，还得改
  float __angle = atan2(VectorY, VectorX)+PI/2;
    if (0 <= __angle && __angle <= PI / 2) 
    {
        angle = __angle;
    } else if(PI / 2 < __angle && __angle <= PI) 
    {
        angle = -(PI - __angle);
    } else if (PI < __angle && __angle <= 3 * PI / 2)
    {
        angle = __angle - PI;
    } else if (3 * PI / 2 < __angle && __angle <= 2 * PI) 
    {
        angle = -(2 * PI - __angle);
    }
  return map_float(angle,0,2*PI,0,((1<<16)-1));
}


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
  delay(2000);
  gm6220.setup();
  M3508.setup();

}

void loop() {
  // vectorSynthesis(remote_data.lx,remote_data.ly);
  M3508.set_speed(800*vectorSynthesis(remote_data.lx,remote_data.ly));
  uint16_t temp=anglecal(remote_data.lx,remote_data.ly);
  gm6220.set_pdes(temp);
  delay(20);
}



