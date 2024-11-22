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
DMMotorMIT gm6220(0,1);

void setup() {
  Serial.begin(115200);

  //遥控器
  ESPNOW::esp_now_setup();
  ESPNOW::add_callback_func("controller_data",test_callback);
  //遥控器
  
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
  M3508.set_speed(800*remote_data.ly);
  delay(20);
}

