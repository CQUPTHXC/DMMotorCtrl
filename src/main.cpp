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
#include "DMCtrlESP.hpp"

#include <map>
M3508_P19 M3508(1);
// float p_des=0,v_des=0.1,Kp=0.1,Kd=0.4,t_ff=0.7;

DMMotorMIT gm6220(0,1,0.1,0.4);

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
  gm6220.setup(1);
  M3508.setup();


}

void loop() {
  M3508.set_speed(800*remote_data.ly);
  gm6220.set_pdes(remote_data.ry+0.5);
  Serial.println(gm6220.POS_raw);
  delay(20);
}

