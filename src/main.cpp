/*
 * @LastEditors: qingmeijiupiao
 * @Description: 主程序测试用
 * @Author: qingmeijiupiao
 * @Date: 2024-10-17 22:00:39
 */
#include <Arduino.h>

#include "CONTROLLER.hpp"
#include "DJIMotorCtrlESP.hpp"
#include "./DMMOTOR/HXC_DMCtrl.hpp"

// M3508_P19 M3508(1);

// // //达妙电机MIT控制类

// DMMotorMIT gm6220(0,1,0.0928*255,0.748*255);

// //二维向量,极坐标表示
// struct dir_and_value
// {
//   float dir=0; //方向,单位弧度
//   float value=0;//值

//   void xy_to_polar(float x,float y){//xy转极坐标
//     if(x==0&&y==0){
//       dir=0;
//       value=0;
//       return;
//     }
//     value=sqrt(x*x+y*y);//值
//     if ((remote_data.lx<0&&remote_data.ly<0)||(remote_data.lx>0&&remote_data.ly<0)||remote_data.lx<0||remote_data.ly<0)
//     {
//       value=-value;
//     }
    
//     dir=atan2(y,x);//方向
//   }

// };
// dir_and_value temp;



// /**
//  * @description: 航向电机劣弧转换函数
//  * @return {float} taget_angle
//  * @Author: qingmeijiupiao
//  * @param {*}
//  */
// float AngleConversion(){
//     //航向电机的实际角度
//   float DMangle=((gm6220.get_pos()-32768)/65535.f)*25;
//   //取到-PI到PI
//   float DMangle_pi=fmod(DMangle,PI);
//   //角度差值
//   float delta_angle=temp.dir-DMangle_pi;
//   //是否需要旋转PI
//   bool need_rotate=abs(delta_angle)>PI/2;
//   float taget_angle=0;
//   float motor_speed=0;
//   if(need_rotate){
//     motor_speed=-1*temp.value;
//     delta_angle=delta_angle>0?delta_angle-PI:delta_angle+PI;
//   }else{
//     motor_speed=temp.value;
//   }
//   taget_angle=DMangle+delta_angle;
//   Serial.println(taget_angle);
//   return taget_angle;
// }


void setup() {
  // Serial.begin(115200);

  // //遥控器
  // ESPNOW::esp_now_setup();
  // ESPNOW::add_callback_func("controller_data",test_callback);
  
  // pinMode(4,OUTPUT);
  // pinMode(5,OUTPUT);
  // digitalWrite(4,HIGH);
  // digitalWrite(5,HIGH);
  // can_init();
  // delay(3000);
  // gm6220.setup();
  // M3508.set_max_curunt(16384);
  // M3508.setup();
  
}
void loop() {
  // temp.xy_to_polar(remote_data.ly,remote_data.lx);

  // M3508.set_speed(temp.value*800);
  // gm6220.set_pdes(32768+AngleConversion()*65535/25);
  // delay(20);
}



