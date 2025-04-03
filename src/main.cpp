/*
 * @LastEditors: qingmeijiupiao
 * @Description: 主程序测试用
 * @Author: qingmeijiupiao
 * @Date: 2024-10-17 22:00:39
 */



#include "CONTROLLER.hpp"
#include "DJIMotorCtrlESP.hpp"
#include "./DMMOTOR/HXC_DMCtrl.hpp"
#include "HXC_TWAI.hpp"
#include "VOFA.hpp"

HXC_TWAI twai(8, 18, CAN_RATE_1MBIT);


VOFA_float motor_kp("motor_kp", 0.00081f);
VOFA_float motor_kd("motor_kd", 0.00012f);
VOFA_float motor_ki("motor_ki", 0.0001f);
VOFA_float motor_targetSpeed("motor_targetSpeed", 120.0f);
VOFA_float motor_targetLocation("motor_targetLocation", 60.0f);


// M3508_P19 M3508(1);
VOFA_float speed_target("speed",0);
VOFA_float location_target("location",0);
//达妙电机MIT控制类

HXC_DMCtrl DMH3510(&twai,0x0,0x1);
HXC_DMCtrl DM3507(&twai,0x1,0x0);
HXC_DMCtrl DM6220(&twai,0x2,0x0);


//二维向量,极坐标表示
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
//   float DMangle=((DMH3510.get_pos()-32768)/65535.f)*25;
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

// #define TARGET_LOCATION 1000
void setup() {

  twai.setup();
  delay(100);
DMH3510.setup(true);
DMH3510.enable();
DM3507.setup(true);
DM3507.enable();
  delay(100);
  Serial.begin(115200);
  VOFA_float::setup(); 
  
}

void loop() { 

float target_speed = motor_targetSpeed;
float target_location = motor_targetLocation;
float target_kp = motor_kp;
float target_kd = motor_kd;
float target_ki = motor_ki;

/*
float now_speed=DM3507.get_vel_rpm();
DM3507.set_speed((int)target_speed);
Serial.printf("%f, %f\n",now_speed,target_speed);

float now_location=DM3507.get_location();
DM3507.set_location(target_location);
Serial.printf("%f, %f\n",now_location,target_location);
*/

DMH3510.set_location_pid(target_kp,target_kd,target_ki); 

float now_location=DMH3510.get_location();
DMH3510.set_location(target_location);
Serial.printf("%f, %f\n",now_location,target_location);
/*
DMH3510.set_speed_pid(target_kp,target_kd,target_ki); 
float now_speed=DMH3510.get_vel_rpm();
DMH3510.set_speed(target_speed);
Serial.printf("%f, %f\n",now_speed,target_speed);


*/
delay(10); 
}



