/*
 * @LastEditors: qingmeijiupiao
 * @Description: HXC达妙电机控制
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-11-25 20:41:56
 */
#ifndef HXC_DMCtrlESP_HPP
#define HXC_DMCtrlESP_HPP
#include "DMCtrlESP.hpp"
#include "PID_CONTROL.hpp"
class HXC_DMCtrl : protected DMMotor{
    HXC_DMCtrl(int MST_ID,int CAN_ID):DMMotor(MST_ID,CAN_ID){};
    ~HXC_DMCtrl();

};






#endif
