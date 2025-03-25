/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 调好默认PID参数的DM电机封装类，继承HXC_DMCtrl
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-03-25 20:17:43
 */


#ifndef HXC_DMMotors_HPP
#define HXC_DMMotors_HPP
#include "HXC_DMCtrl.hpp"

namespace HXC{

//达妙3519
class DM3519 : public HXC_DMCtrl{
    DM3519(HXC_CAN*can,int MST_ID,int CAN_ID):HXC_DMCtrl(can,MST_ID,CAN_ID){
        this->set_Pmax();
        this->set_Vmax();
        this->set_Tmax();
        
        set_speed_pid(pid_param(0.001,0.002,0,1,1));
        set_location_pid(pid_param(0.1,0.1,0,2000,500));
    };
    // 复用基类的构造函数
    DM3519(HXC_CAN* can, int MST_ID, int CAN_ID, pid_param speed_pid, pid_param location_pid)
    :HXC_DMCtrl(can, MST_ID, CAN_ID, speed_pid, location_pid) {
        this->set_Pmax();
        this->set_Vmax();
        this->set_Tmax();
    }
};

class DM6220 : public HXC_DMCtrl{
    DM6220(HXC_CAN*can,int MST_ID,int CAN_ID):HXC_DMCtrl(can,MST_ID,CAN_ID){
        this->set_Pmax();
        this->set_Vmax();
        this->set_Tmax();
        set_speed_pid(pid_param(0.001,0.0006,0,1,1));
        set_location_pid(pid_param(0.047,0.092,0,50,500));
    };
    // 复用基类的构造函数
    DM6220(HXC_CAN* can, int MST_ID, int CAN_ID, pid_param speed_pid, pid_param location_pid)
    :HXC_DMCtrl(can, MST_ID, CAN_ID, speed_pid, location_pid) {
        this->set_Pmax();
        this->set_Vmax();
        this->set_Tmax();
    }

};
}
#endif