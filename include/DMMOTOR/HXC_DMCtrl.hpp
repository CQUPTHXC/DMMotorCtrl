/*
 * @LastEditors: qingmeijiupiao
 * @Description: HXC达妙电机控制
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-11-25 22:09:59
 */
#ifndef HXC_DMCtrlESP_HPP
#define HXC_DMCtrlESP_HPP
#include "DMCtrlESP.hpp"
#include "PID_CONTROL.hpp"
#include "HXCthread.hpp"
class HXC_DMCtrl : protected DMMotor{
    HXC_DMCtrl(int MST_ID,int CAN_ID,float PMAX,float VMAX):DMMotor(MST_ID,CAN_ID),P_MAX(PMAX),V_MAX(VMAX){};

    float get_speed(){
        return (VEL_raw-2048)/4096*V_MAX;
    };
    float get_pos(){
        return (POS_raw-32768)/65535*P_MAX;
    }

    private:
    float P_MAX=0;
    float V_MAX=0;
    //速度控制PID控制器
    PID_CONTROL speed_pid_contraler;
    //速度闭环默认控制参数
    pid_param default_speed_pid_parmater={5,1,0.01,1,10000};
    //位置控制PID控制器
    PID_CONTROL location_pid_contraler;
    //位置闭环默认控制参数
    pid_param default_location_pid_parmater={0.1,0.1,0,2000,3000}; 
    //位置环多圈目标位置
    int location_taget=0;
    //速度环多圈目标位置
    int speed_location_taget=0;
    //单位RPM
    float taget_speed = 0;
    //电机加速度,0为不启用加速度控制
    float acceleration=0;
    //速度环位置误差系数,这里的比例系数需要根据实际情况调整,比例系数speed_location_K可以理解为转子每相差一圈加 speed_location_K RPM速度补偿
    int speed_location_K=1000;
    //控制频率，单位Hz
    int control_frequency=500;
    //力矩范围 0-2047
    uint16_t max_tff=1000;
    //位置到电流的映射函数，默认返回0,当电流非线性时需要重写
    std::function<uint16_t(int n)> location_to_current_func=[](int n){return 0;};
    //速度控制函数
    static void speed_contral_task(HXC_DMCtrl* moto){
        //上次更新时间
        int last_update_speed_time=micros();
        moto->speed_pid_contraler.reset();
        //在unload过后出现扰动，再次load之后不会回到扰动前的位置
        moto->speed_location_taget = moto->location;

        float taget_control_speed = moto->taget_speed;
        float last_taget_control_speed = moto->taget_speed;
        while (1){
            float delta_time=1e-6*(micros()-last_update_speed_time); 
            
            if(moto->acceleration==0){
                taget_control_speed = moto->taget_speed;
            }else{
                //如果启用了加速度控制
                if(abs(taget_control_speed-last_taget_control_speed)>delta_time*moto->acceleration){
                    //根据加速度控制速度
                    taget_control_speed = last_taget_control_speed+(taget_control_speed-last_taget_control_speed)>0?delta_time*moto->acceleration:-delta_time*moto->acceleration;
                }else{
                    taget_control_speed = taget_control_speed;
                }
            }
            //更新上次的设定速度
            last_taget_control_speed = taget_control_speed;

            //由速度计算得到的目标位置
            moto->speed_location_taget+=moto->is_online()*65535.f*taget_control_speed*delta_time/60.f;//位置误差,只有电机在线才计算累计位置

            //更新上次更新时间
            last_update_speed_time=micros();


            //由速度误差和位置误差一同计算电流
            double err = 
            /*速度环的误差=*/(taget_control_speed - moto->get_speed())
            +
            /*速度环位置误差比例系数=*/moto->speed_location_K/*这里的比例系数需要根据实际情况调整,比例系数speed_location_K可以理解为转子每相差一圈加 speed_location_K RPM速度补偿*/
            * 
            /*由速度计算得到的目标位置的误差*/(moto->speed_location_taget-moto->location)/65535.0;


            //计算控制电流
            int16_t cru = 
            /*位置映射的电流=*/moto->location_to_current_func(moto->location)
            +
            /*PID控制器的计算电流=*/moto->speed_pid_contraler.control(moto->is_online()*err);//电机在线才计算电流
            
            twai_message_t can_message;//定义CAN数据对象
            can_message.identifier=moto->CAN_ID;//电机ID就是CAN地址
            can_message.data_length_code=8;//数据长度为8字节
            can_message.self=false;
            memset(can_message.data,0,8);
            can_message.data[6]=(cru>>8)&0x0F;
            can_message.data[7]=cru&0xFF;
            //发送CAN数据包
            twai_transmit(&can_message,portMAX_DELAY);
            
            delay(1000/moto->control_frequency);

        };
    };
    //位置控制函数
    static void location_contral_task(HXC_DMCtrl* moto){
        moto->location_pid_contraler.reset();//重置位置闭环控制器
        float speed=0;
        while (1){
            //位置闭环控制,由位置误差决定速度,再由速度误差决定电流
            speed = moto->location_pid_contraler.control(moto->location_taget - moto->location);
            moto->taget_speed = speed;
            delay(1000/moto->control_frequency);
        }
    };
    //速度控制线程
    HXC::thread<HXC_DMCtrl*> speed_contral_thread=HXC::thread<HXC_DMCtrl*>(speed_contral_task);
    //位置控制线程
    HXC::thread<HXC_DMCtrl*> location_contral_thread=HXC::thread<HXC_DMCtrl*>(location_contral_task);
};






#endif
