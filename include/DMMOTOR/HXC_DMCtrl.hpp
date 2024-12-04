/*
 * @LastEditors: qingmeijiupiao
 * @Description: HXC达妙电机控制
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-11-27 15:29:33
 */
#ifndef HXC_DMCtrlESP_HPP
#define HXC_DMCtrlESP_HPP
#include "DMCtrlESP.hpp"
#include "PID_CONTROL.hpp"
#include "HXCthread.hpp"
class HXC_DMCtrl : protected DMMotor{
    public:
    //禁止电机拷贝传递
    HXC_DMCtrl(const HXC_DMCtrl&) = delete;
    HXC_DMCtrl& operator=(const HXC_DMCtrl&) = delete;

    
    /**
     * @description: 
     * @return {*}
     * @Author: qingmeijiupiao
     * @LastEditTime: Do not edit
     * @param {int} MST_ID 
     * @param {int} CAN_ID
     * @param {float} PMAX 位置最大值
     * @param {float} VMAX 速度最大值
     */
    HXC_DMCtrl(int MST_ID,int CAN_ID,float PMAX,float VMAX):DMMotor(MST_ID,CAN_ID),P_MAX(PMAX),V_MAX(VMAX){};

    ~HXC_DMCtrl(){};
    //初始化,位置闭环,使能
    void setup(){
        enable();//使能
        //速度控制线程
        speed_contral_thread.start(this,"speed_contral_task",4096);
    };

    bool is_online(){return DMMotor::is_online();};

    void stop(bool need_disable=true){
        if(need_disable){
            DMMotor::disable();
        }
        location_taget=get_location();
        speed_location_taget=get_location();
        taget_speed=0;
    };
    //设置位置闭环控制参数
    void set_location_pid(float _location_Kp=0,float _location_Ki=0,float _location_Kd=0,float __dead_zone=0,float _max_speed=0){
            location_pid_contraler.Kp=_location_Kp;
            location_pid_contraler.Ki=_location_Ki;
            location_pid_contraler.Kd=_location_Kd;
            location_pid_contraler._dead_zone=__dead_zone;
            location_pid_contraler._max_value=_max_speed;
    }
    //设置位置闭环控制参数
    void set_location_pid(pid_param pid){
        set_location_pid(pid.Kp,pid.Ki,pid.Kd,pid._dead_zone,pid._max_value);
    }
    //设置速度闭环控制参数
    void set_speed_pid(float _speed_Kp=0,float _speed_Ki=0,float _speed_Kd=0,float __dead_zone=0,float maxtff=0){ 
            speed_pid_contraler.Kp=_speed_Kp;
            speed_pid_contraler.Ki=_speed_Ki;
            speed_pid_contraler.Kd=_speed_Kd;
            speed_pid_contraler._dead_zone=__dead_zone;
            max_tff=maxtff;
            if(max_tff>1||max_tff<=0){//最大电流限制
                max_tff=1;
            }
            speed_pid_contraler._max_value=max_tff;
            speed_pid_contraler.reset();
            location_pid_contraler.reset();
    }
    //设置速度闭环控制参数
    void set_speed_pid(pid_param pid){
        set_speed_pid(pid.Kp,pid.Ki,pid.Kd,pid._dead_zone,pid._max_value);
    }
    //设置最大力矩限制 0-1
    void set_max_tff(float maxtff){
        max_tff=maxtff;
        if(max_tff>1||max_tff<=0){//最大力矩限制
            max_tff=1;
        }
        speed_pid_contraler._max_value=max_tff;
    }
    //卸载使能
    void unload(){
        location_contral_thread.stop();
        this->taget_speed=0;
        this->set_tff=2047;
        delay(30);//等待电流更新到电调
        speed_contral_thread.stop();
        this->disable();
    }
    //设置多圈目标位置
    void set_location(int64_t _location){
        //位置控制线程
        location_contral_thread.start(this,"location_contral_task",4096);
        //赋值
        location_taget=_location;
    }
    //设置目标速度,acce为电机加速度，当acce为0的时候，不启用加速度控制
    virtual void set_speed(float speed,float acce=0){
        acce=acce>0?acce:0;
        location_contral_thread.stop();
        speed_contral_thread.start(this,"speed_contral_task",4096);
        taget_speed = speed;
        acceleration=acce;
    }
    //设置电机加速度,当acce为0的时候,不启用加速度控制
    void set_acceleration(float acce=0){
        acce=acce>0?acce:0;
        acceleration=acce;
    }
    //获取当前速度，单位RPM
    virtual float get_speed(){
        return (VEL_raw-2048)/4096*V_MAX;
    };
    //获取当前位置，单位rad
    float get_pos(){
        return (POS_raw-32768)/65535*P_MAX;
    }
    //设置速度环位置误差系数,可以理解为转子每相差一圈位置误差就加 Kp RPM速度补偿
    void set_speed_location_K(float _K=1000){//可以简单理解为K越大转子越"硬",默认值为1000
        _K=_K>0?_K:-_K;
        speed_location_K=_K;
    }
    //设置闭环控制频率 1-1000 默认为200
    void set_control_frequency(int _control_frequency=200){
        if(_control_frequency<1){
            _control_frequency=50;
        }else if(_control_frequency>1000){
            _control_frequency=1000;
        }
        control_frequency=_control_frequency;
    }
    //获取闭环控制频率
    int get_control_frequency(){
        return control_frequency;
    }
    //获取减速箱减速比
    float get_reduction_ratio(){
        return reduction_ratio;
    }

    int64_t get_location(){return location;}
        /**
     * @description: 当电机的输出电流需要根据位置进行映射的时候，可以传入位置到电流映射函数的指针
     * 例如电机做摇臂控制电机时，电流需要做三角函数映射，那么可以传入经过三角函数的位置到电流映射函数的指针
     * @return {*}
     * @Author: qingmeijiupiao
     * @Date: 2024-04-23 14:00:29
     * @param 返回值为力矩值(float) -1~1，输入为位置(int64_t)的映射函数或者lambda
     */
    void add_location_to_current_func(std::function<float(int64_t)> func){// 建议传入std::function或者lambda
        location_to_current_func=func;
    }
    private:
    float P_MAX=0;
    float V_MAX=0;
    //速度控制PID控制器
    PID_CONTROL speed_pid_contraler;
    //速度闭环默认控制参数
    pid_param default_speed_pid_parmater={5,1,0.01,1,1};
    //位置控制PID控制器
    PID_CONTROL location_pid_contraler;
    //位置闭环默认控制参数
    pid_param default_location_pid_parmater={0.1,0.1,0,2000,3000}; 
    //位置环多圈目标位置
    int64_t location_taget=0;
    //速度环多圈目标位置
    int64_t speed_location_taget=0;
    //单位RPM
    float taget_speed = 0;
    //电机加速度,0为不启用加速度控制
    float acceleration=0;
    //速度环位置误差系数,这里的比例系数需要根据实际情况调整,比例系数speed_location_K可以理解为转子每相差一圈加 speed_location_K RPM速度补偿
    int speed_location_K=1000;
    //控制频率，单位Hz
    int control_frequency=500;
    //力矩范围 0-1
    float max_tff=0.5;
    //减速箱减速比
    float reduction_ratio=1.0;
    //发送的力矩 12bit [0-4095]->[-TMAX-TMAX]
    uint16_t set_tff=2047; 

    //位置到电流的映射函数，默认返回0,当电流非线性时需要重写
    std::function<float(int64_t n)> location_to_current_func=[](int64_t n){return 0;};
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
            float tff = 
            /*位置映射的电流=*/moto->location_to_current_func(moto->location)
            +
            /*PID控制器的计算电流=*/moto->speed_pid_contraler.control(moto->is_online()*err);//电机在线才计算电流
            
            //限制力矩-1~1
            tff=tff<-1?-1:(tff>1?1:tff);
            
            //-1~1 映射为0-4095
            moto->set_tff=tff*2047+2047;

            twai_message_t can_message;//定义CAN数据对象
            can_message.identifier=moto->CAN_ID;//电机ID就是CAN地址
            can_message.data_length_code=8;//数据长度为8字节
            can_message.self=false;
            memset(can_message.data,0,8);
            can_message.data[6]=(moto->set_tff>>8)&0x0F;
            can_message.data[7]=moto->set_tff&0xFF;
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
