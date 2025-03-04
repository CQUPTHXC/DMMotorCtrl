/*
 * @LastEditors: qingmeijiupiao
 * @Description: HXC达妙电机控制，基于MIT控制
 * @Author: qingmeijiupiao
 * @LastEditTime: 2025-03-04 13:33:08
 */
#ifndef HXC_DMCtrlESP_HPP
#define HXC_DMCtrlESP_HPP
#include "DMCtrlMIT.hpp"
#include "PID_CONTROL.hpp"
class HXC_DMCtrl : protected DMMotorMIT{
    public:
    //禁止电机拷贝传递
    HXC_DMCtrl(const HXC_DMCtrl&) = delete;
    HXC_DMCtrl& operator=(const HXC_DMCtrl&) = delete;

    
    /**
     * @description: 
     * @return {*}
     * @Author: qingmeijiupiao
     * @LastEditTime: Do not edit
     * @param {HXC_CAN*} can
     * @param {int} MST_ID 
     * @param {int} CAN_ID
     */
    HXC_DMCtrl(HXC_CAN*can,int MST_ID,int CAN_ID);

    HXC_DMCtrl(HXC_CAN*can,int MST_ID,int CAN_ID,pid_param speed_pid,pid_param location_pid);
    ~HXC_DMCtrl(){};
    //初始化,位置闭环,使能
    void setup(bool is_enable = true)override;

    // 停止电机，并根据需要卸载使能
    void stop(bool need_unload = true);

    // 设置位置闭环控制参数
    void set_location_pid(float _location_Kp = 0, float _location_Ki = 0, float _location_Kd = 0, float __dead_zone = 0, float _max_speed = 0);
    
    // 设置位置闭环控制参数（传入pid_param结构体）
    void set_location_pid(pid_param pid);

    // 设置速度闭环控制参数
    void set_speed_pid(float _speed_Kp = 0, float _speed_Ki = 0, float _speed_Kd = 0, float __dead_zone = 0, float _max_curunt = 0);

    // 设置速度闭环控制参数（传入pid_param结构体）
    void set_speed_pid(pid_param pid);

    // 设置多圈目标位置
    void set_location(int64_t _location);

    // 设置最大力矩（0-1）
    void set_max_Torque(float _max_Torque);

    // 卸载使能
    void unload();

    // 使能电机
    void load();

    // 获取电机是否使能
    bool get_is_load();

    // 获取当前速度（单位：RPM）
    virtual float get_now_speed();

    // 设置目标速度，单位：RPM，加速度控制
    virtual void set_speed(float speed, float acce = 0);

    // 获取转子目标速度
    float get_taget_speed();

    // 设置电机加速度，单位：RPM/s
    void set_acceleration(float acce = 0);

    // 获取减速箱减速比
    float get_reduction_ratio();

    // 设置速度环位置误差系数
    void set_speed_location_K(float _K = 1000);

    // 获取闭环控制频率
    int get_control_frequency();

    // 设置闭环控制频率 0~1000
    void set_control_frequency(int _control_frequency = 1000);

    /**
     * @description: 当电机的输出电流需要根据位置进行映射的时候，可以传入位置到电流映射函数的指针
     * 例如电机做摇臂控制电机时，电流需要做三角函数映射，那么可以传入经过三角函数的位置到电流映射函数的指针
     * @return {*}
     * @Author: qingmeijiupiao
     * @Date: 2024-04-23 14:00:29
     * @param 返回值为力矩 -1~1，输入为位置(int64_t)的映射函数或者lambda
     */
    void add_location_to_Torque_func(std::function<int(int64_t)> func);

    /*基类接口*/

    // 使能电机 对应0xFC命令
    using DMMotor::enable;
    
    // 失能电机 对应0xFD命令
    using DMMotor::disable;
    // 保存电机位置零点
    using DMMotor::save_zero;
    // 清除电机错误
    using DMMotor::clear_error;
    // 获取电机的MST ID
    using DMMotor::get_MST_ID;
    // 获取电机的CAN ID
    using DMMotor::get_CAN_ID;
    // 检查电机是否在线（判断条件：20ms内未收到数据认为掉线）
    using DMMotor::is_online;
    // 获取电机的多圈位置
    using DMMotor::get_location;
    // 重置电机的多圈位置（默认为0）
    using DMMotor::reset_location;
    // 获取电机的原始位置数据（0-65535映射到 -Pmax~Pmax）
    using DMMotor::get_pos_raw;
    // 获取电机的原始速度数据（0-4095映射到 -Vmax~Vmax）
    using DMMotor::get_vel_raw;
    // 获取电机的角度,单位弧度
    using DMMotor::get_pos_rad;
    // 获取电机的角度，单位度
    using DMMotor::get_pos_deg;
    // 获取电机的速度，单位rad/s
    using DMMotor::get_vel_rad;
    // 获取电机的速度，单位rpm
    using DMMotor::get_vel_rpm;
    // 获取电机的原始扭矩数据（0-4095映射到 -Tmax~Tmax）
    using DMMotor::get_torque_raw;
    // 获取电机的错误代码
    using DMMotor::get_error;
    // 获取电机的控制器温度（单位：摄氏度）
    using DMMotor::get_controller_temperature;
    // 获取电机的转子温度（单位：摄氏度）
    using DMMotor::get_motor_temperature;

#ifndef DM_DEBUG
protected:
#endif

    int64_t location_taget = 0;        // 目标位置,回传的编码器值(16bit)作为单位 例如PMAX=12.566 那么旋转一圈为(2*PI/PMAX)*65535=32768
    int64_t speed_location_taget = 0;  // 速度目标位置,这里为了最大化精度,以回传的位置数据作为单位,0-65535映射为0-PMAX
    pid_param default_location_pid_parmater={0.1,0.1,0,2000,500};  // 默认位置PID参数
    PID_CONTROL location_pid_contraler;      // 位置PID控制器
    pid_param default_speed_pid_parmater={0.001,0.002,0.00001,1,1};    // 默认速度PID参数
    PID_CONTROL speed_pid_contraler;         // 速度PID控制器
    
    float max_Torque = 0.8; // 最大扭矩 0-1
    float taget_speed = 0;     // 目标速度



    float acceleration = 0;    // 电机加速度（单位：rad/s²）
    int speed_location_K = 1000;   // 速度环位置误差系数
    int control_frequency = 1000;  // 控制频率（单位：Hz）

    TaskHandle_t torque_func_handle = nullptr;    // 力矩控制任务句柄
    TaskHandle_t speed_func_handle = nullptr;     // 速度闭环控制任务句柄
    TaskHandle_t location_func_handle = nullptr;  // 位置闭环控制任务句柄

    // 力矩控制任务的优先级
    constexpr static int torque_task_Priority = 5;
    // 力矩控制任务的堆栈大小  
    constexpr static int torque_task_stack_size = 4096;
    // 力矩控制更新任务
    static void torque_current_task(void* p);

    // 速度控制任务的优先级
    constexpr static int speed_task_Priority = 5;  
    // 速度控制任务的堆栈大小
    constexpr static int speed_task_stack_size = 4096;
    // 速度闭环控制任务
    static void speed_contral_task(void* n);

    // 位置控制任务的优先级
    constexpr static int location_task_Priority = 5;  
    // 位置控制任务的堆栈大小
    constexpr static int location_task_stack_size = 4096;
    // 位置闭环控制任务
    static void location_contral_task(void* n);

    // 位置到力矩的映射函数 ，输入为位置，输出为力矩[-1~1]，需要在函数中限幅
    std::function<float(int64_t)> location_to_Torque_func = [](int64_t location) { return 0; };

};


HXC_DMCtrl::HXC_DMCtrl(HXC_CAN*can,int MST_ID,int CAN_ID):DMMotorMIT(can,MST_ID,CAN_ID){
    set_speed_pid(default_speed_pid_parmater);
    set_location_pid(default_location_pid_parmater);
};

HXC_DMCtrl::HXC_DMCtrl(HXC_CAN*can,int MST_ID,int CAN_ID,pid_param speed_pid,pid_param location_pid):DMMotorMIT(can,MST_ID,CAN_ID){
    set_speed_pid(speed_pid);
    set_location_pid(location_pid);
}

//初始化,位置闭环,使能
void HXC_DMCtrl::setup(bool is_enable){
    xTaskCreate(torque_current_task, "torque_current_task", torque_task_stack_size, this, torque_task_Priority, &torque_func_handle);
    if (is_enable&&speed_func_handle==nullptr) {
        xTaskCreate(speed_contral_task, "speed_contral_task", speed_task_stack_size, this, speed_task_Priority, &speed_func_handle);  
    }

    //获取三个关键参数
    read_register(Gr,&reduction_ratio);//减速比
    read_register(VMAX,&Vmax);//回传速度映射范围
    read_register(PMAX,&Pmax);//回传位置映射范围

}


// 停止电机，并根据需要卸载使能
void HXC_DMCtrl::stop(bool need_unload){
    set_speed(0);
    if (need_unload) {
        unload();
    }
}

// 设置位置闭环控制参数
void HXC_DMCtrl::set_location_pid(float _location_Kp, float _location_Ki, float _location_Kd, float __dead_zone, float _max_speed){
    location_pid_contraler.setPram(_location_Kp,_location_Ki,_location_Kd,__dead_zone,_max_speed);
}
// 设置位置闭环控制参数（传入pid_param结构体）
void HXC_DMCtrl::set_location_pid(pid_param pid){
    location_pid_contraler.setPram(pid);
}

// 设置速度闭环控制参数
void HXC_DMCtrl::set_speed_pid(float _speed_Kp, float _speed_Ki, float _speed_Kd, float __dead_zone, float _max_curunt){
    speed_pid_contraler.setPram(_speed_Kp,_speed_Ki,_speed_Kd,__dead_zone,_max_curunt);
}

// 设置速度闭环控制参数（传入pid_param结构体）
void HXC_DMCtrl::set_speed_pid(pid_param pid){
    speed_pid_contraler.setPram(pid);
}

// 设置多圈目标位置
void HXC_DMCtrl::set_location(int64_t _location){
    location_taget = _location;
}

// 设置最大力矩（0-1）
void HXC_DMCtrl::set_max_Torque(float _max_Torque){
    max_Torque = _max_Torque;
}

// 卸载使能
void HXC_DMCtrl::unload(){
    if (torque_func_handle != nullptr) {
        vTaskDelete(torque_func_handle);
        torque_func_handle = nullptr;
    }
}
// 使能电机
void HXC_DMCtrl::load(){
    if (torque_func_handle == nullptr) {
        xTaskCreate(torque_current_task, "torque_current_task", torque_task_stack_size, this, torque_task_Priority, &torque_func_handle);
    }
}
// 获取电机是否使能
bool HXC_DMCtrl::get_is_load(){
    return torque_func_handle != nullptr;
}
// 获取当前速度（单位：RPM）
float HXC_DMCtrl::get_now_speed(){
    return (2*Vmax*(get_vel_raw()-2047.f)/4096.f)*60.f/(2*PI);
}

// 设置目标速度，单位：RPM，加速度控制
void HXC_DMCtrl::set_speed(float speed, float acce){
    taget_speed = speed;
    acceleration = acce;
    if (speed_func_handle == nullptr) {
        xTaskCreate(speed_contral_task, "speed_contral_task", speed_task_stack_size, this, speed_task_Priority, &speed_func_handle);
    }
}

// 获取转子目标速度
float HXC_DMCtrl::get_taget_speed(){
    return taget_speed;
}
// 设置电机加速度，单位：RPM/s
void HXC_DMCtrl::set_acceleration(float acce){
    acceleration = acce;
}

// 获取减速箱减速比
float HXC_DMCtrl::get_reduction_ratio(){
    return reduction_ratio;
}
// 设置速度环位置误差系数
void HXC_DMCtrl::set_speed_location_K(float _K){
    speed_location_K = _K;
}

// 获取闭环控制频率
int HXC_DMCtrl::get_control_frequency(){
    return control_frequency;
}

// 设置闭环控制频率 0~1000
void HXC_DMCtrl::set_control_frequency(int _control_frequency){
    control_frequency = _control_frequency;
}

// 当电机的输出电流需要根据位置进行映射的时候，可以传入位置到电流映射函数的指针
// 例如电机做摇臂控制电机时，电流需要做三角函数映射，那么可以传入经过三角函数的位置到电流映射函数的指针
void HXC_DMCtrl::add_location_to_Torque_func(std::function<int(int64_t)> func){
    location_to_Torque_func = func;
}

// 力矩控制任务的入口函数
void HXC_DMCtrl::torque_current_task(void* p){
    HXC_DMCtrl* motor = (HXC_DMCtrl*)p;
    auto xLastWakeTime = xTaskGetTickCount();
    while (1) {
        motor->sendMITpakage();
        //控制频率
        xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / motor->control_frequency);
    }
}

// 速度控制任务的入口函数
void HXC_DMCtrl::speed_contral_task(void* n){
    HXC_DMCtrl* moto = (HXC_DMCtrl*) n;
    //上次更新时间
    int last_update_speed_time=micros();
    moto->speed_pid_contraler.reset();
    //在unload过后出现扰动，再次load之后不会回到扰动前的位置
    moto->speed_location_taget = moto->get_location();

    float taget_control_speed = moto->taget_speed;
    float last_taget_control_speed = moto->taget_speed;
    auto xLastWakeTime = xTaskGetTickCount ();
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
        moto->speed_location_taget+=moto->is_online()*65535
        *((taget_control_speed*2*PI/60.f)/(2*moto->Pmax))
        *delta_time;//位置误差,只有电机在线才计算累计位置
        //更新上次更新时间
        last_update_speed_time=micros();

        //由速度误差和位置误差一同计算电流
        double err = 
        /*速度环的误差=*/(taget_control_speed - moto->get_now_speed())
        +
        /*速度环位置误差比例系数=*/moto->speed_location_K/*这里的比例系数需要根据实际情况调整,比例系数speed_location_K可以理解为转子每相差一圈加 speed_location_K RPM速度补偿*/
        * 
        /*由速度计算得到的目标位置的误差*/(moto->speed_location_taget-moto->get_location())/65535;

        
        //计算控制力矩
        float Torque = 
        /*位置映射的力矩=*/moto->location_to_Torque_func(moto->get_location())
        +
        /*PID控制器的计算力矩=*/moto->speed_pid_contraler.control(moto->is_online()*err);//电机在线才计算电流
        
        //限幅
        Torque=Torque<-1? -1:Torque>1?1:Torque;

        moto->set_tff((Torque*2047)+2047);//设置力矩
        //控制频率
        xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / moto->control_frequency);
    }
}
// 位置控制任务的入口函数
void HXC_DMCtrl::location_contral_task(void* n){
    HXC_DMCtrl* moto = (HXC_DMCtrl*) n;
    moto->location_pid_contraler.reset();//重置位置闭环控制器
    float speed=0;
    auto xLastWakeTime = xTaskGetTickCount ();
    while (1){
        //位置闭环控制,由位置误差决定速度,再由速度误差决定电流
        speed = moto->location_pid_contraler.control(moto->location_taget - moto->get_location());
        moto->taget_speed = speed;
        //控制频率
        xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / moto->control_frequency);
    }
};
#endif
/*
                                              .=%@#=.                                               
                                            -*@@@@@@@#=.                                            
                                         .+%@@@@@@@@@@@@#=                                          
                                       -#@@@@@@@* =@@@@@@@@*:                                       
                                     =%@@@@@@@@=   -@@@@@@@@@#-                                     
                                  .+@@@@@@@@@@-     .@@@@@@@@@@%=                                   
                                .+@@@@@@@@@@@@-     +@@@@@@@@@@@@@+.                                
                               +@@@@@@@@@@@@@@@    .@@@@@@@@@@@@@@@@+.                              
                             =@@@@@@@@@@@@@@@%-     =%@@%@@@@@@@@@@@@@=                             
                           -%@@@@@@@@@@@@+..     .       -@@@@@@@@@@@@@%-                           
                         .#@@@@@@@@@@@@@#       -@+       +@@@@@@@@@@@@@@#:                         
                        +@@@@@@@@@@@@@@@@+     +@@@+     =@@@@@@@@@@@@@@@@@+                        
                      :%@@@@@@@@@@@@@@@@@+    *@@@@*     =@@@@@@@@@@@@@@@@@@%-                      
                     +@@@@@@@@@@@@@@#+*+-   .#@@@@+       :+*+*@@@@@@@@@@@@@@@*                     
                   :%@@@@@@@@@@@@@@+       :%@@@@-    .-       -@@@@@@@@@@@@@@@%:                   
                  =@@@@@@@@@@@@@@@@-      -@@@@%:    .%@+      =@@@@@@@@@@@@@@@@@=                  
                 *@@@@@@@@@@@@@@@@@@.    =@@@@#.    -@@@@+    =@@@@@@@@@@@@@@@@@@@#                 
               .%@@@@@@@@@@@@@@@@@@+    +@@@@*     =@@@@%:    .#@@@@@@@@@@@@@@@@@@@%.               
              :@@@@@@@@@@@@@@@%:::.    #@@@@+     +@@@@#        .::.*@@@@@@@@@@@@@@@@-              
             -@@@@@@@@@@@@@@@%       .%@@@@=     *@@@@*     +-       *@@@@@@@@@@@@@@@@=             
            =@@@@@@@@@@@@@@@@@#.    -@@@@@-    :%@@@@=    .#@@+     +@@@@@@@@@@@@@@@@@@=            
           =@@@@@@@@@@@@@@@@@@@:    =====.     -+===:     :====     @@@@@@@@@@@@@@@@@@@@+           
          +@@@@@@@@@@@@@@@#%%#-                                     :*%%#%@@@@@@@@@@@@@@@+          
         =@@@@@@@@@@@@@@%.       ...........................              *@@@@@@@@@@@@@@@=         
        =@@@@@@@@@@@@@@@+      .#@@@@@@@@@@@@@@@@@@@@@@@@@@#     .*:      =@@@@@@@@@@@@@@@@-        
       -@@@@@@@@@@@@@@@@@=    .%@@@@@@@@@@@@@@@@@@@@@@@@@@#     :@@@-    =@@@@@@@@@@@@@@@@@@:       
      :@@@@@@@@@@@@@@@@@%.   -@@@@%+=====================:     -@@@@%    :%@@@@@@@@@@@@@@@@@@.      
      %@@@@@@@@@@@@@=-=:    =@@@@#.                           +@@@@#.      -=--%@@@@@@@@@@@@@%      
     #@@@@@@@@@@@@@:       +@@@@*      ............. .       *@@@@*             %@@@@@@@@@@@@@+     
    =@@@@@@@@@@@@@@#.     #@@@@+     +@@@@@@@@@@@@@@@#.    .#@@@@+     +#.     +@@@@@@@@@@@@@@@:    
   .@@@@@@@@@@@@@@@@-   .%@@@@=     *@@@@@@@@@@@@@@@#     :%@@@@-     *@@%:    @@@@@@@@@@@@@@@@%    
   %@@@@@@@@@@@%%%#=   :@@@@@:    .#@@@@+-----------     -@@@@@:     #@@@@=    :#%%%@@@@@@@@@@@@*   
  =@@@@@@@@@@@=       -@@@@%.    :%@@@@-                =@@@@%.    .%@@@@=          :%@@@@@@@@@@@:  
  @@@@@@@@@@@%.      =@@@@#     -@@@@%:    .:::-:      +@@@@#     :@@@@@:    .       +@@@@@@@@@@@#  
 +@@@@@@@@@@@@@.    *@@@@*     =@@@@#.    -@@@@@:     #@@@@+     =@@@@%.    -@#     +@@@@@@@@@@@@@- 
.@@@@@@@@@@@@@#    *@%@%=     +@@@@*     =@@@@#.    .#@@@%=     +@@@@#     =@@@%.   =@@@@@@@@@@@@@% 
+@@@@@@@@*-==-                .          .           . ..       .....      .....     .=+=+@@@@@@@@@-
%@@@@@@@+                                                                                 -@@@@@@@@#
@@@@@@@-       =#%#=     -#%%#-     -#%%*.     +%%%*.    .*%%#=     :#%%#-     =%%%*.      .#@@@@@@@
@@@@@@=.::::::*@@@@@*:::-@@@@@@-:::=@@@@@%::::*@@@@@#::::%@@@@@+:---@@@@@@=---+@@@@@%------:=@@@@@@@
=@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@+
 *@@@@@@@@@@@@@@@@@@@@@@@@@@@%%##**++===----:::::------===++***##%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@* 
  -#@@@@@@@@@@@@@@@@%#*+=-:.                                        ..::-=+*##%@@@@@@@@@@@@@@@@@#-  
    :=*%@@@@@%#*=-:                                                             .:-=+*#%%%%##+-.    
                                                                                        
        K####      #####     ###    ###  ######.   ##########     K##    ### ###    ##W    ####W    
       #######    #######    ###    ###  ########  ##########     ###    ### ###   ###   W######    
      W###G####  ###W ####   ###    ###  ######### ##########     ###    ###  ##   ###   ###W####   
      ###   ###  ###   ###   ###    ##  ###    ###    ###         ###    ###  ### t##   ###   ###   
     G##    #   ###    ###   ##     ##  ###    ###    ###         ###    ###  ### ###   ##W         
     ###        ###    ###   ##    ###  ###    ###    ###         ##L    ##   ### ##   ###          
     ###        ###    ###  K##    ###  ###    ###    ###         ##     ##    #####   ###          
     ###       ,##     ###  ###    ###  ###   ###,    ##         G##    ###    ####    ###          
    W##        ###     ###  ###    ###  #########     ##         ##########    ####    ###          
    ###        ###     ###  ###    ###  ########     ###         ##########    ###i   K##           
    ###        ###     ###  ###    ##  #######       ###         ###    ###    ####   ###           
    ###        ###     ###  ##     ##  ###           ###         ###    ###   ##W##   ###           
    ###        ###     ##i  ##    ###  ###           ###         ###    ##    ## ##   ###           
    ###        ###    ###  ,##    ###  ###           ###         ##     ##   ### ##   ###           
    ###    ### ###    ###  K##    ###  ###           ##         t##    ###   ##  ###  ###    ###    
    ###   G##i ###   ###   .##   ###.  ##t           ##         ###    ###  ###  ###  W##,   ###    
     ########  W##W#####    ########   ##           ###         ###    ###  ##    ##   ####W###     
     #######    #######     #######   ###           ###         ###    ### ###    ##.  #######      
      #####      #####       #####    ###           ###         ###    ### ##W    ###   #####       
                   ###                                                                              
                   ###                                                                              
                   #####                                                                            
                    ####                                                                            
                      K                                                                             
*/