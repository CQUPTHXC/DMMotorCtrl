/*
 * @LastEditors: qingmeijiupiao
 * @Description: 遥控器相关
 * @Author: qingmeijiupiao
 * @Date: 2024-11-14 21:40:14
 */
#pragma once
#include <Arduino.h>
#include "ESPNOW.hpp"
struct CONTROLLER_DATA_t {
public:
    float lx,ly,rx,ry,/*-0.5-0.5*/
    lknob,rknob;/*0-1*/
    
    bool LUshouder,LDshouder,RUshouder,RDshouder;

    bool l1sw,l2sw,r1sw,r2sw;

    bool lleft,lright,lup,ldown;

    bool rleft,rright,rup,rdown;
    //编码数据到数组
    void data_to_array(uint8_t* arr){
        *(float*)arr=ly;
        *(float*)(arr+4)=lx;
        *(float*)(arr+8)=ry;
        *(float*)(arr+12)=rx;
        *(float*)(arr+16)=lknob;
        *(float*)(arr+20)=rknob;
        uint8_t UButtonbit=(LUshouder<<0)|(LDshouder<<1)|(RUshouder<<2)|(RDshouder<<3)|(l1sw<<4)|(l2sw<<5)|(r1sw<<6)|(r2sw<<7);
        arr[24]=UButtonbit;
        uint8_t DButtonbit=(lleft<<0)|(lright<<1)|(lup<<2)|(ldown<<3)|(rleft<<4)|(rright<<5)|(rup<<6)|(rdown<<7);
        arr[25]=DButtonbit;
    }
    //解码
    void array_to_data(uint8_t* arr){
        ly=*(float*)arr;
        lx=*(float*)(arr+4);
        ry=*(float*)(arr+8);
        rx=*(float*)(arr+12);
        lknob=*(float*)(arr+16);
        rknob=*(float*)(arr+20);

        uint8_t UButtonbit;
        UButtonbit=arr[24];
        LUshouder=UButtonbit&0x01;
        LDshouder=(UButtonbit&0x02)>>1;
        RUshouder=(UButtonbit&0x04)>>2;
        RDshouder=(UButtonbit&0x08)>>3;
        l1sw=(UButtonbit&0x10)>>4;
        l2sw=(UButtonbit&0x20)>>5;
        r1sw=(UButtonbit&0x40)>>6;
        r2sw=(UButtonbit&0x80)>>7;
        
        uint8_t DButtonbit;
        DButtonbit=arr[25];
        lleft=DButtonbit&0x01;
        lright=(DButtonbit&0x02)>>1;
        lup=(DButtonbit&0x04)>>2;
        ldown=(DButtonbit&0x08)>>3;
        rleft=(DButtonbit&0x10)>>4;
        rright=(DButtonbit&0x20)>>5;
        rup=(DButtonbit&0x40)>>6;
        rdown=(DButtonbit&0x80)>>7;
    }
};
//遥控器数据结构体对象
CONTROLLER_DATA_t remote_data;


void test_callback(ESPNOW::data_package ESPNOW_DATA){
  remote_data.array_to_data(ESPNOW_DATA.data);
}