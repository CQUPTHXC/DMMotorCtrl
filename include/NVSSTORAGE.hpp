/*
 * @LastEditors: qingmeijiupiao
 * @Description: 
 * @Author: qingmeijiupiao
 * @Date: 2024-10-13 17:21:48
 */
#ifndef NVSSTORAGE_HPP
#define NVSSTORAGE_HPP
#include "Preferences.h"
namespace NVSSTORAGE {
    Preferences preferences;
    uint8_t pair_mac[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    void NVS_read() {
        preferences.begin("DATA", false);

        preferences.end();
    }

    void NVS_save() {
        preferences.begin("DATA", false);

        preferences.end();
    }
}
#endif //NVSSTORAGE_HPP