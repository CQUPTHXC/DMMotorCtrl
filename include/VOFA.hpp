#ifndef VOFA_HPP
#define VOFA_HPP

#include "Arduino.h"
#include <map>

class VOFA_float {
public:
    VOFA_float(String _name, float default_value = 0.0f) {
        this->name = _name;
        this->value = default_value;
        VOFA_float::name_to_value_map[this->name] = this;
    }

    void setup(Stream* serial = &Serial) {
        if (!is_setup) {
            serial_port = serial;
            is_setup = true;
            xTaskCreate(read_loop, "VOFA_ReadLoop", 4096, NULL, 5, NULL);
        }
    }

    operator float() const {
        return value;
    }

    void set(float val) {
        value = val;
    }

    float get() const {
        return value;
    }

protected:
    float value;
    String name;

    static std::map<String, VOFA_float*> name_to_value_map;
    static bool is_setup;
    static Stream* serial_port;

    static void read_loop(void* p) {
        while (true) {
            if (serial_port->available()) {
                String str = serial_port->readStringUntil('\n');
                str.trim();

                // 替代 contains 的写法
                if (str.length() == 0 || str.indexOf(':') == -1) continue;

                String key = str.substring(0, str.indexOf(':'));
                float val = str.substring(str.indexOf(':') + 1).toFloat();

                auto it = name_to_value_map.find(key);
                if (it != name_to_value_map.end()) {
                    it->second->set(val);
                } else {
                    serial_port->println("VOFA_float: " + key + " not found");
                }
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
};

// 静态成员变量初始化
std::map<String, VOFA_float*> VOFA_float::name_to_value_map;
bool VOFA_float::is_setup = false;
Stream* VOFA_float::serial_port = nullptr;

#endif
