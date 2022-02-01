#ifndef bmm_h
#define bmm_h

#include "Arduino.h"
#include "Preferences.h"
#include "math.h"
#include <M5Stack.h>
#include "M5_BMM150.h"
#include "M5_BMM150_DEFS.h"

class BMM150{
    private:
        Preferences prefs;
        struct bmm150_dev dev;
        bmm150_mag_data mag_offset; // Compensation magnetometer float data storage 储存补偿磁强计浮子数据
        bmm150_mag_data mag_max;
        bmm150_mag_data mag_min;
        static int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
        static int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
    public:
        BMM150();
        int8_t Initialize();
        void SaveOffset();
        void LoadOffset();
        void Calibrate(uint32_t calibrate_time);
        float ReadHeading();
};

#endif