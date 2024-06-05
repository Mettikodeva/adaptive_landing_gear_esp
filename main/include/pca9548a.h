#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"


// PCA9548A I2C address
#define PCA9548A_I2C_ADDR        0x70  // PCA9548A I2C address

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000           /*!< I2C master clock frequency */

class PCA9548A {
private:
    i2c_port_t i2c_port;
public:
    PCA9548A(i2c_port_t port);
    PCA9548A(int port);
    ~PCA9548A();
    bool init_i2c();
    void writeRegister(uint8_t data);
    void selectChannel(uint8_t channel);
    void selectChannel(int channel);
};

