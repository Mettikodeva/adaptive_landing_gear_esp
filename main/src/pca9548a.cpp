#include "pca9548a.h"

static const char* TAG = "PCA9548A";

PCA9548A::PCA9548A(i2c_port_t port){
    
    i2c_port = port;
}

PCA9548A::PCA9548A(int port) : PCA9548A((i2c_port_t)port) {}

bool PCA9548A::init_i2c() {
    // Initialize I2C bus
    // for more information on I2C configuration, check the API reference
    // https://docs.espressif.com/projects/esp-idf/en/v5.1.3/esp32/api-reference/peripherals/i2c.html

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;    // Adjust these pins as per your setup
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;

    esp_err_t res =  i2c_param_config(i2c_port, &conf);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2C parameters ");
        ESP_LOGE(TAG, "Error code: %s", esp_err_to_name(res));
        return false;
    }
    res = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2C parameters ");
        ESP_LOGE(TAG, "Error code: %s", esp_err_to_name(res));
        return false;
    }
    return true;
}


void PCA9548A::selectChannel(uint8_t channel) {
    uint8_t data = 1 << channel;
    this->writeRegister(data);
}

void PCA9548A::selectChannel(int channel) {
    this->selectChannel((uint8_t)channel);
}


void PCA9548A::writeRegister(uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9548A_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        return;
    } 
    ESP_LOGE(TAG, "Failed to write to PCA9548A");
}
