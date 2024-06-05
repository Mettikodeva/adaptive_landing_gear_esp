#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "Wire.h"
#include "VL53L0X.h"
#include "pca9548a.h"
#include "esp_system.h"
static const char *TAG = "VL53L0X";

//lpf
float alpha = (float)(CONFIG_LIDAR_LPF_ALPHA/100.0);
VL53L0X **sensor = new VL53L0X*[3];
PCA9548A *mux = new PCA9548A(I2C_NUM_0);
uint16_t distance[3];
TickType_t lastTimeLidar = 0;
TickType_t lastTime;

void lidarTask(void *pvParameter){
  //default 12, 31, 28
  int offsets[3] = {CONFIG_LIDAR1_OFFSET, CONFIG_LIDAR2_OFFSET, CONFIG_LIDAR3_OFFSET};
  ESP_LOGI(TAG, "Lidar Task Started!");
  for (int i = 0; i < 3; i++)
  {
    sensor[i] = new VL53L0X();
    delayMicroseconds(10);
    mux->selectChannel(i);
    delayMicroseconds(10);
    sensor[i]->setTimeout(100);

    if (sensor[i]->init())
    {
      ESP_LOGI("VL53L0X", "Sensor %d initialized", i);
    }
    else
    {
      ESP_LOGE("VL53L0X", "Sensor %d not initialized", i);
    }
    sensor[i]->setSignalRateLimit(0.3); // default 0.25

    // //default pre 14 and final 10 valid pre 12 to 18 valid final 8 to 14
    // // even number only!!
    // sensor[i]->setVcselPulsePeriod(VL53L1X::VcselPeriodPreRange, 16);
    // sensor[i]->setVcselPulsePeriod(VL53L1X::VcselPeriodFinalRange, 12);

    sensor[i]->setMeasurementTimingBudget(33000);
    uint32_t timing = sensor[i]->getMeasurementTimingBudget();
    ESP_LOGI("VL53L0X", "%d", i);
    ESP_LOGI("VL53L0X", "Measurement timing budget: %d", (int)timing);
    sensor[i]->startContinuous(0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    for (;;){
      // uint32_t duration[3] = {0,0,0};
      for (int i = 0; i < 3; i++)
      {
        // TickType_t start = xTaskGetTickCount();
        mux->selectChannel(i);
        delayMicroseconds(10);
        uint16_t dist;
        dist = sensor[i]->readRangeContinuousMillimeters();
        // printf("Distance: %d mm\n", dist);
        if (sensor[i]->timeoutOccurred())
        {
          Serial.print(" TIMEOUT");
        }
        dist -= offsets[i]; // distance - offset
        if (dist < CONFIG_LIDAR_MIN)
        {
          distance[i] = 0;
        }
        else if (dist > CONFIG_LIDAR_MAX)
        {
          distance[i] = 0;
        }
        else
        {
          distance[i] = (uint16_t)((1 - alpha) * dist + alpha * distance[i]);
        }
        // duration[i] = xTaskGetTickCount() - start;    
        vTaskDelay(9 / portTICK_PERIOD_MS);
      }
        // printf("Distance %d\t%d\t %d,t: %ld\n", distance[0], distance[1], distance[2], (xTaskGetTickCount()-lastTime)*portTICK_PERIOD_MS);
        // printf("Distance %d %ld \t%d %ld \t %d %ld , t: %ld\n", distance[0],duration[0], distance[1],duration[1], distance[2],duration[2], (xTaskGetTickCount()-lastTime)*portTICK_PERIOD_MS);
        lastTime = xTaskGetTickCount();
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
