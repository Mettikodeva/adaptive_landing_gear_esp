#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Arduino.h"
#include "cstdio"
#include "filters.h"

// MARK: MACRO
#define MAP(x,in_min,in_max,out_min,out_max) (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

int16_t adc_value[3] = {0,0,0};
int16_t adc_value_filtered[3] = {0,0,0};


static int8_t pin[3] = {35,32,33};

Iir **filters = new Iir*[3];

static const char *TAG = "ADC";

float getAngle(int8_t servo){

    /*
        servo 1: 
            deg = 0 -> min adc = 3874, max adc = 4092, mean adc = 3983
            deg = 90 -> min adc = 2002, max adc = 2140, mean adc = 2071
    
        servo 2:
            deg = 0 -> min adc = 3779, max adc = 4089, mean adc = 3934
            deg = 90 -> min adc = 1809, max adc = 2000, mean adc = 1904
        servo 3:
            deg = 0 -> min adc = 3695, max adc = 3919, mean adc = 3807
            deg = 90 -> min adc = 1885, max adc = 2027, mean adc = 1956
    */
    float adc = adc_value_filtered[servo-1];
    float deg = 0;
    if (servo == 1){
        deg = MAP(adc, CONFIG_SERVO1_ADC_MIN, CONFIG_SERVO1_ADC_MAX, CONFIG_SERVO_ANGLE_MIN, CONFIG_SERVO_ANGLE_MAX);
    }
    else if (servo == 2){
        deg = MAP(adc, CONFIG_SERVO2_ADC_MIN, CONFIG_SERVO2_ADC_MAX, CONFIG_SERVO_ANGLE_MIN, CONFIG_SERVO_ANGLE_MAX);
    }
    else if (servo == 3){
        deg = MAP(adc, CONFIG_SERVO3_ADC_MIN, CONFIG_SERVO3_ADC_MAX, CONFIG_SERVO_ANGLE_MIN, CONFIG_SERVO_ANGLE_MAX);
    }
    // printf("Servo %d: %f, %f\n", servo,adc, deg);w


    return deg;
}

void startAdcTask(void *args){
    ESP_LOGI(TAG,"ADC Task\n");
    for (int i = 0; i < 3; i++)
    {
        filters[i] = new Iir(CONFIG_ADC_LPF_ALPHA/100);
    }
    
    pinMode(pin[0], INPUT);
    pinMode(pin[1], INPUT);
    pinMode(pin[2], INPUT);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    TickType_t last_time = xTaskGetTickCount();
    for (;;){
        for (int i = 0; i < 3; i++){
            adc_value[i] = analogRead(pin[i]);
            adc_value_filtered[i] = filters[i]->filter(adc_value[i]);
        }
        // ESP_LOGD(TAG,"ADC Value Raw: %d %d %d\n", adc_value[0], adc_value[1], adc_value[2]);
        // ESP_LOGI(TAG,"ADC Value: %d %d %d\n", adc_value_filtered[0], adc_value_filtered[1], adc_value_filtered[2]);

        vTaskDelayUntil(&last_time, 10/ portTICK_PERIOD_MS);
    }
}