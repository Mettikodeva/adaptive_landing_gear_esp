#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Arduino.h"
#include "cstdio"
#include "filters.h"
#include "driver/gptimer.h"

// MARK: MACRO
#define MAP(x,in_min,in_max,out_min,out_max) (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#define WINDOW 10
int16_t adc_value[3] = {0,0,0};
int16_t adc_value_filtered[3] = {0,0,0};


static int8_t pin[3] = {35,32,33};

Iir **filters = new Iir*[3];

static const char *TAG = "ADC";
extern TaskHandle_t TaskHandle_ADC;

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
        if (adc > CONFIG_SERVO1_ADC_MIN){
            ESP_LOGW(TAG,"1: value out of range");
            adc = CONFIG_SERVO1_ADC_MIN;
        }
        else if (adc < CONFIG_SERVO1_ADC_MAX){
            ESP_LOGW(TAG,"1: value out of range");
            adc = CONFIG_SERVO1_ADC_MAX;
        }
        deg = MAP(adc, (int16_t)CONFIG_SERVO1_ADC_MIN, (int16_t)CONFIG_SERVO1_ADC_MAX, (float)CONFIG_SERVO_ANGLE_MIN, (float)CONFIG_SERVO_ANGLE_MAX);
    }
    else if (servo == 2){
        if (adc > CONFIG_SERVO2_ADC_MIN){
            ESP_LOGW(TAG,"2: value out of range");
            adc = CONFIG_SERVO2_ADC_MIN;
        }
        else if (adc < CONFIG_SERVO2_ADC_MAX){
            ESP_LOGW(TAG,"2: value out of range");
            adc = CONFIG_SERVO2_ADC_MAX;
        }
        deg = MAP(adc, (int16_t)CONFIG_SERVO2_ADC_MIN, (int16_t)CONFIG_SERVO2_ADC_MAX, (float)CONFIG_SERVO_ANGLE_MIN, (float)CONFIG_SERVO_ANGLE_MAX);
    }
    else if (servo == 3){
        if (adc > CONFIG_SERVO3_ADC_MIN){
            ESP_LOGW(TAG,"3: value out of range");
            adc = CONFIG_SERVO3_ADC_MIN;
        }
        else if (adc < CONFIG_SERVO3_ADC_MAX){
            ESP_LOGW(TAG,"3: value out of range");
            adc = CONFIG_SERVO3_ADC_MAX;
        }
        deg = MAP(adc, (int16_t)CONFIG_SERVO3_ADC_MIN, (int16_t)CONFIG_SERVO3_ADC_MAX, (float)CONFIG_SERVO_ANGLE_MIN, (float)CONFIG_SERVO_ANGLE_MAX);
    }
    // printf("Servo %d: %f, %f\n", servo,adc, deg);w
    return deg;
}

void startAdcTask(void *args){
    ESP_LOGI(TAG,"ADC Task\n");
    for (int i = 0; i < 3; i++)
    {
        filters[i] = new Iir((int)CONFIG_ADC_LPF_ALPHA/100.0f);
    }
    pinMode(pin[0], INPUT);
    pinMode(pin[1], INPUT);
    pinMode(pin[2], INPUT);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    TickType_t last_time;
    // int16_t *arr = (int16_t*)malloc(WINDOW * sizeof(int16_t));
    #ifdef CONFIG_LOG_ADC_RAW
        ESP_LOGI("LOG_RAW_ADC","T,a1,a2,a3");
    #endif
    
    // timer 
    // gptimer_handle_t timer_handle;
    // gptimer_config_t config{
    //     .clk_src = GPTIMER_CLK_SRC_APB,
    //     .direction = GPTIMER_COUNT_UP,
    //     .resolution_hz = 10000, // 10 kHz
    // };

    // ESP_ERROR_CHECK(gptimer_new_timer(&config, &timer_handle));
    // gptimer_event_callbacks_t callbacks {
    //     .on_alarm = onTimer,
    // };
    // gptimer_register_event_callbacks(timer_handle, &callbacks, NULL);

    // ESP_ERROR_CHECK(gptimer_enable(timer_handle));

    // gptimer_alarm_config_t alarm_config{
    //     .alarm_count = 329, // 100 us * 330 = 33 ms
    //     .reload_count= 330,
    //     .flags = {
    //         .auto_reload_on_alarm = true,
    //     },
    // };

    // gptimer_set_alarm_action(timer_handle, &alarm_config);
    // ESP_ERROR_CHECK(gptimer_start(timer_handle));

    last_time = xTaskGetTickCount();
    for (;;){
        vTaskDelayUntil(&last_time, 33/ portTICK_PERIOD_MS);
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (int i = 0; i < 3; i++)
        {
            adc_value[i] = analogRead(pin[i]);
            adc_value_filtered[i] = filters[i]->filter(adc_value[i]);
        }
        // adc_value_filtered[0] = filters[0]->filter(adc_value[0]);
        // adc_value_filtered[1] = filters[1]->filter(adc_value[1]);
        // adc_value_filtered[2] = filters[2]->filter(adc_value[2]);
        #ifdef CONFIG_LOG_ADC_RAW
        ESP_LOGI("LOG_RAW_ADC","%d,%d,%d",adc_value[0],adc_value[1],adc_value[2]);
        #endif
    }

    vTaskDelete(NULL);
}