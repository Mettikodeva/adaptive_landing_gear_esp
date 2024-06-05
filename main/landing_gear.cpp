
#include "Arduino.h"
#include "servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "pid.h"
#include "VL53L0X.h"
#include "pca9548a.h"
#include "freertos/queue.h"
#include "helper_3dmath.h"
#include <inttypes.h>
#include "iot_button.h"
#include "headers.h"

// MARK: MACROS
// adjust these values to match the servo's calibration values
#define MAP(x,in_min,in_max,out_min,out_max) (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#define PWM1(x) MAP(x, 0, 90, 2370, 1050)
#define PWM2(x) MAP(x, 0, 90, 2460, 1090)
#define PWM3(x) MAP(x, 0, 90, 2500, 1070)


// MARK: Variables
Servo *servo = new Servo(20);

TaskHandle_t TaskHandle_IMU;
TaskHandle_t TaskHandle_initServo;
TaskHandle_t TaskHandle_sweepServo;
TaskHandle_t TaskHandle_lidar;
TaskHandle_t TaskHandle_lidarControl;
TaskHandle_t TaskHandle_imuControl;
TaskHandle_t TaskHandle_servoWrite;
TaskHandle_t TaskHandle_standby;

SemaphoreHandle_t servo_mutex = xSemaphoreCreateBinary();


static const char *TAG = "lg";
static const char *TAG_LOG = "LOG";
static const char *TAG_BUTTON = "button";
// IMU
extern void imuTask(void *);
extern float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float lidar_pr[2]; // pitch, roll
float prev_ypr[3];
extern SemaphoreHandle_t imu_mutex;
// END IMU

// ADC
extern void startAdcTask(void *);
extern float getAngle(int8_t servo);
// END ADC

// CONTROL //pcval = pitch control value, rcval = roll control value
float kpl = (int)CONFIG_KP_VAL_LIDAR / 100.0f;
float kil = (int)CONFIG_KI_VAL_LIDAR / 100.0f;
float kdl = (int)CONFIG_KD_VAL_LIDAR / 100.0f;
// float kpl = 0.5;
// float kil = 0;
// float kdl = 0.001;
#ifdef CONFIG_INT_SAT_LIDAR_VAL
float IntSatL = CONFIG_INT_SAT_LIDAR_VAL;
#endif

float kpi = (int)CONFIG_KP_VAL_IMU / 100.0f;
float kii = (int)CONFIG_KI_VAL_IMU / 100.0f;
float kdi = (int)CONFIG_KD_VAL_IMU / 100.0f;
// float kpi = 0.5;
// float kii = 0.01;
// float kdi = 0.1;
#ifdef CONFIG_INT_SAT_IMU_VAL
float IntSatI = CONFIG_INT_SAT_IMU_VAL;
#endif

float m1 = 0, m2 = 0, m3 = 0, pcval_imu = 0, rcval_imu = 0, pcval_lidar = 0, rcval_lidar = 0;
SemaphoreHandle_t imu_control_mutex = xSemaphoreCreateBinary();
SemaphoreHandle_t lidar_control_mutex = xSemaphoreCreateBinary();
#ifdef CONFIG_INT_SAT_LIDAR_VAL
    PID * pid_pitch = new PID(kpl, kil, kdl, IntSatL); // lidar
    PID * pid_roll = new PID(kpl, kil, kdl, IntSatL);  // lidar 
#else
    PID * pid_roll = new PID(kpl, kil, kdl);  // lidar 
    PID * pid_pitch = new PID(kpl, kil, kdl); // lidar
#endif
#ifdef CONFIG_INT_SAT_IMU_VAL
    PID * pid_roll2 = new PID(kpi, kii, kdi, IntSatI); // imu
    PID * pid_pitch2 = new PID(kpi, kii, kdi, IntSatI); // imu
#else
    PID * pid_roll2 = new PID(kpi, kii, kdi); // imu
    PID * pid_pitch2 = new PID(kpi, kii, kdi); // imu
#endif

int base_pwm = 1200; // in microseconds
int default_angle = 30;
int prev_pwm[3] = {0, 0, 0}; // in microseconds
VectorInt16 legs[3];    // x, y, z
int pwms[3] = {1200,1200,1200};          // in microseconds

char tolerance = 1; // in degrees
// END CONTROL


// LIDAR
extern VL53L0X **sensor;
extern void lidarTask(void *);
extern uint16_t distance[3];
extern PCA9548A *mux;
// END LIDAR

// main  var
// extern typedef enum
// {
// } State_LG_t;
extern State_LG_t landing_gear_state;


SemaphoreHandle_t pid_mutex = xSemaphoreCreateBinary();


// MARK: Helper Functions
float deg2rad(float deg){
    return deg * PI / 180;
}

float rad2deg(float rad){
    return rad * 180 / PI;
}

VectorInt16 getPlaneNorm(VectorInt16 a, VectorInt16 b, VectorInt16 c){
    VectorInt16 u = {int16_t(b.x - a.x), int16_t(b.y - a.y), int16_t(b.z - a.z)}; 
    VectorInt16 v = {int16_t(c.x - a.x), int16_t(c.y - a.y), int16_t(c.z - a.z)};

    // normal vector of the plane
    /*  | i   j   k |
        | u.x u.y u.z |
        | v.x v.y v.z |
        
        |u.y u.z|       |u.x u.z|       |u.x u.y|
        |v.y v.z| * i - |v.x v.z| * j + |v.x v.y| * k

        i(u.y * v.z - u.z * v.y) - j(u.x * v.z - u.z * v.x) + k(u.x * v.y - u.y * v.x)
    */

    VectorInt16 n = {int16_t(u.y * v.z - u.z * v.y), int16_t(u.z * v.x - u.x * v.z), int16_t(u.x * v.y - u.y * v.x)};
    return n;
}

// MARK: Tasks
void servoWriteTask(void *pvParameter){
    // printf("Servo Write Task\n");
    // xSemaphoreTakeRecursive(servo_mutex, 10/portTICK_PERIOD_MS);
    // for (int i = 2000;i<base_pwm;i-=1){
    //     servo->writeMicroseconds(0,i);
    //     servo->writeMicroseconds(1,i);
    //     servo->writeMicroseconds(2,i);
    //     vTaskDelay(500 / portTICK_PERIOD_MS);
    // }
    // xSemaphoreGiveRecursive(servo_mutex);
    TickType_t lastTime;
    #ifdef CONFIG_LOG_DATA_TO_SERIAL
        char data[50];
        #ifdef CONFIG_LOG_SERVO
            sprintf(data, "T,pwm1,pwm2,pwm3");
            ESP_LOGI("LOG_SERVO", "%s",data);
        #endif
        #ifdef CONFIG_LOG_PID
            sprintf(data, "T,pcl,rcl,pci,rci");
            ESP_LOGI("LOG_PID", "%s",data);
        #endif
    #endif

        lastTime = xTaskGetTickCount();
        for (;;)
        {
            vTaskDelayUntil(&lastTime, 100 / portTICK_PERIOD_MS);

            // if (RAD_TO_DEG * ypr[1] <= tolerance && RAD_TO_DEG * ypr[1] >= -tolerance && RAD_TO_DEG * ypr[0] <= tolerance && RAD_TO_DEG * ypr[0] >= -tolerance){
            //     vTaskDelayUntil(&lastTime, 20 / portTICK_PERIOD_MS);
            //     continue;
            // }
            
            // xQueueTakeMutexRecursive(imu_control_mutex, 10/portTICK_PERIOD_MS);
            // xQueueTakeMutexRecursive(lidar_control_mutex, 10/portTICK_PERIOD_MS);
            
            m1 += (-pcval_lidar) - pcval_imu - 3 * (rcval_imu + rcval_lidar);
            m2 += (-pcval_lidar) - pcval_imu + 3 * (rcval_imu + rcval_lidar);
            m3 += 2 * (pcval_imu + pcval_lidar);
            
            // xQueueGiveMutexRecursive(imu_control_mutex);
            // xQueueGiveMutexRecursive(lidar_control_mutex);
            // ESP_LOGI("SWrite","pcl: %f, rcl: %f, pci: %f, rci: %f\n", pcval_lidar, rcval_lidar, pcval_imu, rcval_imu);

            // m1 = m1 > 400 ? 400 : m1 < -400 ? -400: m1;
            // m2 = m2 > 400 ? 400 : m2 < -400 ? -400: m2;
            // m3 = m3 > 400 ? 400 : m3 < -400 ? -400: m3;
            if (m1 > 500)
            {
                m1 = 500;
            }
            else if (m1 < -500)
            {
                m1 = -500;
            }
            if (m2 > 500)
            {
                m2 = 500;
            }
            else if (m2 < -500)
            {
                m2 = -500;
            }
            if (m3 > 500)
            {
                m3 = 500;
            }
            else if (m3 < -500)
            {
                m3 = -500;
            }
            // ESP_LOGI("SWrite","m1: %f, m2: %f, m3: %f\n", m1, m2, m3);

            pwms[0] = PWM1(default_angle) + m1;
            pwms[1] = PWM2(default_angle) + m2;
            pwms[2] = PWM3(default_angle) + m3;

            // for(int i = 0; i < 3; i++){
            //     if(pwms[i] - prev_pwm[i] > 50){
            //         pwms[i] = prev_pwm[i] + 50;
            //     }
            //     else if(pwms[i] - prev_pwm[i] < -50){
            //         pwms[i] = prev_pwm[i] - 50;
            //     }
            // }
            // xSemaphoreTake(servo_mutex, 10/portTICK_PERIOD_MS);

            servo->writeMicroseconds(0, pwms[0]);
            servo->writeMicroseconds(1, pwms[1]);
            servo->writeMicroseconds(2, pwms[2]);
        // xSemaphoreGive(servo_mutex);
        // prev_pwm[0] = pwms[0];
        // prev_pwm[1] = pwms[1];
        // prev_pwm[2] = pwms[2];
        #ifdef CONFIG_LOG_DATA_TO_SERIAL
            #ifdef CONFIG_LOG_SERVO    
                sprintf(data, "%d,%d,%d", pwms[0], pwms[1], pwms[2]);
                ESP_LOGI("LOG_SERVO", "%s",data);
            #endif
            #ifdef CONFIG_LOG_PID
                sprintf(data, "%3.2f,%3.2f,%3.2f,%3.2f", pcval_lidar, rcval_lidar, pcval_imu, rcval_imu);
                ESP_LOGI("LOG_PID", "%s",data);
            #endif

        #endif
    }
}

void imuControlLoopTask(void *pvParameter){
    
    ESP_LOGI(TAG, "IMU Control Loop Task Started!");
    TickType_t prev_time;
    prev_ypr[1] = ypr[1];
    prev_ypr[2] = ypr[2];
    char deg_per_sec_max = 90;
    
    #ifdef CONFIG_LOG_DATA_TO_SERIAL
    #ifdef CONFIG_LOG_IMU
        char data [50];
        sprintf(data, "T,yaw,pitch,roll");
        ESP_LOGI("LOG_IMU", "%s",data);
    #endif
    #endif
    prev_time = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&prev_time, 50 / portTICK_PERIOD_MS);
        #ifdef CONFIG_LOG_DATA_TO_SERIAL
        #ifdef CONFIG_LOG_IMU
            sprintf(data, "%3.2f, %3.2f, %3.2f",  ypr[0], ypr[1], ypr[2]);
            ESP_LOGI("LOG_IMU", "%s",data);
        #endif
        #endif
        // xQueueTakeMutexRecursive(imu_control_mutex, 10/portTICK_PERIOD_MS);
        if(abs(ypr[1]) < deg2rad(2) && abs(ypr[2]) < deg2rad(2)){
            pcval_imu = 0;
            rcval_imu = 0;
        }
        // else if(abs(prev_ypr[1]-ypr[1]) * 33/1000 > deg_per_sec_max || abs(prev_ypr[2]-ypr[2]) * 33/1000 > deg_per_sec_max){
        //     pcval_imu = 0;
        //     rcval_imu = 0;
        // }
        else{
            pcval_imu = pid_pitch2->update(0 - rad2deg(ypr[1]));
            rcval_imu = pid_roll2->update(0 - rad2deg(ypr[2]));
        }
        // ESP_LOGI("IMUC","pci: %f, rci: %f\n", pcval_imu, rcval_imu);
        prev_ypr[0] = ypr[0];
        prev_ypr[1] = ypr[1];
        prev_ypr[2] = ypr[2];
        // xQueueGiveMutexRecursive(imu_control_mutex);
    }
    ESP_LOGW(TAG, "END");
    vTaskDelete(NULL);
}

// void lidarControlv2Task(void *pvParameter){
//     int8_t L1 = 120, d = 200, L2 = 50, dLidar = 10;
//     // int16_t avg = 0;
//     TickType_t lastTime = 0;
//     // #ifdef CONFIG_LOG_DATA_TO_SERIAL
//     //     char data [50];
//     //     sprintf(data, "Timestamp, x1 ,y1, z1, x2, y2, z2, x3, y3, z3, pitch, roll\n");
//     //     ESP_LOGI(TAG_LOG, "%s",data);
//     // #endif
//     for (;;){
//         // printf("point: ");
//         if(distance[0] == 0 && distance[1] == 0 && distance[2] == 0){
//             pcval_lidar = 0;
//             rcval_lidar = 0;
//             vTaskDelayUntil(&lastTime, 50 / portTICK_PERIOD_MS);
//             continue;
//         }
        
//         // #ifdef CONFIG_LOG_DATA_TO_SERIAL
//         //     sprintf(data, "%ld\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n", pdTICKS_TO_MS(xTaskGetTickCount()), legs[0].x, legs[0].y, legs[0].z, legs[1].x, legs[1].y, legs[1].z, legs[2].x, legs[2].y, legs[2].z, pitch, roll);
//         //     ESP_LOGI(TAG_LOG, "%s",data);
//         // #endif
//         // vTaskDelay(5 / portTICK_PERIOD_MS);
//         // xQueueTakeMutexRecursive(lidar_control_mutex, 10/portTICK_PERIOD_MS);
//         pcval_lidar = pid_pitch->update(0, (distance[0]+distance[1])/2 - distance[2]);
//         rcval_lidar = pid_roll->update(0, distance[0] - distance[1]);
//         // xQueueGiveMutexRecursive(lidar_control_mutex);
//         // ESP_LOGI("LIDARC","pcl: %f, rcl: %f\n", pcval_lidar, rcval_lidar);

//         // printf("Pitch: %f, Roll: %f dur:%ld\n", pitch, roll, xTaskGetTickCount() - lastTime);
        
//         vTaskDelayUntil(&lastTime, 50 / portTICK_PERIOD_MS);
//     }
// }

void lidarControlTask(void *pvParameter){
    int8_t L1 = 120, d = 200, L2 = 50, dLidar = 10;
    float pitch = 0, roll = 0;
    TickType_t lastTime;
    int z_last[3]={0,0,0};
    #ifdef CONFIG_LOG_DATA_TO_SERIAL
    #ifdef CONFIG_LOG_LIDAR
        #ifndef CONFIG_LOG_FK
            char data [50];
        #endif
        #ifdef CONFIG_LOG_FK
            char data [100];
        #endif
        sprintf(data, "T,L1,L2,L3");
        ESP_LOGI("LOG_LIDAR","%s",data);
    #endif
    #ifdef CONFIG_LOG_FK
        sprintf(data, "T,x1,y1,z1,x2,y2,z2,x3,y3,z3,pitch,roll");
        ESP_LOGI("LOG_FK", "%s",data);
    #endif
    #endif
    lastTime = xTaskGetTickCount();
    for (;;){
        vTaskDelayUntil(&lastTime, 200 / portTICK_PERIOD_MS);
        // printf("point: ");
        for (int i = 0; i < 3; i++)
        {
            float angle = getAngle(i + 1);
            legs[i].x = cos(deg2rad(120 * (i)-60)) * (d + L1 * sin(deg2rad(angle)));
            legs[i].y = sin(deg2rad(120 * (i) - 60)) * (d + L1*sin(deg2rad(angle)));
            if(distance[i] == 0){
                legs[i].z = z_last[i];
            }
            else{
                legs[i].z = L2 + L1 * cos(deg2rad(angle)) + (distance[i] - dLidar);
                z_last[i] = legs[i].z;
            }
            // printf("%d-(%d, %d, %d)\t", i, legs[i].x, legs[i].y, legs[i].z);
        }
        #ifdef CONFIG_LOG_DATA_TO_SERIAL
            #ifdef CONFIG_LOG_FK
            sprintf(data, "%d, %d, %d,\t%d, %d, %d,\t%d, %d, %d,\t%3.2f, %3.2f",  legs[0].x, legs[0].y, legs[0].z, legs[1].x, legs[1].y, legs[1].z, legs[2].x, legs[2].y, legs[2].z, pitch, roll);
            ESP_LOGI("LOG_FK", "%s",data);
            #endif
            #ifdef CONFIG_LOG_LIDAR
            sprintf(data, "%d, %d, %d", distance[0],distance[1],distance[2]);
            ESP_LOGI("LOG_LIDAR","%s",data);
            #endif
        #endif
        if(distance[0] == 0 && distance[1] == 0 && distance[2] == 0){
            pcval_lidar = 0;
            rcval_lidar = 0;
            continue;
        }
        // printf("\n");

        VectorInt16 n = getPlaneNorm(legs[0], legs[1], legs[2]);

        // pitch and roll
        pitch = rad2deg(atan2(n.x, n.z));
        roll = -rad2deg(atan2(n.y, n.z));

        // if (pitch > 180 ){
        //     pitch -= 180;
        // }
        // else if (pitch < -180){
        //     pitch += 180;
        // }
        // if (roll > 180){
        //     roll -= 180;
        // }
        // else if (roll < -180){
        //     roll += 180;
        // }

        lidar_pr[0] = pitch;
        lidar_pr[1] = roll;

        pcval_lidar = pid_pitch->update(0, pitch);
        rcval_lidar = pid_roll->update(0, roll);
        printf("pcval_lidar  :%f  \trcval_lidar: %f",pcval_lidar,rcval_lidar);
        // printf("Pitch: %f, Roll: %f dur:%ld\n", pitch, roll, xTaskGetTickCount() - lastTime);
    
    }
}

// MARK: Initialize
void initServoTask(void *pvParameter)
{
    ESP_LOGI(TAG, "Initializing Servos");
    xSemaphoreTake(servo_mutex, 10/portTICK_PERIOD_MS);
    // SemaphoreHandle_t init_mutex = xSemaphoreCreateBinary();
    
    // servo->begin(&init_mutex);
    servo->begin();
    // xSemaphoreTakeRecursive(init_mutex, 10/portTICK_PERIOD_MS);
    xSemaphoreGive(servo_mutex);

    vTaskDelete(NULL);
}


// MARK: Button
const char *button_event_table[] = 
{
    "BUTTON_PRESS_DOWN",
    "BUTTON_PRESS_UP",
    "BUTTON_PRESS_REPEAT",
    "BUTTON_PRESS_REPEAT_DONE",
    "BUTTON_SINGLE_CLICK",
    "BUTTON_DOUBLE_CLICK",
    "BUTTON_MULTIPLE_CLICK",
    "BUTTON_LONG_PRESS_START",
    "BUTTON_LONG_PRESS_HOLD",
    "BUTTON_LONG_PRESS_UP",
};


static void button_event_single_cb(void *arg, void *data)
{
    // printf("single click on button %d\n", (int)data);
    int pin = (int)data;
    if(pin ==5){
        // vTas
    }
    
}

static void button_event_long_cb(void *arg, void *data)
{
    // printf("long press on button %d\n", (int)data);
    // int pin = (int)data;
    
}

void button_event_init(button_handle_t btn, int gpio_num)
{
    button_event_config_t btn_event_cfg {
        .event = BUTTON_LONG_PRESS_START,
        .event_data{ .long_press {
            .press_time = 2000
            }
        }
    };
    iot_button_register_event_cb(btn, btn_event_cfg, button_event_long_cb, (void *)gpio_num);
}

button_handle_t button_init(uint32_t button_num)
{
    button_config_t btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = (int32_t)button_num,
            .active_level = 1,
    #if CONFIG_GPIO_BUTTON_SUPPORT_POWER_SAVE
            .enable_power_save = true,
    #endif
        },
    };
    button_handle_t btn = iot_button_create(&btn_cfg);
    assert(btn);
    esp_err_t err = iot_button_register_cb(btn, BUTTON_PRESS_DOWN, button_event_single_cb, (void *)button_num);
    err |= iot_button_register_cb(btn, BUTTON_PRESS_UP, button_event_single_cb, (void *)button_num);
    err |= iot_button_register_cb(btn, BUTTON_PRESS_REPEAT, button_event_single_cb, (void *)button_num);
    err |= iot_button_register_cb(btn, BUTTON_PRESS_REPEAT_DONE, button_event_single_cb, (void *)button_num);
    err |= iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, button_event_single_cb, (void *)button_num);
    err |= iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, button_event_single_cb, (void *)button_num);
    err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, button_event_single_cb, (void *)button_num);
    err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, button_event_single_cb, (void *)button_num);
    err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, button_event_single_cb, (void *)button_num);
    ESP_ERROR_CHECK(err);
    return btn;
}

void buttonInitTask(void *pvParameter){
    button_event_init(button_init(18),18);
    button_event_init(button_init(5),5);
}

void standbyTask(void *pvParameter){
    for (;;)
    {
        servo->writeMicroseconds(0, 1700);
        servo->writeMicroseconds(1, 1700);
        servo->writeMicroseconds(2, 1700);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void initializeTask(void *pvParameter)
{
    // portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
    // taskENTER_CRITICAL(&myMutex);
    
    ESP_LOGD(TAG, "Initializing Tasks");
    // xTaskCreatePinnedToCore(buttonInitTask, "button init", 2048*2, NULL, 5, NULL, tskNO_AFFINITY);
    
    ESP_LOGI(TAG, "Initializing LiDAR");
    xTaskCreatePinnedToCore(lidarTask, "LiDAR Task", 4096 , NULL, 5, &TaskHandle_lidar, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(startAdcTask, "adc task", 2048*3, NULL, 8, NULL, tskNO_AFFINITY);
    ESP_LOGI(TAG, "Initializing Servos");
    xTaskCreatePinnedToCore(initServoTask, "servo task", 6144, NULL, 5, &TaskHandle_initServo, tskNO_AFFINITY);

    // xTaskCreatePinnedToCore(standbyTask, "standby task", 2048, NULL, 5, &TaskHandle_standby, 1);
    // vTaskSuspend(TaskHandle_standby);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // xTaskCreatePinnedToCore(imuTask, "IMU Task", 3072, NULL, 5, &TaskHandle_IMU, 1);
    // ESP_LOGI(TAG, "Starting Control Tasks");
    // xTaskCreatePinnedToCore(imuControlLoopTask, "imu control", 4096, NULL, 5, &TaskHandle_imuControl, 0);
    // // // xTaskCreatePinnedToCore(lidarControlv2Task, "lidar control", 2048, NULL, 8, &TaskHandle_lidarControl, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(lidarControlTask, "lidar control", 4096, NULL, 5, &TaskHandle_lidarControl, 1);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(servoWriteTask, "servo write", 3072, NULL, 6, &TaskHandle_servoWrite, tskNO_AFFINITY);    
    
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // taskEXIT_CRITICAL(&myMutex);
    landing_gear_state = ACTIVE;
    vTaskDelete(NULL);
}
