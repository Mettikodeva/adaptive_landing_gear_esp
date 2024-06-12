#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "cstdio"
#include "sbus.h"
#include "Arduino.h"
#include "Wire.h"
#include "headers.h"

#ifdef CONFIG_USE_BT
	#include "BluetoothSerial.h"

	const char bt_name[12]  = "LandingGear";

	// Check if Bluetooth is available

	BluetoothSerial SerialBT;

	int vprintf_bt(const char *fmt, va_list args){
			char buf[256];
			vsnprintf(buf, sizeof(buf), fmt, args);
			SerialBT.println(buf);
			return 0;
	}
#endif

static const char *TAG = "main";

extern "C" {
	void app_main(void);
}

extern void initializeTasks();
extern void getTerrainTask(void *);
extern TaskHandle_t TaskHandle_servoWrite;
extern TaskHandle_t TaskHandle_suspended;
extern TaskHandle_t TaskHandle_IMU;
extern TaskHandle_t TaskHandle_initServo;
extern TaskHandle_t TaskHandle_sweepServo;
extern TaskHandle_t TaskHandle_lidar;
extern TaskHandle_t TaskHandle_lidarControl;
extern TaskHandle_t TaskHandle_imuControl;
extern TaskHandle_t TaskHandle_terrain;

State_LG_t landing_gear_state = INIT;
State_LG_t prev_state = INIT;

extern void startIMU();
extern uint16_t distance[3];
extern float target_servo_angle_terrain[3];
// extern void sendLogTask(void *);

bfs::SbusRx sbus_rx(&Serial1,16,17,true);
// bfs::SbusTx sbus_tx(&Serial1,16,17,true);
bfs::SbusData data_sbus;


void app_main(void)
{
	esp_log_level_set("*", ESP_LOG_INFO);
	// xTaskCreatePinnedToCore(check_stack_size, "check stack size", 2048, NULL, 6, NULL, 1);
	initArduino();
	delayMicroseconds(10);
	ESP_LOGD(TAG, "Initializing I2C");
    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C
	ESP_LOGD(TAG, "Initializing IMU");
    delayMicroseconds(1000);

	startIMU();
	ESP_LOGD(TAG, "IMU Initialized");

	#ifdef CONFIG_USE_BT

		bool ret = SerialBT.begin(bt_name);
		if (ret) {
			printf("Bluetooth \"%s\" is started.\n\n", bt_name);
			vprintf_like_t original_vprintf = esp_log_set_vprintf(vprintf_bt);
		} else ESP_LOGW(TAG, "Failed to start Bluetooth, using UART");
	#endif
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	printf("Starting SBUS\n");
	sbus_rx.Begin();
	delayMicroseconds(10);
  	// sbus_tx.Begin();
	delayMicroseconds(10);
	// ESP LOG LEVEL;
	ESP_LOGD(TAG, "Initialize task");
	// xTaskCreatePinnedToCore(initializeTask, "init task", 4096, NULL, 6, NULL, 1);
	initializeTasks();

	vTaskDelay(3000 / portTICK_PERIOD_MS);
	ESP_LOGI("LOG_STATE","T,state");
	TickType_t last_time = xTaskGetTickCount();
	for (;;)
	{
		vTaskDelayUntil(&last_time, 10 / portTICK_PERIOD_MS);
		ESP_LOGI("LOG_STATE","%d",landing_gear_state);
		if (sbus_rx.Read())
		{
			data_sbus = sbus_rx.data();
			// vTaskDelay(5 / portTICK_PERIOD_MS);
			printf("Data SBUS: %d",data_sbus.ch[7]);

			if(data_sbus.ch[7] > 1500){
				// suspended
				if(landing_gear_state == INIT){
					continue;
				}
				
				if (landing_gear_state == TOUCHDOWN || landing_gear_state == STANDBY){
					ESP_LOGW(TAG, "Can't enter suspended while touchdown or standby");
				}
				else{
					landing_gear_state = SUSPENDED;
					target_servo_angle_terrain[0] = 45;
					target_servo_angle_terrain[1] = 45;
					target_servo_angle_terrain[2] = 45;
				}
			}
			if(data_sbus.ch[7] < 1500){
				if(landing_gear_state == INIT){
					continue;
				}
				if(landing_gear_state == SUSPENDED){
					landing_gear_state = DESCENDING;
				}
				if((landing_gear_state == DESCENDING) && ((distance[0] < 10) || (distance[1] < 10) || (distance[2] < 10))){
					landing_gear_state = TOUCHDOWN;
				}
				if((landing_gear_state == TOUCHDOWN ||landing_gear_state == STANDBY) && distance[0] > 10 && distance[1] > 10 && distance[2] > 10){
					landing_gear_state = DESCENDING;
				}
			}

			/*
			mode init -> initializing system, do not accept any command
			
			*/
			eTaskState state;
			ESP_LOGD(TAG,"state: %d", (int)landing_gear_state);
			if(landing_gear_state == SUSPENDED){
				ESP_LOGD(TAG, "Suspended");
				state = eTaskGetState(TaskHandle_suspended);
				if(state == eSuspended){
					vTaskResume(TaskHandle_suspended);
					vTaskDelay(1);
					vTaskSuspend(TaskHandle_servoWrite);
					vTaskSuspend(TaskHandle_terrain);
					vTaskSuspend(TaskHandle_imuControl);
				}
				// else if(state != eSuspended){
				// 	landing_gear_state = DESCENDING;
				// 	vTaskSuspend(TaskHandle_suspended);
				// 	vTaskDelay(5);
				// 	vTaskResume(TaskHandle_servoWrite);
				// 	ESP_LOGD(TAG, "Suspended Task suspended");
				// }
			}				
			else if(landing_gear_state == DESCENDING){
				ESP_LOGD(TAG, "Descending");
				// resume and suspend terrain task, create if not exist
				state = eTaskGetState(TaskHandle_terrain);
				if(state == eSuspended){
					vTaskSuspend(TaskHandle_imuControl);
					vTaskSuspend(TaskHandle_suspended);
					vTaskDelay(1);
					vTaskResume(TaskHandle_terrain);
					vTaskResume(TaskHandle_servoWrite);
				}
			}
			
			else if(landing_gear_state == TOUCHDOWN){
				ESP_LOGD(TAG, "Touchdown");
				// resume and suspend terrain task, create if not exist
				state = eTaskGetState(TaskHandle_imuControl);
				if(state == eSuspended){
					vTaskSuspend(TaskHandle_suspended);
					vTaskSuspend(TaskHandle_terrain);
					vTaskDelay(1);
					vTaskResume(TaskHandle_imuControl);
					vTaskResume(TaskHandle_servoWrite);
				}
			}
			else if(landing_gear_state == STANDBY){
				ESP_LOGD(TAG, "standby");
			}
			else{
				ESP_LOGD(TAG, "State not found");
			}
			prev_state = landing_gear_state;
			vTaskDelay(5 / portTICK_PERIOD_MS);
		}
		
	}
}


