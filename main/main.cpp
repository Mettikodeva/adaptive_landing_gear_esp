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

extern void initializeTask(void *);
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

extern void startIMU();
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
	xTaskCreatePinnedToCore(initializeTask, "init task", 4096, NULL, 6, NULL, 1);
	
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	ESP_LOGI("LOG_STATE","T,state");
	TickType_t last_time = xTaskGetTickCount();
	for (;;)
	{
		vTaskDelayUntil(&last_time, 200 / portTICK_PERIOD_MS);
		ESP_LOGI("LOG_STATE","%d",landing_gear_state);
		if (sbus_rx.Read())
		{
			data_sbus = sbus_rx.data();
			vTaskDelay(5 / portTICK_PERIOD_MS);
			printf("Data SBUS: %d",data_sbus.ch[7]);

			if(data_sbus.ch[7] > 1500){
				if(landing_gear_state == INIT){
					continue;
				}
				else if (landing_gear_state == TOUCHDOWN || landing_gear_state == STANDBY){
					ESP_LOGW(TAG, "Can't enter suspended while touchdown or standby");
				}
				else{
					landing_gear_state = SUSPENDED;
				}
			}
			if(data_sbus.ch[7] < 1500){
				if(landing_gear_state == INIT){
					continue;
				}
				if(landing_gear_state == SUSPENDED){
					landing_gear_state = DESCENDING;
				}
			}

			/*
			mode init -> initializing system, do not accept any command
			
			*/
			switch (landing_gear_state)
			{
			case SUSPENDED:
				ESP_LOGD(TAG, "Suspended");
				vTaskResume(TaskHandle_suspended);
				break;
			case DESCENDING:
				ESP_LOGD(TAG, "Descending");
				// vTaskResume(TaskHandle_lidarControl);
				break;

			default:
				break;
			}

			vTaskDelay(5 / portTICK_PERIOD_MS);
		}
		
	}
}


