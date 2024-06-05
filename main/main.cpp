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
extern TaskHandle_t TaskHandle_standby;
extern TaskHandle_t TaskHandle_IMU;
extern TaskHandle_t TaskHandle_initServo;
extern TaskHandle_t TaskHandle_sweepServo;
extern TaskHandle_t TaskHandle_lidar;
extern TaskHandle_t TaskHandle_lidarControl;
extern TaskHandle_t TaskHandle_imuControl;

State_LG_t landing_gear_state = INIT;

extern void startIMU();
// extern void sendLogTask(void *);

bfs::SbusRx sbus_rx(&Serial1,16,17,true);
// bfs::SbusTx sbus_tx(&Serial1,16,17,true);
bfs::SbusData data_sbus;

void check_stack_size(void *arg){
	for (;;){
		printf("SS lidar: %ld\n", uxTaskGetStackHighWaterMark2(TaskHandle_lidar));
		printf("SS lidarControl: %ld\n", uxTaskGetStackHighWaterMark2(TaskHandle_lidarControl));
		printf("SS imu: %ld\n", uxTaskGetStackHighWaterMark2(TaskHandle_IMU));
		printf("SS imuControl: %ld\n", uxTaskGetStackHighWaterMark2(TaskHandle_imuControl));
		printf("SS initServo: %ld\n", uxTaskGetStackHighWaterMark2(TaskHandle_initServo));
		printf("SS sweepServo: %ld\n", uxTaskGetStackHighWaterMark2(TaskHandle_sweepServo));
		printf("SS servoWrite: %ld\n", uxTaskGetStackHighWaterMark2(TaskHandle_servoWrite));
		printf("SS standby: %ld\n", uxTaskGetStackHighWaterMark2(TaskHandle_standby));
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}


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
	TickType_t last_time = xTaskGetTickCount();
	for (;;)
	{
		vTaskDelayUntil(&last_time, 200 / portTICK_PERIOD_MS);
		if (sbus_rx.Read())
		{
			data_sbus = sbus_rx.data();
			vTaskDelay(5 / portTICK_PERIOD_MS);
			printf("Data SBUS: %d",data_sbus.ch[7]);
			if(data_sbus.ch[7] > 1500){
				if(landing_gear_state == INIT){
					continue;
				}
				landing_gear_state = STANDBY;
				vTaskSuspend(TaskHandle_servoWrite);
				vTaskDelay(5 / portTICK_PERIOD_MS);
				vTaskResume(TaskHandle_standby);
			}
			if(data_sbus.ch[7] < 1500){
				if(landing_gear_state == INIT){
					continue;
				}
				landing_gear_state = ACTIVE;
				vTaskSuspend(TaskHandle_standby);
				vTaskDelay(5 / portTICK_PERIOD_MS);
				vTaskResume(TaskHandle_servoWrite);
			}
			vTaskDelay(5 / portTICK_PERIOD_MS);
		}
		
	}
}


