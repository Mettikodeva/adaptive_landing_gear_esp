// #include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "helper_3dmath.h"
// #include "MPU6050_6Axis_MotionApps20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "Arduino.h"

#define OUTPUT_READABLE_YAWPITCHROLL

#define PIN_SDA 21
#define PIN_CLK 22


VectorInt16 aa;
VectorInt16 gg;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool dmpReady = false;  // set true if DMP init was successful

uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

TickType_t xLastWakeTime;
static const char *TAG = "IMU";
MPU6050 mpu = MPU6050();
SemaphoreHandle_t imu_mutex = xSemaphoreCreateMutex();
// #define I2C_MODE_MASTER 0

void initI2C(){
    i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_21;
	conf.scl_io_num = GPIO_NUM_22;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

void imuTask(void *args){
	printf("IMU Task\n");
	TickType_t last_time = xTaskGetTickCount();
	for (;;){
		vTaskDelayUntil(&last_time, 50 / portTICK_PERIOD_MS);
		// xQueueTakeMutexRecursive(imu_mutex, portMAX_DELAY);
		mpuIntStatus = mpu.getIntStatus();
		fifoCount = mpu.getFIFOCount();
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        // reset so we can continue cleanly
	        mpu.resetFIFO();
		}else if (mpuIntStatus & 0x02) {
			// read a packet from FIFO
			while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
			mpu.dmpGetCurrentFIFOPacket(fifoBuffer); // Get the Latest packet
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            // mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGyro(&gg, fifoBuffer);
			ESP_LOGD(TAG,"aX: %d, aY: %d, aZ: %d", aa.x, aa.y, aa.z);
            ESP_LOGD(TAG,"gX: %d, gY: %d, gZ: %d", gg.x, gg.y, gg.z);
		}
		ESP_LOGI(TAG, "ypr: %f %f %f", ypr[0], ypr[1], ypr[2]);
		
	}
}


void rawImuTask(void *args){
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	for (;;){
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		printf("a:\t");
        printf("%d",ax); printf("\t");
        printf("%d",ay); printf("\t");
        printf("%d",az); printf("\t");
		printf("\n");
		printf("g:\t");
		printf("%d", gx); printf("\t");
		printf("%d",gy); printf("\t");
        printf("%d",gz);
		printf("\n");

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void startIMU(){
	ESP_LOGD(TAG, "Initializing IMU");
    mpu.initialize();
	ESP_LOGD(TAG, "initializing DMP");
	devStatus = mpu.dmpInitialize();
	ESP_LOGD(TAG, "Setting DMP Enabled");
	if(devStatus == 0){
		mpu.CalibrateAccel(10);
		mpu.CalibrateGyro(10);
		mpu.setDMPEnabled(true);
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else{
		ESP_LOGE("IMU","DMP Initialization failed (code ");
		ESP_LOGE("IMU","%d",devStatus);	
		ESP_LOGE("IMU",")\n");
	}
	ESP_LOGD(TAG, "IMU Initialized");
	
}
