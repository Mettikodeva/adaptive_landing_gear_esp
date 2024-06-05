# set(srcs imu.cpp test_ros.cpp)
# set(srcs servos.cpp test_servo.cpp)

# Edit following two lines to set component requirements (see docs)
# set(COMPONENT_PRIV_REQUIRES )

# set(COMPONENT_SRCS "process_ADC.cpp" "landing_gear.bak.cpp" "main.bak.cpp" "lidar.cpp" "send_log.cpp" "myOTA.c" "spiffs_helper.cpp" "pca9548a.cpp" "servo.cpp" "pid.cpp" "imu.cpp" "test_servo.cpp" "test_mpu.cpp" "servos.cpp" "test_ros.cpp" "landing_gear.cpp" "main.cpp" "example.cpp" "components")
# set(COMPONENT_ADD_INCLUDEDIRS "components/include")
# register_component()
# set(COMPONENT_REQUIRES MPU6050)
set(srcs    main.cpp 
landing_gear.cpp 
src/servo.cpp 
src/pid.cpp
src/pca9548a.cpp 
imu.cpp 
src/lidar.cpp
process_ADC.cpp
# src/I2Cdev.cpp 
# src/MPU6050.cpp
# src/MPU6050_6Axis_MotionApps612.cpp
# spiffs_helper.cpp
# send_log.cpp
)


idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include"
                    REQUIRES "esp_timer" "driver"  "vl53l0x" "arduino" "sbus" #"app_update" "spiffs"
                    # PRIV_REQUIRES "protocol_examples_common" "nvs_flash"
                    # EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem
)


