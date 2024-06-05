#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include <cmath>



#define MIN_PULSE_WIDTH       500     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2500     // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
#define SERVO_MAX_DEG        90      // maximum angle in degrees
#define SERVO_MIN_DEG        -90        // minimum angle in degrees

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000 // 20000us, 20ms period

#define SERVO_GROUP 0

/*
    This servo class is used to control a servo motor using the MCPWM peripheral of the ESP32.
    The servo motor is controlled by a PWM signal with a frequency of 50Hz and a duty cycle of 5% to 10%.
    The servo motor is modified to have feedback using the internal potentiometer. 
*/

class Servo{
    public:

        Servo();
        Servo(int8_t update_rate);
        ~Servo();

        void setPwmPin(int pin[]);

        // void setFeedbackPin(int pin);
        
        void begin();
        void begin(SemaphoreHandle_t *mutex);
        // void syncTimerTEZ(mcpwm_timer_handle_t timers[]);
        void setTimer(mcpwm_timer_handle_t *timer);
        void getTimer(mcpwm_timer_handle_t *timer);
        void writeMicroseconds(int gpio, int usec);
        void writeMicroseconds(int usec);

        // void setCalibration(int analog_min, int analog_max, int pwm_min, int pwm_max);
        int getPwm(int);
        int *getPwm();

    private:
        
        int8_t _update_rate;
        TickType_t _last_time_write;

        // MARK: MCPWM
        //  Timer
        mcpwm_timer_handle_t _timer;
        
        // operator
        mcpwm_oper_handle_t _oper[2];
        // comparator
        mcpwm_cmpr_handle_t _comparator[3];
        // generator
        mcpwm_gen_handle_t _generator[3];

        int _pwm_pin[3];
        int _pwm_val[3]; // in microseconds
        int _prev_pwm_val[3]; // in microseconds
 
        // int _feedback_pin;
        // int _feedback;

        // int _feedback_min;
        // int _feedback_max;
        // bool _use_feedback = false;
        char *_name = "SERVO";

        double _map(double val, double in_min, double in_max, double out_min, double out_max);
        float _map(int val, int in_min, int in_max, float out_min, float out_max);
        float _map(float val, float in_min, float in_max, float out_min, float out_max);
    };
