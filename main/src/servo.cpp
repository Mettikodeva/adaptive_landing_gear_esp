#include "servo.h"
#include "driver/gpio.h"
#include "Arduino.h"
// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
// ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO);

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - 0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / (90 - 0) + MIN_PULSE_WIDTH;
}

Servo::Servo():Servo(50){}

Servo::Servo(int8_t update_rate){
    // for constraint the movement of the servo
    // _feedback = -1;
    // _feedback_pin = -1; 
    _update_rate = update_rate;
    _pwm_pin[0] = 25;
    _pwm_pin[1] = 26;
    _pwm_pin[2] = 27;
    _pwm_val[0] = 2000;
    _pwm_val[1] = 2000;
    _pwm_val[2] = 2000;
    _prev_pwm_val[0] = 2000;
    _prev_pwm_val[1] = 2000;
    _prev_pwm_val[2] = 2000;

    // _deg_max = 90.0;
    // _deg_min = 0.0;
    // Constructor
    // _name = (char*)malloc(20);
    // strcpy(_name, "servo name not set");
}

Servo::~Servo(){
    // Destructor
    // free(_name);
    mcpwm_del_timer(_timer);
    mcpwm_del_operator(_oper[0]);
    mcpwm_del_operator(_oper[1]);
    for (int i = 0; i < 3; i++)
    {
        mcpwm_del_comparator(_comparator[i]);
        mcpwm_del_generator(_generator[i]);
    }
    
}

void Servo::setPwmPin(int pin[3]){
    // Set the PWM pin
    for (int i = 0; i < 3; i++)
        _pwm_pin[i] = pin[i];
}


void Servo::begin(SemaphoreHandle_t *mutex){
    xSemaphoreTakeRecursive(*mutex, 10/portTICK_PERIOD_MS);
    this->begin();
    ESP_LOGI(_name, "Servo initialized");
    xSemaphoreGive(*mutex);
}

void Servo::begin(){
    /*
        Servo 1: Timer 0, Operator 0, Comparator 0, Generator 0
        Servo 2: Timer 0, Operator 0, Comparator 1, Generator 1
        Servo 3: Timer 0, Operator 1, Comparator 2, Generator 2
    */
    // MCPWM TIMER
    mcpwm_timer_config_t _timer_conf = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD,

        };

    ESP_LOGD(_name, "Create timer and operator");
    ESP_ERROR_CHECK(mcpwm_new_timer(&_timer_conf, &_timer));

    // MCPWM OPERATOR
    ESP_LOGD(_name, "Create operator");
    mcpwm_operator_config_t  _oper_conf = {
        .group_id = 0, // operator must be in the same group to the timer
    };

    ESP_ERROR_CHECK(mcpwm_new_operator(&_oper_conf, &_oper[0]));
    ESP_ERROR_CHECK(mcpwm_new_operator(&_oper_conf, &_oper[1]));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(_oper[0], _timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(_oper[1], _timer));

    ESP_LOGI(_name, "Create comparator and generator from the operator");

    // MCPWM COMPARATOR
    // mcpwm_comparator_config_t _comparator_conf;

    mcpwm_comparator_config_t _comparator_conf;
    _comparator_conf.flags.update_cmp_on_tez = true;
    _comparator_conf.intr_priority = 0;


    // MCPWM COMPARATOR
    ESP_LOGD(_name, "Create comparator and generator from the operator");
    ESP_ERROR_CHECK(mcpwm_new_comparator(_oper[0], &_comparator_conf, &_comparator[0]));
    ESP_ERROR_CHECK(mcpwm_new_comparator(_oper[0], &_comparator_conf, &_comparator[1]));
    ESP_ERROR_CHECK(mcpwm_new_comparator(_oper[1], &_comparator_conf, &_comparator[2]));
    
    // MCPWM GENERATOR    
    mcpwm_generator_config_t _generator_conf = {
        .gen_gpio_num = 25,
    };

    ESP_ERROR_CHECK(mcpwm_new_generator(_oper[0], &_generator_conf, &_generator[0]));
    _generator_conf.gen_gpio_num = 26;
    ESP_ERROR_CHECK(mcpwm_new_generator(_oper[0], &_generator_conf, &_generator[1]));
    _generator_conf.gen_gpio_num = 27;
    ESP_ERROR_CHECK(mcpwm_new_generator(_oper[1], &_generator_conf, &_generator[2]));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_comparator[0], 2000));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_comparator[1], 2000));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_comparator[2], 2000));


    ESP_LOGD(_name, "set generator action on timer and compare event");
    for (int i = 0; i < 3; i++)
    {
        // go high on counter empty
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(_generator[i],
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        // go low on compare threshold
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(_generator[i],
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _comparator[i], MCPWM_GEN_ACTION_LOW)));    
    }
    
    ESP_LOGD(_name, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP));

    // if (_use_feedback) {
    //     if (_feedback_pin < 0) {
    //         ESP_LOGE(_name, "Feedback pin not set");
    //         return;
    //     }
    //     if (_feedback_min < 0 || _feedback_max < 0) {
    //         ESP_LOGI(_name, "Feedback calibration not set, calibrating...");
    //     }
    // }
}


double Servo::_map(double val, double in_min, double in_max, double out_min, double out_max){
    // map the value from one range to another
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float Servo::_map(int val, int in_min, int in_max, float out_min, float out_max){
    // map the value from one range to another
    return _map((float)val, (float)in_min, (float)in_max, (float)out_min, (float)out_max);
}

float Servo::_map(float val, float in_min, float in_max, float out_min, float out_max){
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Servo::writeMicroseconds(int num_servo, int usec){
    int step = 20;

    // write the microseconds to the servo
    int _pwm_min = MIN_PULSE_WIDTH;
    int _pwm_max = MAX_PULSE_WIDTH;
    if (num_servo < 0 || num_servo > 2) {
        ESP_LOGE(_name, "Invalid servo number: %d. Servo number must be between 0 and 2", num_servo);
        return;
    }
    switch (num_servo)
    {
    case 0:
        _pwm_min = CONFIG_SERVO1_MIN;
        _pwm_max = CONFIG_SERVO1_MAX;
        break;
    case 1:
        _pwm_min = CONFIG_SERVO2_MIN;
        _pwm_max = CONFIG_SERVO2_MAX;
        break;
    case 2:
        _pwm_min = CONFIG_SERVO3_MIN;
        _pwm_max = CONFIG_SERVO3_MAX;
        break;
    }
    _pwm_val[num_servo] = usec;
    if(_pwm_val[num_servo] - _prev_pwm_val[num_servo] > step){
        _pwm_val[num_servo] = _prev_pwm_val[num_servo] + step;
        usec = _pwm_val[num_servo];
    }
    else if(_pwm_val[num_servo] - _prev_pwm_val[num_servo] < -step){
        _pwm_val[num_servo] = _prev_pwm_val[num_servo] - step;
        usec = _pwm_val[num_servo];
    }
    // printf("Servo %d: %d\n", num_servo, usec);
    if (usec < _pwm_min) {
        usec = _pwm_min;
        ESP_LOGE(_name, "Invalid microseconds: %d. Microseconds must be greater than %d", usec, _pwm_min);
        // return;
    }
    if (usec > _pwm_max) {
        ESP_LOGE(_name, "Invalid microseconds: %d. Microseconds must be less than %d", usec, _pwm_max);
        // return;
        usec = _pwm_max;
    }
    _prev_pwm_val[num_servo] = usec;
    // _pwm_val = usec;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_comparator[num_servo], usec));
    // set servo update rate
    // vTaskDelayUntil(&_last_time_write, 1000/_update_rate / portTICK_PERIOD_MS);
}

void Servo::writeMicroseconds(int usec){
    
    // write the microseconds to the servo
    if (usec < MIN_PULSE_WIDTH) {
        ESP_LOGE(_name, "Invalid microseconds: %d. Microseconds must be greater than %d", usec, MIN_PULSE_WIDTH);

        // return;
    }
    if (usec > MAX_PULSE_WIDTH) {
        ESP_LOGE(_name, "Invalid microseconds: %d. Microseconds must be less than %d", usec, MAX_PULSE_WIDTH);
        // return;
    }
    for (int i = 0; i < 3; i++) 
        this->writeMicroseconds(i, usec);
}

int *Servo::getPwm(){
    /*
    return the current pwm value in microseconds (int)
    */
    return _pwm_val;
}

int Servo::getPwm(int gpio){
    return _pwm_val[gpio];
}

void Servo::setTimer(mcpwm_timer_handle_t *timer){
    _timer = *timer;
}

void Servo::getTimer(mcpwm_timer_handle_t *timer){
    *timer = _timer;
}


