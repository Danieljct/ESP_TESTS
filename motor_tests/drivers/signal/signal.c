#include "signal.h"


static const char *TAG = "signal";

pulse_data_t pulse_data;


static ledc_channel_config_t pwm_channel;
static ledc_timer_config_t pwm_timer;

/* Timing control */
static bool timer_running = false;
static double current_time = 0;
static double time_increment = 1.0 / PWM_FREQ;
static signal_t signal_mode;
static bool update_flag = false;

/* GPTimer (sample timer) */
static gptimer_handle_t gptimer = NULL;

// Signal generators
uint16_t SIGNAL_sin23hz(double t)       { return 350 * sin(F23HZ * t) + 600; }
uint16_t SIGNAL_sqr23hz(double t)       { return (sin(F23HZ * t) >= 0) ? 1023 : 512; }
uint16_t SIGNAL_sin23hz_sin1hz8(double t) { return 767 + 255 * sin(F1HZ8 * t) * sin(F23HZ * t); }
uint16_t SIGNAL_sqr23hz_sqr1hz8(double t) { return (sin(F1HZ8 * t) >= 0) ? SIGNAL_sqr23hz(t) : 512; }
uint16_t SIGNAL_sin23hz_sqr1hz8(double t) { return (sin(F1HZ8 * t) >= 0) ? SIGNAL_sin23hz(t) : 512; }

// Utility
void SIGNAL_clearUpdateFlag(void) { update_flag = false; }
bool SIGNAL_getUpdateFlag(void)   { return update_flag; }
void SIGNAL_setCurrentTime(double val) { current_time = val; }
double SIGNAL_getCurrentTime(void) { return current_time; }

uint16_t SIGNAL_getValue(void) {
    switch (signal_mode) {
        case SIGNAL_SIN23HZ: return SIGNAL_sin23hz(current_time);
        case SIGNAL_SQR23HZ: return SIGNAL_sqr23hz(current_time);
        case SIGNAL_SIN23HZ_SIN1HZ8: return SIGNAL_sin23hz_sin1hz8(current_time);
        case SIGNAL_SQR23HZ_SQR1HZ8: return SIGNAL_sqr23hz_sqr1hz8(current_time);
        case SIGNAL_SIN23HZ_SQR1HZ8: return SIGNAL_sin23hz_sqr1hz8(current_time);
        case SIGNAL_MAX: return 512;
        default: return 0;
    }
}

void SIGNAL_pwmSetDutyCycle(uint16_t duty) {
    uint32_t scaled = (duty > 1023 ? 1023 : duty);
    uint32_t ledc_duty = (scaled * PWM_MAX_DUTY) / 1023;
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, ledc_duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

bool IRAM_ATTR gptimer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    current_time += time_increment;
    update_flag = true;
    uint16_t val = SIGNAL_getValue();
    SIGNAL_pwmSetDutyCycle(val);
    return true;
}

void SIGNAL_init(double frequency) {
    current_time = 0;
    time_increment = 1.0 / frequency;

    pwm_timer = (ledc_timer_config_t) {
        .duty_resolution = PWM_RES_BITS,
        .freq_hz = PWM_FREQ,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    pwm_channel = (ledc_channel_config_t) {
        .channel    = PWM_CHANNEL,
        .duty       = 0,
        .gpio_num   = PWM_GPIO,
        .speed_mode = PWM_MODE,
        .hpoint     = 0,
        .timer_sel  = PWM_TIMER
    };
    ledc_channel_config(&pwm_channel);

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = PWM_FREQ * 1000
    };
    gptimer_new_timer(&timer_config, &gptimer);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_isr_callback,
    };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1000,
        .flags.auto_reload_on_alarm = true
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);
    gptimer_enable(gptimer);
    gptimer_start(gptimer);

    timer_running = true;
    ESP_LOGI(TAG, "Signal PWM with real waveform initialized at 1kHz");
}

void SIGNAL_setNextDuty(uint16_t duty) {
    if (duty > 1023) duty = 1023; // Asegurar límites

    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

void SIGNAL_select(signal_t mode) {
    signal_mode = mode;
}

void SIGNAL_stop(void) {
    if (timer_running) {
        gptimer_stop(gptimer);
        timer_running = false;
    }
    current_time = 0.0;
}

void SIGNAL_start(void) {
    SIGNAL_stop();
    gptimer_start(gptimer);
    timer_running = true;
    current_time = 0.0;
}


void SIGNAL_setPulseTiming(uint16_t period_s, uint8_t duty)
{
    if ((period_s > 0) && (duty > 0) && (duty <= 100))
    {
        pulse_data.period = period_s;
        pulse_data.duty = duty;
        pulse_data.periodtime = 0.001 * period_s;
        pulse_data.dutytime = 0.01 * (duty * pulse_data.periodtime);
    }
    else
    {
        if (pulse_data.flag)
        {
            SIGNAL_stop();
        }
        pulse_data.flag = false;
        pulse_data.period = 0;
        pulse_data.duty = 0;
        pulse_data.dutytime = 0.0; 
    }
}
void SIGNAL_setPulseFlag(bool st)
{
    pulse_data.flag = (st && (pulse_data.period > 0));
}