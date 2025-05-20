#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "driver/ledc.h"

static const char *TAG = "main";

#define LED_R 22
#define LED_G 24
#define LED_B 27

#define NUM_TIMERS 1
TimerHandle_t xTimer;

uint8_t led_r_state = 0;
uint8_t led_g_state = 0;
uint8_t led_b_state = 0;

esp_err_t init_led(void);
esp_err_t blink_led(uint8_t led);
esp_err_t set_timer(void);
esp_err_t set_pwm(void);

static void vTimerCallback(TimerHandle_t xTimer)
{
    uint8_t leds[] = {LED_R, LED_G, LED_B};
    uint8_t led = leds[rand() % 3];
  //  blink_led(led);
}

void app_main(void)
{
    //init_led();
    set_pwm();
    set_timer();
}

esp_err_t init_led(void)
{
    gpio_reset_pin(LED_R);
    gpio_reset_pin(LED_G);
    gpio_reset_pin(LED_B);

    gpio_set_direction(LED_R, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_G, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_B, GPIO_MODE_OUTPUT);
    return ESP_OK;
}

esp_err_t blink_led(uint8_t led)
{
    if (led == LED_R)
    {
        led_r_state = !led_r_state;
        gpio_set_level(LED_R, led_r_state);
    }
    else if (led == LED_G)
    {
        led_g_state = !led_g_state;
        gpio_set_level(LED_G, led_g_state);
    }
    else if (led == LED_B)
    {
        led_b_state = !led_b_state;
        gpio_set_level(LED_B, led_b_state);
    }
    return ESP_OK;
}






esp_err_t set_timer(void)
{
    int32_t x;
    for (x = 0; x < NUM_TIMERS; x++)
    {
        xTimer = xTimerCreate("Timer",         // Just a text name, not used by the kernel.
                                  pdMS_TO_TICKS(50), // The timer period in ticks.
                                  pdTRUE,          // The timers will auto-reload themselves when they expire.
                                  (void *)x,       // Assign each timer a unique id equal to its array index.
                                  vTimerCallback   // Each timer calls the same callback when it expires.
        );
        if (xTimer == NULL)
        {
        }
        else
        {
            if (xTimerStart(xTimer, 0) != pdPASS)
            {
            }
        }
    }
    return ESP_OK;
}

esp_err_t set_pwm(void){
    ledc_channel_config_t ledc_channel_r = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 512,
        .gpio_num   = LED_R,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
    };

    ledc_channel_config_t ledc_channel_g = {
        .channel    = LEDC_CHANNEL_1,
        .duty       = 512,
        .gpio_num   = LED_G,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
    };

    ledc_channel_config_t ledc_channel_b = {
        .channel    = LEDC_CHANNEL_2,
        .duty       = 512,
        .gpio_num   = LED_B,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&ledc_channel_r);
    ledc_channel_config(&ledc_channel_g);
    ledc_channel_config(&ledc_channel_b);

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 20000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
    };

    ledc_timer_config(&ledc_timer);
    return ESP_OK;
}