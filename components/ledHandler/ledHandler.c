#include <stdio.h>
#include "math.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "ledHandler.h"

static char* TAG = "LEDHANDLER";

#define GAMMA 4.0

static int redDuty = 0;
static int greenDuty = 0;
static int blueDuty = 0;

static int ledHandler_CalcDuty(int inVal, int brightness)
{
    float retVal;
    retVal = pow(((float)inVal / 255.0), GAMMA) * ((float)brightness/255) * 8192;
    return (int)retVal;
}

void ledHandler_Init()
{
    ledc_timer_config_t ledhandler_timer = {
        .speed_mode       = LEDHANDLER_MODE,
        .duty_resolution  = LEDHANDLER_DUTY_RES,
        .timer_num        = LEDHANDLER_TIMER,
        .freq_hz          = LEDHANDLER_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledhandler_timer));

    ledc_channel_config_t ledhandler_channel_red = {
        .speed_mode     = LEDHANDLER_MODE,
        .channel        = LEDHANDLER_CHANNEL_RED,
        .timer_sel      = LEDHANDLER_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDHANDLER_OUTPUT_IO_RED,
        .duty           = redDuty, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config_t ledhandler_channel_green = {
        .speed_mode     = LEDHANDLER_MODE,
        .channel        = LEDHANDLER_CHANNEL_GREEN,
        .timer_sel      = LEDHANDLER_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDHANDLER_OUTPUT_IO_GREEN,
        .duty           = greenDuty, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config_t ledhandler_channel_blue = {
        .speed_mode     = LEDHANDLER_MODE,
        .channel        = LEDHANDLER_CHANNEL_BLUE,
        .timer_sel      = LEDHANDLER_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDHANDLER_OUTPUT_IO_BLUE,
        .duty           = blueDuty, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledhandler_channel_red));
    ESP_ERROR_CHECK(ledc_channel_config(&ledhandler_channel_green));
    ESP_ERROR_CHECK(ledc_channel_config(&ledhandler_channel_blue));

    gpio_config_t ledhandler_relay_gpio = {
        .pin_bit_mask = LEDHANDLER_OUTPUT_IO_RELAY_BIT,
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = false,
        .pull_up_en  =false  
    };

    ESP_ERROR_CHECK(gpio_config(&ledhandler_relay_gpio));

}

void ledHandler_SetRGB(int redVal,int greenVal, int blueVal, int brightness)
{
    redDuty = ledHandler_CalcDuty(redVal, brightness);
    greenDuty = ledHandler_CalcDuty(greenVal, brightness);
    blueDuty = ledHandler_CalcDuty(blueVal, brightness);

    ESP_ERROR_CHECK(ledc_set_duty(LEDHANDLER_MODE, LEDHANDLER_CHANNEL_RED, redDuty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDHANDLER_MODE, LEDHANDLER_CHANNEL_BLUE, greenDuty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDHANDLER_MODE, LEDHANDLER_CHANNEL_GREEN, blueDuty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDHANDLER_MODE, LEDHANDLER_CHANNEL_RED));
    ESP_ERROR_CHECK(ledc_update_duty(LEDHANDLER_MODE, LEDHANDLER_CHANNEL_GREEN));
    ESP_ERROR_CHECK(ledc_update_duty(LEDHANDLER_MODE, LEDHANDLER_CHANNEL_BLUE));

    ESP_LOGI(TAG, "UPDATED RGB:%d,%d,%d,%d", redDuty, greenDuty, blueDuty, brightness);

}

void ledHandler_SetOnOff(bool stateVal)
{
        gpio_set_level(LEDHANDLER_OUTPUT_IO_RELAY, (uint32_t)stateVal);
}