#include <stdio.h>
#include "math.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "ledHandler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "../../managed_components/espressif__led_strip/include/led_strip.h"

static char *TAG = "LEDHANDLER";

#define GAMMA 4.0

static int ledHandler_ledValue[3][4] = {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}};
static SemaphoreHandle_t ledHandler_FadeSem;

static led_strip_handle_t ledHandler_dLed1Strip;
static led_strip_handle_t ledHandler_dLed2Strip;

static IRAM_ATTR bool ledHandler_FadeEndEventCb(const ledc_cb_param_t *param, void *user_arg)
{
    BaseType_t taskAwoken = pdFALSE;

    if (param->event == LEDC_FADE_END_EVT)
    {
        SemaphoreHandle_t ledHandler_FadeSem_loc = (SemaphoreHandle_t)user_arg;
        xSemaphoreGiveFromISR(ledHandler_FadeSem_loc, &taskAwoken);
    }

    return (taskAwoken == pdTRUE);
}

static int ledHandler_CalcAnalogDuty(int inVal, int brightness)
{
    float retVal;
    retVal = pow(((float)inVal / 255.0), GAMMA) * ((float)brightness / 255) * 8192;
    return (int)retVal;
}

void ledHandler_Init()
{
    ledc_timer_config_t ledhandler_timer = {
        .speed_mode = LEDHANDLER_MODE,
        .duty_resolution = LEDHANDLER_DUTY_RES,
        .timer_num = LEDHANDLER_TIMER,
        .freq_hz = LEDHANDLER_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledhandler_timer));

    ledc_channel_config_t ledhandler_channel_red = {
        .speed_mode = LEDHANDLER_MODE,
        .channel = LEDHANDLER_CHANNEL_RED,
        .timer_sel = LEDHANDLER_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDHANDLER_OUTPUT_IO_RED,
        .duty = ledHandler_ledValue[aLed1][RED], // Set duty to 0%
        .hpoint = 0};
    ledc_channel_config_t ledhandler_channel_green = {
        .speed_mode = LEDHANDLER_MODE,
        .channel = LEDHANDLER_CHANNEL_GREEN,
        .timer_sel = LEDHANDLER_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDHANDLER_OUTPUT_IO_GREEN,
        .duty = ledHandler_ledValue[aLed1][GREEN], // Set duty to 0%
        .hpoint = 0};
    ledc_channel_config_t ledhandler_channel_blue = {
        .speed_mode = LEDHANDLER_MODE,
        .channel = LEDHANDLER_CHANNEL_BLUE,
        .timer_sel = LEDHANDLER_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDHANDLER_OUTPUT_IO_BLUE,
        .duty = ledHandler_ledValue[aLed1][BLUE], // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledhandler_channel_red));
    ESP_ERROR_CHECK(ledc_channel_config(&ledhandler_channel_green));
    ESP_ERROR_CHECK(ledc_channel_config(&ledhandler_channel_blue));

    gpio_config_t ledhandler_relay_gpio = {
        .pin_bit_mask = LEDHANDLER_OUTPUT_IO_RELAY_BIT,
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = false,
        .pull_up_en = false};

    ESP_ERROR_CHECK(gpio_config(&ledhandler_relay_gpio));

    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    ledc_cbs_t ledHandler_FadeCallbacks = {
        .fade_cb = ledHandler_FadeEndEventCb};
    ledHandler_FadeSem = xSemaphoreCreateCounting(LEDHANDLER_CHANNEL_MAX_NUM, 3);
    if (ledHandler_FadeSem == NULL)
    {
        ESP_LOGE(TAG, "Error during semaphore creation!");
    }
    for (int ch = 0; ch < LEDHANDLER_CHANNEL_MAX_NUM; ch++)
    {
        ESP_ERROR_CHECK(ledc_cb_register(LEDHANDLER_MODE, ch, &ledHandler_FadeCallbacks, (void *)ledHandler_FadeSem));
    }

    led_strip_config_t ledhandler_dLed1Strip_cfg = {
        .strip_gpio_num = LEDHANDLER_OUTPUT_IO_DLED1, // The GPIO that connected to the LED strip's data line
        .max_leds = LEDHANDLER_DLED1_LED_COUNT,       // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,                // LED strip model
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,     // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }};

    led_strip_rmt_config_t ledhandler_dLedRmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,               // different clock source can lead to different power consumption
        .resolution_hz = LEDHANDLER_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = 64,                      // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
        }};

    led_strip_config_t ledhandler_dLed2Strip_cfg = {
        .strip_gpio_num = LEDHANDLER_OUTPUT_IO_DLED2, // The GPIO that connected to the LED strip's data line
        .max_leds = LEDHANDLER_DLED1_LED_COUNT,       // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,                // LED strip model
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,     // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }};

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&ledhandler_dLed1Strip_cfg, &ledhandler_dLedRmt_cfg, &ledHandler_dLed1Strip));
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&ledhandler_dLed2Strip_cfg, &ledhandler_dLedRmt_cfg, &ledHandler_dLed2Strip));
}

void ledHandler_SetRGB(int redVal, int greenVal, int blueVal, int brightness, ledHandler_ledType_t ledType)
{
    int newVal[3] = {0, 0, 0};
    newVal[RED] = ledHandler_CalcAnalogDuty(redVal, brightness);
    newVal[GREEN] = ledHandler_CalcAnalogDuty(greenVal, brightness);
    newVal[BLUE] = ledHandler_CalcAnalogDuty(blueVal, brightness);
    bool ValChanged = false;
    for (int i = 0; i < 3; i++)
    {
        if (newVal[i] != ledHandler_ledValue[ledType][i])
        {
            ValChanged = true;
            break;
        }
    }
    if (ValChanged == false)
    {
        ESP_LOGI(TAG, "LED Value not changed");
        return;
    }
    if (false == ledHandler_ledValue[ledType][STATE])
    {
        for (int i = 0; i < 3; i++)
        {
            ledHandler_ledValue[ledType][i] = newVal[i];
        }
        ESP_LOGI(TAG, "LED State off, value state changed but not applied");
        return;
    }
    switch (ledType)
    {
    case aLed1:

        ledHandler_ledValue[aLed1][RED] = newVal[RED];
        ledHandler_ledValue[aLed1][GREEN] = newVal[GREEN];
        ledHandler_ledValue[aLed1][BLUE] = newVal[BLUE];
        for (int i = 0; i < LEDHANDLER_CHANNEL_MAX_NUM; i++)
        {
            xSemaphoreTake(ledHandler_FadeSem, portMAX_DELAY);
        }

        for (int ch = 0; ch < LEDHANDLER_CHANNEL_MAX_NUM; ch++)
        {
            ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDHANDLER_MODE, ch, ledHandler_ledValue[aLed1][ch], LEDHANDLER_FADE_TIME));
            ESP_ERROR_CHECK(ledc_fade_start(LEDHANDLER_MODE, ch, LEDC_FADE_NO_WAIT));
        }

        ESP_LOGI(TAG, "UPDATED RGB aLed1:%d,%d,%d,%d", ledHandler_ledValue[aLed1][RED], ledHandler_ledValue[aLed1][GREEN], ledHandler_ledValue[aLed1][BLUE], brightness);
        break;

    case dLed1:
        ledHandler_ledValue[dLed1][RED] = newVal[RED];
        ledHandler_ledValue[dLed1][GREEN] = newVal[GREEN];
        ledHandler_ledValue[dLed1][BLUE] = newVal[BLUE];

        for (int i = 0; i < LEDHANDLER_DLED1_LED_COUNT; i++)
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(ledHandler_dLed1Strip, i, ledHandler_ledValue[dLed1][RED], ledHandler_ledValue[dLed1][GREEN], ledHandler_ledValue[dLed1][BLUE]));
        }
        ESP_ERROR_CHECK(led_strip_refresh(ledHandler_dLed1Strip));
        ESP_LOGI(TAG, "UPDATED RGB dLed1:%d,%d,%d,%d", ledHandler_ledValue[dLed1][RED], ledHandler_ledValue[dLed1][GREEN], ledHandler_ledValue[dLed1][BLUE], brightness);
        break;

    case dLed2:
        ledHandler_ledValue[dLed2][RED] = newVal[RED];
        ledHandler_ledValue[dLed2][GREEN] = newVal[GREEN];
        ledHandler_ledValue[dLed2][BLUE] = newVal[BLUE];

        for (int i = 0; i < LEDHANDLER_DLED2_LED_COUNT; i++)
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(ledHandler_dLed2Strip, i, ledHandler_ledValue[dLed2][RED], ledHandler_ledValue[dLed2][GREEN], ledHandler_ledValue[dLed2][BLUE]));
        }
        ESP_ERROR_CHECK(led_strip_refresh(ledHandler_dLed2Strip));
        ESP_LOGI(TAG, "UPDATED RGB dLed2:%d,%d,%d,%d", ledHandler_ledValue[dLed2][RED], ledHandler_ledValue[dLed2][GREEN], ledHandler_ledValue[dLed2][BLUE], brightness);
        break;

    default:
        ESP_LOGI(TAG, "Unsupported led type");
        break;
    }
}

void ledHandler_SetOnOff(bool stateVal, ledHandler_ledType_t ledType)
{
    switch (ledType)
    {
    case aLed1:
        if (stateVal != ledHandler_ledValue[aLed1][STATE])
        {
            ledHandler_ledValue[aLed1][STATE] = stateVal;
            ESP_ERROR_CHECK(gpio_set_level(LEDHANDLER_OUTPUT_IO_RELAY, (uint32_t)stateVal));
        }

        break;

    case dLed1:
        if (stateVal != ledHandler_ledValue[dLed1][STATE])
        {
            ledHandler_ledValue[dLed1][STATE] = stateVal;
            if (false == stateVal)
            {
                ESP_ERROR_CHECK(led_strip_clear(ledHandler_dLed1Strip));
            }
            else if (true == stateVal)
            {
                for (int i = 0; i < LEDHANDLER_DLED1_LED_COUNT; i++)
                {
                    ESP_ERROR_CHECK(led_strip_set_pixel(ledHandler_dLed1Strip, i, ledHandler_ledValue[dLed1][RED], ledHandler_ledValue[dLed1][GREEN], ledHandler_ledValue[dLed1][BLUE]));
                }
                ESP_ERROR_CHECK(led_strip_refresh(ledHandler_dLed1Strip));
            }
        }
        break;

    case dLed2:
        if (stateVal != ledHandler_ledValue[dLed2][STATE])
        {
            ledHandler_ledValue[dLed2][STATE] = stateVal;
            if (false == stateVal)
            {
                ESP_ERROR_CHECK(led_strip_clear(ledHandler_dLed2Strip));
            }
            else if (true == stateVal)
            {
                for (int i = 0; i < LEDHANDLER_DLED2_LED_COUNT; i++)
                {
                    ESP_ERROR_CHECK(led_strip_set_pixel(ledHandler_dLed2Strip, i, ledHandler_ledValue[dLed2][RED], ledHandler_ledValue[dLed2][GREEN], ledHandler_ledValue[dLed2][BLUE]));
                }
                ESP_ERROR_CHECK(led_strip_refresh(ledHandler_dLed2Strip));
            }
        }

        break;

    default:
        ESP_LOGI(TAG, "Unsupported led type");
        break;
    }
}
