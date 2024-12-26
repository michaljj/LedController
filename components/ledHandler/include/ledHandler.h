#ifndef LEDHANDLER_H
#define LEDHANDLER_H

#define LEDHANDLER_TIMER LEDC_TIMER_0
#define LEDHANDLER_MODE LEDC_LOW_SPEED_MODE
#define LEDHANDLER_OUTPUT_IO_RED (5)                         // Define the output GPIO
#define LEDHANDLER_OUTPUT_IO_GREEN (4)                       // Define the output GPIO
#define LEDHANDLER_OUTPUT_IO_BLUE (6)                        // Define the output GPIO
#define LEDHANDLER_OUTPUT_IO_RELAY (GPIO_NUM_12)             // Define the output GPIO
#define LEDHANDLER_OUTPUT_IO_RELAY_BIT (1ULL << GPIO_NUM_12) // Define the output GPIO
#define LEDHANDLER_OUTPUT_IO_DLED1 1
#define LEDHANDLER_DLED1_LED_COUNT 3
#define LEDHANDLER_OUTPUT_IO_DLED2 2
#define LEDHANDLER_DLED2_LED_COUNT 3
#define LEDHANDLER_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)
#define LEDHANDLER_CHANNEL_RED LEDC_CHANNEL_0
#define LEDHANDLER_CHANNEL_GREEN LEDC_CHANNEL_1
#define LEDHANDLER_CHANNEL_BLUE LEDC_CHANNEL_2
#define LEDHANDLER_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDHANDLER_FREQUENCY (4000)           // Frequency in Hertz. Set frequency at 4 kHz
#define LEDHANDLER_CHANNEL_MAX_NUM 3
#define LEDHANDLER_FADE_TIME 100

typedef enum ledHandler_ledType
{
    aLed1, // Analog LED
    dLed1, // Digital LED 1
    dLed2  // Digital LED 2
} ledHandler_ledType_t;

typedef enum ledHandler_ledVal
{
    RED,
    GREEN, 
    BLUE,
    STATE
} ledHandler_ledVal_t;

void ledHandler_Init();
void ledHandler_SetRGB(int redVal, int greenVal, int blueVal, int brightness, ledHandler_ledType_t ledType);
void ledHandler_SetOnOff(bool stateVal, ledHandler_ledType_t ledType);

#endif
