#ifndef LEDHANDLER_H
#define LEDHANDLER_H

#define LEDHANDLER_TIMER                    LEDC_TIMER_0
#define LEDHANDLER_MODE                     LEDC_LOW_SPEED_MODE
#define LEDHANDLER_OUTPUT_IO_RED            (5) // Define the output GPIO
#define LEDHANDLER_OUTPUT_IO_GREEN          (4) // Define the output GPIO
#define LEDHANDLER_OUTPUT_IO_BLUE           (6) // Define the output GPIO
#define LEDHANDLER_OUTPUT_IO_RELAY          (GPIO_NUM_12) // Define the output GPIO
#define LEDHANDLER_OUTPUT_IO_RELAY_BIT      (1ULL<<GPIO_NUM_12) // Define the output GPIO
#define LEDHANDLER_CHANNEL_RED              LEDC_CHANNEL_0
#define LEDHANDLER_CHANNEL_GREEN            LEDC_CHANNEL_1
#define LEDHANDLER_CHANNEL_BLUE             LEDC_CHANNEL_2
#define LEDHANDLER_DUTY_RES                 LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDHANDLER_FREQUENCY                (4000) // Frequency in Hertz. Set frequency at 4 kHz

void ledHandler_Init();
void ledHandler_SetRGB(int redVal,int greenVal, int blueVal, int brightness);
void ledHandler_SetOnOff(bool stateVal);


#endif
