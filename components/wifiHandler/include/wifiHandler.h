#define ESP_WIFI_SSID      "Michal_home_2_4"
#define ESP_WIFI_PASS      "Jurek744"
#define ESP_MAXIMUM_RETRY  10

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
extern const char *TAG;

void wifi_init_sta(void);