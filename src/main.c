#include <stdio.h>
#include <string.h>
#include <driver/gpio.h>
#include <driver/timer.h>
#include <hal/timer_types.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <esp_http_client.h>
#include <esp_netif.h>
#include <esp_sleep.h>
#include <constants.h>

#define TRIGPIN GPIO_NUM_26
#define TRIGPIN_MASK GPIO_SEL_26
#define ECHOPIN GPIO_NUM_25
#define ECHOPIN_MASK GPIO_SEL_25
#define SWITCHPIN GPIO_NUM_27
#define SWITCHPIN_MASK GPIO_SEL_27

#define DEEPSLEEPTIME 5*1000000

static const char* TAG = "Main";
volatile uint32_t distance;
volatile bool echoReceived = false;
volatile bool echoTimeout = false;
bool networkConnection = false;

static timer_isr_t IRAM_ATTR pulsegen_isr_callback()
{
    gpio_set_level(TRIGPIN, 0); 
    timer_pause(0,0);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0,0);
     
     
    return false;
}

void echoMeasurement_isr_callback()
{
    
    if(gpio_get_level(ECHOPIN) == 1)//rising edge, start of echo
    {
        timer_start(TIMER_GROUP_0, TIMER_1);
    }
    else
    {
        timer_pause(TIMER_GROUP_0, TIMER_1);
        uint64_t timerCnt;
        timer_get_counter_value(TIMER_GROUP_0, TIMER_1, &timerCnt);
        timer_set_counter_value(TIMER_GROUP_0, TIMER_1,0);
        distance = timerCnt/236; //in mm
        echoReceived = true;
    }
}

void echo_timeout_isr_callback()
{
    timer_pause(TIMER_GROUP_0, TIMER_1);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    echoTimeout = true;
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    ESP_LOGI(TAG, "HTTP Event id %d", evt->event_id);
    return ESP_OK;
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data)
{
    if(event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "WiFi Connecting...");
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "WiFi Connected");
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(TAG, "Sta got IP");
        networkConnection = true;
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {

        ESP_LOGI(TAG, "Sta disconnected"); 
        esp_sleep_enable_timer_wakeup(DEEPSLEEPTIME);
        esp_deep_sleep_start();
    }
}

void pin_init()
{
    gpio_config_t conf;
    //set defined pin as Output, used to trigger the ultrasonic module
    conf.pin_bit_mask = TRIGPIN_MASK;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);
    gpio_set_level(TRIGPIN, 0);
    //set defined pin as input, used to read signal from ultrasonic module 
    conf.pin_bit_mask = ECHOPIN_MASK;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_down_en = GPIO_PULLDOWN_ENABLE;//Enable pulldown to guarantee idle state (when not connected etc.)
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&conf);
    //set defined pin as output, to switch ultrasonic sensor on when needed, to save energy when in deepsleep
    conf.pin_bit_mask = SWITCHPIN_MASK;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_down_en = GPIO_PULLDOWN_ENABLE;//Enable pulldown to disable sensor by default
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);
    //interrupt conf
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ECHOPIN, echoMeasurement_isr_callback, NULL));
}

void tim_init()
{
    timer_config_t conf;
    //configure timergroup0timer0 as upcounter with alarm to use as pulse generator delay for triggering ultrasonic module
    //divider is set to 1 tick = 1us
    conf.counter_dir = TIMER_COUNT_UP;
    conf.counter_en  = TIMER_PAUSE;
    conf.auto_reload = TIMER_AUTORELOAD_EN;
    conf.divider = 80;
    conf.alarm_en  = TIMER_ALARM_EN;
    //conf.intr_type = TIMER_INTR_LEVEL;
    timer_init(TIMER_GROUP_0, TIMER_0, &conf); 
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0,0);    
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0,2);//smaller than mathematically required alarm is used to compensato for overhead, nominal pulselength would be 10us
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, pulsegen_isr_callback, NULL, 0);
    timer_enable_intr(0,0);

    //configure timergroup0timer1 as upcounter to measure length of ultrasonic signal
    //smallest possible prescaler for max resolution (probably not necessary but marginal gain)
    //1 tick = 1/40MHz = 25 ns = 8,5um with 340m/s speed of sound
    //Alarm used as timeout
    conf.counter_dir = TIMER_COUNT_UP;
    conf.counter_en  = TIMER_PAUSE;
    conf.auto_reload = TIMER_AUTORELOAD_DIS;
    conf.divider = 2;
    conf.alarm_en  = TIMER_ALARM_EN;
    timer_init(TIMER_GROUP_0, TIMER_1, &conf); 
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1,0);   
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1,600000);//timeout for distances > 5m = 15ms = 600000 tick
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, echo_timeout_isr_callback, NULL, 0);
    timer_enable_intr(0,0); 
}


void wifi_init()
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t conf = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&conf);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t staConf = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PW,
           }
        };
    esp_wifi_set_config(WIFI_IF_STA, &staConf);
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_connect();
    ESP_LOGI(TAG, "wifiInit completed");
}

void sendDistance(int dist){
    char payload[45];
    sprintf(payload, "distance,location=indoor mm=%d.0", dist);
    ESP_LOGI(TAG, "%s", payload);
    vTaskDelay(1);
    esp_http_client_config_t conf = {   .url = INFLUX_API_URL,
                                        .port = 8086,
                                        .method = HTTP_METHOD_POST,
                                        .auth_type = HTTP_AUTH_TYPE_NONE,
                                        .event_handler = _http_event_handler,
                                        .transport_type = HTTP_TRANSPORT_OVER_TCP
                                        };
    esp_http_client_handle_t client = esp_http_client_init(&conf);
    esp_http_client_set_header(client, "Authorization" , INFLUX_API_AUTH_KEY);
    esp_http_client_set_post_field(client, payload, strlen(payload));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_http_client_perform(client));
}

uint16_t getDistance()
{
    
    echoReceived = false;
    echoTimeout = false;
    gpio_set_level(SWITCHPIN, 1);
    gpio_set_level(TRIGPIN, 1);
    timer_start(TIMER_GROUP_0, TIMER_0);
    while(!echoReceived &&  !echoTimeout){vTaskDelay(1);}
    gpio_set_level(SWITCHPIN, 0);
    if(echoReceived)
    {
        return distance;
    }
    if(echoTimeout)
    {
       return 0;
    }
    return 0;
}

void app_main() {

    pin_init();
    tim_init();
    wifi_init();
    while(!networkConnection){vTaskDelay(1);};
    int dist = getDistance();
    if(echoTimeout)
    {
        ESP_LOGE(TAG, "Echo Timeout");
    }
    if(dist != 0)
    {
        sendDistance(dist);
    }

    esp_sleep_enable_timer_wakeup(DEEPSLEEPTIME);
    esp_deep_sleep_start();

}