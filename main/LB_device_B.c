//Includes
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"

//Pin configuration
#define Red_LED GPIO_NUM_46 //Red LED, ON = LOW
#define Green_LED GPIO_NUM_0 //Green LED, ON = LOW
#define Blue_LED GPIO_NUM_45 //Blue LED, ON = LOW
#define Yellow_LED GPIO_NUM_48 //Yellow LED, OFF = High
#define IR_receiver_1 GPIO_NUM_21 //D10 (Arduino Nano ESP32 Layout)
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#define CONFIG_ESPNOW_CHANNEL 5

//Pin select
#define LED_OUTPUT_PIN_SEL  ((1ULL<<Green_LED) | (1ULL<<Blue_LED) | (1ULL<<Red_LED) | (1ULL<<Yellow_LED))
#define GPIO_INTERRUPT_PIN_SEL  ((1ULL<<IR_receiver_1))
#define ESP_INTR_FLAG_DEFAULT 0

//Meta data
#define SYNC_DEVICES 0
#define TIME_MEASURE 1

//LED Indicator
#define RGB_on 0
#define RGB_off 1
volatile bool one_shot = 0;
volatile bool LED_Indicator_Arr [6] = {1,1,1,1,1,1};

//Global
static const char *TAG = "LB_device_B";
volatile bool reset_time_frame = false;
volatile bool light_barrier_active = false;
volatile char device_role = 0;
static uint8_t broadcastAddress[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

//Device Role status
#define DEVICE_NOT_INITALIZED 0
#define DEVICE_IS_TIMER_SYNC_MASTER 1
#define DEVICE_IS_NOT_TIMER_SYNC_MASTER 2
#define DEVICE_IS_IR_RECEIVER 3

uint64_t ms = 0;
typedef struct {
    uint64_t ms_count;
} queue_timer_1ms_t;

typedef struct PayLoad {
    char meta_data;
    int intVal_ms;
} PayLoad;

PayLoad myPayLoad; 

void GPIO_config (void)
{
gpio_config_t led_io_conf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = LED_OUTPUT_PIN_SEL,
    .pull_down_en = 0,
    .pull_up_en = 0
    };
    gpio_config(&led_io_conf);

    gpio_set_level (Red_LED, 1); //OFF
    gpio_set_level (Green_LED, 1); //OFF
    gpio_set_level (Blue_LED, 1); //OFF
    gpio_set_level (Yellow_LED, 1); //ON
    ESP_LOGI(TAG, "GPIO LED config - done");
}

void GPIO_interrupt_config (void){
    gpio_config_t inter_io_conf = {
    .intr_type = GPIO_INTR_NEGEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = GPIO_INTERRUPT_PIN_SEL,
    .pull_down_en = 0,
    .pull_up_en = 1
    };
    gpio_config(&inter_io_conf);
    ESP_LOGI(TAG, "GPIO interrupt config - done");
}

static void app_wifi_init()
{
ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
}

static void IRAM_ATTR IR_interrupt_isr_handler(void* arg)
{
   reset_time_frame = true;
   light_barrier_active = true;
}

static bool IRAM_ATTR timer_1ms_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    ms++;
    if (light_barrier_active == true && reset_time_frame == false){
        light_barrier_active = false;
        QueueHandle_t queue = (QueueHandle_t)user_data;
        queue_timer_1ms_t ele = {
        .ms_count = ms
        };
        xQueueSendFromISR(queue, &ele, &high_task_awoken);
        one_shot = 1;
    }
    reset_time_frame = false;// insert else/if for the High Task Awoke
    return (high_task_awoken == pdTRUE);// return whether we need to yield at the end of ISR
}

static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
ESP_LOGI(TAG, "Time sending - done");
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    memcpy(&myPayLoad, data, sizeof(myPayLoad));
    if (myPayLoad.meta_data == SYNC_DEVICES){
        ms = myPayLoad.intVal_ms * 1000; // that is not too nice, rework
        device_role = DEVICE_IS_IR_RECEIVER;
    }
    ESP_LOGI(TAG, "%llu", ms);
}

void ESPnow_config(){
//Config ESP_Now
ESP_LOGI(TAG, "Wifi Config");
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK( nvs_flash_erase() );
    ret = nvs_flash_init();}
ESP_ERROR_CHECK( ret );
app_wifi_init();
esp_now_init();
ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
esp_now_peer_info_t peerInfo = {};
memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
if(!esp_now_is_peer_exist(broadcastAddress))
{
    esp_now_add_peer(&peerInfo);}
}

void LED_Indicator_Task(void *params) {
char count = 0;

while (1){

if (one_shot){
    gpio_set_level (Red_LED, 0); 
    gpio_set_level (Green_LED, 1); 
    gpio_set_level (Blue_LED, 1);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    count = 11;
    one_shot = 0;
}

switch (count) {
    case 1:
    gpio_set_level (Red_LED, LED_Indicator_Arr[0]); 
    gpio_set_level (Green_LED, LED_Indicator_Arr[1]); 
    gpio_set_level (Blue_LED, LED_Indicator_Arr[2]);
    break;
    case 11:
    gpio_set_level (Red_LED, LED_Indicator_Arr[3]); 
    gpio_set_level (Green_LED, LED_Indicator_Arr[4]); 
    gpio_set_level (Blue_LED, LED_Indicator_Arr[5]); 
    break;
    case 20:
    count = 0;
    break;
}
vTaskDelay(100 / portTICK_PERIOD_MS);
count++;
}
}

void app_main(void)
{

queue_timer_1ms_t ele; //Create queue for Time Measurments
QueueHandle_t queue_timer_1ms = xQueueCreate(10, sizeof(queue_timer_1ms_t));
if (!queue_timer_1ms) {
    ESP_LOGE(TAG, "Creating queue failed");
    return;}

GPIO_config();
GPIO_interrupt_config();

gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //Install interrupt isr service routinne
gpio_isr_handler_add(IR_receiver_1, IR_interrupt_isr_handler, (void*) IR_receiver_1); //Hook isr handler for TSOPs gpio pin
ESP_LOGI(TAG, "IR_receiver_1 interrupt routine isr added - done");

LED_Indicator_Arr [0] = RGB_off; LED_Indicator_Arr [1] = RGB_off; LED_Indicator_Arr [2] = RGB_on;
LED_Indicator_Arr [3] = RGB_off; LED_Indicator_Arr [4] = RGB_off; LED_Indicator_Arr [5] = RGB_off;
xTaskCreatePinnedToCore(LED_Indicator_Task, "LED_Indicator_Task", 2048, NULL, 1, NULL, 0);

//Config Timer
gptimer_handle_t gptimer_1ms = NULL;
gptimer_config_t timer_config = { 
    .clk_src = GPTIMER_CLK_SRC_XTAL,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000,};
ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_1ms));
gptimer_event_callbacks_t cbs = {
    .on_alarm = timer_1ms_on_alarm, };
ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_1ms, &cbs, queue_timer_1ms)); 
ESP_ERROR_CHECK(gptimer_enable(gptimer_1ms));
gptimer_alarm_config_t alarm_config_1ms = { 
    .reload_count = 0,
    .alarm_count = 1000, 
    .flags.auto_reload_on_alarm = true,};   
ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_1ms, &alarm_config_1ms));
ESP_ERROR_CHECK(gptimer_start(gptimer_1ms));

ESPnow_config();

vTaskDelay(5500 / portTICK_PERIOD_MS); //Wait till a sync is received

while (device_role == DEVICE_NOT_INITALIZED) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

switch (device_role)
{
case DEVICE_IS_TIMER_SYNC_MASTER:
    LED_Indicator_Arr [0] = RGB_on; LED_Indicator_Arr [1] = RGB_on; LED_Indicator_Arr [2] = RGB_on;
    LED_Indicator_Arr [3] = RGB_on; LED_Indicator_Arr [4] = RGB_on; LED_Indicator_Arr [5] = RGB_on;
    ESP_LOGI(TAG, "Device is Timer Sync Master");
    break;
case DEVICE_IS_NOT_TIMER_SYNC_MASTER:
    ESP_LOGI(TAG, "Device is NOT Timer Sync Master");
    LED_Indicator_Arr [0] = RGB_off; LED_Indicator_Arr [1] = RGB_on; LED_Indicator_Arr [2] = RGB_off;
    LED_Indicator_Arr [3] = RGB_off; LED_Indicator_Arr [4] = RGB_on; LED_Indicator_Arr [5] = RGB_off;
    break;
case DEVICE_IS_IR_RECEIVER:
    ESP_LOGI(TAG, "Device is NOT Timer Sync Master");
    LED_Indicator_Arr [0] = RGB_off; LED_Indicator_Arr [1] = RGB_on; LED_Indicator_Arr [2] = RGB_off;
    LED_Indicator_Arr [3] = RGB_off; LED_Indicator_Arr [4] = RGB_on; LED_Indicator_Arr [5] = RGB_off;
    break;
}


while (1) {
    vTaskDelay(200 / portTICK_PERIOD_MS); //if not all Device start at the exact exact time there will be no culliosn
    if (xQueueReceive(queue_timer_1ms, &ele, 10 / portTICK_PERIOD_MS)) {
        myPayLoad.meta_data = TIME_MEASURE;
        myPayLoad.intVal_ms = ele.ms_count;
        esp_now_send(broadcastAddress, (uint8_t *) &myPayLoad, sizeof(myPayLoad));
        ESP_LOGI(TAG, "MS since started the Timer %llu", ele.ms_count);
    }   
}
}