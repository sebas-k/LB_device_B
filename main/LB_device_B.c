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

//Pin configuration
#define Red_LED GPIO_NUM_46 //Red LED, ON = LOW
#define Green_LED GPIO_NUM_0 //Green LED, ON = LOW
#define Blue_LED GPIO_NUM_45 //Blue LED, ON = LOW
#define Yellow_LED GPIO_NUM_48 //Yellow LED, OFF = High
#define IR_receiver_1 GPIO_NUM_21 //D10 (Arduino Nano ESP32 Layout)

//Pin select
#define LED_OUTPUT_PIN_SEL  ((1ULL<<Green_LED) | (1ULL<<Blue_LED) | (1ULL<<Red_LED) | (1ULL<<Yellow_LED))
#define GPIO_INTERRUPT_PIN_SEL  ((1ULL<<IR_receiver_1))
#define ESP_INTR_FLAG_DEFAULT 0

//Global
static const char *TAG = "LB_device_B";
volatile bool reset_time_frame = false;
volatile bool light_barrier_active = false;
uint64_t ms = 0;
typedef struct {
    uint64_t ms_count;
} queue_timer_1ms_t;

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
    }
    reset_time_frame = false;// insert else/if for the High Task Awoke
    return (high_task_awoken == pdTRUE);// return whether we need to yield at the end of ISR
}

void app_main(void)
{

queue_timer_1ms_t ele; //Create queue for Time Measurments
QueueHandle_t queue_timer_1ms = xQueueCreate(10, sizeof(queue_timer_1ms_t));
if (!queue_timer_1ms) {
    ESP_LOGE(TAG, "Creating queue failed");
    return;
}

GPIO_config();
GPIO_interrupt_config();
gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //Install interrupt isr service routinne
gpio_isr_handler_add(IR_receiver_1, IR_interrupt_isr_handler, (void*) IR_receiver_1); //Hook isr handler for TSOPs gpio pin
ESP_LOGI(TAG, "IR_receiver_1 interrupt routine isr added - done");

gptimer_handle_t gptimer_1ms = NULL;// Base Configuration 
gptimer_config_t timer_config = { // .clk_src = GPTIMER_CLK_SRC_DEFAULT
    .clk_src = GPTIMER_CLK_SRC_XTAL,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000,// 1MHz, 1 tick = 1us
    };
ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_1ms)); //Create Timer
gptimer_event_callbacks_t cbs = {
    .on_alarm = timer_1ms_on_alarm, //Set callback event kind and ISR (Interupt Service Routine)
};
ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_1ms, &cbs, queue_timer_1ms)); //Register the callback
ESP_LOGI(TAG, "Enable timer"); //Enable Timer
ESP_ERROR_CHECK(gptimer_enable(gptimer_1ms));
gptimer_alarm_config_t alarm_config_1ms = { //Config the Alarm and start
    .reload_count = 0,
    .alarm_count = 1000, //Period = 1ms
    .flags.auto_reload_on_alarm = true,
};   
ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_1ms, &alarm_config_1ms)); //Start and Enable finally
//ESP_ERROR_CHECK(gptimer_start(gptimer_1ms)); => erst nach dem Sync starten
ESP_LOGI(TAG, "1ms Timer configured - done");
ESP_LOGI(TAG, "Configuration done - device fully booted.");

ESP_ERROR_CHECK(gptimer_start(gptimer_1ms));

while (1) {
//for ( int cnt = 1; cnt <= 10; cnt++) {
    if (xQueueReceive(queue_timer_1ms, &ele, portMAX_DELAY)) {
        ESP_LOGI(TAG, "MS since started the Timer %llu", ele.ms_count);
    }   
}
}