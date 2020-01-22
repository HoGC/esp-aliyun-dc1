#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "dc1_i2c.h"
#include "dc1.h"

static const char *TAG = "main";

void user_dc1_event_cb(int event){
    switch (event) {
        case KEY0_SHORTPRESS_CB:
            ESP_LOGI(TAG, "key0ShortPress_cb");
            break;
        case KEY1_SHORTPRESS_CB:
            ESP_LOGI(TAG, "key1ShortPress_cb");
            break;
        case KEY2_SHORTPRESS_CB:
            ESP_LOGI(TAG, "key2ShortPress_cb");
            break;
        case KEY3_SHORTPRESS_CB:
            ESP_LOGI(TAG, "key3ShortPress_cb");
            break;
        case KEY0_LONGPRESS_CB:
            ESP_LOGI(TAG, "key0LongPress_cb");
            break;
        case KEY1_LONGPRESS_CB:
            ESP_LOGI(TAG, "key1LongPress_cb");
            break;
        case KEY2_LONGPRESS_CB:
            ESP_LOGI(TAG, "key2LongPress_cb");
            break;
        case KEY3_LONGPRESS_CB:
            ESP_LOGI(TAG, "key3LongPress_cb");
            break;

        default:
            break;
    }
}
void app_main(void)
{
    //start i2c task
    dc1_init();
    dc1_event_regist_cb(user_dc1_event_cb);
}