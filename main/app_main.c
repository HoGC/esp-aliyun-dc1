#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"

#include "infra_compat.h"

#include "linkkit_solo.h"
#include "factory_restore.h"

#include "conn_mgr.h"
#include "cJSON.h"

#include "dc1.h"

static const char *TAG = "app main";

static bool linkkit_started = false;

static int user_property_set_event_handler(const int devid, const char *request, const int request_len)
{
    int res = 0;
    cJSON *root = NULL, *PowerSwitch = NULL, *PowerSwitch_1 = NULL, *PowerSwitch_2 = NULL, *PowerSwitch_3 = NULL;
    char retData[50];
    ESP_LOGI(TAG,"Property Set Received, Devid: %d, Request: %s", devid, request);
    
    if (!request) {
        return NULL_VALUE_ERROR;
    }

    /* Parse Root */
    root = cJSON_Parse(request);
    if (!root) {
        ESP_LOGI(TAG,"JSON Parse Error");
        return FAIL_RETURN;
    }

    /** Switch Lightbulb On/Off   */
    PowerSwitch = cJSON_GetObjectItem(root, "PowerSwitch");
    if (PowerSwitch) {
        dc1_set_switch(0, PowerSwitch->valueint);
    }
    PowerSwitch_1 = cJSON_GetObjectItem(root, "PowerSwitch_1");
    if (PowerSwitch_1) {
        dc1_set_switch(1, PowerSwitch_1->valueint);
    } 
    PowerSwitch_2 = cJSON_GetObjectItem(root, "PowerSwitch_2");
    if (PowerSwitch_2) {
        dc1_set_switch(2, PowerSwitch_2->valueint);
    }
    PowerSwitch_3 = cJSON_GetObjectItem(root, "PowerSwitch_3");
    if (PowerSwitch_3) {
        dc1_set_switch(3, PowerSwitch_3->valueint);
    } 
    
    cJSON_Delete(root);

    return SUCCESS_RETURN;
}

static esp_err_t wifi_event_handle(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_GOT_IP:
            if (linkkit_started == false) {
                wifi_config_t wifi_config = {0};
                if (conn_mgr_get_wifi_config(&wifi_config) == ESP_OK &&
                    strcmp((char *)(wifi_config.sta.ssid), HOTSPOT_AP) &&
                    strcmp((char *)(wifi_config.sta.ssid), ROUTER_AP)) {
                    xTaskCreate((void (*)(void *))linkkit_main, "linkkit_main", 10240, NULL, 5, NULL);
                    linkkit_started = true;

                    IOT_RegisterCallback(ITE_PROPERTY_SET, user_property_set_event_handler);
                }
            }
            break;

        default:
            break;
    }

    return ESP_OK;
}

static void linkkit_event_monitor(int event)
{
    switch (event) {
        case IOTX_AWSS_START: // AWSS start without enbale, just supports device discover
            // operate led to indicate user
            ESP_LOGI(TAG, "IOTX_AWSS_START");
            break;

        case IOTX_AWSS_ENABLE: // AWSS enable, AWSS doesn't parse awss packet until AWSS is enabled.
            ESP_LOGI(TAG, "IOTX_AWSS_ENABLE");
            // operate led to indicate user
            break;

        case IOTX_AWSS_LOCK_CHAN: // AWSS lock channel(Got AWSS sync packet)
            ESP_LOGI(TAG, "IOTX_AWSS_LOCK_CHAN");
            // operate led to indicate user
            break;

        case IOTX_AWSS_PASSWD_ERR: // AWSS decrypt passwd error
            ESP_LOGE(TAG, "IOTX_AWSS_PASSWD_ERR");
            // operate led to indicate user
            break;

        case IOTX_AWSS_GOT_SSID_PASSWD:
            ESP_LOGI(TAG, "IOTX_AWSS_GOT_SSID_PASSWD");
            // operate led to indicate user
            break;

        case IOTX_AWSS_CONNECT_ADHA: // AWSS try to connnect adha (device
            // discover, router solution)
            ESP_LOGI(TAG, "IOTX_AWSS_CONNECT_ADHA");
            // operate led to indicate user
            break;

        case IOTX_AWSS_CONNECT_ADHA_FAIL: // AWSS fails to connect adha
            ESP_LOGE(TAG, "IOTX_AWSS_CONNECT_ADHA_FAIL");
            // operate led to indicate user
            break;

        case IOTX_AWSS_CONNECT_AHA: // AWSS try to connect aha (AP solution)
            ESP_LOGI(TAG, "IOTX_AWSS_CONNECT_AHA");
            // operate led to indicate user
            break;

        case IOTX_AWSS_CONNECT_AHA_FAIL: // AWSS fails to connect aha
            ESP_LOGE(TAG, "IOTX_AWSS_CONNECT_AHA_FAIL");
            // operate led to indicate user
            break;

        case IOTX_AWSS_SETUP_NOTIFY: // AWSS sends out device setup information
            // (AP and router solution)
            ESP_LOGI(TAG, "IOTX_AWSS_SETUP_NOTIFY");
            // operate led to indicate user
            break;

        case IOTX_AWSS_CONNECT_ROUTER: // AWSS try to connect destination router
            ESP_LOGI(TAG, "IOTX_AWSS_CONNECT_ROUTER");
            // operate led to indicate user
            break;

        case IOTX_AWSS_CONNECT_ROUTER_FAIL: // AWSS fails to connect destination
            // router.
            ESP_LOGE(TAG, "IOTX_AWSS_CONNECT_ROUTER_FAIL");
            // operate led to indicate user
            break;

        case IOTX_AWSS_GOT_IP: // AWSS connects destination successfully and got
            // ip address
            ESP_LOGI(TAG, "IOTX_AWSS_GOT_IP");
            // operate led to indicate user
            break;

        case IOTX_AWSS_SUC_NOTIFY: // AWSS sends out success notify (AWSS
            // sucess)
            ESP_LOGI(TAG, "IOTX_AWSS_SUC_NOTIFY");
            // operate led to indicate user
            break;

        case IOTX_AWSS_BIND_NOTIFY: // AWSS sends out bind notify information to
            // support bind between user and device
            ESP_LOGI(TAG, "IOTX_AWSS_BIND_NOTIFY");
            // operate led to indicate user
            break;

        case IOTX_AWSS_ENABLE_TIMEOUT: // AWSS enable timeout
            // user needs to enable awss again to support get ssid & passwd of router
            ESP_LOGW(TAG, "IOTX_AWSS_ENALBE_TIMEOUT");
            // operate led to indicate user
            break;

        case IOTX_CONN_CLOUD: // Device try to connect cloud
            ESP_LOGI(TAG, "IOTX_CONN_CLOUD");
            // operate led to indicate user
            break;

        case IOTX_CONN_CLOUD_FAIL: // Device fails to connect cloud, refer to
            // net_sockets.h for error code
            ESP_LOGE(TAG, "IOTX_CONN_CLOUD_FAIL");
            // operate led to indicate user
            break;

        case IOTX_CONN_CLOUD_SUC: // Device connects cloud successfully
            ESP_LOGI(TAG, "IOTX_CONN_CLOUD_SUC");
            // operate led to indicate user
            break;

        case IOTX_RESET: // Linkkit reset success (just got reset response from
            // cloud without any other operation)
            ESP_LOGI(TAG, "IOTX_RESET");
            // operate led to indicate user
            break;

        default:
            break;
    }
}

static void start_conn_mgr()
{
    iotx_event_regist_cb(linkkit_event_monitor);    // awss callback
    conn_mgr_start();

    vTaskDelete(NULL);
}


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
        case SWITCH0_OPEN:
            ESP_LOGI(TAG, "switch0_open");
            user_post_property_str("{\"PowerSwitch\": 1}");
            break;
        case SWITCH0_CLOSE:
            ESP_LOGI(TAG, "switch0_close");
            user_post_property_str("{\"PowerSwitch\": 0}");
            break;
        case SWITCH1_OPEN:
            ESP_LOGI(TAG, "switch1_open");
            user_post_property_str("{\"PowerSwitch_1\": 1}");
            break;
        case SWITCH1_CLOSE:
            ESP_LOGI(TAG, "switch1_close");
            user_post_property_str("{\"PowerSwitch_1\": 0}");
            break;
        case SWITCH2_OPEN:
            ESP_LOGI(TAG, "switch2_open");
            user_post_property_str("{\"PowerSwitch_2\": 1}");
            break;
        case SWITCH2_CLOSE:
            ESP_LOGI(TAG, "switch2_close");
            user_post_property_str("{\"PowerSwitch_2\": 0}");
            break;
        case SWITCH3_OPEN:
            ESP_LOGI(TAG, "switch3_open");
            user_post_property_str("{\"PowerSwitch_3\": 1}");
            break;
        case SWITCH3_CLOSE:
            ESP_LOGI(TAG, "switch3_close");
            user_post_property_str("{\"PowerSwitch_3\": 0}");
            break;

        default:
            break;
    }
}

void app_main()
{
    factory_restore_init();

    dc1_init();
    dc1_event_regist_cb(user_dc1_event_cb);

    conn_mgr_init();
    conn_mgr_register_wifi_event(wifi_event_handle);

    IOT_SetLogLevel(IOT_LOG_INFO);

    xTaskCreate((void (*)(void *))start_conn_mgr, "conn_mgr", 3072, NULL, 5, NULL);
}
