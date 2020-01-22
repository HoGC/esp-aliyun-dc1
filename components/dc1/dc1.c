/*
 * dc1 driver
 * Author: HoGC 
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "dc1_i2c.h"
#include "dc1.h"

// #define DC1_DEBUG

#if defined(DC1_DEBUG)
#define INFO(format, ...) ESP_LOGI("dc1",format, ##__VA_ARGS__)
#else
#define INFO(format, ... )
#endif

dc1_event_cb_t  dc1_event_cb = NULL;

static esp_err_t dc1_i2c_init()
{
    int i2c_master_port = I2C_DC1_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_DC1_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_DC1_SCL_IO;
    conf.scl_pullup_en = 0;
    conf.clk_stretch_tick = 100;
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

static esp_err_t dc1_i2c_write(uint8_t reg_address, uint8_t data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DC1_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    // i2c_master_write(cmd, data, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_DC1_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t dc1_i2c_read(uint8_t reg_address, uint8_t *data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DC1_SENSOR_ADDR << 1 | WRITE_BIT , ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_DC1_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }
    os_delay_us(20);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DC1_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, ACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_DC1_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void switch_set_event(uint8_t num, uint8_t value){

    if(dc1_event_cb != NULL){
        switch (num)
        {
            case 0:
                if(value){
                    dc1_event_cb(SWITCH0_OPEN);
                }else{
                    dc1_event_cb(SWITCH0_CLOSE);
                    dc1_event_cb(SWITCH1_CLOSE);
                    dc1_event_cb(SWITCH2_CLOSE);
                    dc1_event_cb(SWITCH3_CLOSE);
                }
                break;
            case 1:
                if(value){
                    dc1_event_cb(SWITCH0_OPEN);
                    dc1_event_cb(SWITCH1_OPEN);
                }else{
                    dc1_event_cb(SWITCH1_CLOSE);
                }
                break;
            case 2:
                if(value){
                    dc1_event_cb(SWITCH0_OPEN);
                    dc1_event_cb(SWITCH2_OPEN);
                }else{
                    dc1_event_cb(SWITCH2_CLOSE);
                }
                break;
            case 3:
                if(value){
                    dc1_event_cb(SWITCH0_OPEN);
                    dc1_event_cb(SWITCH3_OPEN);
                }else{
                    dc1_event_cb(SWITCH3_CLOSE);
                }
                break;
            default:
                break;
        }
    }

}


static int8_t dc1_read_gpio(){
    uint8_t read_count = 10;
    uint8_t gpio_ret1 = 0;
    uint8_t gpio_ret2 = 0;
    uint8_t gpio16_ret = 0;

    gpio16_ret = gpio_get_level(16);
    while (read_count--)
    {
        dc1_i2c_read(0x00, &gpio_ret1);
        os_delay_us(4000);
        // INFO("gpio_ret1:%02x",gpio_ret1);
        if(gpio_ret1 != 0x00){
            dc1_i2c_read(0x00, &gpio_ret2);
            // INFO("gpio_ret2:%02x",gpio_ret2);
            // INFO("--------------");
            if(gpio_ret1 == gpio_ret2){
                if(gpio16_ret == 0){
                    gpio_ret1 = gpio_ret1 & 0xF7;
                }else{
                    gpio_ret1 = gpio_ret1 | 0x08;
                }
                return gpio_ret1;
            }
            os_delay_us(4000);
        }
    }
    if(gpio16_ret == 0){
        return gpio_ret1 = gpio_ret1 & 0xF7;;
    }else{
        return 0xF8;
    }
}

static bool dc1_write_gpio(uint8_t gpio_data){
    uint8_t write_count = 10;
    uint8_t gpio_ret;

    gpio_data = gpio_data & 0xF0;
    while(write_count--){
        dc1_i2c_write(0x01,gpio_data);
        os_delay_us(4000);
        gpio_ret = dc1_read_gpio();
        if(gpio_ret != 0xF8){
            if((gpio_ret & 0xF0) == gpio_data)
                return true;
        }
        os_delay_us(4000);
    }
    return false;
}

static bool reverse_switch(uint16_t switch_ret, uint8_t num){

    uint16_t switch_bit = 0x10;
    uint8_t switch_status = 0;

    // INFO("switch_ret:%02x",switch_ret);
    if(num == 0){
         switch_ret = switch_ret ^ 0x80;
        if(dc1_write_gpio(switch_ret & 0x80)){
            if((switch_ret & 0x80) == 0x80){
                logo_led_switch(1);
                switch_status = 1;
            }else{
                logo_led_switch(0);
                switch_status = 0;
            }
            switch_set_event(num,switch_status);
            return true;
        }else{
            return false;
        }
    }else{
        switch_bit = switch_bit<<(3-num);
        switch_ret = switch_ret ^ switch_bit;
        if(dc1_write_gpio((switch_ret & 0xF0) | 0x80)){
            if((switch_ret & switch_bit) == switch_bit){
                logo_led_switch(1);
                switch_status = 1;
            }else{
                switch_status = 0;
            }
            switch_set_event(num,switch_status);
            return true;
        }else{
            return false;
        }
    } 
}

void dc1_set_switch(uint8_t num, uint8_t bit_value){

    uint8_t switch_ret=0;
    uint16_t gpio_send=0;
    uint16_t switch_bit=0x10;

    switch_ret = dc1_read_gpio();
    INFO("switch_ret:%02x",switch_ret);
    if(num == 0){
        if(bit_value == 1){
            gpio_send = switch_ret | 0x80;
        }else{
            gpio_send = 0x00;
        }
    }else{
        switch_bit=switch_bit<<(3-num);
        if(bit_value == 1){
            switch_ret = switch_ret | switch_bit;
            // INFO("switch_bit:%02x",switch_ret);
            gpio_send = (switch_ret & 0xF0) | 0x80;
        }else{
            switch_bit=~switch_bit;
            switch_ret = switch_ret & switch_bit;
            gpio_send = (switch_ret & 0xF0) | 0x80;
        }
    }
    if(dc1_write_gpio(gpio_send)){
        if((gpio_send & 0x80) == 0x80){
            logo_led_switch(1); 
        }else{
            if(num == 0){
                logo_led_switch(0); 
            }
        }
        
        // if((num == 0) || (bit_value == 0)){
        //     switch_set_event(0,0);
        //     switch_set_event(1,0);
        //     switch_set_event(2,0);
        //     switch_set_event(3,0);
        // }else{
        //     switch_set_event(0,1);
        //     switch_set_event(num,bit_value);
        // }
        switch_set_event(num,bit_value);
        return true;
    }else{
        return false;
    }
}

static void dc1_key_handler(TimerHandle_t pxTimer){

	uint16_t Key_press = 0;
	static uint16_t Key_Prev = 0;
	static uint8_t Key_State = 0;
	static uint16_t Key_LongCheck = 0;

    uint8_t gpio_ret = 0;

    gpio_ret = dc1_read_gpio();
    if(gpio_ret == 0xF8){
        return;
    }
    Key_press = gpio_ret & 0x0F;
    // INFO("Key_press:%02X",Key_press);

    switch (Key_State) {
    case 0:
        if (Key_press != 0x0F) {
            Key_Prev = Key_press;
            Key_State = 1;
        }
        break;
    case 1:
        if (Key_press == Key_Prev) {
            Key_State = 2;
            INFO("KEY_DOWN\n");
            return;
        } else {
            Key_State = 0;
        }
        break;
    case 2:
        if (Key_press != Key_Prev) {
            Key_State = 0;
            Key_LongCheck = 0;
            INFO("KEY_SHORT_OR_UP");
            xTimerStop(pxTimer, 0);
            switch (Key_Prev & 0x0f)
            {
                case 0x07:
                    INFO("key0ShortPress");
                    reverse_switch(gpio_ret,0);
                    if(dc1_event_cb != NULL){
                        dc1_event_cb(KEY0_SHORTPRESS_CB);
                    }
                    break;
                case 0x0E:
                    INFO("key1ShortPress");
                    reverse_switch(gpio_ret,1);
                    if(dc1_event_cb != NULL){
                        dc1_event_cb(KEY1_SHORTPRESS_CB);
                    }
                    break;
                case 0x0D:
                    INFO("key2ShortPress");
                    reverse_switch(gpio_ret,2);
                    if(dc1_event_cb != NULL){
                        dc1_event_cb(KEY2_SHORTPRESS_CB);
                    }
                    break;
                case 0x0B:
                    INFO("key3ShortPress");
                    reverse_switch(gpio_ret,3);
                    if(dc1_event_cb != NULL){
                        dc1_event_cb(KEY3_SHORTPRESS_CB);
                    }
                    break;
            }
            xTimerStart(pxTimer,0);
            return;
        }
        if (Key_press == Key_Prev) {
            Key_LongCheck++;
            if (Key_LongCheck >= 100){
                Key_LongCheck = 0;
                Key_State = 3;
                INFO("KEY_LONG");
                xTimerStop(pxTimer, 0);
                switch (Key_Prev & 0x0F)
                {
                    case 0x07:
                        INFO("key0LongPress");
                        if(dc1_event_cb != NULL){
                            dc1_event_cb(KEY0_LONGPRESS_CB);
                        }
                        break;
                    case 0x0E:
                        INFO("key1LongPress");
                        if(dc1_event_cb != NULL){
                            dc1_event_cb(KEY1_LONGPRESS_CB);
                        }
                        break;
                    case 0x0D:
                        INFO("key2LongPress");
                        if(dc1_event_cb != NULL){
                            dc1_event_cb(KEY2_LONGPRESS_CB);
                        }
                        break;
                    case 0x0B:
                        INFO("key3LongPress");
                        if(dc1_event_cb != NULL){
                            dc1_event_cb(KEY3_LONGPRESS_CB);
                        }
                        break;
                }
                xTimerStart(pxTimer,0);
                return;
            }
        }
        break;
    case 3:
        if (Key_press != Key_Prev) {
            Key_State = 0;
            INFO("KEY_LONG_UP\n");
            return;
        }
        break;
    }
}

static void dc1_key0_init(void){

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << 16);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(16, 1);
}

static void dc1_led_init(void){

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL << 0) | (1ULL << 14));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(0, 1);
    gpio_set_level(14, 1);
}

void wifi_led_switch(uint8_t bit_value){
    gpio_set_level(0, !bit_value);
}

void logo_led_switch(uint8_t bit_value){
    gpio_set_level(14, !bit_value);
}

void dc1_event_regist_cb(dc1_event_cb_t user_dc1_event_cb){

    dc1_event_cb = user_dc1_event_cb;
}

esp_err_t dc1_init(void)
{
    uint8_t cmd_data;
    vTaskDelay(1000 / portTICK_RATE_MS);
    dc1_i2c_init();
    dc1_led_init();
    dc1_key0_init();
    cmd_data = 0xc0;  
    dc1_i2c_write(0x01, cmd_data);
    TimerHandle_t timer = xTimerCreate("dc1_key_handler", 
                                      (30 / portTICK_RATE_MS),  
                                      pdTRUE, 
                                      NULL, 
                                      dc1_key_handler);

    if(timer != NULL){
        if(xTimerStart(timer,0) != pdPASS ){
            INFO("timer start error\n");
            return ESP_FAIL;
        }
    }else{
        INFO("timer is NULL\n");
        return ESP_FAIL;
    }
    return ESP_OK;
}