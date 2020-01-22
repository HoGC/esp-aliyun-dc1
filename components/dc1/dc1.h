/*
 * dc1 driver
 * Author: HoGC 
 */
#ifndef _DC1_H_
#define	_DC1_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_err.h"

#define I2C_DC1_SCL_IO                  12               /*!< gpio number for I2C master clock */
#define I2C_DC1_SDA_IO                  3                /*!< gpio number for I2C master data  */
#define I2C_DC1_NUM                     I2C_NUM_0        /*!< I2C port number for master dev */

#define DC1_SENSOR_ADDR                 0x20             /*!< slave address for MPU6050 sensor */
#define WRITE_BIT                       I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                        I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                    0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                   0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                         0x0              /*!< I2C ack value */
#define NACK_VAL                        0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                   0x2              /*!< I2C last_nack value */

typedef void (* dc1_event_cb_t)(int event);

typedef enum {
    KEY0_SHORTPRESS_CB = 0,  
    KEY1_SHORTPRESS_CB,
    KEY2_SHORTPRESS_CB,
    KEY3_SHORTPRESS_CB,
    KEY0_LONGPRESS_CB,
    KEY1_LONGPRESS_CB,
    KEY2_LONGPRESS_CB,
    KEY3_LONGPRESS_CB, 
    SWITCH0_OPEN,   
    SWITCH0_CLOSE,
    SWITCH1_OPEN,
    SWITCH1_CLOSE,
    SWITCH2_OPEN,
    SWITCH2_CLOSE,
    SWITCH3_OPEN,
    SWITCH3_CLOSE,
} dc1_event_t;

esp_err_t dc1_init(void);
void dc1_event_regist_cb(dc1_event_cb_t user_dc1_event_cb);

void dc1_set_switch(uint8_t num, uint8_t bit_value);

void wifi_led_switch(uint8_t bit_value);
void logo_led_switch(uint8_t bit_value);

#endif