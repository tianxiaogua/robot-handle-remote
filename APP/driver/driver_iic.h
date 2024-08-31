/*
 * driver_iic.h
 *
 *  Created on: 2024年7月13日
 *      Author: tianxiaohua
 */

#ifndef DRIVER_IIC_H_
#define DRIVER_IIC_H_

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver_gpio.h"

esp_err_t dirver_i2c1_init(void);

esp_err_t driver_i2c1_master_transmit_buf(uint8_t iic_address, uint8_t reg_address, uint8_t *data_buf, uint32_t data_len);

esp_err_t driver_i2c1_master_receive_buf(uint8_t iic_address, uint8_t reg_address, uint8_t *data_buf, uint32_t data_len);

#endif /* APP_DRIVER_DRIVER_IIC_H_ */
