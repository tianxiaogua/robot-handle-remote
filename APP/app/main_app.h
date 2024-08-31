/*
 * main_app.h
 *
 *  Created on: 2024年7月14日
 *      Author: tianxiaohua
 */

#ifndef APP_MAIN_APP_H_
#define APP_MAIN_APP_H_

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
//#include "esp_flash.h"

#include "driver_gpio.h"
#include "driver_tool.h"
#include "driver_spi.h"

#include "lcd.h"
#include "lcd_init.h"
//#include "pic.h"
#include "driver_adc.h"

#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pca9557.h"
#include "driver/i2c.h"

#include "driver_iic.h"

esp_err_t init_app(void);
void start_app(void);


#endif /* APP_MAIN_APP_H_ */
