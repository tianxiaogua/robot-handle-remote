/*
 * led.h
 *
 *  Created on: 2023年2月18日
 *      Author: 29602
 */

#ifndef DRIVER_VNS_H
#define DRIVER_VNS_H

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"


typedef struct nvs_gamepad_ {
	nvs_handle nvs_handler;
	char namespace[32];
	char lift_stick_x[32]; // 左摇杆数值x
	uint32_t lift_stick_x_value;
	char lift_stick_y[32]; // 左摇杆数值y
	uint32_t lift_stick_y_value;
	char right_stick_x[32]; // 右摇杆数值x
	uint32_t right_stick_x_value;
	char right_stick_y[32]; // 右摇杆数值y
	uint32_t right_stick_y_value;
} NVS_GAMEPAD_TYPE;

extern NVS_GAMEPAD_TYPE nvs_gamepad;

void driver_nvs_set_u32(nvs_handle nvs_handler, char* namespace_name, char* key, uint32_t value);
void driver_nvs_get_u32(nvs_handle nvs_handler, char* namespace_name, char* key, uint32_t* out_value);

#endif /* MAIN_LED_H_ */
