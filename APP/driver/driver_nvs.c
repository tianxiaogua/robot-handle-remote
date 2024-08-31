/*
 * nvs.c
 *
 *  Created on: 2023年8月22日
 *      Author: tianxiaohua
 */
#include "driver_nvs.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#define NAMESPACE "list"

#define DEBUGTAG "driver_nvs.c"

NVS_GAMEPAD_TYPE nvs_gamepad = {
	.lift_stick_x        = "lift_stick_x",
	.lift_stick_x_value  = 0,
	.lift_stick_y        = "lift_stick_y",
	.lift_stick_y_value  = 0,
	.right_stick_x       = "right_stick_x",
	.right_stick_x_value = 0,
	.right_stick_y       = "right_stick_y",
	.right_stick_y_value = 0
};


void driver_nvs_set_u32(nvs_handle nvs_handler, char* namespace_name, char* key, uint32_t value)
{
	esp_err_t ret;

	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_LOGE(DEBUGTAG, "init flash error!\n");
	}

	ret = nvs_open(NAMESPACE, NVS_READWRITE, &nvs_handler);
	if (ret != ESP_OK) {
		ESP_LOGE(DEBUGTAG, "open flash error!\n");
	}
	nvs_set_u32(nvs_handler,"lift_stick_x",value);
	nvs_commit(nvs_handler);
	nvs_close(nvs_handler);
}


void driver_nvs_get_u32(nvs_handle nvs_handler, char* namespace_name, char* key, uint32_t* out_value)
{
	esp_err_t ret;

	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_LOGE(DEBUGTAG, "init flash error!\n");
	}

	ret = nvs_open(NAMESPACE, NVS_READONLY, &nvs_handler);
	if (ret != ESP_OK) {
		ESP_LOGE(DEBUGTAG, "open flash error!\n");
	}

	nvs_get_u32(nvs_handler, "lift_stick_x", out_value);
	nvs_commit(nvs_handler); /* 提交 */
	nvs_close(nvs_handler);                     /* 关闭 */
}




