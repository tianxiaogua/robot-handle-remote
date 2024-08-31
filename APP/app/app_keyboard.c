/*
 * app_keyboard.c
 *
 *  Created on: 2024年7月14日
 *      Author: tianxiaohua
 */
#include "main_app.h"
#include "app_keyboard.h"
#include "driver_nvs.h"
#include "driver/gpio.h"

#define KEYTAG    "app_keyboard.c"
#define KEY_PRESS 0x01 // 按键被按下
#define KEY_LIFT  0x02 // 按键被抬起

/* 行输出 */
#define KEY_ROW_1 GPIO_NUM_36
#define KEY_ROW_2 GPIO_NUM_35
#define KEY_ROW_3 GPIO_NUM_34
#define KEY_ROW_4 GPIO_NUM_33
/* 列输入 */
#define KEY_LIN_1 GPIO_NUM_1
#define KEY_LIN_2 GPIO_NUM_14
#define KEY_LIN_3 GPIO_NUM_21
/* 行输出 */
#define KEY_ROW_1_HIG GPIO_SetPin(KEY_ROW_1, GPIO_HIGH)
#define KEY_ROW_1_LOW GPIO_SetPin(KEY_ROW_1, GPIO_LOW)
#define KEY_ROW_2_HIG GPIO_SetPin(KEY_ROW_2, GPIO_HIGH)
#define KEY_ROW_2_LOW GPIO_SetPin(KEY_ROW_2, GPIO_LOW)
#define KEY_ROW_3_HIG GPIO_SetPin(KEY_ROW_3, GPIO_HIGH)
#define KEY_ROW_3_LOW GPIO_SetPin(KEY_ROW_3, GPIO_LOW)
#define KEY_ROW_4_HIG GPIO_SetPin(KEY_ROW_4, GPIO_HIGH)
#define KEY_ROW_4_LOW GPIO_SetPin(KEY_ROW_4, GPIO_LOW)
/* 列输入 */
#define KEY_LIN_1_READ GPIO_GetPin(KEY_LIN_1)
#define KEY_LIN_2_READ GPIO_GetPin(KEY_LIN_2)
#define KEY_LIN_3_READ GPIO_GetPin(KEY_LIN_3)

// 模拟引脚输入

KEY_DETECTION g_key_detection;

void key_scan(uint8 *key_value)
{
	// 四行三列扫描键盘
	KEY_ROW_1_HIG;
	KEY_ROW_2_LOW;
	KEY_ROW_3_LOW;
	KEY_ROW_4_LOW;
	key_value[0] = KEY_LIN_1_READ;
	key_value[1] = KEY_LIN_2_READ;
	key_value[2] = KEY_LIN_3_READ;
	delay_ms(2);

	KEY_ROW_1_LOW;
	KEY_ROW_2_HIG;
	KEY_ROW_3_LOW;
	KEY_ROW_4_LOW;
	key_value[3] = KEY_LIN_1_READ;
	key_value[4] = KEY_LIN_2_READ;
	key_value[5] = KEY_LIN_3_READ;
	delay_ms(2);

	KEY_ROW_1_LOW;
	KEY_ROW_2_LOW;
	KEY_ROW_3_HIG;
	KEY_ROW_4_LOW;
	key_value[6] = KEY_LIN_1_READ;
	key_value[7] = KEY_LIN_2_READ;
//	key_value[8] = KEY_LIN_3_READ;
	delay_ms(2);

	KEY_ROW_1_LOW;
	KEY_ROW_2_LOW;
	KEY_ROW_3_LOW;
	KEY_ROW_4_HIG;
	key_value[8] = KEY_LIN_1_READ;
	key_value[9] = KEY_LIN_2_READ;
//	key_value[11] = KEY_LIN_3_READ;

}
// 校准摇杆
int32 app_calibration_rocker(void)
{
	int32 temp_value[4] = {0};

	driver_nvs_get_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.lift_stick_x, &nvs_gamepad.lift_stick_x_value);
	driver_nvs_get_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.lift_stick_y, &nvs_gamepad.lift_stick_y_value);
	driver_nvs_get_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.right_stick_x, &nvs_gamepad.right_stick_x_value);
	driver_nvs_get_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.right_stick_y, &nvs_gamepad.right_stick_y_value);
	g_key_detection.ca_calue_LX = nvs_gamepad.lift_stick_x_value;
	g_key_detection.ca_calue_LY = nvs_gamepad.lift_stick_y_value;
	g_key_detection.ca_calue_RX = nvs_gamepad.right_stick_x_value;
	g_key_detection.ca_calue_RY = nvs_gamepad.right_stick_y_value;

	nvs_gamepad.lift_stick_x_value = 0;
	if (nvs_gamepad.lift_stick_x_value == 0) { // || nvs_gamepad.lift_stick_y_value == 0 || nvs_gamepad.right_stick_x_value == 0 || nvs_gamepad.right_stick_y_value == 0) { // 没有校准数据，重新校准摇杆
		for (uint32_t i = 0; i < 10; i++) {
			temp_value[0] += get_adc_data(ADC_CHANNEL_3); // LX
			temp_value[1] += get_adc_data(ADC_CHANNEL_4); // LY
			temp_value[2] += get_adc_data(ADC_CHANNEL_2); // RX
			temp_value[3] += get_adc_data(ADC_CHANNEL_1); // RY
			ESP_LOGI(KEYTAG, "get calibration rocker:%d %d %d %d\n", temp_value[0], temp_value[1], temp_value[2], temp_value[3]);
			delay_ms(1);
		}
		ESP_LOGI(KEYTAG, "get average calibration rocker:%d %d %d %d\n", temp_value[0]/10, temp_value[1]/10, temp_value[2]/10, temp_value[3]/10);
		g_key_detection.ca_calue_LX = temp_value[0]/10;
		g_key_detection.ca_calue_LY = temp_value[1]/10;
		g_key_detection.ca_calue_RX = temp_value[2]/10;
		g_key_detection.ca_calue_RY = temp_value[3]/10;
		driver_nvs_set_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.lift_stick_x, temp_value[0]/10);
		driver_nvs_set_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.lift_stick_y, temp_value[1]/10);
		driver_nvs_set_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.right_stick_x, temp_value[2]/10);
		driver_nvs_set_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.right_stick_y, temp_value[3]/10);
	} else {
		ESP_LOGI(KEYTAG, "calibration rocker is ok :%d %d %d %d\n", temp_value[0], temp_value[1], temp_value[2], temp_value[3]);
	}

	return REV_SUCCESS;
}


/**
 * 按键检测任务
 * */
uint16_t shutdown_time = 0;// 关机
uint16_t time_flag;
void app_detection_key(void)
{
	uint8 key_value[10] = {0};

	key_scan(key_value);
	g_key_detection.KEY_UP_   = key_value[0];
	g_key_detection.KEY_DOWN_ = key_value[3];
	g_key_detection.KEY_LEFT  = key_value[6];
	g_key_detection.KEY_RIGHT = key_value[8];
	g_key_detection.KEY_A  = key_value[1];
	g_key_detection.KEY_B  = key_value[4];
	g_key_detection.KEY_X  = key_value[7];
	g_key_detection.KEY_Y  = key_value[9];
	g_key_detection.KEY_LB = key_value[5];
	g_key_detection.KEY_RB = key_value[2];

	updata_pca9557_io_input();
	g_key_detection.KEY_START  = !read_pca9557_io_input(7);
	g_key_detection.KEY_SELECT  = 0; // read_pca9557_io_input(1);
	g_key_detection.KEY_R_KEY  = !read_pca9557_io_input(2);
	g_key_detection.KEY_L_KEY  = !read_pca9557_io_input(3);

	g_key_detection.LX = get_adc_data(ADC_CHANNEL_3);
	g_key_detection.LY = get_adc_data(ADC_CHANNEL_4);
	g_key_detection.RX = get_adc_data(ADC_CHANNEL_2);
	g_key_detection.RY = get_adc_data(ADC_CHANNEL_1);
//	get_adc_data(ADC_CHANNEL_5);

	if (g_key_detection.KEY_LEFT == 1 && g_key_detection.KEY_B == 1) { // 按下的时候是0
		shutdown_time++; // 每次循环约10毫秒 长按3秒关机，相当于循环约290次
		if (shutdown_time >200) {
			ESP_LOGW(KEYTAG, "shutdown!\n");
			set_pca9557_io_low(6);  // 用于关机的IO
			delay_ms(10000);
		}
	} else {
		shutdown_time = 0;
	}

//	time_flag++;
//	if (time_flag >100) {
//		time_flag = 0;
//		printf("UP:%d DOWN:%d LEFT:%d RIGHT:%d A:%d B:%d X:%d Y:%d LB:%d RB:%d START:%d SELECT:%d R_KEY:%d L_KEY:%d  %d %d %d %d\r\n",
//			g_key_detection.KEY_UP_,
//			g_key_detection.KEY_DOWN_,
//			g_key_detection.KEY_LEFT,
//			g_key_detection.KEY_RIGHT,
//			g_key_detection.KEY_A,
//			g_key_detection.KEY_B,
//			g_key_detection.KEY_X,
//			g_key_detection.KEY_Y,
//			g_key_detection.KEY_LB,
//			g_key_detection.KEY_RB,
//			g_key_detection.KEY_START,
//			g_key_detection.KEY_SELECT,
//			g_key_detection.KEY_R_KEY,
//			g_key_detection.KEY_L_KEY,
//			g_key_detection.LX,
//			g_key_detection.LY,
//			g_key_detection.RX,
//			g_key_detection.RY
//			);
//	}
}

