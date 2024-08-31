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

typedef struct nvs_gamepad_ 
{
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

NVS_GAMEPAD_TYPE nvs_gamepad = {
	.namespace           = "key_list",
	.lift_stick_x        = "lift_stick_x",
	.lift_stick_x_value  = 0,
	.lift_stick_y        = "lift_stick_y",
	.lift_stick_y_value  = 0,
	.right_stick_x       = "right_stick_x",
	.right_stick_x_value = 0,
	.right_stick_y       = "right_stick_y",
	.right_stick_y_value = 0
};

typedef struct keyboard_cfg_
{
	uint16_t shutdown_time;// 关机
	uint16_t time_flag;
} KEYBOARD_CFG;

KEYBOARD_CFG keyboard_cfg; // 控制按键的配置参数
KEY_DETECTION g_key_detection;
KEY_DETECTION g_key_detection_his;

static void key_scan(uint8 *key_value)
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
	key_value[8] = KEY_LIN_3_READ;
	delay_ms(2);

	KEY_ROW_1_LOW;
	KEY_ROW_2_LOW;
	KEY_ROW_3_LOW;
	KEY_ROW_4_HIG;
	key_value[9] =  KEY_LIN_1_READ;
	key_value[10] = KEY_LIN_2_READ;
	key_value[11] = KEY_LIN_3_READ;

}
// 校准摇杆
static int32 app_calibration_rocker(void)
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
	GUA_LOGI("read ca_calue_LX value:%d", g_key_detection.ca_calue_LX);
	GUA_LOGI("read ca_calue_LY value:%d", g_key_detection.ca_calue_LY);
	GUA_LOGI("read ca_calue_RX value:%d", g_key_detection.ca_calue_RX);
	GUA_LOGI("read ca_calue_RY value:%d", g_key_detection.ca_calue_RY);

	nvs_gamepad.lift_stick_x_value = 0;
	if (nvs_gamepad.lift_stick_x_value == 0) { // || nvs_gamepad.lift_stick_y_value == 0 || nvs_gamepad.right_stick_x_value == 0 || nvs_gamepad.right_stick_y_value == 0) { // 没有校准数据，重新校准摇杆
		for (uint32_t i = 0; i < 10; i++) {
			temp_value[0] += get_adc_data(ADC_CHANNEL_3); // LX
			temp_value[1] += get_adc_data(ADC_CHANNEL_4); // LY
			temp_value[2] += get_adc_data(ADC_CHANNEL_2); // RX
			temp_value[3] += get_adc_data(ADC_CHANNEL_1); // RY
			// GUA_LOGI("get calibration rocker:%d %d %d %d", temp_value[0], temp_value[1], temp_value[2], temp_value[3]);
			delay_ms(1);
		}
		GUA_LOGW("get average calibration rocker:%d %d %d %d", temp_value[0]/10, temp_value[1]/10, temp_value[2]/10, temp_value[3]/10);
		g_key_detection.ca_calue_LX = temp_value[0]/10;
		g_key_detection.ca_calue_LY = temp_value[1]/10;
		g_key_detection.ca_calue_RX = temp_value[2]/10;
		g_key_detection.ca_calue_RY = temp_value[3]/10;
		driver_nvs_set_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.lift_stick_x, temp_value[0]/10);
		driver_nvs_set_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.lift_stick_y, temp_value[1]/10);
		driver_nvs_set_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.right_stick_x, temp_value[2]/10);
		driver_nvs_set_u32(nvs_gamepad.nvs_handler, nvs_gamepad.namespace, nvs_gamepad.right_stick_y, temp_value[3]/10);
	} else {
		GUA_LOGI("calibration rocker is ok :%d %d %d %d\n", temp_value[0], temp_value[1], temp_value[2], temp_value[3]);
	}

	return REV_OK;
}

int32 app_init_keyboard(void)
{
	int32 ret = 0;

	memset(&g_key_detection, 0, sizeof(g_key_detection));
	memset(&g_key_detection_his, 0, sizeof(g_key_detection_his));
	memset(&keyboard_cfg, 0, sizeof(keyboard_cfg));

	ret = app_calibration_rocker();
	return ret;
}

/**
 * 按键检测任务
 * */

int32 app_detection_key(void)
{
	uint8 key_value[10] = {0};

	key_scan(key_value);
	g_key_detection.KEY_UP_   = key_value[0];
	g_key_detection.KEY_DOWN_ = key_value[3];
	g_key_detection.KEY_LEFT  = key_value[6];
	g_key_detection.KEY_RIGHT = key_value[9];
	g_key_detection.KEY_A  = key_value[1];
	g_key_detection.KEY_B  = key_value[4];
	g_key_detection.KEY_X  = key_value[7];
	g_key_detection.KEY_Y  = key_value[10];
	g_key_detection.KEY_LB = key_value[5];
	g_key_detection.KEY_RB = key_value[2];
	g_key_detection.LT = key_value[11];
	g_key_detection.RT = key_value[8];

	updata_pca9557_io_input();
	g_key_detection.KEY_SELECT = !read_pca9557_io_input(1);
	g_key_detection.KEY_START  = !read_pca9557_io_input(7);

	g_key_detection.KEY_R_KEY  = !read_pca9557_io_input(2);
	g_key_detection.KEY_L_KEY  = !read_pca9557_io_input(3);

	g_key_detection.KEY_CHA    = !GPIO_GetPin(KEY_CHA_NUM);

	g_key_detection.LX = get_adc_data(ADC_CHANNEL_3);
	g_key_detection.LY = get_adc_data(ADC_CHANNEL_4);
	g_key_detection.RX = get_adc_data(ADC_CHANNEL_2);
	g_key_detection.RY = get_adc_data(ADC_CHANNEL_1);
//	get_adc_data(ADC_CHANNEL_5);

	if (g_key_detection.KEY_LEFT == 1 && g_key_detection.KEY_B == 1) { // 按下的时候是0
		keyboard_cfg.shutdown_time++; // 每次循环约10毫秒 长按3秒关机，相当于循环约290次
		if (keyboard_cfg.shutdown_time >200) {
			GUA_LOGW("shutdown!\n");
			set_pca9557_io_low(6);  // 用于关机的IO
			delay_ms(10000);
		}
	} else {
		keyboard_cfg.shutdown_time = 0;
	}

#define DEBUG
#ifdef DEBUG
	if (g_key_detection.KEY_UP_ != g_key_detection_his.KEY_UP_) {
		GUA_LOGI("KEY_UP changed = %d", g_key_detection.KEY_UP_);
	} 
	if (g_key_detection.KEY_DOWN_ != g_key_detection_his.KEY_DOWN_) {
		GUA_LOGI("KEY_DOWN changed = %d", g_key_detection.KEY_DOWN_);
	} 
	if (g_key_detection.KEY_LEFT != g_key_detection_his.KEY_LEFT) {
		GUA_LOGI("KEY_LEFT changed = %d", g_key_detection.KEY_LEFT);
	} 
	if (g_key_detection.KEY_RIGHT != g_key_detection_his.KEY_RIGHT) {
		GUA_LOGI("KEY_RIGHT changed = %d", g_key_detection.KEY_RIGHT);
	} 
	if (g_key_detection.KEY_A != g_key_detection_his.KEY_A) {
		GUA_LOGI("KEY_A changed = %d", g_key_detection.KEY_A);
	} 
	if (g_key_detection.KEY_B != g_key_detection_his.KEY_B) {
		GUA_LOGI("KEY_B changed = %d", g_key_detection.KEY_B);
	} 
	if (g_key_detection.KEY_X != g_key_detection_his.KEY_X) {
		GUA_LOGI("KEY_X changed = %d", g_key_detection.KEY_X);
	} 
	if (g_key_detection.KEY_Y != g_key_detection_his.KEY_Y) {
		GUA_LOGI("KEY_Y changed = %d", g_key_detection.KEY_Y);
	} 
	if (g_key_detection.KEY_LB != g_key_detection_his.KEY_LB) {
		GUA_LOGI("KEY_LB changed = %d", g_key_detection.KEY_LB);
	} 
	if (g_key_detection.KEY_RB != g_key_detection_his.KEY_RB) {
		GUA_LOGI("KEY_RB changed = %d", g_key_detection.KEY_RB);
	} 
	if (g_key_detection.LT != g_key_detection_his.LT) {
		GUA_LOGI("LT changed = %d", g_key_detection.LT);
	} 
	if (g_key_detection.RT != g_key_detection_his.RT) {
		GUA_LOGI("RT changed = %d", g_key_detection.RT);
	} 
	if (g_key_detection.KEY_SELECT != g_key_detection_his.KEY_SELECT) {
		GUA_LOGI("KEY_SELECT changed = %d", g_key_detection.KEY_SELECT);
	} 
	if (g_key_detection.KEY_START != g_key_detection_his.KEY_START) {
		GUA_LOGI("KEY_START changed = %d", g_key_detection.KEY_START);
	} 
	if (g_key_detection.KEY_R_KEY != g_key_detection_his.KEY_R_KEY) {
		GUA_LOGI("KEY_R_KEY changed = %d", g_key_detection.KEY_R_KEY);
	} 
	if (g_key_detection.KEY_CHA != g_key_detection_his.KEY_CHA) {
		GUA_LOGI("KEY_CHA changed = %d", g_key_detection.KEY_CHA);
	} 
	// keyboard_cfg.time_flag++;
	// if (keyboard_cfg.time_flag >5) {
	// 	keyboard_cfg.time_flag = 0;
	// 	printf("CHA:%d UP:%d DOWN:%d LEFT:%d RIGHT:%d A:%d B:%d X:%d Y:%d LB:%d RB:%d LT:%d RT:%d START:%d SELECT:%d R_KEY:%d L_KEY:%d  %d %d %d %d\r\n",
	// 		g_key_detection.KEY_CHA,
	// 		g_key_detection.KEY_UP_,
	// 		g_key_detection.KEY_DOWN_,
	// 		g_key_detection.KEY_LEFT,
	// 		g_key_detection.KEY_RIGHT,
	// 		g_key_detection.KEY_A,
	// 		g_key_detection.KEY_B,
	// 		g_key_detection.KEY_X,
	// 		g_key_detection.KEY_Y,
	// 		g_key_detection.KEY_LB,
	// 		g_key_detection.KEY_RB,
	// 		g_key_detection.LT,
	// 		g_key_detection.RT,
	// 		g_key_detection.KEY_START,
	// 		g_key_detection.KEY_SELECT,
	// 		g_key_detection.KEY_R_KEY,
	// 		g_key_detection.KEY_L_KEY,
	// 		g_key_detection.LX,
	// 		g_key_detection.LY,
	// 		g_key_detection.RX,
	// 		g_key_detection.RY
	// 		);
	// }
#endif
	return ESP_OK;
}


int32 app_get_key_value(KEY_DETECTION *key_value)
{
	if (key_value == NULL) {
		GUA_LOGE("function input value is null!");
		return REV_ERR;
	}

	memcpy(key_value, &g_key_detection, sizeof(KEY_DETECTION));
	return REV_OK;
}

