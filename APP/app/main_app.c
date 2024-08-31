/*
 * main_app.c
 *
 *  Created on: 2024年7月14日
 *      Author: tianxiaohua
 */

#include "main_app.h"
#include "app_keyboard.h"
#include "driver_nvs.h"
#include "driver_pwm.h"
#include "app_tinyusb.h"
#include "driver_timer.h"
#include "driver_pwm_beep.h"

const char debug_app[] = "DEBUG_APP";

/* 开机 */
void muc_starting_up(void)
{
	uint8_t KEY_START;
	uint8_t key_time = 0;
	init_pca9557();
	KEY_START  = read_pca9557_io_input(7);
	ESP_LOGI(debug_app, "KEY_START:%d",KEY_START);
	while(KEY_START == 0) { // 按键按下的时候是0
		delay_ms(100); // 延时100ms消抖
		KEY_START  = read_pca9557_io_input(7);
		key_time++;
		if(key_time >2){ // 超过2秒开机
			set_pca9557_io_high(6);
			ESP_LOGI(debug_app, "start muc");
			return;
		}
	}
	ESP_LOGE(debug_app, "start muc fause");
}





void lcd_task(void * pvParameters)
{
	while(1)
	{
		LCD_Fill(0,0,LCD_W,LCD_H,GREEN);
		delay_ms(1000);
		LCD_Fill(0,0,LCD_W,LCD_H,RED);
		delay_ms(1000);
	}
}


void keyboard_task(void * pvParameters)
{
	driver_kalman_Init();
	app_calibration_rocker();
	while(1) {
		app_detection_key();
		delay_ms(10);
	}
}

void usb_task()
{
	app_tinyusb();
}


esp_err_t init_app(void)
{
	muc_starting_up(); // 开机过程
	init_beep();
	play_beep_ding();
	driver_init_ADC();
	driver_init_gpio();
	spi2_init();
	LCD_Init();//LCD初始化
	LCD_Fill(0,0,LCD_W,LCD_H,0xFFFF);

	return ESP_OK;
}


//创建句柄
TaskHandle_t Handle_lCD_task = NULL;
TaskHandle_t Handle_keyboard_task = NULL;
TaskHandle_t Handle_usb_task = NULL;
void start_app(void)
{

	// 创建LED控制任务
	xTaskCreatePinnedToCore(lcd_task,            //任务函数
							"lcd_task",          //任务名称
							4096,                //堆栈大小
							NULL,                //传递参数
							1,                   //任务优先级
							&Handle_lCD_task,    //任务句柄
							tskNO_AFFINITY);     //无关联，不绑定在任何一个核上
	// 创建LED控制任务
	xTaskCreatePinnedToCore(keyboard_task,            //任务函数
							"keyboard_task",          //任务名称
							4096,                //堆栈大小
							NULL,                //传递参数
							2,                   //任务优先级
							&Handle_keyboard_task,    //任务句柄
							tskNO_AFFINITY);     //无关联，不绑定在任何一个核上
	// 创建USB控制任务
	xTaskCreatePinnedToCore(usb_task,            //任务函数
							"usb_task",          //任务名称
							4096,                //堆栈大小
							NULL,                //传递参数
							3,                   //任务优先级
							&Handle_usb_task,    //任务句柄
							tskNO_AFFINITY);     //无关联，不绑定在任何一个核上
}


