/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "stdio.h"
#include "string.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "app_tinyusb.h"


#define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default

static const char *TAGDEBUG = "TINYUSB";

/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
//    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD) ),
//    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE) ),
    TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(HID_USAGE_DESKTOP_GAMEPAD) )
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 0, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;
  ESP_LOGW(TAGDEBUG, "tud_hid_get_report_cb!!!");
  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
	ESP_LOGW(TAGDEBUG, "tud_hid_set_report_cb!!!");
}

TU_ATTR_WEAK void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol)
{
	ESP_LOGW(TAGDEBUG, "tud_hid_set_protocol_cb!!!");
}

TU_ATTR_WEAK bool tud_hid_set_idle_cb(uint8_t instance, uint8_t idle_rate)
{
	ESP_LOGW(TAGDEBUG, "tud_hid_set_idle_cb!!!");
	return 0;
}

TU_ATTR_WEAK void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
	ESP_LOGW(TAGDEBUG, "tud_hid_report_complete_cb!!! %d %s %d", instance, report, len);
}

typedef struct tinyusb_buttons{
	int8_t x;
	int8_t y;
	int8_t z;
	int8_t rz;
	int8_t rx;
	int8_t ry;
	uint8_t hat;
	uint32_t buttons;
} TINYUSB_BUTTON;
TINYUSB_BUTTON g_tinyusb_button_data;

#define ECC_VALUE      15   // 偏心值 Eccentricity Value
#define SUBDIV_VALUE   128  // 划分数值
#define MAX_ECC_VALUE  4095 // ADC最大划分值
#define MEDIA_VALE     127  // 摇杆中间值 Median
#define MAX_VALE       256  // 摇杆中间值 Median

uint16 get_stick_value(uint16 stick, uint16 media_stick, uint16 filter_channer)
{
	uint16 temp_ecc_value = 0;
	uint16 temp_final_value = 0;

	stick = driver_kalman_filter(filter_channer, stick);
	if (stick > (media_stick - ECC_VALUE) && stick < (media_stick + ECC_VALUE)) {
		temp_final_value = 0;
	}
	else {
		if (stick < media_stick - ECC_VALUE) { // 0 ~ 127
			temp_ecc_value = media_stick  / 128;
			temp_final_value = 127 - stick / temp_ecc_value;
		} else { // 128 ~256 最大值128
			temp_ecc_value = (4095 - media_stick) / 128;
			temp_final_value = 128 + (4095 - stick) / temp_ecc_value;
		}
	}
	if (temp_final_value > 6000) {
		temp_final_value = 0;
	}
	return temp_final_value;
}

void dispose_gamepad_key()
{
	KEY_DETECTION gamepad_key;

	memcpy(&gamepad_key, &g_key_detection, sizeof(KEY_DETECTION));

	if (gamepad_key.KEY_A == BUTTON_DOWN) {
		g_tinyusb_button_data.buttons |= GAMEPAD_BUTTON_A;
	} else {
		g_tinyusb_button_data.buttons &= ~GAMEPAD_BUTTON_A;
	}
	if (gamepad_key.KEY_B == BUTTON_DOWN) {
		g_tinyusb_button_data.buttons |= GAMEPAD_BUTTON_B;
	} else {
		g_tinyusb_button_data.buttons &= ~GAMEPAD_BUTTON_B;
	}
	if (gamepad_key.KEY_X == BUTTON_DOWN) {
		g_tinyusb_button_data.buttons |= GAMEPAD_BUTTON_X;
	} else {
		g_tinyusb_button_data.buttons &= ~GAMEPAD_BUTTON_X;
	}
	if (gamepad_key.KEY_Y == BUTTON_DOWN) {
		g_tinyusb_button_data.buttons |= GAMEPAD_BUTTON_Y;
	} else {
		g_tinyusb_button_data.buttons &= ~GAMEPAD_BUTTON_Y;
	}
	if (gamepad_key.KEY_LB == BUTTON_DOWN) {
		g_tinyusb_button_data.buttons |= GAMEPAD_BUTTON_TL2;
	} else {
		g_tinyusb_button_data.buttons &= ~GAMEPAD_BUTTON_TL2;
	}
	if (gamepad_key.KEY_RB == BUTTON_DOWN) {
		g_tinyusb_button_data.buttons |= GAMEPAD_BUTTON_TR2;
	} else {
		g_tinyusb_button_data.buttons &= ~GAMEPAD_BUTTON_TR2;
	}
	if (gamepad_key.KEY_SELECT == BUTTON_DOWN) {
		g_tinyusb_button_data.buttons |= GAMEPAD_BUTTON_SELECT;
	} else {
		g_tinyusb_button_data.buttons &= ~GAMEPAD_BUTTON_SELECT;
	}
	if (gamepad_key.KEY_START == BUTTON_DOWN) {
		g_tinyusb_button_data.buttons |= GAMEPAD_BUTTON_START;
	} else {
		g_tinyusb_button_data.buttons &= ~GAMEPAD_BUTTON_START;
	}
	if (gamepad_key.KEY_R_KEY == BUTTON_DOWN) {
		g_tinyusb_button_data.buttons |= GAMEPAD_BUTTON_THUMBR;
	} else {
		g_tinyusb_button_data.buttons &= ~GAMEPAD_BUTTON_THUMBR;
	}
	if (gamepad_key.KEY_L_KEY == BUTTON_DOWN) {
		g_tinyusb_button_data.buttons |= GAMEPAD_BUTTON_THUMBL;
	} else {
		g_tinyusb_button_data.buttons &= ~GAMEPAD_BUTTON_THUMBL;
	}

	if (gamepad_key.KEY_UP_ == BUTTON_DOWN) {
		if(gamepad_key.KEY_RIGHT == BUTTON_DOWN) {
			g_tinyusb_button_data.hat = GAMEPAD_HAT_UP_RIGHT;
		} else if (gamepad_key.KEY_LEFT == BUTTON_DOWN){
			g_tinyusb_button_data.hat = GAMEPAD_HAT_UP_LEFT;
		} else {
			g_tinyusb_button_data.hat = GAMEPAD_HAT_UP;
		}
	} else if (gamepad_key.KEY_DOWN_ == BUTTON_DOWN) {
		if(gamepad_key.KEY_RIGHT == BUTTON_DOWN) {
			g_tinyusb_button_data.hat = GAMEPAD_HAT_DOWN_RIGHT;
		} else if (gamepad_key.KEY_LEFT == BUTTON_DOWN){
			g_tinyusb_button_data.hat = GAMEPAD_HAT_DOWN_LEFT;
		} else {
			g_tinyusb_button_data.hat = GAMEPAD_HAT_DOWN;
		}
	} else if (gamepad_key.KEY_RIGHT == BUTTON_DOWN) {
		if(gamepad_key.KEY_UP_ == BUTTON_DOWN) {
			g_tinyusb_button_data.hat = GAMEPAD_HAT_UP_RIGHT;
		} else if (gamepad_key.KEY_DOWN_ == BUTTON_DOWN){
			g_tinyusb_button_data.hat = GAMEPAD_HAT_DOWN_RIGHT;
		} else {
			g_tinyusb_button_data.hat = GAMEPAD_HAT_RIGHT;
		}
	} else if (gamepad_key.KEY_LEFT == BUTTON_DOWN) {
		if(gamepad_key.KEY_UP_ == BUTTON_DOWN) {
			g_tinyusb_button_data.hat = GAMEPAD_HAT_UP_LEFT;
		} else if (gamepad_key.KEY_DOWN_ == BUTTON_DOWN){
			g_tinyusb_button_data.hat = GAMEPAD_HAT_DOWN_LEFT;
		} else {
			g_tinyusb_button_data.hat = GAMEPAD_HAT_LEFT;
		}
	} else {
		g_tinyusb_button_data.hat = GAMEPAD_HAT_CENTERED;
	}

	g_tinyusb_button_data.x = get_stick_value(gamepad_key.RX, gamepad_key.ca_calue_RX, filter_channer1);
	g_tinyusb_button_data.y = get_stick_value(gamepad_key.RY, gamepad_key.ca_calue_RY, filter_channer2);
	g_tinyusb_button_data.rx = get_stick_value(gamepad_key.LX, gamepad_key.ca_calue_LX, filter_channer3);
	g_tinyusb_button_data.ry = get_stick_value(gamepad_key.LY, gamepad_key.ca_calue_LY, filter_channer4);
}



//char tiny_descriptor[15] = ;
void app_tinyusb(void)
{
//	char **cp = tiny_descriptor;
    // Initialize button that will trigger HID reports
    const gpio_config_t boot_button_config = {
        .pin_bit_mask = BIT64(APP_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = true,
        .pull_down_en = false,
    };
    ESP_ERROR_CHECK(gpio_config(&boot_button_config));

    ESP_LOGI(TAGDEBUG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
//        .string_descriptor = "xiaogua gamepad",
//		.string_descriptor_count = 15,
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAGDEBUG, "USB initialization DONE");

    g_tinyusb_button_data.buttons = 0;
    while (1) {
        if (tud_mounted()) {
        	dispose_gamepad_key();
        	tud_hid_gamepad_report(HID_USAGE_DESKTOP_GAMEPAD,
        			g_tinyusb_button_data.x,
					g_tinyusb_button_data.y,
					g_tinyusb_button_data.z,
					g_tinyusb_button_data.rz,
					g_tinyusb_button_data.rx,
					g_tinyusb_button_data.ry,
					g_tinyusb_button_data.hat,
					g_tinyusb_button_data.buttons);
        	vTaskDelay(pdMS_TO_TICKS(10));
        } else {
        	vTaskDelay(pdMS_TO_TICKS(100));
        	ESP_LOGI(TAGDEBUG, "wait usb\n");
        }
    }
}
