#ifndef APP_APP_KEYBOARD_APP_KEYBOARD_H_
#define APP_APP_KEYBOARD_APP_KEYBOARD_H_

#include "app_tinyusb.h"
#include "driver_tool.h"

#define BUTTON_DOWN 1 // 按钮按下 button down
#define BUTTON_UP 0 // 按钮按下 button down

typedef struct key_detection {
	uint8 KEY_CHA;

	uint8 KEY_LEFT;
	uint8 KEY_RIGHT;
	uint8 KEY_UP_;
	uint8 KEY_DOWN_;

	uint8 KEY_A;
	uint8 KEY_B;
	uint8 KEY_X;
	uint8 KEY_Y;

	uint8 KEY_LB;
	uint8 KEY_RB;

	uint8 KEY_SELECT;
	uint8 KEY_START;

	uint8 KEY_R_KEY;
	uint8 KEY_L_KEY;

	uint16 LX;
	uint16 LY;
	uint16 RX;
	uint16 RY;

	uint16 LT;
	uint16 RT;

	uint16 ca_calue_LX;
	uint16 ca_calue_LY;
	uint16 ca_calue_RX;
	uint16 ca_calue_RY;

} KEY_DETECTION;

extern KEY_DETECTION g_key_detection;
/**
 * 按键检测任务
 * */
int32 app_init_keyboard(void);
int32 app_detection_key(void);
#endif /* APP_APP_KEYBOARD_APP_KEYBOARD_H_ */
