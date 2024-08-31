/*
 * tool.h
 *
 *  Created on: 2023年8月1日
 *      Author: tianxiaohua
 */

#ifndef APP_USER_TOOL_H_
#define APP_USER_TOOL_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kalman.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REV_SUCCESS  0
#define REV_ERROR   -1
/*
 * type:           ************size**************
--------------------------------
*p:             所占字节数：4
--------------------------------
bool:           所占字节数：1   最大值：1               最小值：0
--------------------------------
char:           所占字节数：1   最大值：127             最小值：-128
signed char:    所占字节数：1   最大值：127             最小值：-128
unsigned char:  所占字节数：1   最大值：              最小值：
--------------------------------
short:          所占字节数：2   最大值：32767           最小值：-32768
unsigned short: 所占字节数：2   最大值：65535           最小值：0
wchar_t:        所占字节数：2   最大值：65535           最小值：0
--------------------------------

--------------------------------
long:           所占字节数：4   最大值：2147483647      最小值：-2147483648
unsigned long:  所占字节数：4   最大值：4294967295      最小值：0
--------------------------------
double:         所占字节数：8   最大值：1.79769e+308    最小值：2.22507e-308
long double:    所占字节数：8   最大值：1.79769e+308    最小值：2.22507e-308
--------------------------------
float:          所占字节数：4   最大值：3.40282e+38     最小值：1.17549e-38
size_t:         所占字节数：4   最大值：4294967295      最小值：0
string:         所占字节数：28
type:           ************size**************
*/

/*
 * int:            所占字节数：4   最大值：2147483647      最小值：-2147483648
 * unsigned:       所占字节数：4   最大值：4294967295      最小值：0
 * */
typedef int            int32;
typedef unsigned int   uint32;

/*
 * short:          所占字节数：2   最大值：32767           最小值：-32768
 * unsigned short: 所占字节数：2   最大值：65535           最小值：0
 * wchar_t:        所占字节数：2   最大值：65535           最小值：0
*/
typedef short          int16;
typedef unsigned short uint16;

/*
 * char:           所占字节数：1   最大值： 127               最小值：-128
 * signed char:    所占字节数：1   最大值： 127               最小值：-128
 * unsigned char:  所占字节数：1   最大值： 255             最小值：0
*/
typedef char  int8;
//typedef signed char  uint8;
typedef unsigned char  uint8;


enum kalman_filter_channel {
	filter_channer1 = 0,
	filter_channer2,
	filter_channer3,
	filter_channer4
};


void delay_ms(uint32 time);

void driver_kalman_Init(void);
float driver_kalman_filter(uint8 channel, float input);
#endif /* APP_USER_TOOL_H_ */
