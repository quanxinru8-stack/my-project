#ifndef __LED_STRIP_H__
#define __LED_STRIP_H__
#include "sys.h"

/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * WS2812灯带驱动代码
 * Changed.Add some led strip color.
 * All rights reserved
********************************************************************************/

void ledStripInit(void);
void ledStripON(void);
void ledStripOFF(void);
void ledStripGREEN(void);
void ledStripRED(void);
void ledStripBLUE(void);
void ledStripCUSTOM(void);
void ledStripGOLD(void);
void ledStripYELLOW(void);
void ledStripTest(void);

void ledstripTask(void *param);

#endif

