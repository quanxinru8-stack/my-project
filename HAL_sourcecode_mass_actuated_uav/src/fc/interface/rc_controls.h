#ifndef __RC_CONTROL_H
#define __RC_CONTROL_H
#include "sys.h"
#include "rx.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 遥控控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

typedef enum 
{
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

typedef enum 
{
    NOT_CENTERED = 0,
    CENTERED
} rollPitchStatus_e;

typedef enum 
{
    ROL_LO = (1 << (2 * ROLL)),//0001
    ROL_CE = (3 << (2 * ROLL)),//0011
    ROL_HI = (2 << (2 * ROLL)),//0010

    PIT_LO = (1 << (2 * PITCH)),//0100
    PIT_CE = (3 << (2 * PITCH)),//1100
    PIT_HI = (2 << (2 * PITCH)),//1000

    THR_LO = (1 << (2 * THROTTLE)),//010000
    THR_CE = (3 << (2 * THROTTLE)),//110000
    THR_HI = (2 << (2 * THROTTLE)),//100000
	
	YAW_LO = (1 << (2 * YAW)),//0100 0000
    YAW_CE = (3 << (2 * YAW)),//1100 0000
    YAW_HI = (2 << (2 * YAW)),//1000 0000
} stickPositions_e;

typedef enum 
{
	AUX_LO = 0,
    AUX_CE = 1,
    AUX_HI = 2,
} auxPositions_e;

stickPositions_e getRcStickPositions(void);
bool checkStickPosition(stickPositions_e stickPos);

bool areSticksInApModePosition(uint16_t ap_mode);
throttleStatus_e calculateThrottleStatus(void);
rollPitchStatus_e calculateRollPitchCenterStatus(void);
void processRcStickPositions(void);

int32_t getRcStickDeflection(int32_t axis, uint16_t midrc);
void processRcAUXPositions(void);

#endif
