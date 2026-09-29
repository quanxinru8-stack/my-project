#include "ledstrip.h"
#include "ws2812.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "timers.h"

/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * WS2812灯带驱动代码
 * Changed.Add some led strip color.
 * All rights reserved
********************************************************************************/

#define NBR_LEDS  8	//ws2812 RGB灯个数

enum ledStripColor
{
	RED = 0,
	GREEN,
	BLUE,
	WHITE,
	BLACK,//黑色（不亮）
	CUSTOM,//洋红
	GOLD,//金色	#FFD700
	YELLOW,//黄色 #FFFF00
	COLOR_NUM,
};

const uint8_t colorTable[COLOR_NUM][3] =
{
	{0xff, 0x00, 0x00},//红
	{0x00, 0xFF, 0x00},//绿
	{0x00, 0x00, 0xFF},//蓝
	{0xff, 0xff, 0xff},//白
	{0x00, 0x00, 0x00},//黑（不亮）
	{0xff, 0x00, 0xff},//洋红
	{0xff, 0xd7, 0x00},//金色
	{0xff, 0xff, 0x00}//黄色
};

typedef enum
{
    WARNING_LED_OFF = 0,
    WARNING_LED_ON,
} ledStripState_e;

static uint8_t colorBuffer[NBR_LEDS][3];


//填充颜色
static void ledStripFillBufferWitchColor(enum ledStripColor color)
{
	for (int i = 0; i < NBR_LEDS; i++)
	{
		colorBuffer[i][0] = colorTable[color][0];
		colorBuffer[i][1] = colorTable[color][1];
		colorBuffer[i][2] = colorTable[color][2];
	}
}


//灯带初始化
void ledStripInit(void)
{
	ws2812Init();
	ledStripOFF();
}

void ledStripON(void)
{
	ledStripFillBufferWitchColor(RED);
	ws2812Send(colorBuffer, NBR_LEDS);
}

void ledStripOFF(void)
{
	ledStripFillBufferWitchColor(BLACK);
	ws2812Send(colorBuffer, NBR_LEDS);
}

void ledStripGREEN(void)
{
	ledStripFillBufferWitchColor(GREEN);
	ws2812Send(colorBuffer, NBR_LEDS);
}
void ledStripRED(void)
{
	ledStripFillBufferWitchColor(RED);
	ws2812Send(colorBuffer, NBR_LEDS);
}
void ledStripBLUE(void)
{
  ledStripFillBufferWitchColor(BLUE);
	ws2812Send(colorBuffer, NBR_LEDS);
}

void ledStripCUSTOM(void)
{
  ledStripFillBufferWitchColor(CUSTOM);
	ws2812Send(colorBuffer, NBR_LEDS);
}
void ledStripGOLD(void)
{
  ledStripFillBufferWitchColor(GOLD);
	ws2812Send(colorBuffer, NBR_LEDS);
}
void ledStripYELLOW(void)
{
  ledStripFillBufferWitchColor(YELLOW);
	ws2812Send(colorBuffer, NBR_LEDS);
}
void ledStripTest(void)
{
	ledStripRED();
	vTaskDelay(500);
	ledStripGREEN();
	vTaskDelay(500);
	ledStripBLUE();
	vTaskDelay(500);
}
void ledstripTask(void *param)
{
  while(1)
	{
		ledStripTest();
	}
}
