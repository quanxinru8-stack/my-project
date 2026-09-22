#include <stdlib.h>
#include <stdio.h>
#include "system.h"
#include "stabilizer.h"
#include "sensors.h"
#include "commander.h"
#include "state_control.h"
#include "power_control.h"
#include "pos_estimator.h"
#include "gyro.h"
#include "led.h"
#include "runtime_config.h"
#include "stepmotor.h"
#include "uart5.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

/*加入SD卡代码 by jiang*/
#include "MMC_SD.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 四轴自稳控制代码	
 * Changed by jiang, complement. 
 * All rights reserved
********************************************************************************/

static bool isInit;
setpoint_t		setpoint;	/*设置目标状态*/
sensorData_t 	sensorData;	/*传感器数据*/
state_t 		state;		/*四轴姿态*/
control_t 		control;	/*四轴控制参数*/

// void stabilizerInit(void)
// {
// 	if(isInit) return;
	
// 	stateControlInit();		/*姿态PID初始化*/
// 	powerControlInit();		/*电机初始化*/
// 	imuInit();				/*姿态解算初始化*/
// 	isInit = true;
// }
    
// changed
void stabilizerInitChanged(void)
{
	if (isInit) return;
	
	stateControlInit();		        /*姿态PID初始化*/
	powerControlInitChanged();		/*电机初始化*/
	imuInit();				        /*姿态解算初始化*/
	UART5_Config();
    
//	StepMotorInit();
	SD_Initialize();
	isInit = true;
}

// void stabilizerTask(void* param)
// {
// 	u32 tick = 0;
// 	u32 lastWakeTime = getSysTickCnt();////****系统始终为1000hz******/////
	
// 	//等待陀螺仪校准完成
// 	while(!gyroIsCalibrationComplete())
// 	{
// 		vTaskDelayUntil(&lastWakeTime, M2T(1));/////***执行频率是1000hz******////
// 	}
	
// 	while(1) 
// 	{
// 		//1KHz运行频率
// 		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));	
		
// 		//获取传感器数据
// 		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
// 		{
// 			sensorsAcquire(&sensorData, tick);				
// 		}
		
// 		//四元数和欧拉角计算
// 		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
// 		{
// 			imuUpdateAttitude(&sensorData, &state, ATTITUDE_ESTIMAT_DT);	///****包含两个主要函数，计算四元数和转换矩阵、欧拉角计算*****////
// 		}
		
// 		//位置预估计算
// 		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
// 		{  	
// 			updatePositionEstimator(&sensorData, &state, POSITION_ESTIMAT_DT);////***由气压计和加速度计更新位置估计******////
// 		}
		
// 		//目标姿态和飞行模式设定	
// 		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
// 		{
// 			commanderGetSetpoint(&state, &setpoint);////****给出飞行模式以及遥控器控制量*****////
// 			                                        ///****根据选定的飞行模式，得到setpoint中的mode、attitude、attitudeRate的相关量
// 			updateArmingStatus();
// 		}
		
// 		//PID控制器计算控制输出
// 		stateControl(&sensorData, &state, &setpoint, &control, tick); //***不考虑其他模式，只针对自稳模式，得到期望的角速度或角速率
// 		                                                             ///再进行内外环的控制，高度方向不考虑定高模式，遥控器输出的直接是电机控制值
		
// 		//控制电机输出（500Hz）
// 		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
// 		{
// 			powerControl(&control);///按照布局间的对应，给出每个电机的油门值
// 		}
		
// 		tick++;
// 	}
// }

//changed
void stabilizerTaskChanged(void* param)
{
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt();////****系统始终为1000hz******/////
	
	//等待陀螺仪校准完成
	while (!gyroIsCalibrationComplete())
	{
		vTaskDelayUntil(&lastWakeTime, M2T(1));/////***执行频率是1000hz******////
	}
	
	//启动文件系统与SD卡
	ReadyToWrite();

	while (1)
	{
		//1KHz运行频率
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));

		//获取传感器数据
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			sensorsAcquire(&sensorData, tick);//****获取传感器数据，更新到全局变量sensorData中，供后续控制使用******/////
		}

		//四元数和欧拉角计算
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdateAttitude(&sensorData, &state, ATTITUDE_ESTIMAT_DT);	///****包含两个主要函数，计算四元数和转换矩阵、欧拉角计算*****////
		}

		//位置预估计算
		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
		{
			updatePositionEstimator(&sensorData, &state, POSITION_ESTIMAT_DT);////***由气压计和加速度计更新位置估计******////
		}

		//目标姿态和飞行模式设定	
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			commanderGetSetpoint(&state, &setpoint);////****给出飞行模式以及遥控器控制量*****///
			updateArmingStatus();
		}

		//PID控制器计算控制输出
		stateControl(&sensorData, &state, &setpoint, &control, tick); //***不考虑其他模式，只针对自稳模式，得到期望的角速度或角速率
																	 ///再进行内外环的控制，高度方向不考虑定高模式，遥控器输出的直接是电机控制值
		//控制电机输出（500Hz）无刷
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			powerControlChanged(&control);///按照布局间的对应，给出每个电机的油门值
		}	
			
        //step（200Hz）
        if (RATE_DO_EXECUTE(MOTOR_LOOP, tick))
        {
            OutstepControl(&control, &state, &setpoint, &sensorData); // <--- 增加了 &sensorData，地址传进去了
        }
        
		//写SD卡（25Hz）
		//注意:不要频率太高，不然会写不进去！！！经过测试 25HZ是最稳定的！！！
		// if (RATE_DO_EXECUTE(RATE_25_HZ, tick))
		// {
		// 	WriteDataToSDOneLine(&state, &setpoint, &control, tick);
		// }	
		
		tick++;
	}
}
