#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H

#include "sys.h"
#include "config.h"
#include "sensors_types.h"
#include "motors.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "maths.h"
#include "math.h"
#include "power_control.h"
#include "maths.h"
/********************************************************************************
 * ATKflight飞控固件
 * Brief : 步进电机电机驱动代码
 * Author : Jiang Chunli
 * Date : 2022/3/21
 * All rights reserved
********************************************************************************/

//步进电机的DIR PUL ENA 引脚配置
//1.DIR2 PD2
#define STEP_DIR2_PERIPH RCC_AHB1Periph_GPIOD
#define STEP_DIR2_PORT GPIOD
#define STEP_DIR2_PIN GPIO_Pin_2

//2.PUL PA2(TIM5 CH3)
#define STEP_PUL_PERIPH RCC_AHB1Periph_GPIOA
#define STEP_PUL_PORT GPIOA
#define STEP_PUL_PIN GPIO_Pin_2

//3.DIR PC12
//TIPS:高电平失能 低电平使能 可以不接
#define STEP_DIR_PERIPH RCC_AHB1Periph_GPIOC
#define STEP_DIR_PORT GPIOC
#define STEP_DIR_PIN GPIO_Pin_12

//4.PUL2 PB1(TIM3 CH4)
#define STEP_PUL2_PERIPH RCC_AHB1Periph_GPIOB
#define STEP_PUL2_PORT GPIOB
#define STEP_PUL2_PIN GPIO_Pin_1

//5.设置电机转速 CCR/ARR越大越快
#define CCR 500
#define ARR 1000

//6.定义正反转方向
#define FORWARD 1
#define STOP 0
#define BACKWARD -1

//7.定义稳定区间,在此区间内步进电机停转
#define LEFTBOUND -0.8f
#define RIGHTBOUND 0.8f

// limitation of pid output
#define PULSE_PER_DEG_ROLL    40   // 
#define PULSE_PER_DEG_PITCH   25   // 
#define MAX_ROLL_ANGLE        15.0 //
#define MAX_PITCH_ANGLE       10.0 //

#define MOTOR2_POS_0      0     // 0?
#define MOTOR2_POS_45     100   // 45?
#define MOTOR2_POS_90     200   // 90?
#define MOTOR2_POS_NEG45  -100  // -45?

#define MOTOR1_MIN_POSITION   (-650)
#define MOTOR1_MAX_POSITION   (650)
#define MOTOR2_MIN_POSITION   (-200)
#define MOTOR2_MAX_POSITION   (200)

#define MOTOR1_STEP     650    // ??1??????
#define MOTOR2_TARGET   200    // roll??????
#define DEADZONE        0.5f   // ???????
#define DEADZONE_STEP        5

#define MOTOR1_SCALE_FACTOR 15.32f

typedef enum {
    QUADRANT_I,  // pitch>0
    QUADRANT_II,  // pitch<0
    QUADRANT_III,   // roll>0
    QUADRANT_IV,    // roll<0
    NO_INPUT,
    ROLL_POSITIVE,
    ROLL_NEGATIVE,
    PITCH_POSITIVE,
    PITCH_NEGATIVE
    
} Quadrant;

typedef struct {
    volatile uint32_t target_pulses; // 
    volatile uint32_t current_pulses; // 
    volatile uint8_t is_running;      // 
    uint32_t channel;
    int32_t absolute_position;     // 
    int direction;
} MotorControl;

// PID parameter
typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_Param;

// control struct
typedef struct {
    // input
    float delta_control;   // roll error
    float delta_controlp;  // pitch error
    
    // output   
    float output_roll;     // increased roll
    float output_pitch;    // increased pitch

    // PID history
    struct {
        float error[3];    // [k-2, k-1, k]
        float integral;     // 
    } roll, pitch;
} PID_Control;


void Send_Control_Data(control_t *control, state_t *state);

//是否初始化成功
bool IsStepMotorOn(void);
bool IsStepMotorOn2(void);
//启动步进电机开关
void StepMotorON(void);
//启动步进电机开关2
void StepMotorON2(void);
//关闭步进电机开关
void StepMotorOFF(void);
//关闭步进电机开关2
void StepMotorOFF2(void);

//DIR初始化 PD2 
void StepMotor_DIR_Init(void);
//DIR2初始化 
void StepMotor_DIR2_Init(void);

//PUL初始化 PA2 TIM5 CH3
void StepMotor_PUL_Init(void);
//PUL2初始化 PB1 TIM3 CH4
void StepMotor_PUL2_Init(void);

//初始化
void StepMotorInit(void);

//控制移动
void Relative_Move(int direction, int directionp);
//俯仰角与步进电机方向的转换接口
int GetDirection(float control_delta);
//俯仰角与步进电机脉冲数的转换接口
int GetPulse(float control_variable);

//更改电机速度
void SetVelocity(int compare_value);
//步进电机控制接口
void StepMotorControl(float control_variable, float controlp_variable);
//测试函数
void TestStepMotorTask(void *param);

void PID_Incremental(PID_Control* ctrl_pid);

void StartMotor(MotorControl *motor, uint32_t target_pulses, int direction, uint32_t channel);

void allControl(void);

void OutstepControl(control_t *control);

void PrintDebugInfo(void);
#endif /* __STEPMOTOR_H */
