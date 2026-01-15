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
 * Author : Ni Hanyu
 * Date : 2025/1/6
 * All rights reserved.
********************************************************************************/
typedef struct
{
    float kp;
    float ki;
    float kd;

    float integrator;     // 积分状态
    float prev_error;     // 上一次误差

    float out_min;        // 输出限幅
    float out_max;

    float i_min;          // 积分限幅
    float i_max;

    float d_lpf_alpha;    // 微分低通系数(0~1), 0表示不用滤波
    float d_state;        // 微分滤波状态
} PID_t;
typedef struct {
    float rho;
    float psi;
} StepReflectionOut;

void Send_Control_Data(StepReflectionOut *out, state_t *state, setpoint_t *setpoint);

static inline float clampf(float x, float lo, float hi);

static inline void pidReset(PID_t* pid);

static inline float pidUpdateDt(PID_t* pid, float error, float dt);

void OutstepControl(control_t *control, state_t *state, setpoint_t *setpoint);

static inline float wrapDeg180(float x);

StepReflectionOut StepReflection(float ux, float uy);

#endif /* __STEPMOTOR_H */
