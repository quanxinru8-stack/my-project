#include "stepmotor.h"
#include "ledstrip.h"
#include "uart5.h"
/********************************************************************************
 * Project : ATKflight飞控固件
 * Brief : 步进电机电机驱动代码
 * Author : Jiang Chunli 
 * Date : 2022/3/21
 * All rights reserved.
********************************************************************************/
PID_t pid_angle_roll  = {
    .kp = 1.2f,
    .ki = 0.15f,
    .kd = 0.0f,

    .integrator = 0,
    .prev_error = 0,

    .out_min = -1.0f,
    .out_max =  1.0f,

    .i_min = -0.5f,
    .i_max =  0.5f,

    .d_lpf_alpha = 0.0f,
    .d_state = 0
};

PID_t pid_angle_pitch = {
    .kp = 0.8f,
    .ki = 0.15f,
    .kd = 0.0f,

    .integrator = 0,
    .prev_error = 0,

    .out_min = -1.0f,
    .out_max =  1.0f,

    .i_min = -0.5f,
    .i_max =  0.5f,

    .d_lpf_alpha = 0.0f,
    .d_state = 0
};

#define STEPMOTOR_DT    (1.0f / 500.0f) // step control loop dt (s)
float dt = STEPMOTOR_DT;
static float psi = 0;
static float rho = 0;
static float psi_prev = 0;
static uint8_t psi_fold_mode = 0; // 0: normal branch, 1: folded branch

void Send_Control_Data(StepReflectionOut *out, state_t *state, setpoint_t *setpoint)
{
    float tx_buf[5];
    tx_buf[0] = out->rho;
    tx_buf[1] = out->psi;
    tx_buf[2] = state->attitude.roll;
    tx_buf[3] = state->attitude.pitch;
    tx_buf[4] = state->attitude.yaw;

    UART5_Send_Float_Packet(tx_buf, 5);
    
    //printf("\n\rdelta_control:%.3f delta_controlp:%.3f roll:%.3f pitch:%.3f raw:%.3f\n\r",tx_buf[0],tx_buf[1],tx_buf[2],tx_buf[3],tx_buf[4]);
    //printf("\n\rset_roll: %.3f \n\r set_pitch: %.3f", setpoint->attitude.roll, setpoint->attitude.pitch);
    printf("\n\rrho: %.3f psi: %.3f", out->rho, out->psi);
}

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline void pidReset(PID_t* pid)
{
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->d_state    = 0.0f;
}

// 位置/角度 PID：输入 error（deg），输出一般建议是期望角速度（deg/s）或控制量
static inline float pidUpdateDt(PID_t* pid, float error, float dt)
{
    if (dt <= 0.0f) dt = 1e-3f;

    // P
    const float p = pid->kp * error;

    // I (先积分再限幅)
    // ===== I: 积分分离 + 泄放 =====
    const float e_abs = fabsf(error);

    //积分分离阈值：误差大就不积分（0.2~0.4，归一化误差[-1,1]）
    const float I_ENABLE_E = 0.25f;

    //泄放系数：每秒衰减比例（0.5~2.0，越大回中越快）
    const float I_LEAK_PER_S = 1.0f;

    // 先泄放（避免积分长期残留）
    pid->integrator *= (1.0f - I_LEAK_PER_S * dt);
    pid->integrator = clampf(pid->integrator, pid->i_min, pid->i_max);

    // 再决定是否积分
    if (e_abs < I_ENABLE_E) {
        pid->integrator += pid->ki * error * dt;
        pid->integrator = clampf(pid->integrator, pid->i_min, pid->i_max);
    }

    // D（对误差求导）
    float d_raw = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // 可选：微分低通滤波
    if (pid->d_lpf_alpha > 0.0f && pid->d_lpf_alpha < 1.0f) {
        pid->d_state = pid->d_state + pid->d_lpf_alpha * (d_raw - pid->d_state);
        d_raw = pid->d_state;
    }

    const float d = pid->kd * d_raw;

    // 未限幅输出
    float out = p + pid->integrator + d;

    // 输出限幅
    float out_sat = clampf(out, pid->out_min, pid->out_max);

    // 抗饱和（最简单的：输出打满时，禁止继续往同方向积分）
    // 这能显著减少“积分冲过头”
    if (out != out_sat) {
        // out_sat - out 的符号表示被削掉的方向
        // 若 error 与 (out - out_sat) 同号，说明积分在推动更饱和，撤销本次积分
        const float cut = out - out_sat;
        if ((error > 0.0f && cut > 0.0f) || (error < 0.0f && cut < 0.0f)) {
            pid->integrator -= pid->ki * error * dt;
            pid->integrator = clampf(pid->integrator, pid->i_min, pid->i_max);
        }
    }

    return out_sat;
}

static inline float wrapDeg180(float x)
{
    while (x > 180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}

StepReflectionOut StepReflection(float ux, float uy)
{
    StepReflectionOut out;

    //  幅值（偏心量）
    rho = sqrtf(ux*ux + uy*uy);
    rho = clampf(rho, 0.0f, 1.0f); 

    // 方向角（圆环）
    psi = atan2f(uy, ux) * RAD2DEG; // CCW+
    // psi = -psi;                     // 电机2：CW+

    // 在 |psi|≈90° 附近加入滞回死区，避免 roll 主导时因 pitch 微小正负抖动而反复翻转 rho
    const float PSI_FOLD_ENTER = 92.0f; // 进入折返分支阈值（度）
    const float PSI_FOLD_EXIT  = 88.0f; // 退出折返分支阈值（度）
    const float psi_abs = fabsf(psi);

    if (psi_fold_mode) {
        if (psi_abs < PSI_FOLD_EXIT) {
            psi_fold_mode = 0;
        }
    } else {
        if (psi_abs > PSI_FOLD_ENTER) {
            psi_fold_mode = 1;
        }
    }

    if (psi_fold_mode) {
        // 折返表示：psi 移到 [-90,90]，并同步翻转 rho（两者必须成对）
        float psi_fold = (psi >= 0.0f) ? (psi - 180.0f) : (psi + 180.0f);
        psi = clampf(psi_fold, -90.0f, 90.0f);
        rho = -rho;
    } else {
        psi = clampf(psi, -90.0f, 90.0f);
    }

    psi_prev = psi;
    rho = clampf(rho, -1.0f, 1.0f); 

    if (fabsf(rho) < 0.05f) 
    {
       rho = 0.0f;
    }

    out.rho = rho;
    out.psi = psi;
    return out;
}



void OutstepControl(control_t *control, state_t *state, setpoint_t *setpoint)
{
    float e_roll  = control->delta_control;
    float e_pitch = control->delta_controlp;
    // float e_roll  = 28;
    // float e_pitch = -1.5;

    if (fabsf(e_roll) < 2.0f) 
    { // e_roll的死区保护，防止滑块在两个步进格之间来回横跳。
        e_roll = 0;             // 忽略不计
    }

    if (fabsf(e_pitch) < 2.0f) 
    { // e_roll的死区保护，防止滑块在两个步进格之间来回横跳。
        e_pitch = 0;             // 忽略不计
    }

    // 2) roll 做 wrap（pitch 不用）
    e_roll = wrapDeg180(e_roll);

    e_roll  = clampf(e_roll,  -30.0f, 30.0f);
    e_pitch = clampf(e_pitch, -30.0f, 30.0f);

    float e_roll_n  = e_roll / 30.0f;  // [-1,1]
    float e_pitch_n = e_pitch / 30.0f;

    e_roll_n  = clampf(e_roll_n,  -1.0f, 1.0f);
    e_pitch_n = clampf(e_pitch_n, -1.0f, 1.0f);

    float uy = -pidUpdateDt(&pid_angle_roll, e_roll_n, dt);
    float ux = -pidUpdateDt(&pid_angle_pitch, e_pitch_n, dt);

    // ===== stick center detect (use your own threshold) =====
    const float CENTER_E = 0.05f;   // 归一化误差阈值：0.02~0.08 自己调

    if (fabsf(e_roll_n) < CENTER_E && fabsf(e_pitch_n) < CENTER_E)
    {
        // 回中：强制回零
        StepReflectionOut out;
        out.psi = 0.0f;
        out.rho = 0.0f;   // 质量块也回到中心

        // 清积分，防止松手后被 I 项拖偏
        pidReset(&pid_angle_roll);
        pidReset(&pid_angle_pitch);

        Send_Control_Data(&out, state, setpoint);
        return;
    }

    StepReflectionOut out = StepReflection(ux, uy);
    Send_Control_Data(&out, state, setpoint);
    
}

