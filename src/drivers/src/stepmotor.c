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
static const PID_Param pid_roll = {0.4f, 0.0f, 0.0f};
static const PID_Param pid_pitch = {0.4f, 0.0f, 0.0f};

MotorControl stepmotor1 = {0, 0, 0, TIM_Channel_1, 0, FORWARD};
MotorControl stepmotor2 = {0, 0, 0, TIM_Channel_3, 0, FORWARD};

PID_Control ctrl_pid ={0};

Quadrant DetermineQuadrant(float roll, float pitch) {
     if(roll>DEADZONE && pitch>DEADZONE)
     {
        return QUADRANT_I;
     }
     
     else if(roll>DEADZONE && pitch<DEADZONE && pitch>-DEADZONE)
     {
        return ROLL_POSITIVE;
     }
     
     else if(roll>DEADZONE && pitch<-DEADZONE)
     {
        return QUADRANT_IV;
     }
     
     else if(roll<-DEADZONE && pitch<-DEADZONE)
     {
        return QUADRANT_III;
     }
     
     else if(roll<-DEADZONE && pitch>DEADZONE)
     {
        return QUADRANT_II;
     }
     
     else if(roll<-DEADZONE && pitch<DEADZONE && pitch>-DEADZONE)
     {
        return ROLL_NEGATIVE;
     }
        
     else if(roll>-DEADZONE && roll<DEADZONE && pitch<-DEADZONE)
     {
        return PITCH_NEGATIVE;
     }    
     
     else if(roll>-DEADZONE && roll<DEADZONE && pitch>DEADZONE)
     {
        return PITCH_POSITIVE;
     }    
     
     else 
     {
       return NO_INPUT;
     }    
}

//DIR初始化 PD2 
void StepMotor_DIR_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(STEP_DIR_PERIPH , ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=STEP_DIR_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;
	
	GPIO_Init(STEP_DIR_PORT,&GPIO_InitStructure);
	GPIO_ResetBits(STEP_DIR_PORT,STEP_DIR_PIN);
	
}

//DIR2初始化 PC12 
void StepMotor_DIR2_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(STEP_DIR2_PERIPH, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin= STEP_DIR2_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed; 
	
	GPIO_Init(STEP_DIR2_PORT,&GPIO_InitStructure);
	GPIO_ResetBits(STEP_DIR2_PORT, STEP_DIR2_PIN);
	
}

//PUL初始化 PA2 TIM5 CH3 PA2 CH1
void StepMotor_PUL_Init(void)
{
	GPIO_InitTypeDef 				GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef 	    TIM_TimeBaseStructure;
	TIM_OCInitTypeDef 				TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	 	 	//TIM5时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 			//使能PORTA时钟	 
	
	//引脚复用配置
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5);   		//PA2复用为TIMER5_CH2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);   
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_0;          //GPIOA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        			//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;				//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      			//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       				//上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              				//初始化PA2
	 
	//时基单元配置
	TIM_TimeBaseStructure.TIM_Prescaler=83;  						//定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		//向上计数模式
	TIM_TimeBaseStructure.TIM_Period=ARR-1;   						//自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);					//初始化TIMER5
	
	//TIMER5_CH1 PWM模式配置	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 		//输出极性:TIM输出比较极性低
    
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  						//根据T指定的参数初始化外设TIM5 CH3
    TIM_OC3Init(TIM5, &TIM_OCInitStructure);  
    
    
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  				//使能TIM5在CCR1上的预装载寄存器
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  	
    
    TIM_ARRPreloadConfig(TIM5,ENABLE);								//ARPE使能 
     
	TIM_SetCompare1(TIM5, 0);
	TIM_SetCompare3(TIM5, 0);	//修改比较值，修改占空比
	TIM_Cmd(TIM5, ENABLE);  										//使能TIM5
	
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
}

//TIMER5更新中断处理函数
//TODO: 可以用来计数脉冲个数，并且判断是否已经达到指定脉冲
void TIM5_IRQHandler(void)   
{ 
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) 	
	{ 
        __disable_irq();
		// motor1 count
        if (stepmotor1.is_running) 
        {
            stepmotor1.current_pulses++;
            if(stepmotor1.direction == FORWARD){
                  stepmotor1.absolute_position ++ ;
                }
                else{
                  stepmotor1.absolute_position -- ;
                }
            if (stepmotor1.current_pulses >= stepmotor1.target_pulses) 
            {
                TIM_SetCompare1(TIM5, 0); 
                stepmotor1.is_running = 0;           
            }
        }

        // motor2 count
        if (stepmotor2.is_running) 
        {
            stepmotor2.current_pulses++;
            if(stepmotor2.direction == FORWARD){
                  stepmotor2.absolute_position ++ ;
                }
                else{
                  stepmotor2.absolute_position -- ;
                }
            if (stepmotor2.current_pulses >= stepmotor2.target_pulses) 
            {
                TIM_SetCompare3(TIM5, 0); //
                stepmotor2.is_running = 0;
            }
        }
        __enable_irq();
        
        TIM_ClearITPendingBit(TIM5 ,TIM_IT_Update );					//清除中断更新标志位  
	} 
}

//步进电机初始化
void StepMotorInit(void)
{
	StepMotor_DIR2_Init();
	StepMotor_DIR_Init();
	StepMotor_PUL_Init();	
}


void StartMotor(MotorControl *motor, uint32_t target_pulses, int direction, uint32_t channel) 
{
    int32_t available_steps = 0;

    if(motor == &stepmotor1) {
        if(direction == FORWARD) {
            available_steps = MOTOR1_MAX_POSITION - motor->absolute_position;
        } else if(direction == BACKWARD) {
            available_steps = motor->absolute_position - MOTOR1_MIN_POSITION;
        }
    } else if(motor == &stepmotor2) {
        if(direction == FORWARD) {
            available_steps = MOTOR2_MAX_POSITION - motor->absolute_position;
        } else if(direction == BACKWARD) {
            available_steps = motor->absolute_position - MOTOR2_MIN_POSITION;
        }
    }

    // stop
    if(available_steps <= 0 || direction == STOP) {
        motor->target_pulses = 0;
        motor->is_running = 0;
        return;
    }

    // limit
    if (target_pulses > (uint32_t)available_steps) {
        motor->target_pulses = available_steps;
    } else {
        motor->target_pulses = target_pulses;
    }
    // set 
//    TIM_Cmd(TIM5, ENABLE);

    motor->current_pulses = 0;
    motor->is_running = 1;
    motor->direction = direction;
    
    // direction
    if (motor == &stepmotor1)
    {
      if(direction==FORWARD) 
      {
        GPIO_SetBits(STEP_DIR_PORT,STEP_DIR_PIN);
      }

      else if(direction==BACKWARD)
      {
         GPIO_ResetBits(STEP_DIR_PORT,STEP_DIR_PIN);
      }
    } 
    else if (motor == &stepmotor2)
    {
      if(direction==FORWARD) 
      {
        GPIO_ResetBits(STEP_DIR2_PORT,STEP_DIR2_PIN);
      }

      else if(direction==BACKWARD)
      {
        GPIO_SetBits(STEP_DIR2_PORT,STEP_DIR2_PIN);
      }
       
    }
    
    // channel
    switch (channel)
    {
        case TIM_Channel_1:
            TIM_SetCompare1(TIM5, CCR);
            break;
        case TIM_Channel_3:
            TIM_SetCompare3(TIM5, CCR);
            break;
    }
}


//俯仰角与步进电机方向的转换接口: roll
//TODO
int GetDirection(float control_delta)
{
	if (control_delta > RIGHTBOUND)
	{
		return FORWARD;
	}
	else if(control_delta < LEFTBOUND)
	{
		return BACKWARD;
	}
	else
	{
		return STOP;
	}
}

void PID_Incremental(PID_Control* ctrl)
{
    /* roll calculate */
    // update error 
    float error_roll = ctrl->delta_control;  // current error
    ctrl->roll.integral += error_roll;       // integral 
    ctrl->roll.integral = constrain(ctrl->roll.integral, -100.0f, 100.0f);  // limit

    // calculate difference
    float derivative_roll = error_roll - ctrl->roll.error[0];

    // calculate output
    ctrl->output_roll = pid_roll.Kp * error_roll + 
                        pid_roll.Ki * ctrl->roll.integral +
                        pid_roll.Kd * derivative_roll;
    // keep error
    ctrl->roll.error[0] = error_roll;
    
    /* pitch */
    // update error
    float error_pitch = ctrl->delta_controlp;
    ctrl->pitch.integral += error_pitch;
    ctrl->pitch.integral = constrain(ctrl->pitch.integral, -50.0f, 50.0f);

    // difference
    float derivative_pitch = error_pitch - ctrl->pitch.error[0];

    // output
    ctrl->output_pitch = pid_pitch.Kp * error_pitch +
                         pid_pitch.Ki * ctrl->pitch.integral +
                         pid_pitch.Kd * derivative_pitch;
                         
    ctrl->pitch.error[0] = error_pitch;

    /* limit */
    ctrl->output_roll = constrain(ctrl->output_roll, -30.0f, 30.0f);// 
    ctrl->output_pitch = constrain(ctrl->output_pitch, -30.0f, 30.0f);
    
}
//相对移动方向控制
void StrategicControl(PID_Control* ctrl) 
{
    // calculate control 
//    float u = abs(ctrl->output_roll)/30;
//    float v = abs(ctrl->output_pitch)/30;
    
    // determine phase
    Quadrant q = DetermineQuadrant(ctrl->output_roll, ctrl->output_pitch);
    
    double ans1 = atan2(ctrl->output_pitch , ctrl->output_roll) * RAD2DEG ;
    float v1 = 90 - ans1 ;  //positive
    float v2 = ans1 - 90 ;  //positive
    float v3 = -90 - ans1 ;  //positive
    float v4 = 90 + ans1 ;  //positive
    
    float u1 = 20 * v1 / 9 ;  //positive
    float u2 = - 20 * v2 / 9 ; //negative
    float u3 = 20 * v3 / 9 ;  //positive
    float u4 = - 20 * v4 / 9 ; //negative
    
    int tar1 = (int)(u1 + 0.5f); //positive
    int tar2 = (int)(u2 - 0.5f); //negative
    int tar3 = (int)(u3 + 0.5f); //positive
    int tar4 = (int)(u4 - 0.5f); //negative
    
    // target   
    //stop
    uint32_t motor1_pulses = 0;
    int motor1_dir = STOP;
    
    // stop
    uint32_t motor2_pulses = 0;
    
    int motor2_dir = STOP;
    switch(q) {
        
        case NO_INPUT:
          motor1_pulses = abs(stepmotor1.absolute_position);
          motor2_pulses = 0;
          motor1_dir = (stepmotor1.absolute_position > 0) ? BACKWARD : FORWARD;
          motor2_dir = STOP;
          break;
        
        case PITCH_POSITIVE:
            motor2_pulses = abs(MOTOR2_POS_0 - stepmotor2.absolute_position);
            motor2_dir = (MOTOR2_POS_0 > stepmotor2.absolute_position) ? FORWARD : BACKWARD;
            motor1_pulses = abs(MOTOR1_STEP-stepmotor1.absolute_position);
            motor1_dir = FORWARD ;
            break;
            
        case PITCH_NEGATIVE:
            motor2_pulses = abs(MOTOR2_POS_0 - stepmotor2.absolute_position);
            motor2_dir = (MOTOR2_POS_0 > stepmotor2.absolute_position) ? FORWARD : BACKWARD;
            motor1_pulses = abs(MOTOR1_STEP-stepmotor1.absolute_position);
            motor1_dir = BACKWARD ;
            break;
            
        case ROLL_POSITIVE:
            motor2_pulses = abs(MOTOR2_POS_90 - stepmotor2.absolute_position) ;
            motor2_dir = (MOTOR2_POS_90 > stepmotor2.absolute_position) ? FORWARD : BACKWARD ;
            motor1_pulses = abs(MOTOR1_STEP - stepmotor1.absolute_position) ;
            motor1_dir = FORWARD ;
            break;
            
        case ROLL_NEGATIVE:
            motor2_pulses = abs(MOTOR2_POS_90 - stepmotor2.absolute_position) ;
            motor2_dir = (MOTOR2_POS_90 > stepmotor2.absolute_position) ? FORWARD : BACKWARD ;
            motor1_pulses = abs(MOTOR1_STEP - stepmotor1.absolute_position) ;
            motor1_dir = BACKWARD ;
            break;
        
        case QUADRANT_I:
            motor2_pulses = abs( tar1 - stepmotor2.absolute_position) ;
            motor2_dir = (tar1 > stepmotor2.absolute_position) ? FORWARD : BACKWARD ;
            motor1_pulses = abs(MOTOR1_STEP - stepmotor1.absolute_position) ;
            motor1_dir = FORWARD ;
            break;
        
        case QUADRANT_II:
            motor2_pulses = abs(tar2 - stepmotor2.absolute_position) ;
            motor2_dir = (tar2 > stepmotor2.absolute_position) ? FORWARD : BACKWARD ;
            motor1_pulses = abs(MOTOR1_STEP - stepmotor1.absolute_position) ;
            motor1_dir = FORWARD ;
            break;
        
        case QUADRANT_III:
            motor2_pulses = abs(tar3 - stepmotor2.absolute_position) ;
            motor2_dir = (tar3 > stepmotor2.absolute_position) ? FORWARD : BACKWARD ;
            motor1_pulses = abs(MOTOR1_STEP - stepmotor1.absolute_position) ;
            motor1_dir = BACKWARD ;
            break;
        
        case QUADRANT_IV:
            motor2_pulses = abs(tar4 - stepmotor2.absolute_position) ;
            motor2_dir = (tar4 > stepmotor2.absolute_position) ? FORWARD : BACKWARD ;
            motor1_pulses = abs(MOTOR1_STEP - stepmotor1.absolute_position) ;
            motor1_dir = BACKWARD ;
            break;
            
    }

    // motor2 control (round) !!deadzone
    if(motor2_pulses > DEADZONE_STEP) {
        StartMotor(&stepmotor2, motor2_pulses, motor2_dir, TIM_Channel_3);
    } else {
        stepmotor2.target_pulses = 0;
        stepmotor2.is_running = 0;
        TIM_SetCompare3(TIM5, 0); 
    }

    // motor1 contorl (line)
    if(motor1_pulses > DEADZONE_STEP) {
        StartMotor(&stepmotor1, motor1_pulses, motor1_dir, TIM_Channel_1);
    } else {
        stepmotor1.target_pulses = 0;
        stepmotor1.is_running = 0;
        TIM_SetCompare1(TIM5, 0); 
    }
}


//步进电机控制接口 控制量: roll pitch
void StepMotorControl(float control_variable, float controlp_variable)
{
    ctrl_pid.delta_control = control_variable;
    ctrl_pid.delta_controlp = controlp_variable;
    
    PID_Incremental(&ctrl_pid);
    StrategicControl(&ctrl_pid);  
}

void OutstepControl(control_t *control)
{
   StepMotorControl(30,30);
//    PrintDebugInfo();
}

void Send_Control_Data(control_t *control, state_t *state)
{
    float tx_buf[5];
    tx_buf[0] = control->delta_control;
    tx_buf[1] = control->delta_controlp;
    tx_buf[2] = state->attitude.roll;
    tx_buf[3] = state->attitude.pitch;
    tx_buf[4] = state->attitude.yaw;

    UART5_Send_Float_Packet(tx_buf, 5);
    
    printf("\n\rdelta_control:%.3f delta_controlp:%.3f roll:%.3f pitch:%.3f raw:%.3f\n\r",tx_buf[0],tx_buf[1],tx_buf[2],tx_buf[3],tx_buf[4]);
}


void allControl(void)
{
//    for(int i=0; i<5; i++){                                                           
//    // ???+200
//    StepMotorControl(5.0f, 0.0f); // QUADRANT_V
//    while(stepmotor2.is_running);
//    
//    // ??0
//    StepMotorControl(0.0f, 0.0f); 
//    while(stepmotor2.is_running);
//    }
}

void PrintDebugInfo() {
    printf("Motor1: Pos=%ld Dir=%s Running=%d\n", 
          stepmotor1.absolute_position,
          (stepmotor1.direction==FORWARD)?"FWD":"REV", 
          stepmotor1.is_running);
          
    printf("Motor2: Pos=%ld Dir=%s Running=%d\n", 
          stepmotor2.absolute_position,
          (stepmotor2.direction==FORWARD)?"FWD":"REV",
          stepmotor2.is_running);
}


