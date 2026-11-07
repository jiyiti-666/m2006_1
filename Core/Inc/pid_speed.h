#ifndef __PID_SPEED_H__
#define __PID_SPEED_H__

#include "main.h"
#include <stdint.h>

/* 输出限幅：M2006 电流指令范围 ±10000 */
#define PID_SPEED_OUT_MAX   8000.0f
/* 积分限幅：防止积分饱和 */
#define PID_SPEED_I_MAX     40000.0f

/* 默认参数（M2006 速度环，10ms 控制周期） */
#define SPEED_KP_DEFAULT    20.0f
#define SPEED_KI_DEFAULT    0.10f
#define SPEED_KD_DEFAULT    0.0f

typedef struct
{
    volatile float SetPoint;      /* 目标速度 rpm */
    volatile float ActualValue;   /* 输出电流 */
    volatile float SumError;      /* 积分累加 */
    volatile float Proportion;    /* Kp */
    volatile float Integral;      /* Ki */
    volatile float Derivative;    /* Kd */
    volatile float Error;
    volatile float LastError;
} PID_TypeDef;

extern PID_TypeDef g_speed_pid;

void    pid_speed_init(float Kp, float Ki, float Kd, float SetPoint);
void    pid_speed_clear(void);
void    pid_speed_clear_accum(PID_TypeDef *pid);
int16_t pid_speed_update(PID_TypeDef *pid, float currentValue);

#endif
