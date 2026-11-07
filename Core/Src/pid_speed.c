#include "pid_speed.h"
#include "main.h"
#include <stdint.h>

PID_TypeDef g_speed_pid;

void pid_speed_init(float Kp, float Ki, float Kd, float Setpoint)
{
    g_speed_pid.SetPoint    = Setpoint;
    g_speed_pid.ActualValue = 0.0f;
    g_speed_pid.SumError    = 0.0f;
    g_speed_pid.Proportion  = Kp;
    g_speed_pid.Integral    = Ki;
    g_speed_pid.Derivative  = Kd;
    g_speed_pid.Error       = 0.0f;
    g_speed_pid.LastError   = 0.0f;
}

void pid_speed_clear(void)
{
    g_speed_pid.SumError    = 0.0f;
    g_speed_pid.SetPoint    = 0.0f;
    g_speed_pid.Error       = 0.0f;
    g_speed_pid.LastError   = 0.0f;
    g_speed_pid.ActualValue = 0.0f;
}

/* 只清积分和误差，保留 SetPoint（切模式时用） */
void pid_speed_clear_accum(PID_TypeDef *pid)
{
    pid->SumError  = 0.0f;
    pid->Error     = 0.0f;
    pid->LastError = 0.0f;
}

int16_t pid_speed_update(PID_TypeDef *pid, float currentValue)
{
    pid->Error = pid->SetPoint - currentValue;

    pid->SumError += pid->Error;
    if (pid->SumError >  PID_SPEED_I_MAX) pid->SumError =  PID_SPEED_I_MAX;
    if (pid->SumError < -PID_SPEED_I_MAX) pid->SumError = -PID_SPEED_I_MAX;

    pid->ActualValue = (pid->Proportion * pid->Error)
                     + (pid->Integral   * pid->SumError)
                     + (pid->Derivative * (pid->Error - pid->LastError));

    pid->LastError = pid->Error;

    if (pid->ActualValue >  PID_SPEED_OUT_MAX) pid->ActualValue =  PID_SPEED_OUT_MAX;
    if (pid->ActualValue < -PID_SPEED_OUT_MAX) pid->ActualValue = -PID_SPEED_OUT_MAX;

    return (int16_t)pid->ActualValue;
}
