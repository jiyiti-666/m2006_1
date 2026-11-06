#include "pid_location.h"
#include "main.h"
#include "pid_speed.h"
#include <stdint.h>

PID_TypeDef g_location_pid;

void pid_location_init(float Kp, float Ki, float Kd, float SetPoint)
{
    g_location_pid.SetPoint    = SetPoint;
    g_location_pid.ActualValue = 0.0f;
    g_location_pid.SumError    = 0.0f;
    g_location_pid.Proportion  = Kp;
    g_location_pid.Integral    = Ki;
    g_location_pid.Derivative  = Kd;
    g_location_pid.Error       = 0.0f;
    g_location_pid.LastError   = 0.0f;
}

void pid_location_clear(void)
{
    g_location_pid.SumError    = 0.0f;
    g_location_pid.SetPoint    = 0.0f;
    g_location_pid.Error       = 0.0f;
    g_location_pid.LastError   = 0.0f;
    g_location_pid.ActualValue = 0.0f;
}

void pid_location_clear_accum(void)
{
    g_location_pid.SumError  = 0.0f;
    g_location_pid.Error     = 0.0f;
    g_location_pid.LastError = 0.0f;
}

int16_t pid_location_update(PID_TypeDef *pid, float currentValue)
{
    pid->Error = pid->SetPoint - currentValue;

    pid->SumError += pid->Error;
    if (pid->SumError >  PID_LOC_I_MAX) pid->SumError =  PID_LOC_I_MAX;
    if (pid->SumError < -PID_LOC_I_MAX) pid->SumError = -PID_LOC_I_MAX;

    pid->ActualValue = (pid->Proportion * pid->Error)
                     + (pid->Integral   * pid->SumError)
                     + (pid->Derivative * (pid->Error - pid->LastError));

    pid->LastError = pid->Error;

    if (pid->ActualValue >  PID_LOC_OUT_MAX) pid->ActualValue =  PID_LOC_OUT_MAX;
    if (pid->ActualValue < -PID_LOC_OUT_MAX) pid->ActualValue = -PID_LOC_OUT_MAX;

    return (int16_t)pid->ActualValue;
}
