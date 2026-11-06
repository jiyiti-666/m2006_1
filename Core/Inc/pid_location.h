#ifndef __PID_LOCATION_H__
#define __PID_LOCATION_H__

#include "main.h"
#include <stdint.h>
#include "pid_speed.h"

/* 位置环输出是「速度给定 rpm」，限幅要小于速度环能达到的最大值 */
#define PID_LOC_OUT_MAX     400.0f
/* 积分限幅 */
#define PID_LOC_I_MAX       50000.0f

/* 默认参数（输入单位是编码器计数，8192 = 1 圈） */
#define LOC_KP_DEFAULT      0.05f
#define LOC_KI_DEFAULT      0.0f
#define LOC_KD_DEFAULT      0.0f

extern PID_TypeDef g_location_pid;

void    pid_location_init(float Kp, float Ki, float Kd, float SetPoint);
int16_t pid_location_update(PID_TypeDef *pid, float currentValue);
void    pid_location_clear(void);
void    pid_location_clear_accum(void);

#endif
