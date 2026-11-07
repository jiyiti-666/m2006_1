#ifndef __PID_LOCATION_H__
#define __PID_LOCATION_H__

#include "main.h"
#include <stdint.h>
#include "pid_speed.h"

/* 位置环输出是「速度给定」，单位和速度环一致（转子 rpm）。
   2880 = 80（输出轴 rpm 限速）× 36（减速比），与 M2006_SPEED_LIMIT 保持一致 */
#define PID_LOC_OUT_MAX     2880.0f
/* 积分限幅 */
#define PID_LOC_I_MAX       50000.0f

/* 默认参数（输入单位是转子计数，294912 = 输出轴 1 圈）
   误差 1 圈会输出约 8800，被 2880 的限幅截住，属正常的饱和区 */
#define LOC_KP_DEFAULT      0.03f
#define LOC_KI_DEFAULT      0.0f
#define LOC_KD_DEFAULT      0.0f

extern PID_TypeDef g_location_pid;

void    pid_location_init(float Kp, float Ki, float Kd, float SetPoint);
int16_t pid_location_update(PID_TypeDef *pid, float currentValue);
void    pid_location_clear(void);
void    pid_location_clear_accum(PID_TypeDef *pid);

#endif
