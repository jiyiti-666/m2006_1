#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"

/* ===== 机械参数：按你的车改这三项 ===== */

/* 轮距：左右两轮中心之间的距离，单位 mm */
#define CHASSIS_TRACK_MM        167.0f

/* 轮子直径（含轮胎外缘），单位 mm */
#define CHASSIS_WHEEL_DIAM_MM   85.0f

/* 转弯半径：车中心到转弯圆心的距离，单位 mm。这是「想怎么转」，不是量出来的。
   R = 0           → 原地自旋（两轮等距反向）
   R = TRACK/2     → 绕内侧轮转，内轮静止
   R < TRACK/2     → 内侧轮反转
   R 越大          → 转弯越平缓 */
#define CHASSIS_TURN_RADIUS_MM  250.0f

void Chassis_Go(float rev);         /* 直行 rev 圈（输出轴） */
void Chassis_Turn(float deg);       /* 按当前半径转 deg 度 */
void Chassis_Spin(float deg);       /* 原地自旋 deg 度 */
void Chassis_SetRadius(float r_mm); /* 运行时改半径 */

#endif
