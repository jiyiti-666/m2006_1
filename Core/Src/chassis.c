#include "chassis.h"
#include "m2006.h"
#include "m2006_ctrl.h"
#include <math.h>

/* main.c 里的总线实例 */
extern M2006_Bus moto;

static float s_radius = CHASSIS_TURN_RADIUS_MM;

void Chassis_SetRadius(float r_mm)
{
    s_radius = r_mm;
}

/* 直行：两台的目标位置同时加一个偏移 */
void Chassis_Go(float rev)
{
    M2006_Motor *ma = M2006_Get(&moto, M2006_ID_A);
    M2006_Motor *mb = M2006_Get(&moto, M2006_ID_B);
    if (ma == NULL || mb == NULL) return;

    int32_t delta = (int32_t)(rev * M2006_COUNT_PER_REV);

    g_loc_target_a = ma->total_cnt + delta;
    g_loc_target_b = mb->total_cnt + delta;

    Ctrl_SetMode(CTRL_MODE_LOC);
}

/*
 * 差速转弯运动学：
 *   车中心转 deg 度时，左右轮走过的弧长不同
 *   外轮圈数 = |deg| × (R + L/2) / (180 × D)
 *   内轮圈数 = |deg| × (R - L/2) / (180 × D)
 *
 * 左转（deg>0）：A 是内圈，B 是外圈
 * 右转（deg<0）：A 是外圈，B 是内圈
 * R = 0 时退化为原地自旋（两轮等距反向）
 */
void Chassis_Turn(float deg)
{
    M2006_Motor *ma = M2006_Get(&moto, M2006_ID_A);
    M2006_Motor *mb = M2006_Get(&moto, M2006_ID_B);
    if (ma == NULL || mb == NULL) return;

    float half = CHASSIS_TRACK_MM * 0.5f;
    float D    = CHASSIS_WHEEL_DIAM_MM;
    float ad   = fabsf(deg);

    float rev_out = ad * (s_radius + half) / (180.0f * D);
    float rev_in  = ad * (s_radius - half) / (180.0f * D);

    int32_t d_out = (int32_t)(rev_out * M2006_COUNT_PER_REV);
    int32_t d_in  = (int32_t)(rev_in  * M2006_COUNT_PER_REV);

    int32_t delta_a, delta_b;
    if (deg > 0.0f) {
        delta_a = d_in;    /* 左转：A 内圈 */
        delta_b = d_out;
    } else {
        delta_a = d_out;   /* 右转：A 外圈 */
        delta_b = d_in;
    }

    g_loc_target_a = ma->total_cnt + delta_a;
    g_loc_target_b = mb->total_cnt + delta_b;

    Ctrl_SetMode(CTRL_MODE_LOC);
}

/* 原地自旋：左右轮反向等距，等价于半径为 0 的转弯 */
void Chassis_Spin(float deg)
{
    float saved = s_radius;
    s_radius = 0.0f;
    Chassis_Turn(deg);
    s_radius = saved;
}
