#include "m2006_ctrl.h"
#include "pid_speed.h"
#include "pid_location.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

static M2006_Bus *s_bus  = NULL;
static uint8_t    s_id_a = 1;
static uint8_t    s_id_b = 2;
static volatile CtrlMode s_mode = CTRL_MODE_OPEN;

/* 两台电机各自独立的 PID 状态；Kp/Ki/Kd 每次从全局变量同步（供 VOFA 在线调） */
static PID_TypeDef s_spd_a, s_spd_b;
static PID_TypeDef s_loc_a, s_loc_b;

/* 上一周期的速度给定，供 VOFA 显示「目标速度」 */
static volatile float s_spd_set_a = 0.0f;
static volatile float s_spd_set_b = 0.0f;

volatile int16_t g_open_current = 0;
volatile int16_t g_speed_target = 0;
volatile int32_t g_loc_target   = 0;

/* ============ 串口接收缓冲 ============ */
static uint8_t  rx_byte;
static char     rx_line[48];
static char     rx_work[48];
static volatile uint8_t rx_idx   = 0;
static volatile uint8_t rx_ready = 0;

/* 自己实现的字符串转浮点，不依赖库里的 scanf %f */
static float str2float(const char *s)
{
    int   sign = 1;
    long  ip   = 0;
    float frac = 0.0f, div = 1.0f;

    while (*s == ' ' || *s == '\t') s++;
    if (*s == '-')      { sign = -1; s++; }
    else if (*s == '+') { s++; }

    while (*s >= '0' && *s <= '9') { ip = ip * 10 + (*s - '0'); s++; }

    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            frac = frac * 10.0f + (float)(*s - '0');
            div *= 10.0f;
            s++;
        }
    }
    return sign * ((float)ip + frac / div);
}

/* 把全局参数同步到某一路 PID，保证 VOFA 改 Kp 对两台都生效 */
static void sync_param(PID_TypeDef *dst, const PID_TypeDef *src)
{
    dst->Proportion = src->Proportion;
    dst->Integral   = src->Integral;
    dst->Derivative = src->Derivative;
}

/* 只清运行状态，不动参数 */
static void pid_reset_state(PID_TypeDef *p)
{
    p->SetPoint    = 0.0f;
    p->ActualValue = 0.0f;
    p->SumError    = 0.0f;
    p->Error       = 0.0f;
    p->LastError   = 0.0f;
}

/* ============ 初始化 ============ */
void Ctrl_Init(M2006_Bus *bus, uint8_t id_a, uint8_t id_b)
{
    s_bus  = bus;
    s_id_a = id_a;
    s_id_b = id_b;

    pid_speed_init(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT, SPEED_KD_DEFAULT, 0.0f);
    pid_location_init(LOC_KP_DEFAULT, LOC_KI_DEFAULT, LOC_KD_DEFAULT, 0.0f);

    pid_reset_state(&s_spd_a); sync_param(&s_spd_a, &g_speed_pid);
    pid_reset_state(&s_spd_b); sync_param(&s_spd_b, &g_speed_pid);
    pid_reset_state(&s_loc_a); sync_param(&s_loc_a, &g_location_pid);
    pid_reset_state(&s_loc_b); sync_param(&s_loc_b, &g_location_pid);

    g_open_current = 0;
    g_speed_target = 0;
    g_loc_target   = 0;
    s_mode         = CTRL_MODE_OPEN;

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* ============ 串口接收中断 ============ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_idx > 0 && !rx_ready) {
                memcpy(rx_work, rx_line, rx_idx);
                rx_work[rx_idx] = '\0';
                rx_ready = 1;
                rx_idx   = 0;
            }
        } else if (rx_idx < sizeof(rx_line) - 1) {
            rx_line[rx_idx++] = (char)rx_byte;
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/* ============ 命令解析 ============ */
static void apply_gain(PID_TypeDef *p, char param, float v)
{
    if (param == 'P')      p->Proportion = v;
    else if (param == 'I') p->Integral   = v;
    else                   p->Derivative = v;
}

/* which: 0 = 两台一起改, 'A' = 只改 A, 'B' = 只改 B
   改哪个环由当前模式决定：位置模式改位置环，其余改速度环 */
static void set_gain(char which, char param, float v)
{
    int loc = (s_mode == CTRL_MODE_LOC);

    if (which == 0 || which == 'A') apply_gain(loc ? &s_loc_a : &s_spd_a, param, v);
    if (which == 0 || which == 'B') apply_gain(loc ? &s_loc_b : &s_spd_b, param, v);
}

static void parse_cmd(const char *s)
{
    if (strncmp(s, "stop", 4) == 0) {
        g_open_current = 0;
        s_mode = CTRL_MODE_OPEN;
        pid_speed_clear();
        pid_location_clear();
        pid_reset_state(&s_spd_a); pid_reset_state(&s_spd_b);
        pid_reset_state(&s_loc_a); pid_reset_state(&s_loc_b);
        return;
    }

    /* 把当前物理位置作为新的零点，清零多圈计数和目标 */
    if (strncmp(s, "zero", 4) == 0) {
        M2006_Motor *mA = M2006_Get(s_bus, s_id_a);
        M2006_Motor *mB = M2006_Get(s_bus, s_id_b);

        g_open_current = 0;
        g_loc_target   = 0;
        g_speed_target = 0;
        s_mode = CTRL_MODE_OPEN;

        if (mA != NULL) mA->total_cnt = 0;
        if (mB != NULL) mB->total_cnt = 0;

        pid_reset_state(&s_spd_a); pid_reset_state(&s_spd_b);
        pid_reset_state(&s_loc_a); pid_reset_state(&s_loc_b);
        return;
    }

    if (strncmp(s, "speed:", 6) == 0) {
        /* 命令按「输出轴 rpm」输入，超过 M2006_SPEED_LIMIT 会被截断 */
        float rpm = str2float(s + 6);
        if (rpm >  M2006_SPEED_LIMIT) rpm =  M2006_SPEED_LIMIT;
        if (rpm < -M2006_SPEED_LIMIT) rpm = -M2006_SPEED_LIMIT;
        g_speed_target = (int16_t)(rpm * M2006_REDUCTION);
        pid_reset_state(&s_spd_a); pid_reset_state(&s_spd_b);
        s_mode = CTRL_MODE_SPEED;
        return;
    }

    /* 同时兼容 "loc:1.0" 和 "location:1.0" 两种写法 */
    {
        const char *num = NULL;
        if (strncmp(s, "location:", 9) == 0)   num = s + 9;
        else if (strncmp(s, "loc:", 4) == 0)   num = s + 4;

        if (num != NULL) {
            g_loc_target = (int32_t)(str2float(num) * M2006_COUNT_PER_REV);
            pid_reset_state(&s_spd_a); pid_reset_state(&s_spd_b);
            pid_reset_state(&s_loc_a); pid_reset_state(&s_loc_b);
            s_mode = CTRL_MODE_LOC;
            return;
        }
    }

    if (strncmp(s, "cur:", 4) == 0) {
        g_open_current = (int16_t)str2float(s + 4);
        s_mode = CTRL_MODE_OPEN;
        return;
    }

    /* —— PID 参数：不带后缀改两台，带 A/B 只改那一台 —— */
    if (strncmp(s, "KpA:", 4) == 0) { set_gain('A', 'P', str2float(s + 4)); return; }
    if (strncmp(s, "KpB:", 4) == 0) { set_gain('B', 'P', str2float(s + 4)); return; }
    if (strncmp(s, "Kp:",  3) == 0) { set_gain(0,   'P', str2float(s + 3)); return; }

    if (strncmp(s, "KiA:", 4) == 0) { set_gain('A', 'I', str2float(s + 4)); return; }
    if (strncmp(s, "KiB:", 4) == 0) { set_gain('B', 'I', str2float(s + 4)); return; }
    if (strncmp(s, "Ki:",  3) == 0) { set_gain(0,   'I', str2float(s + 3)); return; }

    if (strncmp(s, "KdA:", 4) == 0) { set_gain('A', 'D', str2float(s + 4)); return; }
    if (strncmp(s, "KdB:", 4) == 0) { set_gain('B', 'D', str2float(s + 4)); return; }
    if (strncmp(s, "Kd:",  3) == 0) { set_gain(0,   'D', str2float(s + 3)); return; }
}

void Ctrl_Process(void)
{
    if (!rx_ready) return;
    parse_cmd(rx_work);
    rx_ready = 0;
}

/* ============ 控制任务（10ms 一次） ============ */
void Ctrl_Task(void)
{
    if (s_bus == NULL) return;

    M2006_Motor *ma = M2006_Get(s_bus, s_id_a);
    M2006_Motor *mb = M2006_Get(s_bus, s_id_b);

    uint8_t ok_a = (ma != NULL) && ma->online;
    uint8_t ok_b = (mb != NULL) && mb->online;

    int16_t out_a = 0, out_b = 0;

    switch (s_mode)
    {
    case CTRL_MODE_OPEN:
        out_a = (int16_t)(M2006_SIGN_A * g_open_current);
        out_b = (int16_t)(M2006_SIGN_B * g_open_current);
        break;

    case CTRL_MODE_SPEED:
        /* 反装的那台目标取反，两台在「世界坐标系」下朝同一方向 */
        s_spd_a.SetPoint = (float)(M2006_SIGN_A * g_speed_target);
        s_spd_b.SetPoint = (float)(M2006_SIGN_B * g_speed_target);
        if (ok_a) out_a = pid_speed_update(&s_spd_a, (float)ma->speed);
        if (ok_b) out_b = pid_speed_update(&s_spd_b, (float)mb->speed);
        break;

    case CTRL_MODE_LOC:
    {
        /* 反装的那台，目标位置符号相反，其余逻辑完全对称 */
        s_loc_a.SetPoint = (float)(M2006_SIGN_A * g_loc_target);
        s_loc_b.SetPoint = (float)(M2006_SIGN_B * g_loc_target);

        int16_t spd_a = 0, spd_b = 0;

        if (ok_a) spd_a = pid_location_update(&s_loc_a, (float)ma->total_cnt);
        if (ok_b) spd_b = pid_location_update(&s_loc_b, (float)mb->total_cnt);

        /* 同步补偿：进度落后的多给点速度，超前的少给点 */
        if (ok_a && ok_b && SYNC_GAIN > 0.0f) {
            float diff = (s_loc_a.Error - s_loc_b.Error) * 0.5f * SYNC_GAIN;
            if (diff >  SYNC_MAX) diff =  SYNC_MAX;
            if (diff < -SYNC_MAX) diff = -SYNC_MAX;
            spd_a = (int16_t)(spd_a + diff);
            spd_b = (int16_t)(spd_b - diff);
        }

        /* 到位死区：停输出、清积分，防止在目标点来回抖 */
        if (s_loc_a.Error < LOC_DEADBAND && s_loc_a.Error > -LOC_DEADBAND) {
            spd_a = 0;
            pid_location_clear_accum(&s_loc_a);
            pid_speed_clear_accum(&s_spd_a);
        }
        if (s_loc_b.Error < LOC_DEADBAND && s_loc_b.Error > -LOC_DEADBAND) {
            spd_b = 0;
            pid_location_clear_accum(&s_loc_b);
            pid_speed_clear_accum(&s_spd_b);
        }

        s_spd_a.SetPoint = (float)spd_a;
        s_spd_b.SetPoint = (float)spd_b;

        if (ok_a) out_a = pid_speed_update(&s_spd_a, (float)ma->speed);
        if (ok_b) out_b = pid_speed_update(&s_spd_b, (float)mb->speed);
        break;
    }

    default:
        break;
    }

    /* 记录速度给定，供波形对比 */
    s_spd_set_a = s_spd_a.SetPoint;
    s_spd_set_b = s_spd_b.SetPoint;

    /* 掉线的电机强制卸力 */
    M2006_SetCurrent(s_bus, s_id_a, ok_a ? out_a : 0);
    M2006_SetCurrent(s_bus, s_id_b, ok_b ? out_b : 0);
    M2006_Flush(s_bus);
}

/* ============ VOFA 波形输出（5 通道） ============ */
void Ctrl_SendData(void)
{
    if (s_bus == NULL) return;

    M2006_Motor *ma = M2006_Get(s_bus, s_id_a);
    M2006_Motor *mb = M2006_Get(s_bus, s_id_b);
    if (ma == NULL || mb == NULL) return;

    /* VOFA 波形 8 通道：
       0 目标速度  1 A速度  2 B速度
       3 目标圈数  4 A圈数  5 B圈数
       6 A在线     7 B在线
       位置都乘了方向系数，统一到同一坐标系，可以直接比同步性 */
    /* VOFA 波形 8 通道（速度已换算成输出轴 rpm）：
       0 目标速度  1 A速度  2 B速度
       3 目标圈数  4 A圈数  5 B圈数
       6 A在线     7 B在线 */
    printf("motor:%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%d\r\n",
           (double)((M2006_SIGN_A * s_spd_set_a) / M2006_REDUCTION),
           (double)((M2006_SIGN_A * ma->speed)     / M2006_REDUCTION),
           (double)((M2006_SIGN_B * mb->speed)     / M2006_REDUCTION),
           (double)((float)g_loc_target / M2006_COUNT_PER_REV),
           (double)((float)(M2006_SIGN_A * ma->total_cnt) / M2006_COUNT_PER_REV),
           (double)((float)(M2006_SIGN_B * mb->total_cnt) / M2006_COUNT_PER_REV),
           ma->online ? 1 : 0,
           mb->online ? 1 : 0);
}

CtrlMode Ctrl_GetMode(void)
{
    return s_mode;
}
