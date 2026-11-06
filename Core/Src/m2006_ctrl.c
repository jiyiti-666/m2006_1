#include "m2006_ctrl.h"
#include "pid_speed.h"
#include "pid_location.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

static M2006_Bus *s_bus = NULL;
static uint8_t    s_id  = 1;
static volatile CtrlMode s_mode = CTRL_MODE_OPEN;

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

/* ============ 初始化 ============ */
void Ctrl_Init(M2006_Bus *bus, uint8_t id)
{
    s_bus = bus;
    s_id  = id;

    pid_speed_init(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT, SPEED_KD_DEFAULT, 0.0f);
    pid_location_init(LOC_KP_DEFAULT, LOC_KI_DEFAULT, LOC_KD_DEFAULT, 0.0f);

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
static void parse_cmd(const char *s)
{
    float v;

    if (strncmp(s, "stop", 4) == 0) {
        g_open_current = 0;
        s_mode = CTRL_MODE_OPEN;
        pid_speed_clear();
        pid_location_clear();
        return;
    }

    if (strncmp(s, "speed:", 6) == 0) {
        g_speed_target = (int16_t)str2float(s + 6);
        pid_speed_clear_accum();
        s_mode = CTRL_MODE_SPEED;
        return;
    }

    if (strncmp(s, "loc:", 4) == 0) {
        g_loc_target = (int32_t)(str2float(s + 4) * M2006_COUNT_PER_REV);
        pid_location_clear_accum();
        pid_speed_clear_accum();
        s_mode = CTRL_MODE_LOC;
        return;
    }

    if (strncmp(s, "cur:", 4) == 0) {
        g_open_current = (int16_t)str2float(s + 4);
        s_mode = CTRL_MODE_OPEN;
        return;
    }

    if (strncmp(s, "Kp:", 3) == 0) {
        v = str2float(s + 3);
        if (s_mode == CTRL_MODE_LOC) g_location_pid.Proportion = v;
        else                         g_speed_pid.Proportion    = v;
        return;
    }

    if (strncmp(s, "Ki:", 3) == 0) {
        v = str2float(s + 3);
        if (s_mode == CTRL_MODE_LOC) g_location_pid.Integral = v;
        else                         g_speed_pid.Integral    = v;
        return;
    }

    if (strncmp(s, "Kd:", 3) == 0) {
        v = str2float(s + 3);
        if (s_mode == CTRL_MODE_LOC) g_location_pid.Derivative = v;
        else                         g_speed_pid.Derivative    = v;
        return;
    }
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

    M2006_Motor *m = M2006_Get(s_bus, s_id);
    if (m == NULL) return;

    /* 掉线保护：100ms 没收到反馈就卸力 */
    if (!m->online) {
        M2006_SetCurrent(s_bus, s_id, 0);
        M2006_Flush(s_bus);
        return;
    }

    int16_t out = 0;

    switch (s_mode)
    {
    case CTRL_MODE_OPEN:
        out = g_open_current;
        break;

    case CTRL_MODE_SPEED:
        g_speed_pid.SetPoint = (float)g_speed_target;
        out = pid_speed_update(&g_speed_pid, (float)m->speed);
        break;

    case CTRL_MODE_LOC:
    {
        g_location_pid.SetPoint = (float)g_loc_target;

        int16_t spd_tgt = pid_location_update(&g_location_pid, (float)m->total_cnt);

        /* 进入死区：停止输出并清积分，防止在目标点来回抖 */
        if (g_location_pid.Error < LOC_DEADBAND &&
            g_location_pid.Error > -LOC_DEADBAND) {
            spd_tgt = 0;
            pid_location_clear_accum();
            pid_speed_clear_accum();
        }

        g_speed_pid.SetPoint = (float)spd_tgt;
        out = pid_speed_update(&g_speed_pid, (float)m->speed);
        break;
    }

    default:
        out = 0;
        break;
    }

    M2006_SetCurrent(s_bus, s_id, out);
    M2006_Flush(s_bus);
}

/* ============ VOFA 波形输出 ============ */
void Ctrl_SendData(void)
{
    if (s_bus == NULL) return;

    M2006_Motor *m = M2006_Get(s_bus, s_id);
    if (m == NULL) return;

    printf("motor:%.1f,%.1f,%.1f,%.1f\r\n",
           (double)g_speed_pid.SetPoint,
           (double)m->speed,
           (double)((float)g_loc_target / M2006_COUNT_PER_REV),
           (double)((float)m->total_cnt / M2006_COUNT_PER_REV));
}

CtrlMode Ctrl_GetMode(void)
{
    return s_mode;
}
