#ifndef __POWER_CONTRL_H
#define __POWER_CONTRL_H

#include "main.h"
#include "cmsis_os.h"
#include "chongshan_motor.h"

#define Wheel_MOTOR_NUM         4

#define CURRENT_TO_AMP          (20.0f / 16384.0f)

#define MAX_CURRENT             16384
#define MIN_CURRENT             (-16384)

/*
	单轮母线功率模型（根据本战队的舵轮小车实测的数据）:
	  P = K1*|KT*I*w| + K2*|w| + K3*(KT*I)^2 + P0/Wheel_MOTOR_NUM
*/
#define M3508_KT                0.3f    /* 3508输出轴转矩常数 N*m/A */
#define PWR_K1                  1.0f    /* 机械功率项, 有效系数 = K1*M3508_KT */
#define PWR_K2                  0.3f    /* 转速相关损项 */
#define PWR_K3                  1.56f   /* 铜损项, 有效系数 = K3*M3508_KT^2 */
#define PWR_P0                  4.5f    /* 四轮静态功耗合计 W */
#define POWER_K_CAL             1.0f    /* 整体标定系数 */


#define PWR_MAX               88.0f   /* 底盘功率上限 W, TODO: 按裁判系统实际值填 */
#define PWR_I_MAX             19.5f  



extern uint8_t power_cal_on;

void Power_Cal_Init(void);

/* 实测功率送进来(W)，结合27赛季重山自制功率计*/
void Power_Cal_Ref(float w);

/* 每个控制周期调一次, 只记下当前四轮电流/转速 */
void Power_Cal_Push(const float *cur_a, const float *vel_rads, uint8_t num);

/* 把标定通道追加到 vofa_buf, 返回填充后的总通道数 */
uint8_t Power_Cal_Fill(float *buf, float w_yuce,uint8_t offset);

void Power_task(const float *w_feedback, const float *w_predict,float *cur,float *vel);
float Power_Calc(const float *cur_a, const float *vel_rads);

#endif
