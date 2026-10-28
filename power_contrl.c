#include "power_contrl.h"
#include <math.h>
#include "chongshan_motor.h"
#include "chongshan_controller.h"


/*==================== 功率标定采集 ====================*/
/*
	目的: 把上面的 PWR_K1/PWR_K2/PWR_K3/PWR_P0 换成实测值

	Power_Cal_Push 在 chassis_task 每个控制周期调一次, 只记下当前四轮电流/转速
	Power_Cal_Fill 在 vofa_task 里调, 把 9 个通道追加到 vofa_buf 后面
	通道顺序(接在你原有通道之后):
	  +0~3   四轮电流 A
	  +4~7   四轮转速 rad/s
	  +8     实测底盘功率 W (裁判系统反馈的)

	拟合: 先把波形按 100ms 窗口平均(与功率表口径对齐), 再最小二乘
	  P = p*mean(sum|I*w|) + q*mean(sum I^2) + c*mean(sum|w|) + d
	  PWR_K1 = p / M3508_KT
	  PWR_K3 = q / (M3508_KT*M3508_KT)
	  PWR_K2 = c
	  PWR_P0 = d

	注意: 采集时限幅不要起作用(k 恒为 1), 否则拟合到的是限幅后的系统
*/

uint8_t power_cal_on = 1U;

static float cal_cur[Wheel_MOTOR_NUM];
static float cal_vel[Wheel_MOTOR_NUM];
float cal_ref;

float W_last;
float w_new=0.0f;

PID_TypeDef power_pid;

void Power_Cal_Init(void)
{
		for(int i=0;i<Wheel_MOTOR_NUM;i++)
		{
				cal_cur[i]=0.0f;
				cal_vel[i]=0.0f;
		}
		cal_ref=0.0f;
		
		PID_Init(&power_pid,1.0f,0.0f,0.0f,10.0f,0.0f,0.0f);
}

/* 实测功率送进来(W) ,裁判系统*/
void Power_Cal_Ref(float w)
{
		cal_ref=w;
}

/* 每个控制周期调一次: 只记下当前四轮电流/转速 */
void Power_Cal_Push(const float *cur_a, const float *vel_rads, uint8_t num)
{
		if(cur_a==NULL||vel_rads==NULL) return;
		if(num>Wheel_MOTOR_NUM) num=Wheel_MOTOR_NUM;

		for(uint8_t i=0;i<num;i++)
		{
				cal_cur[i]=cur_a[i];
				cal_vel[i]=vel_rads[i];
		}
}

/* 四轮功率预测: 返回瞬时值, */
/* cur_a 单位 A, vel_rads 单位 rad/s, 返回四轮合计功率 W */
float Power_Calc(const float *cur_a, const float *vel_rads)
{
		float W=0.0f;

		for(uint8_t i=0;i<Wheel_MOTOR_NUM;i++)
		{
				W+= PWR_K1*fabsf(M3508_KT*cur_a[i]*vel_rads[i])
					+ PWR_K2*fabsf(vel_rads[i])
					+ PWR_K3*(M3508_KT*cur_a[i])*(M3508_KT*cur_a[i])
					+ PWR_P0/(float)Wheel_MOTOR_NUM;
		}

		return W;
}
/*每个任务周期真正的功率控制流程*/
/*
	cur_a:底盘的电机pid解算之后算出的电流*M3508_KT，
	vel_rads：电机的反馈 fdb.meas.vel_rads
*/


void Power_task(const float *w_feedback, const float *w_predict,float *cur,float *vel)
{
		float P_init[Wheel_MOTOR_NUM];
		float P_total=0.0f;
		float P0_s=PWR_P0/(float)Wheel_MOTOR_NUM;	

		for(uint8_t i=0;i<Wheel_MOTOR_NUM;i++)
		{
				P_init[i]= PWR_K1*fabsf(M3508_KT*cur[i]*vel[i])
						 + PWR_K2*fabsf(vel[i])
						 + PWR_K3*(M3508_KT*cur[i])*(M3508_KT*cur[i])
						 + P0_s;

				P_total+=P_init[i];
		}

		if(P_total<=PWR_MAX) return;		

		float scale=PWR_MAX/P_total;


		for(uint8_t i=0;i<Wheel_MOTOR_NUM;i++)
		{
				float Aq=PWR_K3*M3508_KT*M3508_KT;
				float Bq=PWR_K1*M3508_KT*fabsf(vel[i])*((cur[i]>=0.0f)?1.0f:-1.0f);
				float Cq=PWR_K2*fabsf(vel[i]) + P0_s - P_init[i]*scale;
				float delta=Bq*Bq - 4.0f*Aq*Cq;

				float I_new;

				if(delta<0.0f)
				{
						I_new=-Bq/(2.0f*Aq);
						if((cur[i]>=0.0f)!=(I_new>=0.0f)) I_new=0.0f;
				}
				else
				{
						float sq=sqrtf(delta);
						I_new=(cur[i]>=0.0f) ? ((-Bq+sq)/(2.0f*Aq))
											 : ((-Bq-sq)/(2.0f*Aq));
				}

				if(cur[i]>=0.0f)
				{
						if(I_new> cur[i]) I_new= cur[i];
						if(I_new< 0.0f)   I_new= 0.0f;
				}
				else
				{
						if(I_new< cur[i]) I_new= cur[i];
						if(I_new> 0.0f)   I_new= 0.0f;
				}
				if(I_new> PWR_I_MAX) I_new= PWR_I_MAX;
				if(I_new<-PWR_I_MAX) I_new=-PWR_I_MAX;

				cur[i]=I_new;
		}

		w_new=Power_Calc(cur,vel);	
}
	

uint8_t Power_Cal_Fill(float *buf, float w_yuce,uint8_t offset)
{
		uint8_t n=offset;

		if(buf==NULL) return offset;

		for(int i=0;i<Wheel_MOTOR_NUM;i++) buf[n++]=cal_cur[i];
		for(int i=0;i<Wheel_MOTOR_NUM;i++) buf[n++]=cal_vel[i];
		buf[n++]=cal_ref;
		buf[n++]=w_yuce;
	
		return n;
}
