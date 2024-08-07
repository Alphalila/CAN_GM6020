/*
 * pid.h
 *
 *  Created on: Jul 5, 2024
 *      Author: DELL
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
#include "stm32f4xx.h"
/***
 * 说明：PID结构体
 * 作用：存放PID计算的各项数据
 */
typedef struct _pid_struct_t
{
  float kp;		//比例
  float ki;		//积分
  float kd;		//微分
  float i_max;	//积分限幅
  float out_max;//输出限幅

  float target_value;   //target value设定值
  float feedback_value; //feedback value反馈值
  float err[2];			//error and last error两次误差

  float p_out;	//比例输出
  float i_out;	//积分输出
  float d_out;	//微分输出
  float output;	//总输出，即p+i+d
}pid_struct_t;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);

void gimbal_PID_init(void);
float pid_calc(pid_struct_t *pid, float target_value, float feedback_value);

#endif /* INC_PID_H_ */
