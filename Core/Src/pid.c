/*
 * pid.c
 *
 *  Created on: Jul 5, 2024
 *      Author: DELL
 */

#include "pid.h"

pid_struct_t gimbal_yaw_speed_pid1, gimbal_yaw_speed_pid2;
pid_struct_t gimbal_yaw_angle_pid1, gimbal_yaw_angle_pid2;

/***
 * 函数：PID初始化赋值函数
 * 参数1：PID指针
 * 参数2：kp比例
 * 参数3：ki积分
 * 参数4：kd微分
 * 参数5：i_max积分限幅
 * 参数6：out_max输出限幅
 */
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

/***
 * 函数：PID运算函数
 * 参数1：PID结构体指针
 * 参数2：target_value设定值
 * 参数3：feedback_value反馈值
 */
float pid_calc(pid_struct_t *pid, float target_value, float feedback_value)
{
  pid->target_value = target_value;
  pid->feedback_value = feedback_value;
  pid->err[1] = pid->err[0];//上一次误差
  pid->err[0] = pid->target_value - pid->feedback_value;//本次误差

  pid->p_out  = pid->kp * pid->err[0];//kp比例调整
  pid->i_out += pid->ki * pid->err[0];//ki积分调整
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//kd微分调整
  //LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

  pid->output = pid->p_out + pid->i_out + pid->d_out;//总输出，p+i+d
  //LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;//函数返回值为总输出
}

/***
 * 函数：电机角度环和速度环PID初始化赋值函数
 * 备注：只是初测出来的数据，具体还需要测试
 */
void gimbal_PID_init()
{
	//PID1赋值
	pid_init(&gimbal_yaw_angle_pid1, 7, 0.01, 0, 0, 320);//角度P=7,I=0.01,D=0
	pid_init(&gimbal_yaw_speed_pid1, 10, 0.05, 0, 30000, 30000);//速度P=10,I=0.05,D=0
	//PID2赋值
	pid_init(&gimbal_yaw_angle_pid2, 7, 0.015, 0, 0, 320);//角度P=7,I=0.01,D=0
	pid_init(&gimbal_yaw_speed_pid2, 10, 0.05, 0, 30000, 30000);//速度P=10,I=0.05,D=0
}
