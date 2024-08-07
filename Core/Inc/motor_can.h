/*
 * bsp_can.h
 *
 *  Created on: Jul 4, 2024
 *      Author: DELL
 */

#ifndef INC_MOTOR_CAN_H_
#define INC_MOTOR_CAN_H_

#include "main.h"
#include "can.h"
#include "stm32f4xx.h"

/***
 * 电机结构体，用于统一处理数据
 */
typedef struct
{
    uint16_t can_id;		//电机ID
    int16_t  set_voltage;	//设定的电压值
    uint16_t rotor_angle;	//机械角度
    int16_t  rotor_speed;	//转速
    int16_t  torque_current;//扭矩电流
    uint8_t  temperate;		//温度
}moto_info_t;				//电机结构体

void can_filter_init(void);
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void set_GM6020_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1,int16_t v2);


#endif /* INC_MOTOR_CAN_H_ */
