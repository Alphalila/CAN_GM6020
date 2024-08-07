/*
 * bsp_can.c
 *
 *  Created on: Jul 4, 2024
 *      Author: DELL
 */

#include <motor_can.h>

moto_info_t motor_yaw_info1, motor_yaw_info2;
uint16_t can_cnt;


/***
 * 函数：筛选器配置
 * 作用：过滤器设置为0，即无过滤
 * 备注：需要配置过滤器之后才能正常使用CAN
 * 		本程序参数参考https://blog.csdn.net/weixin_73037889/article/details/130696750
 */
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);	//开启CAN1
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	//当FIFO0中有消息的时候进入中断
}

/***
 * 函数：CAN的FIFO 0中断回调函数
 * 作用：接收电机的反馈报文，并与电机结构体拼接
 * 参数：CAN的位号，即电机底部的小拨码位号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8] = {0};


  if(hcan->Instance == CAN1)		//验证一下是CAN1通道
  {
	  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //自带接收数据函数。

	  switch(rx_header.StdId)
		{
	  	  	//拼接电机参数
			case 0x205:	//ID1的电机反馈报文标识符为0x205
			{
				motor_yaw_info1.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);//角度，角度值范围：0 ~ 8191
				motor_yaw_info1.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);//速度，转速值单位：rpm
				motor_yaw_info1.torque_current = ((rx_data[4] << 8) | rx_data[5]);//转矩电流
				motor_yaw_info1.temperate      =   rx_data[6];					 //温度
				break;
			}
			case 0x206:	//ID2的电机反馈报文标识符为0x206
			{
				motor_yaw_info2.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);//角度，角度值范围：0 ~ 8191
				motor_yaw_info2.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);//速度，转速值单位：rpm
				motor_yaw_info2.torque_current = ((rx_data[4] << 8) | rx_data[5]);//转矩电流
				motor_yaw_info2.temperate      =   rx_data[6];					 //温度
				break;
			}
		}

//	  switch(rx_header2.StdId)
//	  {
//	  	  	//拼接电机参数
//	  		case 0x205:	//ID1的电机反馈报文标识符为0x205
//	  		{
//	  			motor_yaw_info1.rotor_angle    = ((rx_data1[0] << 8) | rx_data1[1]);//角度，角度值范围：0 ~ 8191
//	  			motor_yaw_info1.rotor_speed    = ((rx_data1[2] << 8) | rx_data1[3]);//速度，转速值单位：rpm
//	  			motor_yaw_info1.torque_current = ((rx_data1[4] << 8) | rx_data1[5]);//转矩电流
//	  			motor_yaw_info1.temperate      =   rx_data1[6];					 //温度
//	  			break;
//	  		}
//	  		case 0x206:	//ID2的电机反馈报文标识符为0x206
//	  		{
//	  			motor_yaw_info2.rotor_angle    = ((rx_data2[0] << 8) | rx_data2[1]);//角度，角度值范围：0 ~ 8191
//	  			motor_yaw_info2.rotor_speed    = ((rx_data2[2] << 8) | rx_data2[3]);//速度，转速值单位：rpm
//	  			motor_yaw_info2.torque_current = ((rx_data2[4] << 8) | rx_data2[5]);//转矩电流
//	  			motor_yaw_info2.temperate      =   rx_data2[6];					 //温度
//	  			break;
//	  		}
//	  	}
  }
}

/***
 * 函数：GM6020控制发送函数
 * 参数1：CAN的位号
 * 参数2：v1控制ID1的电机
 * 参数3：v2控制ID2的电机
 * 备注：一台电机使用tx_data的两位数据
 */
void set_GM6020_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1,int16_t v2)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8] = {0};		//中间量，可写入8位控制数据

  tx_header.StdId = 0x1ff;			//标准ID的电机控制报文的标识符
  tx_header.IDE   = CAN_ID_STD;		//标准帧
  tx_header.RTR   = CAN_RTR_DATA;	//数据帧
  tx_header.DLC   = 8;				//数据长度

  tx_data[0] = (v1>>8)&0xff;		//控制ID1的电机
  tx_data[1] =    (v1)&0xff;		//数据帧的第0、1位

  tx_data[2] = (v2>>8)&0xff;		//控制ID2的电机
  tx_data[3] =    (v2)&0xff;		//数据帧的第2、3位

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);
}

