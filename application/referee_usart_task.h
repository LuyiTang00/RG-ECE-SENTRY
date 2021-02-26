/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H
#include "main.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void referee_usart_task(void const * argument);

/*
@brief         打包发送机器人数据    （UART1)
                Rui Peng 2021/2/25
								测试版本
*/
void sendPack(uint8_t cmd_ID,uint8_t level, uint8_t robot_ID);
	
/*
@brief  各种CV——EE/通讯配置
*/


#define HEADER 0xaf; //包头
#define AUTOFIRE 0xff //自动开火
#define AUTOAIM 0x00 //自动瞄准
#define MANUEL  0xfa //纯手动模式
#define BIGBUFF 0xbb //打大符
#define SINBIGBUFF 0xaa //变速大符

#define LEVEL_I  0x01 //1级
#define LEVEL_II 0x02 //2级
#define LEVEL_III 0x03 //3级

#define ROBOTID_RED 0xff  //己方机器人颜色
#define ROBOTID_BLUE 0x00  //己方机器人颜色






/*
@brief         与上位机通讯  任务   （发送） （UART1)
                Rui Peng 2021/2/25
								测试版本
*/



void sendData_Task_EE_To_PC(void);



#endif
