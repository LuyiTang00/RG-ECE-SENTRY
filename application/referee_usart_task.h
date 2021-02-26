/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM����ϵͳ���ݴ���
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
  * @brief          ����ϵͳ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void referee_usart_task(void const * argument);

/*
@brief         ������ͻ���������    ��UART1)
                Rui Peng 2021/2/25
								���԰汾
*/
void sendPack(uint8_t cmd_ID,uint8_t level, uint8_t robot_ID);
	
/*
@brief  ����CV����EE/ͨѶ����
*/


#define HEADER 0xaf; //��ͷ
#define AUTOFIRE 0xff //�Զ�����
#define AUTOAIM 0x00 //�Զ���׼
#define MANUEL  0xfa //���ֶ�ģʽ
#define BIGBUFF 0xbb //����
#define SINBIGBUFF 0xaa //���ٴ��

#define LEVEL_I  0x01 //1��
#define LEVEL_II 0x02 //2��
#define LEVEL_III 0x03 //3��

#define ROBOTID_RED 0xff  //������������ɫ
#define ROBOTID_BLUE 0x00  //������������ɫ






/*
@brief         ����λ��ͨѶ  ����   �����ͣ� ��UART1)
                Rui Peng 2021/2/25
								���԰汾
*/



void sendData_Task_EE_To_PC(void);



#endif
