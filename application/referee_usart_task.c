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
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "stm32f4xx_hal.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "user_lib.h"

\


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          ���ֽڽ��
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

 
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

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
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
    pc_control_init();

    while(1)
    {
        sendData_Task_EE_To_PC(); //ͨѶ ��������
        referee_unpack_fifo_data();
        osDelay(5);
    }
}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          ���ֽڽ��
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}


void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
            detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
            detect_hook(REFEREE_TOE);
        }
    }
}

/*
@brief         ������ͻ��������ݣ�UART1)
                Rui Peng 2021/2/25
								���԰汾
*/
uint32_t sendTask_TimeStamp = 0; //����ʱ���
const uint16_t sendFreq = 10; //���ͼ�� ��ms)

void sendPack(uint8_t cmd_ID,uint8_t level, uint8_t robot_ID){
			
	if(xTaskGetTickCount()-sendFreq >sendTask_TimeStamp){ //��ʱ�����ѵ������Ͳ�����ʱ���
		 
		 sendTask_TimeStamp = xTaskGetTickCount(); //����ʱ���   
		 uint8_t dataToSend = HEADER; 
     HAL_UART_Transmit(&huart1, &dataToSend, 1, 3); // ���Ͱ�ͷ
		 
		 dataToSend = cmd_ID; 
		 HAL_UART_Transmit(&huart1, &dataToSend, 1, 3); // ����CMD_ID
		
		 dataToSend = level; 
		 HAL_UART_Transmit(&huart1, &dataToSend, 1, 3); // ����level
		 
		 dataToSend = robot_ID; 
		 HAL_UART_Transmit(&huart1, &dataToSend, 1, 3); // ����robot_ID
		 
		 uint8_t head = HEADER;
		 uint8_t checkSum = head + cmd_ID + level + robot_ID; //����У��� ���𶯡�uint8_t����
		
		 dataToSend = checkSum; 
		 HAL_UART_Transmit(&huart1, &dataToSend, 1, 3); // ����У���
	}
}


/*
@brief         ����λ��ͨѶ �����ͣ� ��UART1)
                Rui Peng 2021/2/25
								���԰汾
*/


void sendData_Task_EE_To_PC(void){
	
 uint8_t task_cmdID = MANUEL;   //Ĭ��ֵ
 uint8_t task_level = LEVEL_I;  //Ĭ��ֵ
 uint8_t task_robotID = ROBOTID_RED; //Ĭ��ֵ
 sendPack(task_cmdID,task_level,task_robotID); //������ͻ�������������λ����UART1)
}


//=====================================================



#define PC_RX_BUF_NUM 50
uint8_t pc_rx_buf[2][PC_RX_BUF_NUM];

 gimbal_control_t* p_gimbal_control = NULL;

struct gimbal_cmd gimbal_cmd_t;

void pc_control_init(void)
{
    usart1_init(pc_rx_buf[0], pc_rx_buf[1], PC_RX_BUF_NUM);
	  p_gimbal_control = get_gimbal_pointer();
}


uint8_t shootCommand = 0x00;//�Զ�����ָ��  0x00 = ͣ��  0xff = ����
uint8_t fricCommand = 0x01;// Ħ����ת��ָ��  0x01 =��ת  0x02 = ��ת
fp32 yawMove = 0;  //��̨����
fp32 pitchMove = 0;  //��̨����
uint32_t timeFlag = 0; //

static void pc_command_unpack(uint8_t *buf, uint16_t len)
{  
	if(len == PACK_LENG){
		if( buf[0] == HEADER){ //��Ϊ��ͷ
			uint8_t checkSum = 0;  
			for(uint8_t i = 0;i<PACK_LENG-1;i++)
			{
			checkSum = buf[i]+ checkSum;
			}
			if(checkSum == buf[PACK_LENG-1]){ //���У��� if checkSum OK		  
			  int16_t yawCommand = buf[2];
        yawCommand <<= 8;
        yawCommand |= buf[1];   //�ϲ��ߵ�λ
			  
			  int16_t pitchCommand = buf[4];
        pitchCommand <<= 8;
        pitchCommand |= buf[3];//�ϲ��ߵ�λ

			 yawMove = -(fp32)yawCommand*(3.14159f/180.0f)*0.008f;
			 pitchMove = -(fp32)pitchCommand*(3.14159f/180.0f)*0.008f;
					
			 	if(buf[6]== 0xff){  //����ָ��
				  shootCommand = 0xff;
				}else{
				  shootCommand = 0x00;
				}
	 }else{ //if checkSum bad��set all to zero for safty
	  
			shootCommand = 0x00;//�Զ�����ָ��  0x00 = ͣ��  0xff = ����
			yawMove = 0;  //��̨����
			pitchMove = 0;  //��̨����
	 
	 }
	}
 }
}

//void USART1_IRQHandler(void)
//{
//    static volatile uint8_t res;
//    if(USART1->SR & UART_FLAG_IDLE)
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart1);

//        static uint16_t this_time_rx_len = 0;

//        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            __HAL_DMA_DISABLE(huart1.hdmarx);
//            this_time_rx_len = PC_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart1.hdmarx, PC_RX_BUF_NUM);
//            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(huart1.hdmarx);
//           // fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
//					pc_command_unpack(pc_rx_buf[0], this_time_rx_len);
//					//usart1_tx_dma_enable(pc_rx_buf[0], this_time_rx_len);
//         //   detect_hook(REFEREE_TOE);
//        }
//        else
//        {
//            __HAL_DMA_DISABLE(huart1.hdmarx);
//            this_time_rx_len = PC_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart1.hdmarx, PC_RX_BUF_NUM);
//            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
//            __HAL_DMA_ENABLE(huart1.hdmarx);
//            //fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
//					pc_command_unpack(pc_rx_buf[1], this_time_rx_len);
//				//		usart1_tx_dma_enable(pc_rx_buf[1], this_time_rx_len);
//         //   detect_hook(REFEREE_TOE);
//        }
//    }
//}
/*
static void pc_command_unpack(uint8_t *buf, uint16_t len)
{
	if (buf[0] == 0xaf) {
		if (buf[1] == GIMBAL_MOVEMENT) {
			memcpy(&gimbal_cmd_t, buf, sizeof(gimbal_cmd_t));
			fp32 yaw = rad_format((fp32)gimbal_cmd_t.yaw/10000);
			fp32 pitch = rad_format((fp32)gimbal_cmd_t.pitch/10000);
			p_gimbal_control->gimbal_yaw_motor.absolute_angle_set = rad_format(p_gimbal_control->gimbal_yaw_motor.absolute_angle + yaw);
			p_gimbal_control->gimbal_pitch_motor.absolute_angle_set = rad_format(p_gimbal_control->gimbal_pitch_motor.absolute_angle + pitch);

		}
	}
}

*/	

//�����ж�

void USART1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = PC_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

            //reset set_data_lenght
            //�����趨���ݳ���
            huart1.hdmarx->Instance->NDTR = PC_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);

						HAL_UART_Receive_DMA(&huart1,(uint8_t*)pc_rx_buf[0], this_time_rx_len);
						pc_command_unpack((uint8_t*)pc_rx_buf[0], this_time_rx_len);
						//��¼���ݽ���ʱ��
						//detect_hook(DBUS_TOE);
            
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(huart1.hdmarx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = PC_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            huart1.hdmarx->Instance->NDTR = PC_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);

						
						HAL_UART_Receive_DMA(&huart1,(uint8_t*)pc_rx_buf[1], this_time_rx_len);
						pc_command_unpack((uint8_t*)pc_rx_buf[1], this_time_rx_len);
						//��¼���ݽ���ʱ��
						//detect_hook(DBUS_TOE);
        }
    } 
		
		HAL_UART_IRQHandler(&huart1);
}