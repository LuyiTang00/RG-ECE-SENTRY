#include "communication_task.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "user_lib.h"
#include "main.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

#define PC_RX_BUF_NUM 50
static uint8_t pc_rx_buf[2][PC_RX_BUF_NUM];

gimbal_control_t* p_gimbal_control = NULL;

struct gimbal_cmd gimbal_cmd_t;


void pc_control_init(void)
{
    usart1_init(pc_rx_buf[0], pc_rx_buf[1], PC_RX_BUF_NUM);
		p_gimbal_control = get_gimbal_pointer();
}


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

//串口中断
void USART1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = PC_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = PC_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            pc_command_unpack(pc_rx_buf[0], this_time_rx_len);
						usart1_tx_dma_enable(pc_rx_buf[0], this_time_rx_len);
						//记录数据接收时间
						//detect_hook(DBUS_TOE);
            
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = PC_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = PC_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

						pc_command_unpack(pc_rx_buf[1], this_time_rx_len);
						usart1_tx_dma_enable(pc_rx_buf[1], this_time_rx_len);
						//记录数据接收时间
						//detect_hook(DBUS_TOE);
        }
    }

}


