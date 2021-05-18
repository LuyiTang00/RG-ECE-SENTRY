#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stm32f4xx_hal.h"
#include "gimbal_task.h"
#include "communication_task.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "user_lib.h"
#include "main.h"


void pc_control_init(void);

#define GIMBAL_MOVEMENT 1

struct gimbal_cmd
{
	uint8_t head;
	uint8_t id;
	int16_t pitch;
	int16_t yaw;
};

#endif
