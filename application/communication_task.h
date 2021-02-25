#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stm32f4xx_hal.h"

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
