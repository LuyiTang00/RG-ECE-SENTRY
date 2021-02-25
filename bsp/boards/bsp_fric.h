#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"
//1320 speed level 2 single shoot - fric_dow
//1400 speed level 2 continuous shoot- fric_up
//1206 speed level 1 single 18m/s
//1170 speed level 0 15m/s
//1265 spedd level 2 22 m/s

#define FRIC_UP 1400
#define FRIC_DOWN 1265
#define FRIC_OFF 1000

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
