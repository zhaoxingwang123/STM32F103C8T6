#ifndef __AS608_H
#define __AS608_H
#include "stm32f10x.h"
#include "usart1.h"
#include "delay.h"
#include "sys.h"
#include<stdio.h>
#include <stdlib.h>
#include <string.h>

#define PS_Sta   PCin(14)//指纹模块感应引脚

void PS_StaGPIO_Init(void);
unsigned char FINGERPRINT_Cmd_Delete_All_Model(void);
unsigned short AS608_Find_Fingerprint(void);
unsigned short AS608_Add_Fingerprint(unsigned short ID);
void FINGERPRINT_Cmd_Delete_Model(unsigned short uiID_temp);

#endif

