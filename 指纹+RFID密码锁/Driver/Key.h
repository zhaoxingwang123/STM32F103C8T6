#ifndef __KEY_H
#define __KEY_H
#include "stm32f10x.h"
#include "delay.h"


#define BEEP  PCout(13)	
#define RELAY PBout(12)	

void BeepAndRelay_Init(void);
void Key_Init(void);
void PA15_PB3to5OUT_AND_PB6to9IN(void);
void PA15_PB3to5IN_AND_PB6to9OUT(void);
u8 Key_Scan(void);

#endif
