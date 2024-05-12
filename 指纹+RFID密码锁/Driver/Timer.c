#include "Timer.h"

void TIM2_Init(u16 arr , u16 psc){
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2 ,&TIM_TimeBaseStructure);     //初始化时基单元
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_ITConfig(TIM2 , TIM_IT_Update , ENABLE);
    TIM_Cmd(TIM2 , ENABLE);
}

extern u8 RELAY_TIME; 
extern u8 InitDisplay; 
extern u8 pass;
extern u8 ICpass;
extern u8 ReInputEn;

void TIM2_IRQHandler(void){
    static u8 time_count = 0;
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
        if(time_count ++ >= 100){                       //一秒
            time_count = 0;
            if(RELAY_TIME )  RELAY_TIME--;
            else{
                if(pass==1 || ICpass==1){
                    if(ReInputEn==0){
                        pass = 0;
                        ICpass=0;
                        InitDisplay = 1;
                    }
                }
            }
        }
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

