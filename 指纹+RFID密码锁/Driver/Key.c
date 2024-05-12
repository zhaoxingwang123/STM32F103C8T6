#include "Key.h"

void BeepAndRelay_Init(void){
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC , ENABLE);
    
    //初始化继电器，并且是默认低电平，引脚为PB12
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB , &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB,GPIO_Pin_12);
    
    //初始化蜂鸣器，并且默认低电平，引脚为PC13
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC , &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC,GPIO_Pin_13);
}

void Key_Init(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOA , ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE); //关闭JTAG模式使A15,B3,B4变成普通IO口
}

void PA15_PB3to5OUT_AND_PB6to9IN(void){
    //PA15，PB3-5为推挽输出 ， PB6-9为下拉输入
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA , &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB , &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 |GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB , &GPIO_InitStructure);
}

void PA15_PB3to5IN_AND_PB6to9OUT(void){
    //PA15，PB3-5为下拉输入 ， PB6-9为推挽输出
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA , &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB , &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 |GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB , &GPIO_InitStructure);
}


void KEY_DELAY(void)
{
    u16 i = 10000;
    while(i--);
}

u8 Key_Scan(void){
    u8 tmp = 0 ,num = 99;
    //检测矩阵键盘为行输出列输出检测哪列，再切换列输出行输出检测出为哪行
    PA15_PB3to5OUT_AND_PB6to9IN();
    GPIO_WriteBit(GPIOA,GPIO_Pin_15,Bit_SET);
    GPIO_Write(GPIOB,(GPIO_ReadOutputData(GPIOB)|0X0038));
    KEY_DELAY();
    //delay_ms(10);
    if((GPIO_ReadInputData(GPIOB) & 0X03C0) != 0){
        tmp = (u8)((GPIO_ReadInputData(GPIOB) & 0X03C0) >> 2);
        PA15_PB3to5IN_AND_PB6to9OUT();
        GPIO_Write(GPIOB,(GPIO_ReadOutputData(GPIOB)|0X03C0));
        KEY_DELAY();
        //delay_ms(10);
        if((GPIO_ReadInputData(GPIOB) & 0X0038) != 0 || GPIO_ReadInputDataBit(GPIOA , GPIO_Pin_15)){
            tmp += (u8)((GPIO_ReadInputData(GPIOB) & 0X0038)>>2 ) + GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15);
            while((GPIO_ReadInputData(GPIOB) & 0X0038) != 0 || GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15));
            switch(tmp){
                case 65 : num = 0 ; break;
                case 130: num = 1 ; break;
                case 66 : num = 2 ; break;
                case 34 : num = 3 ; break;
                case 132: num = 4 ; break;
                case 68 : num = 5 ; break;
                case 36 : num = 6 ; break;
                case 136: num = 7 ; break;
                case 72 : num = 8 ; break;
                case 40 : num = 9 ; break;
                case 24 : num = 10; break;
                case 20 : num = 11; break;
                case 18 : num = 12; break;
                case 17 : num = 13; break;
                case 129: num = 14; break;
                case 33 : num = 15; break;
                default: break;
            }
        }
    }
    return num;
}


