# 基于STM32C8T6的RFID&指纹的密码锁

## 一、原理图

![](.\部分参考资料\原理图.png)



## 二、模块

### 1.蜂鸣器及继电器

使用继电器实现门锁的开与关，本代码使用继电器连接小灯来模拟门锁的开关，继电器使用PB12口

蜂鸣器与继电器相似，都是只需要初始化GPIO口，并使用位带操作实现开关即可，使用PC13口

蜂鸣器采用推挽输出，继电器使用开漏输出

位带操作

#define BEEP  PCout(13)	
#define RELAY PBout(12)

```c
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
```



### 2.矩阵键盘

由原理图可见矩阵键盘使用到8个端口，其中有A15,B3,B4默认位JTAG接口，需要将其回复成普通端口

```c
void Key_Init(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOA , ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE); //关闭JTAG模式使A15,B3,B4变成普通IO口
}
```

矩阵键盘检测的方式，即行输出列输入检测哪一行，再列输出行输入，检测出位哪行哪列，最后根据值来判断位哪个按键

```c
void Key_Init(void);
void PA15_PB3to5OUT_AND_PB6to9IN(void);//四列输入，四行输出
void PA15_PB3to5IN_AND_PB6to9OUT(void);//四列输出，四行输入
u8 Key_Scan(void);					   //对键盘扫描出按键对应的值
```

<img src=".\部分参考资料\按键分布图.png" style="zoom:200%;" />



### 3.LCD1602

参考[lcd1602原理讲解](https://blog.csdn.net/as480133937/article/details/113148712)

LCD模块为显示屏模块，对LCD1602操作无非就是写命令写数据两种

```c
#ifndef __LCD_H
#define __LCD_H
#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"

#define LCD1602_RS PBout(10)    //数据命令
#define LCD1602_RW PBout(1)     //读写
#define LCD1602_EN PBout(0)     //使能

void Lcd_Init(void);
u8 Dao_xu(u8 data);             //根据原理图GPIOA0-7与D0-7为反向的所以需要将其写的数据倒过来
void LCD_Write_Com(unsigned char com);                                      //写命令
void LCD_Write_Data(unsigned char Data);                                    //写数据
void LCD_Write_Char(unsigned char x,unsigned char y,unsigned char Data);    //写字符
void LCD_Write_String(unsigned char x,unsigned char y,unsigned char *str);  //写字符串
void LCD_Clear(void);

#endif
```



### 4.RC522

[参考博客](https://www.cnblogs.com/quan0311/p/15025135.html#4_26)

RC522为RFID模块，用于存储门禁卡

RC522模块非常复杂，通过大量的寄存器操作实现寻卡，防碰撞，CRC计算，选定卡等操作。

定义RC522的配置以及寄存器rc522_config，实现RC522的各种操作rc522_function，参考上面博客实现即可

由于只涉及到存储卡号进行解锁，并没有涉及到向卡内写数据，改数据，读数据等操作，所以实现获取ID号即可

```c
u8 GET_PID( u8 *PID)
{
    u8 receivezt=0;
    receivezt=PcdRequest ( 0x52, PID );     //寻卡
    if(receivezt!=MI_OK) return 0;
    receivezt=PcdAnticoll ( PID );          //防冲撞
    if(receivezt!=MI_OK) return 0;
    receivezt=PcdSelect ( PID );            //选定卡
    if(receivezt!=MI_OK) return 0;
    return MI_OK;
}
```



### 5.AS608

指纹是通过串口进行数据传输，所以一般波特率选择是的57600（32单片机），通过连接到单片机的串口和指纹进行通信，指纹有很多io口，但是对于本次设计只需要6个，分别是2个3.3V，gnd，txd和rxd和TCH 6根数据线进行连接。其中txd表示发送刚给单片机RXD表示接收单片机的指令。其中3.3V表示的是供电电压，TXD表示发送，RXD表示接收，TCH表示判断手指是否是否在指纹模块上，有的话输出高电平。

串口通信是指两个有串口通信协议的设备间以串行的方式互相传输数据。。 串口通信(serial communications)的概念非常简单,串口按位(bit)发送和接收字节。尽管比按字节(byte)的并行通信慢,但是串口可以在使用一根线发送数据的同时用另一根线接收数据。它很简单并且能够实现远距离通信。比如ieee488定义并行通行状态时，规定设备线总长不得超过20米，并且任意两个设备间的长度不得超过2米；而对于串口而言，长度可达1200米。典型地，串口用于ascii码字符的传输。通信使用3根线完成，分别是地线、发送、接收。

由于串口通信是异步的，端口能够在一根线上发送数据同时在另一根线上接收数据。其他线用于握手，但不是必须的。串口通信最重要的参数是波特率、数据位、停止位和奇偶校验。对于两个进行通信的端口，这些参数必须匹配

```c
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

```



### 6.TIM2

通用定时器，整个项目只用到的通用定时器最基础的定时功能，只需要进行初始化以及中断服务函数的编写即可

```c
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

```



### 7.FLASH

将密码与IC卡的数据存放到FLASH里这样就能实现，掉电不丢失的功能

```c
#ifndef __MYFLASH_H__
#define __MYFLASH_H__
#include "stm32f10x.h"
#include "sys.h"

#define STM32_FLASH_SIZE 128            //所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN 1              //使能FLASH写入(0，不使能;1，使能)
//#define STM_SECTOR_SIZE  1024

#define STM32_FLASH_BASE 0x08000000     //STM32 FLASH的起始地址

u16 STMFLASH_ReadHalfWord(u32 addr);                                    //读半字
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);            //指定地址读数据
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite); //指定地址无需检查写数据
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);         //指定地址写数据

#endif

```



## 三、 主函数

主函数将各个模块整合，主函数要实现的功能有：

​	1.初始密码为000000	

​	2.输入密码LCD显示开

​	3.密码正确后，继电器通过TIM2实现15秒自动关闭

​	4.输入错误计数，输入错误超过三次之后锁死，智能通过重启电源解决

​	5.输入两次正确密码后进入修改密码程序，输入新的密码后，按下修改按键会提示再次输入，两次输入一样后密码修改成功

​	6.031407进入管理员模式，管理员模式提供注册IC卡，删除IC卡，以及注册指纹功能

​	7.031408删除所有指纹

​	8.031406将密码初始化为000000

```c
#define uchar            unsigned char
#define uint             unsigned int
#define FLASH_SAVE_ADDR  0X08004096     //设置FLASH 保存地址(必须为偶数)
#define MAX_PEOPLE       5              //最大存储5张IC卡
#define SIZE             10             //写卡号的地址长度

uchar FlagKeyPress = 0;                 //是否有按键按下
uchar keycode;                          //按键值
uchar pass = 0;                         //密码是否正确标志
uchar ICpass = 0;                       //IC卡是否正确标志
uchar CurrentPassword[6]={0,0,0,0,0,0}; //存储当前密码
uchar InputData[6]={0,0,0,0,0,0};       //输入密码
uchar initpassword[6]={0,0,0,0,0,0};    //初始密码
uchar TempPassword[6];                  //重置密码缓存数组
uchar PressNum=0;                       //密码输入位数记数
uchar CorrectCont;                      //正确输入计数
uchar ReInputEn=0;                      //重置密码输入允许
uchar ReInputCont;                      //重新输入计数
uchar ErrorCont;                        //错误次数计数
uchar RELAY_TIME;                       //继电器开启时间
uchar InitDisplay=1;                    //返回主页面标志
uchar Register = 0;                     //注册卡标志
uchar Delete = 0;                       //删除卡标志
uchar RegFingerprint = 0;               //注册指纹标志
u8 ucArray_ID [ 4 ] ;                   //存放IC卡号
u8 ID_BUF[8];                           //扫描卡获取卡号
u8 ID_TEMP_Buffer[10];                  //ID_TEMP_Buffer注册过的卡号
uchar Manager = 0;                      //管理注册指纹，注册IC卡与删除IC卡
unsigned short user_id ;                //用户指纹ID

void CHECK_NEW_MCU(void);                                   //检查是否是新的单片机，是的话清空存储区，否则保留
void KeyPress(unsigned char code);                          //键盘功能判断
void BuzzerRingsNum(u8 num);                                //蜂鸣器控制
void Display_user_id(void);                                 //显示指纹用户id
void ResetPassword(void);                                   //重置密码
void DataInit(void);                                        //将输入密码清空
void WRITE_IC_NUM_TO_FLASH(u8* ID_Buffer,u8 space);         //IC卡号写入STM32内部FLASH
void READ_IC_NUM_FOR_FLASH(u8* ID_TEMP_Buffer ,u8 space);   //从STM32内部FLASH读出IC卡号
void WRITE_DATA_TO_FLASH(u8* ID_Buffer,u8 LEN);             //密码写入STM32内部FLASH
void READ_DATA_FOR_FLASH(u8* ID_TEMP_Buffer ,u8 LEN);       //从STM32内部FLASH读出密码
void Cancel(void);											//取消操作，将正在输入的密码清楚
void Ensure(void);											//确认按键操作
void finger_ctrl(void);										//指纹控制函数
u8 RC522_SCAN(u8* BUF);                                     //扫描IC卡
void COMPER_ID_MODE(void);                                  //正常待机刷卡函数
void ADD_ID_MODE(void);                                     //注册IC函数
void DEL_ID_MODE(void);                                     //删除IC卡函数

int main(void){
    delay_init();
    NVIC_Configuration();
    BeepAndRelay_Init();
    Key_Init();
    delay_ms(500);
    DataInit();
    Lcd_Init();
    LCD_Write_String( 0 ,0 ,(uchar*)"***Loading***");
    LCD_Write_String(0,1,(uchar*)"****************");
    CHECK_NEW_MCU();                    //单片机校验,如果为新单片机，将IC卡存储的空间清楚出来，并将初始密码写入
    PS_StaGPIO_Init();
    RC522_Init();
    PcdReset();                         //复位RC522 
    M500PcdConfigISOType('A');          //设置工作方式
    uart1_Init(57600);
    TIM2_Init(999 , 719);               //以100Hz计数，定时10ms
    while(1){
        if(InitDisplay==1){
            InitDisplay = 0;
            BEEP = 0;
            DataInit();
            CorrectCont=0;              //正确计数器清零
            RELAY = 0;                  //继电器关闭
            LCD_Write_String(0,0,(uchar*)"===Coded Lock===");
            LCD_Write_String(0,1,(uchar*)"password:       ");
            LCD_Write_Com(0x80+0x40+9);
            LCD_Write_Com(0x0F);        //光标闪烁
        }
        finger_ctrl();                  //指纹处理函数
        if(Register == 0&&Delete == 0)
        COMPER_ID_MODE();
        ADD_ID_MODE();
        DEL_ID_MODE();
        keycode = Key_Scan();
        if ((keycode<16)&&(FlagKeyPress==0)){
            FlagKeyPress = 1;           //每次只处理一个按键
            KeyPress(keycode);
            FlagKeyPress = 0;
        }
        delay_ms(10);
    }
}
```

