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
