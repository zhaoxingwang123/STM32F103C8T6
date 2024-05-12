#ifndef __LCD_H
#define __LCD_H
#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"

#define LCD1602_RS PBout(10)    //��������
#define LCD1602_RW PBout(1)     //��д
#define LCD1602_EN PBout(0)     //ʹ��

void Lcd_Init(void);
u8 Dao_xu(u8 data);             //����ԭ��ͼGPIOA0-7��D0-7Ϊ�����������Ҫ����д�����ݵ�����
void LCD_Write_Com(unsigned char com);                                      //д����
void LCD_Write_Data(unsigned char Data);                                    //д����
void LCD_Write_Char(unsigned char x,unsigned char y,unsigned char Data);    //д�ַ�
void LCD_Write_String(unsigned char x,unsigned char y,unsigned char *str);  //д�ַ���
void LCD_Clear(void);

#endif
