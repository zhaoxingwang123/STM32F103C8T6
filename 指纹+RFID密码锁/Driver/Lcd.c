#include "Lcd.h"


u8 Dao_xu(u8 data)//倒叙函数
{
    u8 i = 0 ,temp = 0;;
	  
	  for(i = 0; i < 8; i++)
	 {
	   temp += (((data >> i) & 0x01) << (7 - i));
	 }
	 return temp;
}

//写命令函数
void LCD_Write_Com(unsigned char com){
    LCD1602_RS = 0;
    delay_us(1);
    LCD1602_RW = 0;
    delay_us(1);
    LCD1602_EN = 1;
    delay_us(1);
    GPIO_Write(GPIOA , (GPIO_ReadOutputData(GPIOA )& 0XFF00) + Dao_xu(com));
    delay_us(100);
    LCD1602_EN = 0;
}

//写数据函数
void LCD_Write_Data(unsigned char Data){
    LCD1602_RS = 1;
    delay_us(1);
    LCD1602_RW = 0;
    delay_us(1);
    LCD1602_EN = 1;
    delay_us(1);
    GPIO_Write(GPIOA , (GPIO_ReadOutputData(GPIOA )& 0XFF00) + Dao_xu(Data));
    delay_us(100);
    LCD1602_EN = 0;
}

//写字符函数
void LCD_Write_Char(unsigned char x,unsigned char y,unsigned char Data){
    if(y == 0){
        LCD_Write_Com(0X80 + x);    //第一行
    }else{
        LCD_Write_Com(0XC0 + x);    //第二行
    }
    LCD_Write_Data(Data);
}

//写字符串函数
void LCD_Write_String(unsigned char x,unsigned char y,unsigned char *str){
    if(y == 0){
        LCD_Write_Com(0X80 + x);    //第一行
    }else{
        LCD_Write_Com(0XC0 + x);    //第二行
    }
    while(*str){
        LCD_Write_Data(*str);
        str++;
    }
}

//清屏函数
void LCD_Clear(void){
    LCD_Write_Com(0x01); 
    delay_ms(5);
}

void Lcd_Init(void){
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
    GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //输出速度50MHZ
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化GPIOA
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1|GPIO_Pin_10;  // LCD1602 EN-RW-RS
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //输出速度50MHZ
    GPIO_Init(GPIOB, &GPIO_InitStructure);   //GPIOB
    
    //初始化LCD1602
    LCD_Write_Com(0X38);
    delay_ms(5);
    LCD_Write_Com(0X38);
    delay_ms(5);
    LCD_Write_Com(0X38);
    delay_ms(5);
    LCD_Write_Com(0X08);    //显示关闭
    delay_ms(5);
    LCD_Write_Com(0X01);    //显示清屏
    delay_ms(5);
    LCD_Write_Com(0X06);    //显示光标移动位置
    delay_ms(5);
    LCD_Write_Com(0X0C);    //显示开及光标设置
    delay_ms(5);
    
}

