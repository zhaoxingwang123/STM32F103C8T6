#ifndef __MYFLASH_H__
#define __MYFLASH_H__
#include "stm32f10x.h"
#include "sys.h"

#define STM32_FLASH_SIZE 128            //��ѡSTM32��FLASH������С(��λΪK)
#define STM32_FLASH_WREN 1              //ʹ��FLASHд��(0����ʹ��;1��ʹ��)
//#define STM_SECTOR_SIZE  1024

#define STM32_FLASH_BASE 0x08000000     //STM32 FLASH����ʼ��ַ

u16 STMFLASH_ReadHalfWord(u32 addr);                                    //������
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);            //ָ����ַ������
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite); //ָ����ַ������д����
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);         //ָ����ַд����

#endif

