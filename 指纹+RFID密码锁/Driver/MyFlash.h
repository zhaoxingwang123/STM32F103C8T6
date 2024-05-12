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

