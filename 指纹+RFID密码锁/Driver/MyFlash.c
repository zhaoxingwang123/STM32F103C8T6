#include "Myflash.h"

//读指定地址的半字 ，16位
u16 STMFLASH_ReadHalfWord(u32 addr){
    return *(vu16*)addr;
}

#if STM32_FLASH_WREN                            //如果使能了写  

//不检查的写入
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite){
    u16 i;
    for(i = 0 ; i<NumToWrite ;i++){
        FLASH_ProgramHalfWord(WriteAddr , pBuffer[i]);
        WriteAddr+=2;
    }
}

#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //字节
#else 
#define STM_SECTOR_SIZE	2048
#endif		 

//从指定地址开始写入指定长度的数据
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//最多是2K字节
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite){
    u32 secpos;                                 //扇区地址
    u16 secoff;                                 //扇区内偏移地址(16位计算)
    u16 secremain;                              //扇区内剩余地址(16位计算)
    u16 i;    
    u32 offaddr;                                //相对0X08000000偏移后的地址
    if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//非法地址
    FLASH_Unlock();                             //解锁
    offaddr = WriteAddr - STM32_FLASH_BASE;     //实际偏移地址
    secpos=offaddr/STM_SECTOR_SIZE;             //第几页
    secoff=(offaddr%STM_SECTOR_SIZE)/2;         //扇区内的偏移（2个字节为基本单位）
    secremain=STM_SECTOR_SIZE/2-secoff;         //扇区内的剩余大小
    if(NumToWrite<=secremain) secremain = NumToWrite;                       //要写入的数据不大于扇区剩余的位置
    while(1){
        STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE , STMFLASH_BUF ,STM_SECTOR_SIZE/2);
        for(i=0;i<secremain;i++){
            if(STMFLASH_BUF[secoff+i]!=0XFFFF) break;                       //需要擦除
        }
        if(i<secremain){                                                    //判断是否需要擦除
            FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);      //擦除整页，再写入整页
            for(i=0;i<secremain;i++){
                STMFLASH_BUF[i+secoff] = pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE , STMFLASH_BUF ,STM_SECTOR_SIZE/2);
        }else STMFLASH_Write_NoCheck(WriteAddr , pBuffer , secremain);      //不需要擦除，直接写入剩余区域
        if(NumToWrite == secremain) break;      //写完了
        else{
            secpos++;
            secoff = 0;
            pBuffer += secremain ;              //指针偏移
            WriteAddr += secremain ;            //地址偏移
            NumToWrite -= secremain ;           //剩余未写入字节
            if(NumToWrite > (STM_SECTOR_SIZE/2)) 
                secremain = STM_SECTOR_SIZE/2;  //下一个扇区还是写不完
            else secremain = NumToWrite ;       //下一页可以写完了
        }
    }
    FLASH_Lock();                               //上锁
}

#endif

//从指定地址开始读出指定长度的数据
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead){
    u16 i;
    for(i=0;i<NumToRead;i++){
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr);
        ReadAddr+=2;
    }
}



