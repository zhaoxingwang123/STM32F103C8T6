#include "Myflash.h"

//��ָ����ַ�İ��� ��16λ
u16 STMFLASH_ReadHalfWord(u32 addr){
    return *(vu16*)addr;
}

#if STM32_FLASH_WREN                            //���ʹ����д  

//������д��
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite){
    u16 i;
    for(i = 0 ; i<NumToWrite ;i++){
        FLASH_ProgramHalfWord(WriteAddr , pBuffer[i]);
        WriteAddr+=2;
    }
}

#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //�ֽ�
#else 
#define STM_SECTOR_SIZE	2048
#endif		 

//��ָ����ַ��ʼд��ָ�����ȵ�����
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//�����2K�ֽ�
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite){
    u32 secpos;                                 //������ַ
    u16 secoff;                                 //������ƫ�Ƶ�ַ(16λ����)
    u16 secremain;                              //������ʣ���ַ(16λ����)
    u16 i;    
    u32 offaddr;                                //���0X08000000ƫ�ƺ�ĵ�ַ
    if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//�Ƿ���ַ
    FLASH_Unlock();                             //����
    offaddr = WriteAddr - STM32_FLASH_BASE;     //ʵ��ƫ�Ƶ�ַ
    secpos=offaddr/STM_SECTOR_SIZE;             //�ڼ�ҳ
    secoff=(offaddr%STM_SECTOR_SIZE)/2;         //�����ڵ�ƫ�ƣ�2���ֽ�Ϊ������λ��
    secremain=STM_SECTOR_SIZE/2-secoff;         //�����ڵ�ʣ���С
    if(NumToWrite<=secremain) secremain = NumToWrite;                       //Ҫд������ݲ���������ʣ���λ��
    while(1){
        STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE , STMFLASH_BUF ,STM_SECTOR_SIZE/2);
        for(i=0;i<secremain;i++){
            if(STMFLASH_BUF[secoff+i]!=0XFFFF) break;                       //��Ҫ����
        }
        if(i<secremain){                                                    //�ж��Ƿ���Ҫ����
            FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);      //������ҳ����д����ҳ
            for(i=0;i<secremain;i++){
                STMFLASH_BUF[i+secoff] = pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE , STMFLASH_BUF ,STM_SECTOR_SIZE/2);
        }else STMFLASH_Write_NoCheck(WriteAddr , pBuffer , secremain);      //����Ҫ������ֱ��д��ʣ������
        if(NumToWrite == secremain) break;      //д����
        else{
            secpos++;
            secoff = 0;
            pBuffer += secremain ;              //ָ��ƫ��
            WriteAddr += secremain ;            //��ַƫ��
            NumToWrite -= secremain ;           //ʣ��δд���ֽ�
            if(NumToWrite > (STM_SECTOR_SIZE/2)) 
                secremain = STM_SECTOR_SIZE/2;  //��һ����������д����
            else secremain = NumToWrite ;       //��һҳ����д����
        }
    }
    FLASH_Lock();                               //����
}

#endif

//��ָ����ַ��ʼ����ָ�����ȵ�����
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead){
    u16 i;
    for(i=0;i<NumToRead;i++){
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr);
        ReadAddr+=2;
    }
}



