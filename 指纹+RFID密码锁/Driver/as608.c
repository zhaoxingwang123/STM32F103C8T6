#include "as608.h"


#define STM32_RX1_BUF       Usart1RecBuf 
#define STM32_Rx1Counter    RxCounter
#define STM32_RX1BUFF_SIZE  USART1_RXBUFF_SIZE

unsigned char AS608_RECEICE_BUFFER[24];//指纹数据存储数组

//FINGERPRINT通信协议定义
unsigned char AS608_Pack_Head[6] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF};                     //协议包头
unsigned char AS608_Get_Img[6] = {0x01,0x00,0x03,0x01,0x0,0x05};                        //获得指纹图像
unsigned char FP_Search[11]= {0x01,0x0,0x08,0x04,0x01,0x0,0x0,0x03,0xA1,0x0,0xB2};      //搜索指纹搜索范围0 - 929
unsigned char FP_Img_To_Buffer1[7]= {0x01,0x0,0x04,0x02,0x01,0x0,0x08};                 //将图像放入到BUFFER1
unsigned char FP_Img_To_Buffer2[7]= {0x01,0x0,0x04,0x02,0x02,0x0,0x09};                 //将图像放入到BUFFER2
unsigned char FP_Reg_Model[6]= {0x01,0x0,0x03,0x05,0x0,0x09};                           //将BUFFER1跟BUFFER2合成特征模版
unsigned char FP_Delet_All_Model[6]= {0x01,0x0,0x03,0x0d,0x00,0x11};                    //删除指纹模块里所有的模版
unsigned char  FP_Save_Finger[9]= {0x01,0x00,0x06,0x06,0x01,0x00,0x0B,0x00,0x19};       //将BUFFER1中的特征码存放到指定的位置
unsigned char  FP_Delete_Model[10]= {0x01,0x00,0x07,0x0C,0x0,0x0,0x0,0x1,0x0,0x0};      //删除指定的模版

void PS_StaGPIO_Init(void){                                                             //读出感应状态（有指纹按下时，输出高电平）
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC , &GPIO_InitStructure);
}

unsigned char uart_recv(unsigned char *bufs, unsigned short timeout){                   //接收数据
    unsigned char i;
    unsigned short RX_timeout;
    RX_timeout = timeout;
    do{
        delay_ms(1);
        if(RX_timeout > 0){
            if(--RX_timeout == 0){
                unsigned char ret;
                ret = STM32_Rx1Counter;
                if(STM32_Rx1Counter > 0){
                    for(i = 0 ; i<STM32_Rx1Counter ;i++){
                        *bufs = STM32_RX1_BUF[i];
                        bufs++;
                    }
                }
                STM32_Rx1Counter = 0;
                return ret;
            }
        }
    }while(1);
}


unsigned char  AS608_Receive_Data(void){                                                //接收反馈数据缓冲
    memset(AS608_RECEICE_BUFFER,0,24);                                                  //清空数据
    return uart_recv(AS608_RECEICE_BUFFER,1000);
}

void AS608_Cmd_Send_Pack_Head(void){                                                    //发送包头
    uart1_send(AS608_Pack_Head,6);
}

unsigned char AS608_Cmd_Get_Img(void){                                                  //FINGERPRINT_获得指纹图像命令判断接收到的确认码,等于0指纹获取成功
    AS608_Cmd_Send_Pack_Head();
    uart1_send(AS608_Get_Img , 6);
    if(AS608_Receive_Data() > 0){
        return AS608_RECEICE_BUFFER[9];
    }else{
        return 0XFE;
    }
}

unsigned char  FINGERPRINT_Cmd_Img_To_Buffer1(void){                                    //讲图像转换成特征码存放在Buffer1中
    AS608_Cmd_Send_Pack_Head();
    uart1_send(FP_Img_To_Buffer1 , 7);
    if(AS608_Receive_Data() > 0){
        return AS608_RECEICE_BUFFER[9];
    }else{
        return 0XFE;
    }
}

unsigned char  FINGERPRINT_Cmd_Img_To_Buffer2(void){                                    //将图像转换成特征码存放在Buffer2中
    AS608_Cmd_Send_Pack_Head();
    uart1_send(FP_Img_To_Buffer2,7);
    if (AS608_Receive_Data() > 0)
    {
        return AS608_RECEICE_BUFFER[9];
    }
    else
    {
        return 0xFE;
    }
}

unsigned char FINGERPRINT_Cmd_Reg_Model(void){                                          //将BUFFER1 跟 BUFFER2 中的特征码合并成指纹模版
    AS608_Cmd_Send_Pack_Head();
    uart1_send(FP_Reg_Model,6);
    if (AS608_Receive_Data() > 0)
    {
        return AS608_RECEICE_BUFFER[9];
    }
    else
    {
        return 0xFE;
    }
}

unsigned char FINGERPRINT_Cmd_Delete_All_Model(void){                                   //删除指纹模块里的所有指纹模版
    AS608_Cmd_Send_Pack_Head(); 
    uart1_send(FP_Delet_All_Model,6);
    if (AS608_Receive_Data() > 0)
    {
        return AS608_RECEICE_BUFFER[9];
    }
    else
    {
        return 0xFE;
    }
}

void FINGERPRINT_Cmd_Delete_Model(unsigned short uiID_temp){                            //删除指纹模块里的指定指纹模版
    unsigned short uiSum_temp = 0;
    unsigned char i;

    FP_Delete_Model[4]=(uiID_temp&0xFF00)>>8;
    FP_Delete_Model[5]=(uiID_temp&0x00FF);

    for(i=0; i<8; i++)
        uiSum_temp = uiSum_temp + FP_Delete_Model[i];

    FP_Delete_Model[8]=(uiSum_temp&0xFF00)>>8;
    FP_Delete_Model[9]=uiSum_temp&0xFF;


    AS608_Cmd_Send_Pack_Head(); 
    uart1_send(FP_Delete_Model,10);
}

unsigned short FINGERPRINT_Cmd_Search_Finger(void){                                     //搜索全部用户
    AS608_Cmd_Send_Pack_Head();
    uart1_send(FP_Search,11);
    if (AS608_Receive_Data() > 0 && AS608_RECEICE_BUFFER[9] == 0)
    {
        return (AS608_RECEICE_BUFFER[10]*256 + AS608_RECEICE_BUFFER[11]);
    }
    else
    {
        return 0xFFFE;
    }
}

unsigned char FINGERPRINT_Cmd_Save_Finger(unsigned short storeID){
    unsigned short temp = 0;
    unsigned char i;

    FP_Save_Finger[5] =(storeID&0xFF00)>>8;
    FP_Save_Finger[6] = (storeID&0x00FF);

    for(i=0; i<7; i++)                                                                  //计算校验和
        temp = temp + FP_Save_Finger[i];

    FP_Save_Finger[7]=(temp & 0xFF00) >> 8;                                             //存放校验数据
    FP_Save_Finger[8]= temp & 0xFF;

    AS608_Cmd_Send_Pack_Head();                                                         //发送通信协议包头

    uart1_send(FP_Save_Finger,9);
    if (AS608_Receive_Data() > 0)
    {
        return AS608_RECEICE_BUFFER[9];
    }
    else
    {
        return 0xFE;
    }
}

unsigned short AS608_Find_Fingerprint(void){                                            //寻找指纹是否存在
    if(AS608_Cmd_Get_Img() == 0)
    {
        delay_ms(100);
        if (FINGERPRINT_Cmd_Img_To_Buffer1() == 0)
        {
            return FINGERPRINT_Cmd_Search_Finger();
        }
    }
    return 0xFFFE;
}

unsigned short AS608_Add_Fingerprint(unsigned short ID){                                //指纹添加新用户
    if(AS608_Cmd_Get_Img() == 0)
    {
        //delay_ms(100);
        if (FINGERPRINT_Cmd_Img_To_Buffer1() == 0)
        {
            if(AS608_Cmd_Get_Img() == 0)
            {
                //delay_ms(100);
                if (FINGERPRINT_Cmd_Img_To_Buffer2() == 0)
                {
                    if (FINGERPRINT_Cmd_Reg_Model() == 0)
                    {
                        return FINGERPRINT_Cmd_Save_Finger(ID) ;
                    }
                }
            }
        }
    }
    return 0xFFFE;
}


