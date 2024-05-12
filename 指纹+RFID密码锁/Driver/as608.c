#include "as608.h"


#define STM32_RX1_BUF       Usart1RecBuf 
#define STM32_Rx1Counter    RxCounter
#define STM32_RX1BUFF_SIZE  USART1_RXBUFF_SIZE

unsigned char AS608_RECEICE_BUFFER[24];//ָ�����ݴ洢����

//FINGERPRINTͨ��Э�鶨��
unsigned char AS608_Pack_Head[6] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF};                     //Э���ͷ
unsigned char AS608_Get_Img[6] = {0x01,0x00,0x03,0x01,0x0,0x05};                        //���ָ��ͼ��
unsigned char FP_Search[11]= {0x01,0x0,0x08,0x04,0x01,0x0,0x0,0x03,0xA1,0x0,0xB2};      //����ָ��������Χ0 - 929
unsigned char FP_Img_To_Buffer1[7]= {0x01,0x0,0x04,0x02,0x01,0x0,0x08};                 //��ͼ����뵽BUFFER1
unsigned char FP_Img_To_Buffer2[7]= {0x01,0x0,0x04,0x02,0x02,0x0,0x09};                 //��ͼ����뵽BUFFER2
unsigned char FP_Reg_Model[6]= {0x01,0x0,0x03,0x05,0x0,0x09};                           //��BUFFER1��BUFFER2�ϳ�����ģ��
unsigned char FP_Delet_All_Model[6]= {0x01,0x0,0x03,0x0d,0x00,0x11};                    //ɾ��ָ��ģ�������е�ģ��
unsigned char  FP_Save_Finger[9]= {0x01,0x00,0x06,0x06,0x01,0x00,0x0B,0x00,0x19};       //��BUFFER1�е��������ŵ�ָ����λ��
unsigned char  FP_Delete_Model[10]= {0x01,0x00,0x07,0x0C,0x0,0x0,0x0,0x1,0x0,0x0};      //ɾ��ָ����ģ��

void PS_StaGPIO_Init(void){                                                             //������Ӧ״̬����ָ�ư���ʱ������ߵ�ƽ��
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC , &GPIO_InitStructure);
}

unsigned char uart_recv(unsigned char *bufs, unsigned short timeout){                   //��������
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


unsigned char  AS608_Receive_Data(void){                                                //���շ������ݻ���
    memset(AS608_RECEICE_BUFFER,0,24);                                                  //�������
    return uart_recv(AS608_RECEICE_BUFFER,1000);
}

void AS608_Cmd_Send_Pack_Head(void){                                                    //���Ͱ�ͷ
    uart1_send(AS608_Pack_Head,6);
}

unsigned char AS608_Cmd_Get_Img(void){                                                  //FINGERPRINT_���ָ��ͼ�������жϽ��յ���ȷ����,����0ָ�ƻ�ȡ�ɹ�
    AS608_Cmd_Send_Pack_Head();
    uart1_send(AS608_Get_Img , 6);
    if(AS608_Receive_Data() > 0){
        return AS608_RECEICE_BUFFER[9];
    }else{
        return 0XFE;
    }
}

unsigned char  FINGERPRINT_Cmd_Img_To_Buffer1(void){                                    //��ͼ��ת��������������Buffer1��
    AS608_Cmd_Send_Pack_Head();
    uart1_send(FP_Img_To_Buffer1 , 7);
    if(AS608_Receive_Data() > 0){
        return AS608_RECEICE_BUFFER[9];
    }else{
        return 0XFE;
    }
}

unsigned char  FINGERPRINT_Cmd_Img_To_Buffer2(void){                                    //��ͼ��ת��������������Buffer2��
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

unsigned char FINGERPRINT_Cmd_Reg_Model(void){                                          //��BUFFER1 �� BUFFER2 �е�������ϲ���ָ��ģ��
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

unsigned char FINGERPRINT_Cmd_Delete_All_Model(void){                                   //ɾ��ָ��ģ���������ָ��ģ��
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

void FINGERPRINT_Cmd_Delete_Model(unsigned short uiID_temp){                            //ɾ��ָ��ģ�����ָ��ָ��ģ��
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

unsigned short FINGERPRINT_Cmd_Search_Finger(void){                                     //����ȫ���û�
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

    for(i=0; i<7; i++)                                                                  //����У���
        temp = temp + FP_Save_Finger[i];

    FP_Save_Finger[7]=(temp & 0xFF00) >> 8;                                             //���У������
    FP_Save_Finger[8]= temp & 0xFF;

    AS608_Cmd_Send_Pack_Head();                                                         //����ͨ��Э���ͷ

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

unsigned short AS608_Find_Fingerprint(void){                                            //Ѱ��ָ���Ƿ����
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

unsigned short AS608_Add_Fingerprint(unsigned short ID){                                //ָ��������û�
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


