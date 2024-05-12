#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "usart1.h"
//#include "BeepAndRelay.h"
#include "Key.h"
#include "Lcd.h"
#include "Timer.h"
#include "Myflash.h"
#include "rc522_config.h"
#include "rc522_function.h"
#include "as608.h"

#define uchar            unsigned char
#define uint             unsigned int
#define FLASH_SAVE_ADDR  0X08004096     //����FLASH �����ַ(����Ϊż��)
#define MAX_PEOPLE       5              //���洢5��IC��
#define SIZE             10             //д���ŵĵ�ַ����

uchar FlagKeyPress = 0;                 //�Ƿ��а�������
uchar keycode;                          //����ֵ
uchar pass = 0;                         //�����Ƿ���ȷ��־
uchar ICpass = 0;                       //IC���Ƿ���ȷ��־
uchar CurrentPassword[6]={0,0,0,0,0,0}; //�洢��ǰ����
uchar InputData[6]={0,0,0,0,0,0};       //��������
uchar initpassword[6]={0,0,0,0,0,0};    //��ʼ����
uchar TempPassword[6];                  //�������뻺������
uchar PressNum=0;                       //��������λ������
uchar CorrectCont;                      //��ȷ�������
uchar ReInputEn=0;                      //����������������
uchar ReInputCont;                      //�����������
uchar ErrorCont;                        //�����������
uchar RELAY_TIME;                       //�̵�������ʱ��
uchar InitDisplay=1;                    //������ҳ���־
uchar Register = 0;                     //ע�Ῠ��־
uchar Delete = 0;                       //ɾ������־
uchar RegFingerprint = 0;               //ע��ָ�Ʊ�־
u8 ucArray_ID [ 4 ] ;                   //���IC����
u8 ID_BUF[8];                           //ɨ�迨��ȡ����
u8 ID_TEMP_Buffer[10];                  //ID_TEMP_Bufferע����Ŀ���
uchar Manager = 0;                      //����ע��ָ�ƣ�ע��IC����ɾ��IC��
unsigned short user_id ;                //�û�ָ��ID

void CHECK_NEW_MCU(void);                                   //����Ƿ����µĵ�Ƭ�����ǵĻ���մ洢����������
void KeyPress(unsigned char code);                          //���̹����ж�
void BuzzerRingsNum(u8 num);                                //����������
void Display_user_id(void);                                 //��ʾָ���û�id
void ResetPassword(void);                                   //��������
void DataInit(void);                                        //�������������
void WRITE_IC_NUM_TO_FLASH(u8* ID_Buffer,u8 space);         //IC����д��STM32�ڲ�FLASH
void READ_IC_NUM_FOR_FLASH(u8* ID_TEMP_Buffer ,u8 space);   //��STM32�ڲ�FLASH����IC����
void WRITE_DATA_TO_FLASH(u8* ID_Buffer,u8 LEN);             //����д��STM32�ڲ�FLASH
void READ_DATA_FOR_FLASH(u8* ID_TEMP_Buffer ,u8 LEN);       //��STM32�ڲ�FLASH��������
void Cancel(void);
void Ensure(void);
void finger_ctrl(void);
u8 RC522_SCAN(u8* BUF);                                     //ɨ��IC��
void COMPER_ID_MODE(void);                                  //��������ˢ������
void ADD_ID_MODE(void);                                     //ע��IC����
void DEL_ID_MODE(void);                                     //ɾ��IC������

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
    CHECK_NEW_MCU();                    //��Ƭ��У��
    PS_StaGPIO_Init();
    RC522_Init();
    PcdReset();                         //��λRC522 
    M500PcdConfigISOType('A');          //���ù�����ʽ
    uart1_Init(57600);
    TIM2_Init(999 , 719);               //��100Hz��������ʱ10ms
    while(1){
        if(InitDisplay==1){
            InitDisplay = 0;
            BEEP = 0;
            DataInit();
            CorrectCont=0;              //��ȷ����������
            RELAY = 0;                  //�̵����ر�
            LCD_Write_String(0,0,(uchar*)"===Coded Lock===");
            LCD_Write_String(0,1,(uchar*)"password:       ");
            LCD_Write_Com(0x80+0x40+9);
            LCD_Write_Com(0x0F);        //�����˸
        }
        finger_ctrl();                  //ָ�ƴ�����
        if(Register == 0&&Delete == 0)
        COMPER_ID_MODE();
        ADD_ID_MODE();
        DEL_ID_MODE();
        keycode = Key_Scan();
        if ((keycode<16)&&(FlagKeyPress==0)){
            FlagKeyPress = 1;           //ÿ��ֻ����һ������
            KeyPress(keycode);
            FlagKeyPress = 0;
        }
        delay_ms(10);
    }
}

void CHECK_NEW_MCU(void){
    u8 comper_str[6],i=0;
    u8 clear_str[10] = {0};
    memset(clear_str,'0',sizeof(clear_str));
    STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)comper_str,5); 
    comper_str[5] = '\0';
    if(strstr((char*)comper_str,"FDYDZ") == NULL){          //�µĵ�Ƭ��
        WRITE_DATA_TO_FLASH(initpassword,6);
        for(i = 0; i < MAX_PEOPLE; i++){
            WRITE_IC_NUM_TO_FLASH(clear_str,i);             //��IC�ĵ�ַ����ȫ������
        }
        STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)"FDYDZ",5);    //д�롰FDYDZ���������´�У��
    }
    READ_DATA_FOR_FLASH(CurrentPassword,6);                 //��STM32�ڲ�FLASH������洢����
}

void DataInit(void){
    uchar i =0;                         //��������λ������
    PressNum = 0;                       //��������λ������
    for(i=0;i<6;i++)InputData[i]=0;     //�����ݴ�������
}

void WRITE_IC_NUM_TO_FLASH(u8* ID_Buffer,u8 space){
    STMFLASH_Write(FLASH_SAVE_ADDR+0x10*space , (u16*)ID_Buffer , SIZE);
    delay_ms(100);
}

void READ_IC_NUM_FOR_FLASH(u8* ID_TEMP_Buffer ,u8 space){
    STMFLASH_Read(FLASH_SAVE_ADDR + 0x10 * space,(u16*)ID_TEMP_Buffer,SIZE);
}

void WRITE_DATA_TO_FLASH(u8* ID_Buffer,u8 LEN){
    STMFLASH_Write(FLASH_SAVE_ADDR+0x10*MAX_PEOPLE*2 , (u16*)ID_Buffer , LEN);
    delay_ms(100);
}

void READ_DATA_FOR_FLASH(u8* ID_TEMP_Buffer ,u8 LEN){
    STMFLASH_Read(FLASH_SAVE_ADDR+0x10*MAX_PEOPLE*2 ,(u16*)ID_TEMP_Buffer,LEN);
}

void Display_user_id(void){
    LCD_Write_Com(0XC0);
    LCD_Write_Data('U');
    LCD_Write_Data('s');
    LCD_Write_Data('e');
    LCD_Write_Data('r');
    LCD_Write_Data('s');
    LCD_Write_Data(' ');
    LCD_Write_Data('i');
    LCD_Write_Data('d');
    LCD_Write_Data(user_id+0x30);       //OX30Ϊ��ʾ���ֵ�ƫ����
    LCD_Write_Data(' ');
    LCD_Write_Data('r');
    LCD_Write_Data('e');
    LCD_Write_Data('g');
    LCD_Write_Data(' ');
    LCD_Write_Data(' ');
    LCD_Write_Data(' ');
    LCD_Write_Data(' ');
}

void BuzzerRingsNum(u8 num){
    uchar i =0;
    for(i = 0; i<num ; i++){
        BEEP = 1;
        delay_ms(100);
        BEEP = 0;
        if(num != 1) delay_ms(50);
    }
}

void ResetPassword(void){
    uchar i = 0;
    
    if(pass == 0){
        BuzzerRingsNum(3);              //��������3��
        LCD_Write_String(0,1,(uchar*)"      error     "); 
        delay_ms(1000);
        LCD_Write_String(0,1,(uchar*)"password:       ");
        LCD_Write_Com(0x80+0x40+9);
        LCD_Write_Com(0x0F);            //�����˸
        DataInit();                     //���������ݼ���������
    }else{
        if(ReInputEn == 1){
            if(PressNum == 6){
                ReInputCont++;
                if(ReInputCont == 2){
                    if((TempPassword[0]==InputData[0])&&(TempPassword[1]==InputData[1])&&(TempPassword[2]==InputData[2])&&  
                        (TempPassword[3]==InputData[3])&&(TempPassword[4]==InputData[4])&&(TempPassword[5]==InputData[5])){  //��������������������Ա�
                            LCD_Write_String(0,1,(uchar*)"ResetPasswordOK ");
                            BuzzerRingsNum(2);
                            WRITE_DATA_TO_FLASH(TempPassword,6);                // ��������д��STM32�ڲ�FLASH
                            delay_ms(100);
                            READ_DATA_FOR_FLASH(CurrentPassword,6);             //��STM32�ڲ�FLASH������洢����
                            delay_ms(1000);
                            LCD_Write_String(0,1,(uchar*)"password:       ");
                            LCD_Write_Com(0x80+0x40+9);
                            LCD_Write_Com(0x0F);                                //�����˸
                    }else{                                                      //������������벻һ��
                        BuzzerRingsNum(3);
                        LCD_Write_String(0,1,(uchar*)"      error     "); 
                        delay_ms(1000);
                        LCD_Write_String(0,1,(uchar*)"password:       ");
                        LCD_Write_Com(0x80+0x40+9);
                        LCD_Write_Com(0x0F);                                    //�����˸
                    }
                    ReInputEn = 0;                                              //�ر����ù���
                    ReInputCont = 0;
                    CorrectCont = 0;
                }else{
                    BuzzerRingsNum(1);
                    LCD_Write_String(0,1,(uchar*)"input again     ");           //��ʾ���ٴ�����
                    for(i=0 ; i<6 ;i++){
                        TempPassword[i]=InputData[i];                           //����һ������������ݴ�����
                    }
                }
                DataInit();
            }
        }
    }
}

void Cancel(void){
    unsigned char i;
    
    LCD_Write_String(0,1,(uchar*)"password:       ");
    BuzzerRingsNum(2);                                                          //��ʾ��,������
    for(i=0;i<6;i++)InputData[i]=0;                                             //�����ݴ�������
    RELAY=0;                                                                    //�ر���
    BEEP =0;                                                                    //������
    pass=0;                                                                     //������ȷ��־����
    ReInputEn=0;                                                                //������������־����
    ErrorCont=0;                                                                //������������������
    CorrectCont=0;                                                              //������ȷ�����������
    ReInputCont=0;                                                              //������������������� 
    PressNum=0;                                                                 //����λ������������
    ICpass = 0;
    InitDisplay=1;
}

void Ensure(void){
    if(PressNum == 6){
        if((InputData[0]==0)&&(InputData[1]==3)&&(InputData[2]==1)&&(InputData[3]==4)&&(InputData[4]==0)&&(InputData[5]==6)){           //031406��ʼ������
            WRITE_DATA_TO_FLASH(initpassword , 6);
            delay_ms(100);
            READ_DATA_FOR_FLASH(CurrentPassword , 6);
            LCD_Write_String(0,1,(uchar*)"Init password...");
            BEEP = 1;
            delay_ms(1000);
            BEEP = 0;
            LCD_Write_String(0,1,(uchar*)"password:       ");
            LCD_Write_Com(0x80+0x40+9);
            LCD_Write_Com(0x0F);                                                //�����˸
        }else if((InputData[0]==0)&&(InputData[1]==3)&&(InputData[2]==1)&&(InputData[3]==4)&&(InputData[4]==0)&&(InputData[5]==7)){     //031407�������Աģʽ��ע�Ῠɾ����ע��ָ��
            LCD_Write_String(0,1,(uchar*)"                ");
            LCD_Write_String(0,2,(uchar*)"    manager     ");
            BuzzerRingsNum(2);
            Manager = 1;
        }else if((InputData[0]==0)&&(InputData[1]==3)&&(InputData[2]==1)&&(InputData[3]==4)&&(InputData[4]==0)&&(InputData[5]==8)){     //031408ָ�����
            if(FINGERPRINT_Cmd_Delete_All_Model() == 0){
                LCD_Write_String(0,1,(uchar*)" clear finger ok");
                BuzzerRingsNum(2);
            }
            user_id = 0;
            delay_ms(1000);
            InitDisplay = 1;
        }else{
            if((InputData[0]==CurrentPassword[0])&&(InputData[1]==CurrentPassword[1])&&(InputData[2]==CurrentPassword[2])&&  
                (InputData[3]==CurrentPassword[3])&&(InputData[4]==CurrentPassword[4])&&(InputData[5]==CurrentPassword[5])){
                CorrectCont++;
                if(CorrectCont == 1){                                           //��ֻ��һ����ȷ����ʱ������
                    LCD_Write_String(0,1,(uchar*)"      open      ");
                    RELAY = 1;                                                  //�̵�����
                    RELAY_TIME = 15;                                            //����15��
                    pass = 1;                                                   //������ȷ��־
                    BuzzerRingsNum(2);
                }else{                                                          //��������ȷ����ʱ�����������������빦��
                    LCD_Write_String(0,1,(uchar*)"SetNewWordEnable");
                    BuzzerRingsNum(2);
                    ReInputEn=1;
                    CorrectCont = 0;
                }
            }else{
                ErrorCont ++;
                LCD_Write_String(0,1,(uchar*)"      error     ");
                if(ErrorCont == 3){
                    do{
                          LCD_Write_String(0,1,(uchar*)"Keyboard Locked!");
                          RELAY=0;                                              //�ر���
                          BEEP = !BEEP;
                          delay_ms(55);
                    }while(1);
                }else{
                    BEEP = 1;
                    delay_ms(1000);
                    BEEP = 0;
                    LCD_Write_String(0,1,(uchar*)"password:       ");
                    LCD_Write_Com(0x80+0x40+9);
                    LCD_Write_Com(0x0F);                                        //�����˸
                }
            }
        }
        DataInit();
    }
}


//������Ӧ���򣬲����Ǽ�ֵ
//���ؼ�ֵ��
//         7       8      9      10(A)//ע��IC��
//         4       5      6      11(B)//ɾ��IC��
//         1       2      3      12(C)//ע��ָ��
//        14(����) 0   15(ȷ��)  13(D)//�޸�����
void KeyPress(uchar keycode){
    uchar i;

    switch(keycode){
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            if(Register==0&&Delete==0&&RegFingerprint==0){                      //û�������¼�����ʱ
                if(PressNum < 6){                                               //����λ������6ʱ�Ž�������
                    LCD_Write_String(0,1,(uchar*)"input:          ");           //��ʾ����
                    BuzzerRingsNum(1);
                    for(i = 0; i<=PressNum ; i++){                               //��ʾ���룬��ʾ*
                        LCD_Write_Com(0X80 + 0X46 + i);
                        LCD_Write_Data('*');
                    }
                    InputData[PressNum] = keycode;
                    PressNum++;
                }else PressNum = 6;                                             //����λ������6ʱ���Բ���
            }
            break;
        case 10:
            if(RegFingerprint == 0 && Manager == 1){
                Register = !Register;
                Delete = 0;
                LCD_Write_Com(0X0C);
                if(Register == 1){
                    LCD_Write_String(0,0,(uchar*)"  Add IC MODE   ");
                }else InitDisplay = 1;
            }
            break;
        case 11:
            if(RegFingerprint == 0 && Manager == 1){
                Delete = !Delete;
                Register = 0;
                LCD_Write_Com(0X0C);
                if(Delete == 1){
                    LCD_Write_String(0,0,(uchar*)" Delete IC MODE   "); 
                }else InitDisplay = 1;
            }
            break;
        case 12:
            if(Register == 0 && Delete == 0 && Manager == 1){
                RegFingerprint = !RegFingerprint;
                if(RegFingerprint == 1){
                    Display_user_id();
                }else{
                    InitDisplay = 1;
                    Manager = 0;
                }
            }
            break;
        case 13:
            if(Register == 0 && Delete == 0 && RegFingerprint == 0){
                ResetPassword();
            }
            break;
        case 14:
            if(Register == 0 && Delete == 0 && RegFingerprint == 0){
                Cancel();
            }
            break;
        case 15:
            if(Register == 0 && Delete == 0 && RegFingerprint == 0){
                Ensure();
            }
            break;
    }
}

void finger_ctrl(void){
    u8 ret;
    if(PS_Sta == 1){
        if(RegFingerprint == 1){
            ret = AS608_Add_Fingerprint(user_id);
            if(ret == 0){
                user_id++;
                if(user_id > 9) user_id = 0;
                Display_user_id();
                BuzzerRingsNum(1);
            }
        }
        if(RegFingerprint == 0){
            unsigned short ret_userid = 0;
            
            ret_userid = AS608_Find_Fingerprint();
            if(ret_userid != 0xFFFE){
                LCD_Write_String(0,1,(uchar*)"      open      ");
                RELAY = 1;                                                      //�̵�������
                RELAY_TIME = 15;                                                //�̵�������15��
                pass = 1;                                                       //������ȷ��־
                BuzzerRingsNum(2);                                              //��ʾ��,������
            }else{
                ErrorCont++;
                LCD_Write_String(0,1,(uchar*)"      error     ");
                if(ErrorCont == 3){
                    do{
                          LCD_Write_String(0,1,(uchar*)"Keyboard Locked!");
                          RELAY=0;                                              //�ر���
                          BEEP = !BEEP;
                          delay_ms(55);
                    }while(1);
                }else{
                    BEEP = 1;
                    delay_ms(1000);
                    BEEP = 0;
                    LCD_Write_String(0,1,(uchar*)"password:       ");
                    LCD_Write_Com(0x80+0x40+9);
                    LCD_Write_Com(0x0F);                                        //�����˸
                }
            }
        }
    }
}

u8 RC522_SCAN(u8* BUF){
    if(Delete == 1||Register==1){
        static u8 i=0;
        switch(i){
            case 0:    LCD_Write_String(0,1,(uchar*)"      ()        "); break;
            case 1:    LCD_Write_String(0,1,(uchar*)"     (())       "); break;
            case 2:    LCD_Write_String(0,1,(uchar*)"    ((()))      "); break;
            case 3:    LCD_Write_String(0,1,(uchar*)"   (((())))     "); break;
            case 4:    LCD_Write_String(0,1,(uchar*)"  ((((()))))    "); break;
            case 5:    LCD_Write_String(0,1,(uchar*)"                "); break;
            default :  i = 0; break;
        }
        i++;
    }
    if(GET_PID(ucArray_ID) == MI_OK){
        sprintf((char*)BUF,"%02X%02X%02X%02X", ucArray_ID [ 0 ], ucArray_ID [ 1 ], ucArray_ID [ 2 ], ucArray_ID [ 3 ] );//��IC�������ַ�������ʽ������BUF��
        return 1;
    }
    return 0;
}

void COMPER_ID_MODE(void){
    if(RC522_SCAN(ID_BUF)){
        u8 i = 0;
        ICpass = 0;
        LCD_Write_Com(0x0C);                                                    //�رչ��
        LCD_Write_String(0,1,(uchar*)" IC:             ");
        LCD_Write_String(4,1,ID_BUF);                                           //��ʾ����
        BEEP = 1; //��������
        delay_ms(500);
        BEEP = 0;
        for(i=0 ; i<MAX_PEOPLE ; i++){
            READ_IC_NUM_FOR_FLASH(ID_TEMP_Buffer,i);                            //��ȡSTM32�ڲ�FLASH�洢�Ŀ���
            if(strstr((char*)ID_TEMP_Buffer,(char*)ID_BUF) != NULL ){           //����ƥ����ȷ
                LCD_Write_String(0,1,(uchar*)"      open      ");
                RELAY = 1;
                RELAY_TIME = 15;
                ICpass = 1;
                ErrorCont = 0;
                break;
            }else ICpass = 0;
        }
        if(ICpass == 0){
            ErrorCont++;
            LCD_Write_String(0,1,(uchar*)"      error     "); 
            if(ErrorCont==3){
                do{
                        LCD_Write_String(0,1,(uchar*)"Keyboard Locked!");
                        RELAY=0;                                                //�ر���
                        BEEP = !BEEP;
                        delay_ms(55);
                }while(1);
            }else{
                BEEP=1;
                delay_ms(1000);
                BEEP=0;
                LCD_Write_String(0,1,(uchar*)"password:       ");
                LCD_Write_Com(0x80+0x40+9);
                LCD_Write_Com(0x0F);                                            //�����˸
            }
            InitDisplay = 1;
        }
    }
}

void ADD_ID_MODE(void){
    if(Register == 1){
        if(RC522_SCAN(ID_BUF)){
            u8 i = 0;
            Register = 0;
            Manager = 0;
            LCD_Write_String(0,1,(uchar*)" IC:             ");
            LCD_Write_String(4,1,ID_BUF);
            BEEP = 1;
            delay_ms(500);
            BEEP = 0;
            for(i=0;i<MAX_PEOPLE ; i++){
                 READ_IC_NUM_FOR_FLASH(ID_TEMP_Buffer,i);
                 if(strstr((char*)ID_TEMP_Buffer,(char*)ID_BUF) != NULL)         //��⵽��ͬ�Ŀ�
                     break;
                 if(ID_TEMP_Buffer[8] == '0' )                                   //ID_TEMP_Buffer��8λ��0��˵����û�д洢���� XXXXXXXXD
                     break;  
            }
            delay_ms(1000);
            if(i == MAX_PEOPLE)    LCD_Write_String(0,1,(uchar*)"   memery full  ");
            else{
                sprintf((char*)ID_TEMP_Buffer,"%s%d",ID_BUF,1);                 //��IC���ſ�����ID_TEMP_Buffer�У���8λ�洢1��˵�����ű��洢
                WRITE_IC_NUM_TO_FLASH(ID_TEMP_Buffer,i);
                LCD_Write_String(0,1,(uchar*)"   save ok!  ");
            }
            delay_ms(1000); 
            InitDisplay = 1;
        }
    }
}

void DEL_ID_MODE(void){
    if(Delete == 1){
        if(RC522_SCAN(ID_BUF)){
            u8 i =0;
            Delete = 0;
            Manager = 0;
            LCD_Write_String(0,1,(uchar*)" IC:             ");
            LCD_Write_String(4,1,ID_BUF);
            BEEP = 1;
            delay_ms(500);
            BEEP = 0;
            for(i=0 ; i<MAX_PEOPLE ; i++){
                READ_IC_NUM_FOR_FLASH(ID_TEMP_Buffer,i);
                if(strstr((char*)ID_TEMP_Buffer,(char*)ID_BUF) != NULL)
                    break;
            }
            delay_ms(1000);
            if(i == MAX_PEOPLE)    LCD_Write_String(0,1,(uchar*)"   find fail  ");
            else{
                sprintf((char*)ID_TEMP_Buffer,"%02X%02X%02X%02X%d",0,0,0,0,0);  //ID_TEMP_Buffer����ȫ��д��0
                WRITE_IC_NUM_TO_FLASH(ID_TEMP_Buffer,i);
                LCD_Write_String(0,1,(uchar*)"   Delete ok!     ");
            }
            delay_ms(1000);
            InitDisplay = 1;
        }
    }
}

