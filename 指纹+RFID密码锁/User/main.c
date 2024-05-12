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
#define FLASH_SAVE_ADDR  0X08004096     //设置FLASH 保存地址(必须为偶数)
#define MAX_PEOPLE       5              //最大存储5张IC卡
#define SIZE             10             //写卡号的地址长度

uchar FlagKeyPress = 0;                 //是否有按键按下
uchar keycode;                          //按键值
uchar pass = 0;                         //密码是否正确标志
uchar ICpass = 0;                       //IC卡是否正确标志
uchar CurrentPassword[6]={0,0,0,0,0,0}; //存储当前密码
uchar InputData[6]={0,0,0,0,0,0};       //输入密码
uchar initpassword[6]={0,0,0,0,0,0};    //初始密码
uchar TempPassword[6];                  //重置密码缓存数组
uchar PressNum=0;                       //密码输入位数记数
uchar CorrectCont;                      //正确输入计数
uchar ReInputEn=0;                      //重置密码输入允许
uchar ReInputCont;                      //重新输入计数
uchar ErrorCont;                        //错误次数计数
uchar RELAY_TIME;                       //继电器开启时间
uchar InitDisplay=1;                    //返回主页面标志
uchar Register = 0;                     //注册卡标志
uchar Delete = 0;                       //删除卡标志
uchar RegFingerprint = 0;               //注册指纹标志
u8 ucArray_ID [ 4 ] ;                   //存放IC卡号
u8 ID_BUF[8];                           //扫描卡获取卡号
u8 ID_TEMP_Buffer[10];                  //ID_TEMP_Buffer注册过的卡号
uchar Manager = 0;                      //管理注册指纹，注册IC卡与删除IC卡
unsigned short user_id ;                //用户指纹ID

void CHECK_NEW_MCU(void);                                   //检查是否是新的单片机，是的话清空存储区，否则保留
void KeyPress(unsigned char code);                          //键盘功能判断
void BuzzerRingsNum(u8 num);                                //蜂鸣器控制
void Display_user_id(void);                                 //显示指纹用户id
void ResetPassword(void);                                   //重置密码
void DataInit(void);                                        //将输入密码清空
void WRITE_IC_NUM_TO_FLASH(u8* ID_Buffer,u8 space);         //IC卡号写入STM32内部FLASH
void READ_IC_NUM_FOR_FLASH(u8* ID_TEMP_Buffer ,u8 space);   //从STM32内部FLASH读出IC卡号
void WRITE_DATA_TO_FLASH(u8* ID_Buffer,u8 LEN);             //密码写入STM32内部FLASH
void READ_DATA_FOR_FLASH(u8* ID_TEMP_Buffer ,u8 LEN);       //从STM32内部FLASH读出密码
void Cancel(void);
void Ensure(void);
void finger_ctrl(void);
u8 RC522_SCAN(u8* BUF);                                     //扫描IC卡
void COMPER_ID_MODE(void);                                  //正常待机刷卡函数
void ADD_ID_MODE(void);                                     //注册IC函数
void DEL_ID_MODE(void);                                     //删除IC卡函数

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
    CHECK_NEW_MCU();                    //单片机校验
    PS_StaGPIO_Init();
    RC522_Init();
    PcdReset();                         //复位RC522 
    M500PcdConfigISOType('A');          //设置工作方式
    uart1_Init(57600);
    TIM2_Init(999 , 719);               //以100Hz计数，定时10ms
    while(1){
        if(InitDisplay==1){
            InitDisplay = 0;
            BEEP = 0;
            DataInit();
            CorrectCont=0;              //正确计数器清零
            RELAY = 0;                  //继电器关闭
            LCD_Write_String(0,0,(uchar*)"===Coded Lock===");
            LCD_Write_String(0,1,(uchar*)"password:       ");
            LCD_Write_Com(0x80+0x40+9);
            LCD_Write_Com(0x0F);        //光标闪烁
        }
        finger_ctrl();                  //指纹处理函数
        if(Register == 0&&Delete == 0)
        COMPER_ID_MODE();
        ADD_ID_MODE();
        DEL_ID_MODE();
        keycode = Key_Scan();
        if ((keycode<16)&&(FlagKeyPress==0)){
            FlagKeyPress = 1;           //每次只处理一个按键
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
    if(strstr((char*)comper_str,"FDYDZ") == NULL){          //新的单片机
        WRITE_DATA_TO_FLASH(initpassword,6);
        for(i = 0; i < MAX_PEOPLE; i++){
            WRITE_IC_NUM_TO_FLASH(clear_str,i);             //存IC的地址内容全部清零
        }
        STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)"FDYDZ",5);    //写入“FDYDZ”，方便下次校验
    }
    READ_DATA_FOR_FLASH(CurrentPassword,6);                 //从STM32内部FLASH里读出存储密码
}

void DataInit(void){
    uchar i =0;                         //密码输入位数记数
    PressNum = 0;                       //密码输入位数记数
    for(i=0;i<6;i++)InputData[i]=0;     //密码暂存区清零
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
    LCD_Write_Data(user_id+0x30);       //OX30为显示数字的偏移量
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
        BuzzerRingsNum(3);              //蜂鸣器响3声
        LCD_Write_String(0,1,(uchar*)"      error     "); 
        delay_ms(1000);
        LCD_Write_String(0,1,(uchar*)"password:       ");
        LCD_Write_Com(0x80+0x40+9);
        LCD_Write_Com(0x0F);            //光标闪烁
        DataInit();                     //将输入数据计数器清零
    }else{
        if(ReInputEn == 1){
            if(PressNum == 6){
                ReInputCont++;
                if(ReInputCont == 2){
                    if((TempPassword[0]==InputData[0])&&(TempPassword[1]==InputData[1])&&(TempPassword[2]==InputData[2])&&  
                        (TempPassword[3]==InputData[3])&&(TempPassword[4]==InputData[4])&&(TempPassword[5]==InputData[5])){  //将两次输入的新密码作对比
                            LCD_Write_String(0,1,(uchar*)"ResetPasswordOK ");
                            BuzzerRingsNum(2);
                            WRITE_DATA_TO_FLASH(TempPassword,6);                // 将新密码写入STM32内部FLASH
                            delay_ms(100);
                            READ_DATA_FOR_FLASH(CurrentPassword,6);             //从STM32内部FLASH里读出存储密码
                            delay_ms(1000);
                            LCD_Write_String(0,1,(uchar*)"password:       ");
                            LCD_Write_Com(0x80+0x40+9);
                            LCD_Write_Com(0x0F);                                //光标闪烁
                    }else{                                                      //两次输入的密码不一致
                        BuzzerRingsNum(3);
                        LCD_Write_String(0,1,(uchar*)"      error     "); 
                        delay_ms(1000);
                        LCD_Write_String(0,1,(uchar*)"password:       ");
                        LCD_Write_Com(0x80+0x40+9);
                        LCD_Write_Com(0x0F);                                    //光标闪烁
                    }
                    ReInputEn = 0;                                              //关闭重置功能
                    ReInputCont = 0;
                    CorrectCont = 0;
                }else{
                    BuzzerRingsNum(1);
                    LCD_Write_String(0,1,(uchar*)"input again     ");           //提示你再次输入
                    for(i=0 ; i<6 ;i++){
                        TempPassword[i]=InputData[i];                           //将第一次输入的数据暂存起来
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
    BuzzerRingsNum(2);                                                          //提示音,响两声
    for(i=0;i<6;i++)InputData[i]=0;                                             //密码暂存区清零
    RELAY=0;                                                                    //关闭锁
    BEEP =0;                                                                    //报警关
    pass=0;                                                                     //密码正确标志清零
    ReInputEn=0;                                                                //重置输入充许标志清零
    ErrorCont=0;                                                                //密码错误输入次数清零
    CorrectCont=0;                                                              //密码正确输入次数清零
    ReInputCont=0;                                                              //重置密码输入次数清零 
    PressNum=0;                                                                 //输入位数计数器清零
    ICpass = 0;
    InitDisplay=1;
}

void Ensure(void){
    if(PressNum == 6){
        if((InputData[0]==0)&&(InputData[1]==3)&&(InputData[2]==1)&&(InputData[3]==4)&&(InputData[4]==0)&&(InputData[5]==6)){           //031406初始化密码
            WRITE_DATA_TO_FLASH(initpassword , 6);
            delay_ms(100);
            READ_DATA_FOR_FLASH(CurrentPassword , 6);
            LCD_Write_String(0,1,(uchar*)"Init password...");
            BEEP = 1;
            delay_ms(1000);
            BEEP = 0;
            LCD_Write_String(0,1,(uchar*)"password:       ");
            LCD_Write_Com(0x80+0x40+9);
            LCD_Write_Com(0x0F);                                                //光标闪烁
        }else if((InputData[0]==0)&&(InputData[1]==3)&&(InputData[2]==1)&&(InputData[3]==4)&&(InputData[4]==0)&&(InputData[5]==7)){     //031407进入管理员模式，注册卡删除卡注册指纹
            LCD_Write_String(0,1,(uchar*)"                ");
            LCD_Write_String(0,2,(uchar*)"    manager     ");
            BuzzerRingsNum(2);
            Manager = 1;
        }else if((InputData[0]==0)&&(InputData[1]==3)&&(InputData[2]==1)&&(InputData[3]==4)&&(InputData[4]==0)&&(InputData[5]==8)){     //031408指纹清空
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
                if(CorrectCont == 1){                                           //当只有一次正确输入时，开锁
                    LCD_Write_String(0,1,(uchar*)"      open      ");
                    RELAY = 1;                                                  //继电器开
                    RELAY_TIME = 15;                                            //开启15秒
                    pass = 1;                                                   //密码正确标志
                    BuzzerRingsNum(2);
                }else{                                                          //当两次正确输入时，代表开启了重置密码功能
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
                          RELAY=0;                                              //关闭锁
                          BEEP = !BEEP;
                          delay_ms(55);
                    }while(1);
                }else{
                    BEEP = 1;
                    delay_ms(1000);
                    BEEP = 0;
                    LCD_Write_String(0,1,(uchar*)"password:       ");
                    LCD_Write_Com(0x80+0x40+9);
                    LCD_Write_Com(0x0F);                                        //光标闪烁
                }
            }
        }
        DataInit();
    }
}


//按键响应程序，参数是键值
//返回键值：
//         7       8      9      10(A)//注册IC卡
//         4       5      6      11(B)//删除IC卡
//         1       2      3      12(C)//注册指纹
//        14(返回) 0   15(确定)  13(D)//修改密码
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
            if(Register==0&&Delete==0&&RegFingerprint==0){                      //没有其他事件输入时
                if(PressNum < 6){                                               //密码位数少于6时才进行输入
                    LCD_Write_String(0,1,(uchar*)"input:          ");           //显示输入
                    BuzzerRingsNum(1);
                    for(i = 0; i<=PressNum ; i++){                               //显示密码，显示*
                        LCD_Write_Com(0X80 + 0X46 + i);
                        LCD_Write_Data('*');
                    }
                    InputData[PressNum] = keycode;
                    PressNum++;
                }else PressNum = 6;                                             //输入位数大于6时忽略不计
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
                RELAY = 1;                                                      //继电器开启
                RELAY_TIME = 15;                                                //继电器开启15秒
                pass = 1;                                                       //密码正确标志
                BuzzerRingsNum(2);                                              //提示音,响两声
            }else{
                ErrorCont++;
                LCD_Write_String(0,1,(uchar*)"      error     ");
                if(ErrorCont == 3){
                    do{
                          LCD_Write_String(0,1,(uchar*)"Keyboard Locked!");
                          RELAY=0;                                              //关闭锁
                          BEEP = !BEEP;
                          delay_ms(55);
                    }while(1);
                }else{
                    BEEP = 1;
                    delay_ms(1000);
                    BEEP = 0;
                    LCD_Write_String(0,1,(uchar*)"password:       ");
                    LCD_Write_Com(0x80+0x40+9);
                    LCD_Write_Com(0x0F);                                        //光标闪烁
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
        sprintf((char*)BUF,"%02X%02X%02X%02X", ucArray_ID [ 0 ], ucArray_ID [ 1 ], ucArray_ID [ 2 ], ucArray_ID [ 3 ] );//把IC卡号以字符串的形式拷贝到BUF中
        return 1;
    }
    return 0;
}

void COMPER_ID_MODE(void){
    if(RC522_SCAN(ID_BUF)){
        u8 i = 0;
        ICpass = 0;
        LCD_Write_Com(0x0C);                                                    //关闭光标
        LCD_Write_String(0,1,(uchar*)" IC:             ");
        LCD_Write_String(4,1,ID_BUF);                                           //显示卡号
        BEEP = 1; //蜂鸣器响
        delay_ms(500);
        BEEP = 0;
        for(i=0 ; i<MAX_PEOPLE ; i++){
            READ_IC_NUM_FOR_FLASH(ID_TEMP_Buffer,i);                            //读取STM32内部FLASH存储的卡号
            if(strstr((char*)ID_TEMP_Buffer,(char*)ID_BUF) != NULL ){           //查找匹配正确
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
                        RELAY=0;                                                //关闭锁
                        BEEP = !BEEP;
                        delay_ms(55);
                }while(1);
            }else{
                BEEP=1;
                delay_ms(1000);
                BEEP=0;
                LCD_Write_String(0,1,(uchar*)"password:       ");
                LCD_Write_Com(0x80+0x40+9);
                LCD_Write_Com(0x0F);                                            //光标闪烁
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
                 if(strstr((char*)ID_TEMP_Buffer,(char*)ID_BUF) != NULL)         //检测到相同的卡
                     break;
                 if(ID_TEMP_Buffer[8] == '0' )                                   //ID_TEMP_Buffer第8位是0，说明是没有存储过的 XXXXXXXXD
                     break;  
            }
            delay_ms(1000);
            if(i == MAX_PEOPLE)    LCD_Write_String(0,1,(uchar*)"   memery full  ");
            else{
                sprintf((char*)ID_TEMP_Buffer,"%s%d",ID_BUF,1);                 //把IC卡号拷贝到ID_TEMP_Buffer中，第8位存储1，说明卡号被存储
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
                sprintf((char*)ID_TEMP_Buffer,"%02X%02X%02X%02X%d",0,0,0,0,0);  //ID_TEMP_Buffer缓存全部写入0
                WRITE_IC_NUM_TO_FLASH(ID_TEMP_Buffer,i);
                LCD_Write_String(0,1,(uchar*)"   Delete ok!     ");
            }
            delay_ms(1000);
            InitDisplay = 1;
        }
    }
}

