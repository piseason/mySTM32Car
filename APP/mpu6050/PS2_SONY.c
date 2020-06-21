
#include "SysTick.h"
#include "PS2_SONY.h"

u8 psx_buf[PSX_BUF_SIZE];                                       //按键数据
u8 deal_flag = 0;                           //处理标志
u8 release_flag = 0;                            //弹起标志
u8 PS2Count = 0;

/*
    PS2_DAT PB3         IN  DI
    PS2_CMD PB4         OUT DO
    PS2_ATT PB5         OUT CS
    PS2_CLK PB6         OUT
*/
void psx_io_config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);         //使能 PB 端口时钟
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //使能禁止JTAG,否则PB3&4不能置0
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;            //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;           //IO 翻转 50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
}

/*=========================================手柄码解析函数=======================================
函数名：psx_transfer
功能介绍：解析手柄传输的数据
函数参数：dat（手柄源码）
返回值：rd_data（接）
===============================================================================================*/
u8 psx_transfer(u8 dat) {
    u8 rd_data;
    volatile u16 i=0x01;
    rd_data = 0;
    for(i=0x01;i<0x0100;i<<=1){
        if(i&dat) PS2_CMD = 1;
        else PS2_CMD = 0;
    PS2_CLK = 1;
        delay_us(5);
        PS2_CLK = 0;
        delay_us(5);
        PS2_CLK = 1;
        if(PS2_DAT) {
            rd_data |= i;
        }
    }
    delay_us(16);
    return rd_data;
}

//short poll
void PS2_ShortPoll(void)
{
    PS2_ATT = 0;
    delay_us(16);
    psx_transfer(0x01);  
    psx_transfer(0x42);  
    psx_transfer(0X00);
    psx_transfer(0x00);
    psx_transfer(0x00);
    PS2_ATT = 1;
    delay_us(16);   
}
//进入配置
void PS2_EnterConfing(void)
{
    PS2_ATT = 0;
    delay_us(16);
    psx_transfer(0x01);  
    psx_transfer(0x43);  
    psx_transfer(0X00);
    psx_transfer(0x01);
    psx_transfer(0x00);
    psx_transfer(0X00);
    psx_transfer(0X00);
    psx_transfer(0X00);
    psx_transfer(0X00);
    PS2_ATT = 1;
    delay_us(16);
}
//发送模式设置
void PS2_TurnOnAnalogMode(void)
{
    PS2_ATT = 0;
    psx_transfer(0x01);  
    psx_transfer(0x44);  
    psx_transfer(0X00);
    psx_transfer(0x01); //analog=0x01;digital=0x00  软件设置发送模式
    psx_transfer(0x03); //Ox03锁存设置，即不可通过按键“MODE”设置模式。
                   //0xEE不锁存软件设置，可通过按键“MODE”设置模式。
    psx_transfer(0X00);
    psx_transfer(0X00);
    psx_transfer(0X00);
    psx_transfer(0X00);
    PS2_ATT = 1;
    delay_us(16);
}
//振动设置
void PS2_VibrationMode(void)
{
    PS2_ATT = 0;
    delay_us(16);
    psx_transfer(0x01);  
    psx_transfer(0x4D);  
    psx_transfer(0X00);
    psx_transfer(0x00);
    psx_transfer(0X01);
    PS2_ATT = 1;
    delay_us(16);   
}
//完成并保存配置
void PS2_ExitConfing(void)
{
    PS2_ATT = 0;
    delay_us(16);
    psx_transfer(0x01);  
    psx_transfer(0x43);  
    psx_transfer(0X00);
    psx_transfer(0x00);
    psx_transfer(0x5A);
    psx_transfer(0x5A);
    psx_transfer(0x5A);
    psx_transfer(0x5A);
    psx_transfer(0x5A);
    PS2_ATT = 1;
    delay_us(16);
}
/*=========================================手柄初始化============================================
函数名：psx_init
功能介绍：PS2引脚拉高
函数参数：无
返回值：无
===============================================================================================*/
void PSX_init(void) {
    psx_io_config();
//    PS2_ATT = 1;
//    PS2_CMD = 1;
//    PS2_CLK = 1;
    //PS2_DAT = 0;
    //PS2_ACK = 1;
    PS2_ShortPoll();
    //PS2_ShortPoll();
    //PS2_ShortPoll();
    //PS2_EnterConfing();       //进入配置模式
    //PS2_TurnOnAnalogMode();   //“红绿灯”配置模式，并选择是否保存
    //PS2_VibrationMode();  //开启震动模式
    //PS2_ExitConfing();        //完成并保存配置
    delay_ms(1000);
}

/*=========================================手柄原码获取函数=======================================
函数名：psx_write_read
功能介绍：解析并保存手柄传输的数据
函数参数：*get_buf 获取数据
返回值：无
===============================================================================================*/
void psx_write_read(u8 *get_buf) {
    
//  volatile u8 byte=0;
//  volatile u16 ref=0x01;
//  PS2_ATT = 0;
//  get_buf[0] = psx_transfer(START_CMD);  //开始命令
//  get_buf[1] = psx_transfer(ASK_DAT_CMD);  //请求数据
//  for(byte=2;byte<9;byte++)          //开始接受数据
//  {
//      for(ref=0x01;ref<0x100;ref<<=1)
//      {
//          PS2_CLK = 1;
//          delay_us(5);
//          PS2_CLK = 0;
//          delay_us(5);
//          PS2_CLK = 1;
//          if(PS2_DAT) get_buf[byte] |= ref;
//      }
//    delay_us(16);
//  }
//  PS2_ATT = 1;
    
    
    PS2_ATT = 0;
    get_buf[0] = psx_transfer(START_CMD);
    get_buf[1] = psx_transfer(ASK_DAT_CMD);
    get_buf[2] = psx_transfer(get_buf[0]);
    get_buf[3] = psx_transfer(get_buf[0]);
    get_buf[4] = psx_transfer(get_buf[0]);
    get_buf[5] = psx_transfer(get_buf[0]);
    get_buf[6] = psx_transfer(get_buf[0]);
    get_buf[7] = psx_transfer(get_buf[0]);
    get_buf[8] = psx_transfer(get_buf[0]);  
    PS2_ATT = 1;
    
    return;
}

//弹起后才能按别的
void getStateHold(void) {
    static u8 temp_alv, temp_alh, temp_arv, temp_arh;
    psx_write_read(psx_buf);//读取手柄
    if((psx_buf[1] == PSX_RED_MODE)||(psx_buf[1] == PSX_GREEN_MODE)) {                   //共用
        switch(psx_buf[3]) {               
            case 0xef: {if(deal_flag < 1) {release_flag = RELEASE_UP;deal_flag++;}break;}       //Up     
            case 0xbf: {if(deal_flag < 1) {release_flag = RELEASE_DOWN;deal_flag++;}break;}     //Down   
            case 0x7f: {if(deal_flag < 1) {release_flag = RELEASE_LEFT;deal_flag++;}break;}     //Left   
            case 0xdf: {if(deal_flag < 1) {release_flag = RELEASE_RIGHT;deal_flag++;}break;}    //Right   
            case 0xfe: {if(deal_flag < 1) {release_flag = RELEASE_SELECT;deal_flag++;}break;}   //Select 
            //红灯模式特有
            case 0xf7: {if(deal_flag < 1) {release_flag = RELEASE_START;deal_flag++;}break;}    //Start
            case 0xfd: {if(deal_flag < 1) {release_flag = RELEASE_L3;deal_flag++;}break;}       //L3
            case 0xfb: {if(deal_flag < 1) {release_flag = RELEASE_R3;deal_flag++;}break;}       //R3
            default: {
                deal_flag = 0;release_flag = 0;break;       //弹起
            }
        }   
        if(deal_flag) return;
        switch(psx_buf[4]) {                                   
            case 0xfb: {if(deal_flag < 1) {release_flag = RELEASE_L1;deal_flag++;}break;}        //L1     
            case 0xfe: {if(deal_flag < 1) {release_flag = RELEASE_L2;deal_flag++;}break;}        //L2     
            case 0xf7: {if(deal_flag < 1) {release_flag = RELEASE_R1;deal_flag++;}break;}        //R1     
            case 0xfd: {if(deal_flag < 1) {release_flag = RELEASE_R2;deal_flag++;}break;}        //R2     
            case 0xef: {if(deal_flag < 1) {release_flag = RELEASE_TRI;deal_flag++;}break;}       //Tri    
            case 0xbf: {if(deal_flag < 1) {release_flag = RELEASE_CROSS;deal_flag++;}break;}     //Cross  
            case 0x7f: {if(deal_flag < 1) {release_flag = RELEASE_RECT;deal_flag++;}break;}      //Rect   
            case 0xdf: {if(deal_flag < 1) {release_flag = RELEASE_CIRCLE;deal_flag++;}break;}    //Circle 
            default:{
                deal_flag = 0;release_flag = 0;break;       //弹起
            }
       }
    }   
    if(psx_buf[1] == PSX_RED_MODE) {                                                        //判断是否为红灯模式
        temp_alv = psx_buf[8];           //读取左摇杆垂直数据
        temp_arv = psx_buf[6];           //读取右摇杆垂直数据
        temp_alh = psx_buf[7];           //读取左摇杆水平数据
        temp_arh = psx_buf[5];           //读取右摇杆水平数据
        if (temp_alv<64) {if(deal_flag < 2) {release_flag = RELEASE_UP;deal_flag++;}}
        if (temp_alv>192) {if(deal_flag < 2) {release_flag = RELEASE_DOWN;deal_flag++;}}
        if (temp_alh<64) {if(deal_flag < 2) {release_flag = RELEASE_LEFT;deal_flag++;}}
        if (temp_alh>192) {if(deal_flag < 2) {release_flag = RELEASE_RIGHT;deal_flag++;}}
        if (temp_arv<64) {if(deal_flag < 2) {release_flag = RELEASE_TRI;deal_flag++;}}
        if (temp_arv>192) {if(deal_flag < 2) {release_flag = RELEASE_CROSS;deal_flag++;}}
        if (temp_arh<64) {if(deal_flag < 2) {release_flag = RELEASE_RECT;deal_flag++;}}
        if (temp_arh>192) {if(deal_flag < 2) {release_flag = RELEASE_CIRCLE;deal_flag++;}}
    }                                                       
    //return release_flag;    
}
