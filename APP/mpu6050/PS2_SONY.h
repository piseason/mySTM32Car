#ifndef __PS2_SONY_H__
#define __PS2_SONY_H__

#include "system.h"

#define PS2_DAT	    PBin(3)
#define PS2_CMD			PBout(4)
#define PS2_ATT			PBout(5)
#define PS2_CLK			PBout(6)

/***********************************宏定义*******************************************/
#define START_CMD           0x01      //手柄起始指令
#define ASK_DAT_CMD         0x42      //手柄应答指令
#define PSX_GREEN_MODE      0x41      //手柄绿灯模式对应码
#define PSX_RED_MODE        0x73      //手柄红灯模式对应码                                
#define PSX_BUF_SIZE        9         //手柄数组大小
#define CMD_RETURN_SIZE     50        //串口打印数组大小

#define       RELEASE_L1       0x01              //设置L1键弹起标志
#define       RELEASE_L2       0x02              //设置L2键弹起标志
#define       RELEASE_R1       0x03              //设置R1键弹起标志
#define       RELEASE_R2       0x04              //设置R2键弹起标志
#define       RELEASE_UP            0x05              //设置上键弹起标志
#define       RELEASE_DOWN          0x06              //设置下键弹起标志
#define       RELEASE_LEFT          0x07              //设置左键弹起标志
#define       RELEASE_RIGHT         0x08              //设置右键弹起标志
#define       RELEASE_TRI           0x09              //设置三角键弹起标志
#define       RELEASE_CROSS         0x10              //设置交叉键弹起标志
#define       RELEASE_RECT          0x11              //设置方块键弹起标志
#define       RELEASE_CIRCLE        0x12              //设置圆圈键弹起标志
#define       RELEASE_SELECT        0x13              //设置选择键弹起标志
#define       RELEASE_START         0x14              //设置开始键弹起标志
#define       RELEASE_L3       0x15              //设置L3键弹起标志
#define       RELEASE_R3       0x16              //设置R3键弹起标志

extern u8 PS2Count;
extern u8 release_flag;

void PSX_init(void);		     //配置初始化
void getStateHold(void);		//弹起后才能按别的

#endif
