#include "pwm.h"

volatile u16 cntL = 0;
volatile u16 cntR = 0;

/*******************************************************************************
* 函 数 名         : TIM1_CH1~4_PWM_Init
* 函数功能		   : TIM1通道1~4 PWM初始化函数
* 输    入         : arr:重装载值
					 psc:分频系数
* 输    出         : 无
*******************************************************************************/
void TIM1_PWM_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);        //使能定时器1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 		//使能GPIO外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	//设置该引脚为复用输出功能,输出TIM1 CH1234的PWM脉冲波形        GPIOA 8 9 10 11
    GPIO_InitStructure.GPIO_Pin = PWM_PIN_TIM1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 		//初始化GPIO
    GPIO_SetBits(GPIOA, PWM_PIN_TIM1);

    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);//改变指定管脚的映射	

    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM1 Channel1~4 PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_Low //输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC1
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC3
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC4

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR2上的预装载寄存器
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR3上的预装载寄存器
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR4上的预装载寄存器

    TIM_Cmd(TIM1, ENABLE);  //使能TIM1
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void TIM3_PWM_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);        //使能定时器1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);       //使能GPIO外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

    //设置该引脚为复用输出功能,输出TIM3 CH1234的PWM脉冲波形        GPIOA 6 7 GPIOB 0 1
    GPIO_InitStructure.GPIO_Pin = PWM_PIN_TIM3_A;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);       //初始化GPIOA
    //GPIO_SetBits(GPIOA, PWM_PIN_TIM3_A);

    GPIO_InitStructure.GPIO_Pin = PWM_PIN_TIM3_B;
    GPIO_Init(GPIOB, &GPIO_InitStructure);       //初始化GPIOB
    //GPIO_SetBits(GPIOB, PWM_PIN_TIM3_B);

    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);//改变指定管脚的映射   

    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM1 Channel1~4 PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_Low //输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC1
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC3
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC4

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR3上的预装载寄存器
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR4上的预装载寄存器
    TIM_ARRPreloadConfig(TIM3,ENABLE);//使能预装载寄存器

    TIM_Cmd(TIM3, ENABLE);  //使能TIM3
}

void TIM2_CH12_PWM_Init(u16 arr,u16 psc){
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);        //使能定时器2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);       //使能GPIO外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

    //设置该引脚为复用输出功能,输出TIM2 CH12的PWM脉冲波形        GPIOA  0 1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);       //初始化GPIOA


    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);//改变指定管脚的映射   

    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM2 Channel1~2 PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_Low //输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2 OC1
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2 OC2

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
    TIM_ARRPreloadConfig(TIM2,ENABLE);//使能预装载寄存器

    TIM_Cmd(TIM2, ENABLE);  //使能TIM2
}

void TIM2_CH34_Input_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 开启TIMx_CLK,x[1,8] 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;//管脚设置
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;	 //设置下拉输入模式
	GPIO_Init(GPIOA,&GPIO_InitStructure); 	   /* 初始化GPIO */
	
  //TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF-1; 	
	// 定时器时钟源TIMxCLK = HCLK=72MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10KHz
  //TIM_TimeBaseInitStructure.TIM_Prescaler = 7200-1;	
  // 计数方式
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //自动装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //分频系数
  TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	// 初始化定时器TIMx, x[1,8]
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	/* IC1捕获：上升沿触发 TI1FP1 */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0xF;		//重要	否则数据非常飘
  TIM_ICInit(TIM2,&TIM_ICInitStructure);
	TIM_ITConfig(TIM2,TIM_IT_CC3,ENABLE);
	
	/* IC2捕获：下降沿触发 TI1FP2 */	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0xF;			//重要	否则数据非常飘
  TIM_ICInit(TIM2,&TIM_ICInitStructure);
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4,ENABLE);
	//TIM_ARRPreloadConfig(TIM2,ENABLE);//使能预装载寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);

  /* 使能定时器 */
  //TIM_Cmd(TIM2, ENABLE);
}

void TIM4_CH12_PWM_Init(u16 arr,u16 psc){
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);        //使能定时器2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);       //使能GPIO外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

    //设置该引脚为复用输出功能,输出TIM2 CH12的PWM脉冲波形        GPIOB  6 7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);       //初始化GPIOA


    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);//改变指定管脚的映射   

    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM2 Channel1~2 PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_Low //输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2 OC1
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2 OC2

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
    TIM_ARRPreloadConfig(TIM4,ENABLE);//使能预装载寄存器

    TIM_Cmd(TIM4, ENABLE);  //使能TIM2
}

void startSpeedCount(void){
  cntL = 0;
  cntR = 0;
  TIM_SetCounter(TIM2,0);
  TIM_Cmd(TIM2, ENABLE);
}

/*******************************************************************************
* 函 数 名         : TIM5_IRQHandler
* 函数功能		   : TIM5中断函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void TIM2_IRQHandler(void)
{
	/* 获取输入捕获值 */
	//IC3Value = TIM_GetCapture3(TIM2);
	//IC4Value = TIM_GetCapture4(TIM2);
	
	if(TIM_GetITStatus(TIM2,TIM_IT_CC3)) //发生捕获中断
	{
	  /* 清除定时器捕获中断 */
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
		cntL++;
	}
	if(TIM_GetITStatus(TIM2,TIM_IT_CC4)) //发生捕获中断
	{
	  /* 清除定时器捕获中断 */
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		cntR++;
	}
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)) //发生溢出中断
	{
	  /* 清除定时器溢出中断 */
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		TIM_Cmd(TIM2, DISABLE);
	}
	
//	/* 频率计算 */
//  if (IC3Value>IC3Value0)
//  {
//    FreqL = 2000/(float)(IC3Value-IC3Value0);
//		IC3Value0 = IC3Value;
//  }
//	else if (IC3Value<IC3Value0){
//		FreqL = 2000/(float)(0xFFFF-IC3Value0+IC3Value);
//		IC3Value0 = IC3Value;
//	}
//  else
//  {
//    FreqL = 0;
//  }

}

void TIM4_counter_Init(u16 arr,u16 psc)
{         
        GPIO_InitTypeDef GPIO_InitStructure;
        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
        TIM_ICInitTypeDef  TIM_ICInitStructure;
        NVIC_InitTypeDef NVIC_InitStructure;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,  ENABLE);          //使能TIM5时钟
         RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //使能GPIOA时钟
        
        GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;              //PA7 清除之前设置  
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //PA7 浮空输入  
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_ResetBits(GPIOB,GPIO_Pin_7);                                                              //PA7 下拉
        
        //初始化定时器5 TIM5         
        TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值
        TIM_TimeBaseStructure.TIM_Prescaler =psc;         //预分频器   
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
        TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
        //初始化TIM5输入捕获参数
        TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;                   //         选择输入端 IC1映射到TI1上
        TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;           //上升沿捕获
        TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
        TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;                             //配置输入分频,不分频
        TIM_ICInitStructure.TIM_ICFilter = 0x0F;                                                            //IC1F=0000 配置输入滤波器 不滤波
        TIM_ICInit(TIM4, &TIM_ICInitStructure);

        TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2); //选择IC2为始终触发源
        TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);//TIM从模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件
        TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable); //启动定时器的被动触发
        
        //中断分组初始化
        NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级2级
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道被使能
        NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
        
        TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC2,ENABLE);//不能允许更新中断 ,允许CC2IE捕获中断        
        
        TIM_Cmd(TIM4,ENABLE);         //使能定时器3
  
}

void TIM4_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM4, TIM_IT_CC2) == SET)  
        {
					cntL++;
          TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);             //清楚TIM的中断待处理位                                
				}
				if(TIM_GetITStatus(TIM4,TIM_IT_Update)) //发生溢出中断
				{
					TIM_Cmd(TIM2, DISABLE);
					TIM_ClearITPendingBit(TIM4,TIM_IT_Update);		/* 清除定时器溢出中断 */
				}
}
