#include "stdlib.h"
#include "math.h"
#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "pwm.h"
#include "usart.h"
#include "iompu6050.h" 
#include "PS2_SONY.h"
#include "M5StickC.h"



//移植USB时候的头文件
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"

//*********************************************************
//USB虚拟串口初始化
//*********************************************************
void main_USB_Init()
{
    USART1_Init(9600);
    delay_ms(1800);
    USB_Port_Set(0);    //USB先断开
    delay_ms(700);
    USB_Port_Set(1);    //USB再次连接
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();
}

//*********************************************************
//测试用输入捕获来测速
//*********************************************************
int testGetSpeed()
{
    u16 times=0;
    float tmp;
    
    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
    LED_Init();
    main_USB_Init();



    TIM2_CH34_Input_Init(10*msSpeed, 7200-1); //计一个数0.1ms   计数频率为10000/s
    //TIM4_counter_Init(10*msSpeed, 7200-1);
    
    TIM3_PWM_Init(999, 72-1); //频率是1KHz
    TIM_SetCompare1(TIM3,999);
    TIM_SetCompare2(TIM3,0);
    TIM_SetCompare3(TIM3,0);
    TIM_SetCompare4(TIM3,999);
    
    while(1)
    {
        times++;
        if(times>320)
        {
            times=0;
            usb_printf("\r\nRestart...\r\n");
        }
        if(TIM2->CNT==0){   //计数完成
            tmp = 1000.0/msSpeed;
            usb_printf("Frequency = L:%f  R:%f\r\n",tmp*cntL,tmp*cntR);
            startSpeedCount();
        }
        if(times%20==0){
            tmp = times*3;
            TIM_SetCompare1(TIM3,tmp);
            TIM_SetCompare4(TIM3,tmp);
        } 
        if(times%10==0)led=!led;    //闪烁LED,提示系统正在运行.
        delay_ms(100);                  
    }   
}

int testMPU6050()
{
    u16 times=0;
    s16 G_X,G_Y,G_Z,W_X,W_Y,W_Z;
    //float gx,gy,gz,wx,wy,wz;
    
    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
    LED_Init();
    main_USB_Init();
    
    IOMPU6050_Init();   //软件IIC
    
    while(1)
    {
        times++;
        if(times%5==0){
            //mpu6050_get_data(&G_X, &G_Y, &G_Z, &W_X, &W_Y, &W_Z, &temperature);
              MPU6050_Get_Accelerometer(&G_X,&G_Y,&G_Z);    //得到加速度传感器数据
        MPU6050_Get_Gyroscope(&W_X,&W_Y,&W_Z);          //得到陀螺仪数据
//            G_X-=362;
//            G_Y-=270;
//            G_Z+=2308;
//            W_X+=7;
//            W_Y-=33;
//            W_Z+=19;

//            gx = G_X/16384.0;
//            gy = G_Y/16384.0;
//            gz = G_Z/16384.0;
//            wx = W_X/16.384;
//            wy = W_Y/16.384;
//            wz = W_Z/16.384;
                usb_printf("read:%5d %5d %5d %5d %5d %5d\r\n",G_X,G_Y,G_Z,W_X,W_Y,W_Z);
								//usb_printf("read:%5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\r\n",gx,gy,gz,wx,wy,wz);
        } 
        if(times%10==0)led=!led;    //闪烁LED,提示系统正在运行.
        delay_ms(100);                  
    }   
}

int testPS2()
{
    u16 times=0;
    
    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
    LED_Init();
    main_USB_Init();

    PSX_init();

    while(1)
    {
        times++;
        if(times%5==0){    
						
        getStateHold();
			  usb_printf("%d, Waiting for Data...\r\n",release_flag);
            switch(release_flag)
            {
                case(RELEASE_UP):
                    usb_printf("RELEASE_UP\r\n");          break;
                case(RELEASE_DOWN):
                    usb_printf("RELEASE_DOWN\r\n");        break;
                case(RELEASE_LEFT):
                    usb_printf("RELEASE_LEFT\r\n");       break;
                case(RELEASE_RIGHT):
                    usb_printf("RELEASE_RIGHT\r\n");         break; 
                case(RELEASE_TRI):
                    usb_printf("RELEASE_TRI\r\n");          break;
                case(RELEASE_CROSS):
                    usb_printf("RELEASE_CROSS\r\n");        break;
                case(RELEASE_SELECT):
                    usb_printf("RELEASE_SELECT\r\n");       break;
                case(RELEASE_RECT):
                    usb_printf("RELEASE_RECT\r\n");         break;  
                case(RELEASE_CIRCLE):
                    usb_printf("RELEASE_CIRCLE\r\n");       break;  
                case(RELEASE_R1):
                    usb_printf("RELEASE_R1\r\n");           break;
                case(RELEASE_L1):
                    usb_printf("RELEASE_L1\r\n");           break;
                case(RELEASE_L2):
                    usb_printf("RELEASE_L2\r\n");           break;
                case(RELEASE_R2):
                    usb_printf("RELEASE_R2\r\n");           break;
                default:{
                    usb_printf("%d\r\n",release_flag);           break;
                }
            }
        } 
        if(times%100==0)led=!led;    //闪烁LED,提示系统正在运行.
        delay_ms(10);                   
    }       
}

void updatePara(float *paras, u8 len){
    u16 t,i=0,tmp=0;
    for (t = 0; t < USB_USART_RX_STA; ++t){
        if (USB_USART_RX_BUF[t]==',')
        {
            USB_USART_RX_BUF[t]='\0';
            paras[i]=atof((const char*)(USB_USART_RX_BUF+tmp));
            tmp=t+1;i++;if (i>len) return;
        }        
    }
    if (tmp<USB_USART_RX_STA)   //处理最后没有分隔符的数据
    {
        USB_USART_RX_BUF[USB_USART_RX_STA]='\0';
        paras[i]=atof((const char*)(USB_USART_RX_BUF+tmp));
    }
	
}

#define paraNum 5

int testM5StickC()
{
    u16 times=0;
    float paras[paraNum]; //Kp,Ki,Kd,Vmin,Vmax
		float tmp,Vdata,power = 0.0;
		
		paras[0] = 0.4;
		paras[1] = 0.01;
		paras[2] = 0.2; 
		paras[3] = 3;
		paras[4] = 5;
	
    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
    LED_Init(); 
		main_USB_Init();
    delay_ms(100);
	
    IOMPU6050_Init();   //软件IIC
    delay_ms(100);
    ini_theta();      //在电机上电之前拿到测量偏差和统计信息
		delay_ms(100);
		led=0;						//点亮LED,提示系统初始化完成
	
    TIM3_PWM_Init(999, 72-1); //频率是1KHz
    TIM_SetCompare1(TIM3,0);
    TIM_SetCompare2(TIM3,0);
    TIM_SetCompare3(TIM3,0);
    TIM_SetCompare4(TIM3,0);

    while(1)
    {
				//USB通信
        if(USB_USART_RX_STA)
        {            
            updatePara(paras,paraNum);
            USB_USART_RX_STA=0;
        }
			  else
        {
					if(times%100==0) usb_printf("Kp=%7.3f,Ki=%7.3f,Kd=%7.3f,Vmin=%7.3f,Vmax=%7.3f\r\n",paras[0],paras[1],paras[2],paras[3],paras[4]); 
				}  
				
        //PID控制
        update_theta();
				if(fabs(thetaP) > 0.5){
						power=0;
						thetaI=0;
				}
				else{
						power = thetaP * paras[0] + thetaI * paras[1] + thetaD * paras[2];
						power = fmaxf(-1.0, fminf(1.0, power));
				}
				Vdata = ((paras[4] - paras[3]) * fabs(power)) + paras[3];
			  tmp = Vdata*100;
				if (power>0)
				{   
						TIM_SetCompare1(TIM3,(u16)tmp);     //前进
						TIM_SetCompare2(TIM3,0);
						TIM_SetCompare3(TIM3,0);
						TIM_SetCompare4(TIM3,(u16)tmp);
				}
				else if (power<0)
				{
						TIM_SetCompare1(TIM3,0);    //后退
						TIM_SetCompare2(TIM3,(u16)tmp);
						TIM_SetCompare3(TIM3,(u16)tmp);
						TIM_SetCompare4(TIM3,0);
				}
				else{
						TIM_SetCompare1(TIM3,0);        //停止
						TIM_SetCompare2(TIM3,0);
						TIM_SetCompare3(TIM3,0);
						TIM_SetCompare4(TIM3,0);
				}
				
				//分频
				times++;
				if(times%100==0) led=!led;    //闪烁LED,提示系统正在运行.
				delay_ms(10);   
                 
    }   
}

int main(){
    testGetSpeed();
    testMPU6050();
		testPS2();
		testM5StickC();
}
