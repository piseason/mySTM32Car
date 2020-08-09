
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
//测试PWM占空比对速度的影响
//*********************************************************
int testPWM()
{
    u16 len=0,times=0,t;
    u8 usbstatus=0;
    
    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
    LED_Init(); 
    main_USB_Init();

    TIM3_PWM_Init(999, 72-1); //频率是1KHz
    TIM_SetCompare1(TIM3,999);
    TIM_SetCompare2(TIM3,0);
    TIM_SetCompare3(TIM3,0);
    TIM_SetCompare4(TIM3,999);
    
    while(1)
    {
        if(usbstatus!=bDeviceState)//USB连接状态发生了改变.
        {
            usbstatus=bDeviceState;//记录新的状态
            if(usbstatus==CONFIGURED)
            {
                usb_printf("USB Connected\r\n");
            }
            else
            {
                usb_printf("USB disConnected\r\n");
            }
        }
        if(USB_USART_RX_STA&0x8000)
        {                      
            len=USB_USART_RX_STA&0x3FFF;//得到此次接收到的数据长度
            usb_printf("\r\nData Length:%d\r\n\r\n",len);
            for(t=0;t<len;t++)
            {
                USB_USART_SendData(USB_USART_RX_BUF[t]);//以字节方式,发送给USB 
            }
            usb_printf("\r\n\r\n");//插入换行
            USB_USART_RX_STA=0;
        }
        else
        {
            times++;
            if(times>320)
            {
                times=0;
                usb_printf("\r\nmySTM32Car, Waiting for Data...\r\n");
            }
            if(times%20==0){
                len=times*3;
                usb_printf("Duty Cycle:%d\r\n",len); 
                TIM_SetCompare1(TIM3,len);
                TIM_SetCompare4(TIM3,len);
            } 
            if(times%10==0)led=!led;    //闪烁LED,提示系统正在运行.
            delay_ms(100);   
        }                   
    }   
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
		usb_printf("DutyCycle = %f\r\n",tmp);
        } 
        if(times%10==0)led=!led;    //闪烁LED,提示系统正在运行.
        delay_ms(100);                  
    }   
}

int testMPU6050()
{
    u16 times=0;
    s16 G_X,G_Y,G_Z,W_X,W_Y,W_Z;
//    float gx,gy,gz,wx,wy,wz;
    
    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
    LED_Init();
    main_USB_Init();
    
//  iic_init();
//  delay_ms(100);
//  mpu6050_init();
    IOMPU6050_Init();   //硬件IIC会卡住
    
    while(1)
    {
        times++;
        if(times%5==0){
            //mpu6050_get_data(&G_X, &G_Y, &G_Z, &W_X, &W_Y, &W_Z, &temperature);
              MPU6050_Get_Accelerometer(&G_X,&G_Y,&G_Z);    //得到加速度传感器数据
                        MPU6050_Get_Gyroscope(&W_X,&W_Y,&W_Z);          //得到陀螺仪数据
//            G_X-=800;
//            G_Y-=60;
//            G_Z+=2128;
//            W_X+=72;
//            W_Y-=17;
//            W_Z+=6;

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
		
		paras[0] = 20;
		paras[1] = 1;
		paras[2] = 4; 
		paras[3] = 3;
		paras[4] = 9;
	
    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
    LED_Init(); 
		main_USB_Init();
    delay_ms(100);
	
    IOMPU6050_Init();   //硬件IIC会卡住
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
				Kp = paras[0];Ki = paras[1];Kd = paras[2];
				//USB通信
        if(USB_USART_RX_STA)
        {            
            updatePara(paras,paraNum);
            USB_USART_RX_STA=0;
        }
			  else
        {
					if(times%500==0) usb_printf("Kp=%7.3f,Ki=%7.3f,Kd=%7.3f,Vmin=%7.3f,Vmax=%7.3f,P=%7.3f,I=%7.3f,D=%7.3f,power=%7.3f\r\n",paras[0],paras[1],paras[2],paras[3],paras[4],thetaP,thetaI,thetaD,power); 
				}  
				
        //PID控制
        update_theta();
				if(fabs(thetaP) > 0.4){
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
						TIM_SetCompare1(TIM3,0);    //后退
						TIM_SetCompare2(TIM3,(u16)tmp);
						TIM_SetCompare3(TIM3,(u16)tmp);
						TIM_SetCompare4(TIM3,0);
				}
				else if (power<0)
				{
						TIM_SetCompare1(TIM3,(u16)tmp);     //前进
						TIM_SetCompare2(TIM3,0);
						TIM_SetCompare3(TIM3,0);
						TIM_SetCompare4(TIM3,(u16)tmp);
				}
				else{
						TIM_SetCompare1(TIM3,0);        //停止
						TIM_SetCompare2(TIM3,0);
						TIM_SetCompare3(TIM3,0);
						TIM_SetCompare4(TIM3,0);
				}
				
				//分频
				times++; 
				if(times%500==0){
					led=!led;    //闪烁LED,提示系统正在运行.
				}
				delay_ms(2);   
                 
    }   
}

int testMove()
{
    u16 times=0;
    float paras[paraNum+2]; //Kp,Ki,Kd,Vmin,Vmax,Speed,Turn
		float tmpL,tmpR,VdataL,VdataR,powerL=0,powerR=0;
		
		paras[0] = 100;
		paras[1] = 0.3;
		paras[2] = 3; 
		paras[3] = 2;
		paras[4] = 5;
	  paras[5] = 0;
	  paras[6] = 0;
	
    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
    LED_Init(); 
		main_USB_Init();
    delay_ms(100);
	
    IOMPU6050_Init();   //硬件IIC会卡住
    delay_ms(100);
    ini_theta();      //在上电之前拿到测量偏差和统计信息
		delay_ms(100);
		led=0;						//点亮LED,提示系统初始化完成
	
    TIM3_PWM_Init(999, 72-1); //频率是1KHz
    TIM_SetCompare1(TIM3,0);
    TIM_SetCompare2(TIM3,0);
    TIM_SetCompare3(TIM3,0);
    TIM_SetCompare4(TIM3,0);

    while(1)
    {
				Kp = paras[0];Ki = paras[1];Kd = paras[2];
				//USB通信
        if(USB_USART_RX_STA)
        {            
            updatePara(paras,paraNum);
            USB_USART_RX_STA=0;
        }
			  else
        {
					if(times%500==0) usb_printf("Kp=%7.3f,Ki=%7.3f,Kd=%7.3f,Vmin=%7.3f,Vmax=%7.3f,P=%7.3f,I=%7.3f,D=%7.3f,power=%7.3f\r\n",paras[0],paras[1],paras[2],paras[3],paras[4],thetaP,thetaI,thetaD,power); 
				}  
				
        //PID控制
        update_theta2(paras[5]);
				if(fabs(thetaP) > 0.5){
						powerL=0;powerR=0;
						thetaI=0;
				}
				else{
//						power = thetaP * paras[0] + thetaI * paras[1] + thetaD * paras[2];
//						power = fmaxf(-1.0, fminf(1.0, power));
					powerL = (thetaP-paras[6]) * paras[0] + thetaI * paras[1] + thetaD * paras[2];
					powerR = (thetaP+paras[6]) * paras[0] + thetaI * paras[1] + thetaD * paras[2];
					powerL = fmaxf(-1.0, fminf(1.0, powerL));
					powerR = fmaxf(-1.0, fminf(1.0, powerR));
				}
				VdataL = ((paras[4] - paras[3]) * fabs(powerL)) + paras[3];
				VdataR = ((paras[4] - paras[3]) * fabs(powerR)) + paras[3];
			  tmpL = VdataL*100;
				tmpR = VdataR*100;
				if (powerL>0)
				{   
						TIM_SetCompare1(TIM3,0);
						TIM_SetCompare2(TIM3,(u16)tmpL);
				}
				else if (powerL<0)
				{
						TIM_SetCompare1(TIM3,(u16)tmpL);
						TIM_SetCompare2(TIM3,0);
				}
				else{
						TIM_SetCompare1(TIM3,0);        //停止
						TIM_SetCompare2(TIM3,0);
				}
				if (powerR>0)
				{   
						TIM_SetCompare3(TIM3,0);
						TIM_SetCompare4(TIM3,(u16)tmpR);
				}
				else if (powerR<0)
				{
						TIM_SetCompare3(TIM3,(u16)tmpR);
						TIM_SetCompare4(TIM3,0);
				}
				else{
						TIM_SetCompare3(TIM3,0);        //停止
						TIM_SetCompare4(TIM3,0);
				}
				
				//分频
				times++;
				if(times%500==0){
					led=!led;    //闪烁LED,提示系统正在运行.
				}
				delay_ms(2);     
                 
    }   
}

#define paraNumEx 8
extern int16_t gyroX, gyroY, gyroZ;
int testOffsetTheta()
{
		float tmpWy;
    u16 times=0;						//控制角度					控制角度的累计
    float paras[paraNumEx+3]; //Kp,Ki,Kd,Vmin,Vmax,Kvp,Kvi,Kvd,shiftT,Speed,Turn
		float tmpL,tmpR,VdataL,VdataR,powerL=0,powerR=0;
		
		paras[0] = 100;
		paras[1] = 5.6;
		paras[2] = 2.8; 
		paras[3] = 1.5;
		paras[4] = 2.7;
		//速度PID
		paras[5] = 0;
		paras[6] = 0.2;
		paras[7] = 0;
	
	  paras[8] = 0;
	  paras[9] = 0;
		paras[10] = 0;
	
    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
    LED_Init(); 
		//main_USB_Init();
    //delay_ms(100);
	
    IOMPU6050_Init();   //硬件IIC会卡住
    delay_ms(100);
    ini_theta();      //在上电之前拿到测量偏差和统计信息
		delay_ms(100);
		led=0;						//点亮LED,提示系统初始化完成
	
		TIM3_PWM_Init(999, 144-1); //频率是500Hz, 降低频率可以降低声音, 由于主循环为500Hz, 这里也取PWM为500Hz
    TIM_SetCompare1(TIM3,0);
    TIM_SetCompare2(TIM3,0);
    TIM_SetCompare3(TIM3,0);
    TIM_SetCompare4(TIM3,0);

    while(1)
    {
				Kp = paras[0];Ki = paras[1];Kd = paras[2];
				Kvp = paras[5];Kvi = paras[6];Kvd = paras[7];
			  thetaV = paras[8];
				//USB通信
        if(USB_USART_RX_STA)
        {            
            updatePara(paras,paraNumEx);
            USB_USART_RX_STA=0;
        }
			  else
        {
					//if(times%500==0) usb_printf("Kp=%7.3f,Ki=%7.3f,Kd=%7.3f,Vmin=%7.3f,Vmax=%7.3f,P=%7.3f,I=%7.3f,D=%7.3f,powerL=%7.3f,powerR=%7.3f\r\n",paras[0],paras[1],paras[2],paras[3],paras[4],thetaP,thetaI,thetaD,powerL,powerR); 
				}  
				
        //PID控制
        update_thetaEx(paras[9]);	//ThetaITar
				if(fabs(thetaP) > 0.5){
						powerL=0;powerR=0;
						thetaI=0;thetaII=0;
					  thetaOffset=0;thetaV=0;
				}
				else{
					//角度PID, 控制thetaP
					if( abs(paras[10])<0.1/210 ) tmpWy = (gyroZ/65.536) /9000.0;
					else tmpWy = 0;
					powerL = (thetaP-paras[10]+tmpWy) * paras[0] + thetaI * paras[1] + thetaD * paras[2];
					powerR = (thetaP+paras[10]-tmpWy) * paras[0] + thetaI * paras[1] + thetaD * paras[2];
					powerL = fmaxf(-1.0, fminf(1.0, powerL));
					powerR = fmaxf(-1.0, fminf(1.0, powerR));
					
					//速度PID, 控制thetaI
					paras[8] = -(thetaI * paras[5] + thetaII * paras[6] + thetaP * paras[7]);
					paras[8] = fmaxf(-10.0, fminf(10.0, paras[8]));		//假定角度的偏差在10度之内
				}
				VdataL = ((paras[4] - paras[3]) * fabs(powerL)) + paras[3];
				VdataR = ((paras[4] - paras[3]) * fabs(powerR)) + paras[3];
			  tmpL = VdataL*100;
				tmpR = VdataR*100;
				if (powerL>0)
				{   
						TIM_SetCompare1(TIM3,0);
						TIM_SetCompare2(TIM3,(u16)tmpL);
				}
				else if (powerL<0)
				{
						TIM_SetCompare1(TIM3,(u16)tmpL);
						TIM_SetCompare2(TIM3,0);
				}
				else{
						TIM_SetCompare1(TIM3,0);        //停止
						TIM_SetCompare2(TIM3,0);
				}
				if (powerR>0)
				{   
						TIM_SetCompare3(TIM3,0);
						TIM_SetCompare4(TIM3,(u16)tmpR);
				}
				else if (powerR<0)
				{
						TIM_SetCompare3(TIM3,(u16)tmpR);
						TIM_SetCompare4(TIM3,0);
				}
				else{
						TIM_SetCompare3(TIM3,0);        //停止
						TIM_SetCompare4(TIM3,0);
				}
				
				//分频
				times++;
				if(times%10==0 && fabs(paras[9])<0.05){
					angleCorrection();
				}
				if(times%500==0){
					led=!led;    //闪烁LED,提示系统正在运行.
				}
				delay_ms(2);     
                 
    }   
}


int main(){
    //testPWM();
    //testGetSpeed();
    //testMPU6050();
    //testPS2();
    //testM5StickC();
	  //testMove();
		testOffsetTheta();
}
