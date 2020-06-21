
#include "system.h"
#include "SysTick.h"
#include "iompu6050.h"

//初始化IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD ;   //推挽输出为0xD1,读数不变
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	IIC_SCL=1;
	IIC_SDA=1;
 
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：0，接收应答失败
//        1，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 0;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 1;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}	
/*********************************通用**************************************/				 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
  	for(i=0;i<8;i++ )
	{
	    IIC_SCL=0; 
	    delay_us(2);
		IIC_SCL=1;
	    receive<<=1;
	    if(READ_SDA)receive++;   
		delay_us(1); 
	}					 
	if (!ack)
		IIC_NAck();//发送nACK
	else
		IIC_Ack(); //发送ACK   
	return receive;
}
/***********************************从机*************************************/	
//IIC发送给从机一个字节
//addr:寄存器地址
//data:数据
//返回值:1,成功
//    	0,失败
u8 I2C_WriteByte(u8 addr, u8 data, u8 device_addr)
{
	IIC_Start();  

	IIC_Send_Byte(device_addr);	    //发器件地址
	if(!IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 0;		
	}
	IIC_Send_Byte(addr);   //发送低地址
	IIC_Wait_Ack(); 
	IIC_Send_Byte(data);     //发送字节							   
  	if(!IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 0;		
	} 		    	   
  	IIC_Stop();//产生一个停止条件 
	delay_us(2);
	return 1;
}
 
//IIC接收来自从机的一个字节
//addr:寄存器地址
//返回值:1,成功
//    	0,失败
u8 I2C_ReadByte(u8 addr, u8 device_addr)  //读寄存器或读数据
{	
	u8 data=0;
	IIC_Start();  
	IIC_Send_Byte(device_addr);	
  	if(!IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 0;		
	} 
	IIC_Send_Byte(addr);   //发送低地址
	IIC_Wait_Ack(); 

	IIC_Start();  	
	IIC_Send_Byte(device_addr+1);	    //发器件地址
  	if(!IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 0;		
	} 

	data=IIC_Read_Byte(0);
	IIC_Stop();//产生一个停止条件	    
	return data;
}
//IIC发送给从机多个字节
//addr:寄存器地址
//len:字节数
//data:数据
//返回值:1,成功
//    	0,失败
u8 I2C_WriteBytes(u8 addr,u8 len,u8 *buf,u8 device_addr)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte(device_addr);//发送器件地址
	if(!IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 0;		
	}
    IIC_Send_Byte(addr);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(!IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	 
			return 0;		 
		}		
	}    
    IIC_Stop();	 
	return 1;	
} 
//IIC接收从机的多个字节
//addr:寄存器地址
//len:字节数
//data:数据
//返回值:1,成功
//    	0,失败
u8 I2C_ReadBytes(u8 addr,u8 len,u8 *buf,u8 device_addr)
{ 
	IIC_Start();  
	IIC_Send_Byte(device_addr);	
	if(!IIC_Wait_Ack())		//等待ACK
	{
		IIC_Stop();	 
		return 0;		 
	}
	IIC_Send_Byte(addr);   //发送低地址
	IIC_Wait_Ack(); 

	IIC_Start();  	
	IIC_Send_Byte(device_addr+1);	    //发器件地址
	if(!IIC_Wait_Ack())		//等待ACK
	{
		IIC_Stop();	 
		return 0;		 
	}

	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//产生一个停止条件 
	return 1;	
}


//初始化MPU6050
//返回值:1,成功
//    	0,失败
u8 IOMPU6050_Init(void)
{ 
	u8 res;
	IIC_Init();//初始化IIC总线
	MPU6050_Write_Byte(MPU6050_PWR_MGMT1_REG,0X80);	//复位MPU6050
    delay_ms(100);
	MPU6050_Write_Byte(MPU6050_PWR_MGMT1_REG,0X00);	//唤醒MPU6050
	MPU6050_Write_Byte(MPU6050_PWR_MGMT2_REG,0X00);	//no standby mode
	//MPU6050_Write_Byte(MPU6050_CFG_REG,0x06);		//低通滤波频率
	MPU6050_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU6050_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU6050_Set_Rate(125);						//设置采样率50Hz
	MPU6050_Write_Byte(MPU6050_INT_EN_REG,0X00);	//关闭所有中断
	MPU6050_Write_Byte(MPU6050_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU6050_Write_Byte(MPU6050_FIFO_EN_REG,0X00);	//关闭FIFO
	//MPU6050_Write_Byte(MPU6050_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	//MPU6050_Write_Byte(MPU6050_INTBP_CFG_REG,0X02);	//唤醒bypass en
	res=MPU6050_Read_Byte(MPU6050_DEVICE_ID_REG);
	if( res==(MPU6050_ADDR>>1) )//器件ID正确
	{
		//MPU6050_Write_Byte(MPU6050_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		//MPU6050_Write_Byte(MPU6050_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		//MPU6050_Set_Rate(50);						//设置采样率为50Hz
 	}else return 0;
	return 1;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:1,设置成功
//    	0,设置失败 
u8 MPU6050_Set_Gyro_Fsr(u8 fsr)
{
	return MPU6050_Write_Byte(MPU6050_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:1,设置成功
//    	0,设置失败 
u8 MPU6050_Set_Accel_Fsr(u8 fsr)
{
	return MPU6050_Write_Byte(MPU6050_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:1,设置成功
//    	0,设置失败 
u8 MPU6050_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU6050_Write_Byte(MPU6050_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:1,设置成功
//    	0,设置失败  
u8 MPU6050_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU6050_Write_Byte(MPU6050_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU6050_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU6050_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU6050_Read_Len(MPU6050_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:1,成功
//    	0,失败
u8 MPU6050_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU6050_Read_Len(MPU6050_GYRO_XOUTH_REG,6,&buf[0]);
	if(res==1)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:1,成功
//    	0,失败
u8 MPU6050_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU6050_Read_Len(MPU6050_ACCEL_XOUTH_REG,6,&buf[0]);
	if(res==1)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:1,成功
//    	0,失败
u8 MPU6050_Write_Len(u8 reg,u8 len,u8 *buf)
{
	u8 res;
	res=I2C_WriteBytes(reg, len, buf, MPU6050_ADDR);	 
	return res;
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:1,成功
//    	0,失败
u8 MPU6050_Read_Len(u8 reg,u8 len,u8 *buf)
{ 

	u8 res;
	res=I2C_ReadBytes(reg, len, buf, MPU6050_ADDR);	 
	return res;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:1,成功
//    	0,失败
u8 MPU6050_Write_Byte(u8 reg,u8 data) 				 
{ 
	u8 res;
	res=I2C_WriteByte(reg, data, MPU6050_ADDR);	 
	return res;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU6050_Read_Byte(u8 reg)
{
	u8 res;
	res=I2C_ReadByte(reg, MPU6050_ADDR);
	return res;		
}
