#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "pwm.h"
#include "led.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "mpu6050_usart.h"

 int main(void)
 {	
	/*****************变量定义*******************/
	u8 t=0;																//次数统计每十次更新PWM波
	u8 report=1;													//默认开启上报（可在此修改是否默认上报）
  float pitch = 0,roll = 0,yaw = 0; 		//欧拉角
	float Pitch = 0,Roll = 0,Yaw = 0;			//每十次欧拉角之和用于简单滤波
	short aacx,aacy,aacz;									//加速度传感器原始数据
	short gyrox,gyroy,gyroz;							//陀螺仪原始数据  
	u16 temp;  
	u8 dir=1;	
	u8 i;
	 
	/*****************初始化阶段*****************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断分组
	delay_init();	    	            		//延时函数初始化	
	uart_init(500000);	 								//串口初始化为9600	 
	TIM2_PWM_Init(10000-1,144-1);   	 	//PWM频率=72000k/(10000)/144=50Hz 
	MPU_Init();													//初始化MPU6050
	LED_Init();					            		//初始化LED
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);	//点亮蓝灯表明初始化完成 硬件运转
	GPIO_ResetBits(GPIOB, GPIO_Pin_15); //点亮红灯表明MPU6050未工作
  GPIO_ResetBits(GPIOB, GPIO_Pin_14); 
	/****************MPU6050自检*****************/
	//检查m6050是否运作正常，正常则会跳过该while
  //并且mpu_dmp_init的参数传递将会
	//会影响后续欧拉角的读数		相当重要
	//1---表示从起始位置开始    起始欧拉角为0
	//0---表示以地标为准        起始欧拉角不为0
	/*****************硬件自检*******************/
	TIM_SetCompare3(TIM2,500);
	for(temp = 500; temp < 1000; temp++)
	{
		TIM_SetCompare1(TIM2,temp);
		TIM_SetCompare2(TIM2,temp);
		delay_ms(5);
	}
	for(temp = 1000; temp > 750; temp--)
	{
		TIM_SetCompare1(TIM2,temp);
		TIM_SetCompare2(TIM2,temp);
		delay_ms(5);
	}
	TIM_SetCompare1(TIM2,750);
  TIM_SetCompare2(TIM2,750);
	for(temp = 500; temp < 600; temp++)
	{
		TIM_SetCompare3(TIM2,temp);
		delay_ms(10);
	}
	for(temp = 600; temp > 500; temp--)
	{
		TIM_SetCompare3(TIM2,temp);
		delay_ms(10);
	}
	TIM_SetCompare3(TIM2,500);
	
	while(mpu_dmp_init(0))				              //设置为0
	{
		for(i = mpu_dmp_init(0); i > 0; i--)
		{
		delay_ms(200);														//延时200ms
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);					//红灯闪烁表明MPU6050未初始化完成
		delay_ms(200);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		delay_ms(1000);	
		}
		
 	}
		delay_ms(1000);
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);	
	/******************执行程序******************/
	 while(1)
	{
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0)
		{
			
		}
		GPIO_SetBits(GPIOB, GPIO_Pin_15);					//正常关闭红灯
		
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			
		if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧向匿名上位机发送加速度和陀螺仪原始数据
		if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			
		Pitch += pitch;						//俯仰  10组相加
		Roll += roll;             //横滚
		Yaw += yaw;								//偏航
			
		t++;
		if(t % 5 == 0)           //10次更新
		{

			Pitch /= 5.0f;					//取平均值
			Roll /= 5.0f;
			Yaw /= 5.0f;
			
			//防止过载机制
			if(Pitch >= 90.0f)Pitch =  90.0f;else if(Pitch <= -90.0f)Pitch = -90.0f;
			if(Roll >= 90.0f)Roll =  90.0f;else if(Roll <= -90.0f)Roll = -90.0f;			
			if(Yaw >= 90.0f)Yaw =  90.0f;else if(Yaw <= -90.0f)Yaw = -90.0f;
			
			temp = 750 + (int)(Pitch * 500.0f/180.0f); //云台俯仰
			TIM_SetCompare1(TIM2,temp);
				
			temp = 750+ (int)(Yaw * 500.0f/180.0f);		//云台偏航
			TIM_SetCompare2(TIM2,temp);
				
			if(Pitch >= 0)																//涵道
			{
				temp = (500.0f + (int)(Pitch * 250.0f/90.0f));
			}
			else
			{
				temp = (500.0f - (int)(Pitch * 250.0f/90.0f));
			}
			TIM_SetCompare3(TIM2,temp);

			if(dir)
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_14);
				dir = 0;
			}
			else
			{
				GPIO_ResetBits(GPIOB, GPIO_Pin_14);
				dir = 1;
			}
				
			t = 0;
			Pitch = 0;
			Roll = 0;
			Yaw = 0;		
		}	
	}
		

	
}

