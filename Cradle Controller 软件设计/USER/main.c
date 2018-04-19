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
	/*****************��������*******************/
	u8 t=0;																//����ͳ��ÿʮ�θ���PWM��
	u8 report=1;													//Ĭ�Ͽ����ϱ������ڴ��޸��Ƿ�Ĭ���ϱ���
  float pitch = 0,roll = 0,yaw = 0; 		//ŷ����
	float Pitch = 0,Roll = 0,Yaw = 0;			//ÿʮ��ŷ����֮�����ڼ��˲�
	short aacx,aacy,aacz;									//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;							//������ԭʼ����  
	u16 temp;  
	u8 dir=1;	
	u8 i;
	 
	/*****************��ʼ���׶�*****************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����жϷ���
	delay_init();	    	            		//��ʱ������ʼ��	
	uart_init(500000);	 								//���ڳ�ʼ��Ϊ9600	 
	TIM2_PWM_Init(10000-1,144-1);   	 	//PWMƵ��=72000k/(10000)/144=50Hz 
	MPU_Init();													//��ʼ��MPU6050
	LED_Init();					            		//��ʼ��LED
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);	//�������Ʊ�����ʼ����� Ӳ����ת
	GPIO_ResetBits(GPIOB, GPIO_Pin_15); //������Ʊ���MPU6050δ����
  GPIO_ResetBits(GPIOB, GPIO_Pin_14); 
	/****************MPU6050�Լ�*****************/
	//���m6050�Ƿ������������������������while
  //����mpu_dmp_init�Ĳ������ݽ���
	//��Ӱ�����ŷ���ǵĶ���		�൱��Ҫ
	//1---��ʾ����ʼλ�ÿ�ʼ    ��ʼŷ����Ϊ0
	//0---��ʾ�Եر�Ϊ׼        ��ʼŷ���ǲ�Ϊ0
	/*****************Ӳ���Լ�*******************/
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
	
	while(mpu_dmp_init(0))				              //����Ϊ0
	{
		for(i = mpu_dmp_init(0); i > 0; i--)
		{
		delay_ms(200);														//��ʱ200ms
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);					//�����˸����MPU6050δ��ʼ�����
		delay_ms(200);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		delay_ms(1000);	
		}
		
 	}
		delay_ms(1000);
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);	
	/******************ִ�г���******************/
	 while(1)
	{
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0)
		{
			
		}
		GPIO_SetBits(GPIOB, GPIO_Pin_15);					//�����رպ��
		
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			
		if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡��������λ�����ͼ��ٶȺ�������ԭʼ����
		if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			
		Pitch += pitch;						//����  10�����
		Roll += roll;             //���
		Yaw += yaw;								//ƫ��
			
		t++;
		if(t % 5 == 0)           //10�θ���
		{

			Pitch /= 5.0f;					//ȡƽ��ֵ
			Roll /= 5.0f;
			Yaw /= 5.0f;
			
			//��ֹ���ػ���
			if(Pitch >= 90.0f)Pitch =  90.0f;else if(Pitch <= -90.0f)Pitch = -90.0f;
			if(Roll >= 90.0f)Roll =  90.0f;else if(Roll <= -90.0f)Roll = -90.0f;			
			if(Yaw >= 90.0f)Yaw =  90.0f;else if(Yaw <= -90.0f)Yaw = -90.0f;
			
			temp = 750 + (int)(Pitch * 500.0f/180.0f); //��̨����
			TIM_SetCompare1(TIM2,temp);
				
			temp = 750+ (int)(Yaw * 500.0f/180.0f);		//��̨ƫ��
			TIM_SetCompare2(TIM2,temp);
				
			if(Pitch >= 0)																//����
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

