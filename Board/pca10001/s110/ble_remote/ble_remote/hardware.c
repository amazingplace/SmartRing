#include "hardware.h"
#include "nrf_delay.h"
//uint8_t user_send[9];
//����3��ȫ�ֵĽṹ�壬���ڱ���������Ϣ��
shine_type led_config;
vibrate_type motor_config;
alarm_type clock_config;

//��ʼ�������ܵĽṹ������
void led_config_init(void)
{
	led_config.led_type = 'R';
	led_config.shine_mode = 0;//ģʽ0,��1s��һ��
	led_config.shine_times = 0;
}
void motor_config_init(void)
{	
	motor_config.motor_type = 0;//0Ĭ��LDA���͵�����
	motor_config.vibrate_mode = 0;
	motor_config.vitrate_times = 0;
}
void clock_config_init(void)
{
	clock_config.year = 2015;
	clock_config.month = 6;
	clock_config.day = 1;
	clock_config.hours = 23;
	clock_config.minutes = 59;
	clock_config.seconds = 59;
	clock_config.alarm_mode = 0;//ÿ������
	clock_config.alarm_time =  86399;//��������
	clock_config.alarm_way = 0;//������ʾ�ķ�ʽ��Ĭ��0������ʾ
}
//��Ϊ�û���ʾ����������LED��ʼ��******************//
void Function_Init(void)
{
	//�����������Ϊ�ߣ��͵�ƽ��Ч
	nrf_gpio_cfg_output(LED_RED);     //���ú��
	nrf_gpio_cfg_output(LED_BLUE);  //��������
	nrf_gpio_cfg_output(LED_YELLOW);  //���ûƵ�
  nrf_gpio_cfg_output(LED_0);
	nrf_gpio_cfg_output(LED_1);
	nrf_gpio_pin_set(LED_RED);      //�ߵ�ƽ����
	nrf_gpio_pin_set(LED_BLUE);
	nrf_gpio_pin_set(LED_YELLOW);
	nrf_gpio_cfg_output(MOTOR);
	nrf_gpio_pin_clear(MOTOR);

}
void Red_shine(uint8_t n)
{
	  while(n--)
		{
		nrf_gpio_pin_clear(LED_RED);//�����
	  nrf_delay_ms(1000);
	  nrf_gpio_pin_set(LED_RED);
	  nrf_delay_ms(1000);
		}
}

void Blue_shine(uint8_t n)
{
		  while(n--)
		{
		nrf_gpio_pin_clear(LED_BLUE);//�����
	  nrf_delay_ms(1000);
	  nrf_gpio_pin_set(LED_BLUE);
	  nrf_delay_ms(1000);
		}
}	
void Yellow_shine(uint8_t n)
{
			  while(n--)
		{
		nrf_gpio_pin_clear(LED_YELLOW);//�����
	  nrf_delay_ms(1000);
	  nrf_gpio_pin_set(LED_YELLOW);
	  nrf_delay_ms(1000);
		}
}
void Motor(uint8_t n)
{ 
	while(n--)
		{
		nrf_gpio_pin_set(MOTOR);//��
 	  nrf_delay_ms(1000);
	  nrf_gpio_pin_clear(MOTOR);
	  nrf_delay_ms(1000);
		}
}
void Red_togger()
{
	  nrf_gpio_pin_toggle(LED_RED);
}
void Red_fastshine(uint8_t n)
{
			  while(n--)
		{
		nrf_gpio_pin_clear(LED_YELLOW);//�����
	  nrf_delay_ms(50);
	  nrf_gpio_pin_set(LED_YELLOW);
	  nrf_delay_ms(50);
		}


}
void Motor_set(void)
{
		nrf_gpio_pin_set(MOTOR);//��
}

uint8_t AddAlarmClock(uint8_t*alarm)
{
	uint8_t i=0;
	for(;i<99;i++)
	{
		if(0==alarmClock[i].isUsed)//ֱ���ҵ�δ�õ�
		{
			for(uint8_t k=0;k<11;k++)
				alarmClock[i].alarm[k]=alarm[k];//�Ѵ�pdata[2]��ַ��ʼ�����ݴ浽������
			alarmClock[i].isUsed=1;//��Ǳ�ʹ��
			return i;
		}
	}
	return i;//�ѽ�����50������
}
void GetAllClock(uint8_t* alarms)//alarms��СΪCLOCK_COUNT_MAX*(sizeof(alarm)+sizeof(clockHandler))+2,��һλ��ʾ���Ӹ���
{
	 uint16_t index=0,clock_num = 0;
	for(uint8_t i=0;i<99;i++)
	{
		if(1==alarmClock[i].isUsed)
		{
			  //alarms[index]=i+1;//��¼���ӵڼ�����Ч���ӣ���1��ʼ����¼�±�
				alarms[index++] = (i+1)/10+'0';   //ʮλ
        alarms[index++] = (i+1)%10+'0'; //��λ	
			  alarms[index++] = '\r';
			//index++;
			for(uint8_t k=0;k<11;k++)
				alarms[index++]=alarmClock[i].alarm[k];			    //ÿ��������һ�����кţ��Ͷ�ʱ�������
					alarms[index++] = 0;  
			   clock_num++;
		}
//		else if(alarmClock[i].isUsed==0 && index != 0)
//			alarms[index-1] = 1;   //���һ�����Ӻ���0x64,��Ϊ��־λ
//		else 
//			break;
	}
   alarms[index-1] = 1;
}
