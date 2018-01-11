#include "hardware.h"
#include "nrf_delay.h"
//uint8_t user_send[9];
//定义3个全局的结构体，用于保存设置信息。
shine_type led_config;
vibrate_type motor_config;
alarm_type clock_config;

//初始化三大功能的结构体配置
void led_config_init(void)
{
	led_config.led_type = 'R';
	led_config.shine_mode = 0;//模式0,亮1s灭一秒
	led_config.shine_times = 0;
}
void motor_config_init(void)
{	
	motor_config.motor_type = 0;//0默认LDA类型的震动器
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
	clock_config.alarm_mode = 0;//每天提醒
	clock_config.alarm_time =  86399;//最大的秒数
	clock_config.alarm_way = 0;//闹钟提示的方式。默认0，震动提示
}
//作为用户提示的三个功能LED初始化******************//
void Function_Init(void)
{
	//设置引脚输出为高，低电平有效
	nrf_gpio_cfg_output(LED_RED);     //配置红灯
	nrf_gpio_cfg_output(LED_BLUE);  //配置蓝灯
	nrf_gpio_cfg_output(LED_YELLOW);  //配置黄灯
  nrf_gpio_cfg_output(LED_0);
	nrf_gpio_cfg_output(LED_1);
	nrf_gpio_pin_set(LED_RED);      //高电平不亮
	nrf_gpio_pin_set(LED_BLUE);
	nrf_gpio_pin_set(LED_YELLOW);
	nrf_gpio_cfg_output(MOTOR);
	nrf_gpio_pin_clear(MOTOR);

}
void Red_shine(uint8_t n)
{
	  while(n--)
		{
		nrf_gpio_pin_clear(LED_RED);//红灯亮
	  nrf_delay_ms(1000);
	  nrf_gpio_pin_set(LED_RED);
	  nrf_delay_ms(1000);
		}
}

void Blue_shine(uint8_t n)
{
		  while(n--)
		{
		nrf_gpio_pin_clear(LED_BLUE);//红灯亮
	  nrf_delay_ms(1000);
	  nrf_gpio_pin_set(LED_BLUE);
	  nrf_delay_ms(1000);
		}
}	
void Yellow_shine(uint8_t n)
{
			  while(n--)
		{
		nrf_gpio_pin_clear(LED_YELLOW);//红灯亮
	  nrf_delay_ms(1000);
	  nrf_gpio_pin_set(LED_YELLOW);
	  nrf_delay_ms(1000);
		}
}
void Motor(uint8_t n)
{ 
	while(n--)
		{
		nrf_gpio_pin_set(MOTOR);//震动
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
		nrf_gpio_pin_clear(LED_YELLOW);//红灯亮
	  nrf_delay_ms(50);
	  nrf_gpio_pin_set(LED_YELLOW);
	  nrf_delay_ms(50);
		}


}
void Motor_set(void)
{
		nrf_gpio_pin_set(MOTOR);//震动
}

uint8_t AddAlarmClock(uint8_t*alarm)
{
	uint8_t i=0;
	for(;i<99;i++)
	{
		if(0==alarmClock[i].isUsed)//直到找到未用的
		{
			for(uint8_t k=0;k<11;k++)
				alarmClock[i].alarm[k]=alarm[k];//把从pdata[2]地址开始的数据存到闹钟数
			alarmClock[i].isUsed=1;//标记被使用
			return i;
		}
	}
	return i;//已将超过50个闹钟
}
void GetAllClock(uint8_t* alarms)//alarms大小为CLOCK_COUNT_MAX*(sizeof(alarm)+sizeof(clockHandler))+2,第一位表示闹钟个数
{
	 uint16_t index=0,clock_num = 0;
	for(uint8_t i=0;i<99;i++)
	{
		if(1==alarmClock[i].isUsed)
		{
			  //alarms[index]=i+1;//记录闹钟第几个有效闹钟，从1开始，记录下标
				alarms[index++] = (i+1)/10+'0';   //十位
        alarms[index++] = (i+1)%10+'0'; //个位	
			  alarms[index++] = '\r';
			//index++;
			for(uint8_t k=0;k<11;k++)
				alarms[index++]=alarmClock[i].alarm[k];			    //每个闹钟由一个序列号，和定时内容组成
					alarms[index++] = 0;  
			   clock_num++;
		}
//		else if(alarmClock[i].isUsed==0 && index != 0)
//			alarms[index-1] = 1;   //最后一个闹钟后添0x64,作为标志位
//		else 
//			break;
	}
   alarms[index-1] = 1;
}
