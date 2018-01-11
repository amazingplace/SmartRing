#ifndef _HARDWARE_H
#define _HARDWARE_H
//#include "pca10001.h"
#include "nrf_gpio.h"
#include "nrf.h"
#define LED_RED           4
#define LED_BLUE          5
#define LED_YELLOW        6	
#define MOTOR							3
#define LED_0          18
#define LED_1          19
typedef struct led_set
{
	uint8_t led_type;   //led灯的类型，指颜色
	uint8_t shine_mode;
	uint8_t shine_times;
}shine_type;
typedef struct alarm_clock_set
{
	uint16_t year;
	uint8_t  month;
	uint8_t  day;
	uint8_t  hours;
	uint8_t  minutes;
	uint8_t  seconds;
	uint8_t  alarm_mode;//四种模式，值为1,2,3,4
	uint32_t alarm_time;//闹钟的时间，这里将其转化为秒，便于计时
	uint8_t alarm_way;//闹钟的提醒方式
}alarm_type;


//********************************************
//star添加
typedef struct AlarmClock
{
	uint8_t isUsed;//0表示未使用，1表示已经被使用
	uint32_t clockHandler;//定时器句柄
	uint8_t alarm[11];//时间格式为12341234XYZ，前1234表当前时间，后1234表示闹钟时间,X为灯颜色，Y为亮灯次数，Z为振动次数
}AlarmClockType;

extern AlarmClockType alarmClock[99];//结构体数组
uint8_t AddAlarmClock(uint8_t*alarm);//返回新添加到闹钟的数组下标
void GetAllClock(uint8_t*alarms);

	//闹钟编号为数组下标

//end
//********************************************


typedef struct motor_set
{
	uint8_t motor_type;
	uint8_t vibrate_mode;
	uint8_t vitrate_times;
}vibrate_type;
extern void Red_shine(uint8_t n);
extern void Blue_shine(uint8_t n);
extern void Yellow_shine(uint8_t n);
extern void Motor(uint8_t n);
extern void Function_Init(void);
extern void Red_togger(void);
extern void Motor_set(void);
extern void Red_fastshine(uint8_t n);
#endif
