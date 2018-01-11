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
	uint8_t led_type;   //led�Ƶ����ͣ�ָ��ɫ
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
	uint8_t  alarm_mode;//����ģʽ��ֵΪ1,2,3,4
	uint32_t alarm_time;//���ӵ�ʱ�䣬���ｫ��ת��Ϊ�룬���ڼ�ʱ
	uint8_t alarm_way;//���ӵ����ѷ�ʽ
}alarm_type;


//********************************************
//star���
typedef struct AlarmClock
{
	uint8_t isUsed;//0��ʾδʹ�ã�1��ʾ�Ѿ���ʹ��
	uint32_t clockHandler;//��ʱ�����
	uint8_t alarm[11];//ʱ���ʽΪ12341234XYZ��ǰ1234��ǰʱ�䣬��1234��ʾ����ʱ��,XΪ����ɫ��YΪ���ƴ�����ZΪ�񶯴���
}AlarmClockType;

extern AlarmClockType alarmClock[99];//�ṹ������
uint8_t AddAlarmClock(uint8_t*alarm);//��������ӵ����ӵ������±�
void GetAllClock(uint8_t*alarms);

	//���ӱ��Ϊ�����±�

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
