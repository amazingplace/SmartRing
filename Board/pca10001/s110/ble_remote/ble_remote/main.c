/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h" 
#include "app_button.h"
#include "ble_nus.h"
#include "simple_uart.h"
#include "boards.h"
#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"
#include "nrf_delay.h"
#include "hardware.h"
#include "string.h"
extern shine_type led_config;
extern vibrate_type motor_config;
extern alarm_type clock_config;
 AlarmClockType alarmClock[99];
#define ADVERTISING_LED_PIN_NO          LED_0                                       //广播指示灯
#define CONNECTED_LED_PIN_NO            LED_1                                       //连接指示灯

#define DEVICE_NAME                     "BLE_REMOTE"                                //蓝牙设备名称

#define APP_ADV_INTERVAL                64                                          //广播间隔
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         //广播超时时间

#define APP_TIMER_PRESCALER             4095                                           //RTC1 分频器的值
#define APP_TIMER_MAX_TIMERS            8                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         12                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               16                                          /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               60                                          /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           //潜伏期
#define CONN_SUP_TIMEOUT                400                                         //连接断开超时时间
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (20 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

/**************闹钟相关宏定义**********************/
#define ALARM_CLOCK_TICK APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
//#define LED_ON_TIME     APP_TIMER_TICKS(150, APP_TIMER_PRESCALER) /*LED?????*/
//#define LED_OFF_TIME    APP_TIMER_TICKS(350, APP_TIMER_PRESCALER) /*LED?????*/
//#define BLINK_LED_MAX   5                      /*??LED??? ????5*/
//static app_timer_id_t BlinkHandle;//blink???
//static uint8_t BlinkCount = 0; //???????
//static bool IsLinkLedON = false; //?????

//#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
//static app_timer_id_t AlarmClockHandler;//闹钟执行句柄
//static app_timer_id_t AlarmClockHandler2;//闹钟执行句柄
static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_nus_t                        m_nus; 
  //  static uint32_t test;/**< Structure to identify the Nordic UART Service. */
/////*****电池相关**************************/
//static uint32_t power_evt_id;     
void setLedMotor(uint8_t*context);
void AlarmClockTimeoutHandler2(void *p_context);
/**@brief     Error handler function, which is called when an error has occurred.
 *
 * @warning   This handler is an example only and does not fit a final product. You need to analyze
 *            how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief       Assert macro callback function.
 *
 * @details     This function will be called in case of an assert in the SoftDevice.
 *
 * @warning     This handler is an example only and does not fit a final product. You need to
 *              analyze how your product is supposed to react in case of Assert.
 * @warning     On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief   Function for Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
  // Initialize timer module
  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
}


/**@brief   Function for the GAP initialization.
 *
 * @details This function will setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)            //用于调试
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for the Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)    //初始化广播机制
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    
    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}


/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordic UART BLE Service and send
 *           it to the UART module.
 */
/**@snippet [Handling the data received over BLE] */

//**********************************************
//star添加
void AlarmClockTimeoutHandler2(void *p_context)
{
	AlarmClockType*alarmClock=(AlarmClockType*)p_context;
	setLedMotor(alarmClock->alarm+8);//让设备做运动
	memset(alarmClock,0,sizeof(AlarmClockType));//删除队列中的闹钟
	//setLedMotor("R11");//让设备做运动
}
void setLedMotor(uint8_t*context)
{
	switch(context[0])
	{
	case 'R':
		Red_shine(context[1]-'0');
		Motor(context[2]-'0');
		break;
	case 'B':
		Blue_shine(context[1]-'0');
		Motor(context[2]-'0');
		break;
	case 'Y':
		Yellow_shine(context[1]-'0');
		Motor(context[2]-'0');
		break;
	}
}
void nus_data_handler2(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	uint8_t flag='0';
	uint32_t errorcode;
	if('S'==p_data[0])
	{
			if('S'==p_data[1])//搜索当前闹钟
			{
				uint8_t alarm[14];//每个闹钟13个字符，共99个闹钟，格式为IIABCDABCDXYZM，II为闹钟序列，ABCDABCDXYZ为闹钟,M为'1'表示最后一个闹钟
				uint8_t N=0;
				for(uint8_t i=0;i<99;i++)
				{//统计闹钟个数
					if(alarmClock[i].isUsed)
						N++;
				}
				for(uint8_t i=0;i<99;i++)
				{//发送每个闹钟
					if(alarmClock[i].isUsed)
					{
						memset(alarm,0,14);
						alarm[0]=i/10+'0';
						alarm[1]=i%10+'0';
						memcpy(&alarm[2],alarmClock[i].alarm,11);
						N--;
						alarm[13]= (N==0)?'0':'1';
					errorcode = ble_nus_send_string(&m_nus,alarm,14);//发送该闹钟内容
				//	APP_ERROR_CHECK(errorcode);
					}
				}
      }
			else if('A'==p_data[1])//添加闹钟格式为SA12341234XYZ，前1234表示当前时间，后1234表示待设时间
			{
				uint8_t index=AddAlarmClock(&p_data[2]);//返回分配给闹钟的号
				errorcode = app_timer_create(&alarmClock[index].clockHandler,APP_TIMER_MODE_SINGLE_SHOT,AlarmClockTimeoutHandler2);
				APP_ERROR_CHECK(errorcode);
				uint32_t clock_sec=((p_data[6]-p_data[2])*10+p_data[7]-p_data[3])*60*60+((p_data[8]-p_data[4])*10+p_data[9]-p_data[5])*60;
				while(clock_sec>604800)//当为负（大于7天）数，则闹钟需要加上一天的秒数
					clock_sec+=24*60*60;
				app_timer_start(alarmClock[index].clockHandler,clock_sec*8,&alarmClock[index]);//启动定时器，发送闹钟地址
				errorcode = ble_nus_send_string(&m_nus,&flag,1);//发送添加成功消息
				APP_ERROR_CHECK(errorcode);
			}
			else if('D'==p_data[1])//删除闹钟
			{
				uint8_t temp = (p_data[2]-'0')*10+(p_data[3]-'0');//删除第n个闹钟
				if(alarmClock[temp].isUsed==1)
				{
					app_timer_stop(alarmClock[temp].clockHandler);
					memset(&alarmClock[temp],0,sizeof(AlarmClockType));
				errorcode = ble_nus_send_string(&m_nus,&flag,1);//发送删除闹钟
				APP_ERROR_CHECK(errorcode);
				}
				else
				{
					flag='1';
					errorcode = ble_nus_send_string(&m_nus,&flag,1);//发送删除闹钟
					APP_ERROR_CHECK(errorcode);
				}
			}
	}
	else
	{
		setLedMotor(p_data);
		errorcode = ble_nus_send_string(&m_nus,&flag,1);//发送控制成功信号
		APP_ERROR_CHECK(errorcode);
	}
	
}
//**********************************************
/**@snippet [Handling the data received over BLE] */



/**@brief Function for initializing services that will be used by the application.
 */
	
//service 初始化
static void services_init(void)
{
    uint32_t         err_code;
    ble_nus_init_t   nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));      //清空nus_init结构体数据

    //nus_init.data_handler = nus_data_handler;    //设定service的handler
	
		nus_init.data_handler = nus_data_handler2;    //设定service的handler，新的函数
    
    err_code = ble_nus_init(&m_nus, &nus_init);  //初始化nus service
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief       Function for handling an event from the Connection Parameters Module.
 *
 * @details     This function will be called for all events in the Connection Parameters Module
 *              which are passed to the application.
 *
 * @note        All this function does is to disconnect. This could have been done by simply setting
 *              the disconnect_on_fail config parameter, but instead we use the event handler
 *              mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief       Function for handling errors from the Connection Parameters module.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
//连接参数的初始化
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));     //清除结构体

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
//开始广播
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));           //清除结构体
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;            //广播间隔
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;  //广播超时时

    err_code = sd_ble_gap_adv_start(&adv_params);         //调用广播开始函数
    APP_ERROR_CHECK(err_code);                            //检测有无ERROR产生

    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);             //广播状态指示灯亮
}

/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
//每次连接事件产生的处理函数
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:                             //已经连接上
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);             //已连接指示灯亮
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);         //广播指示灯灭
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            advertising_start();

            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            { 
             nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

                // Configure buttons with sense level low as wakeup source.
             //nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
             //                             BUTTON_PULL,
             //                             NRF_GPIO_PIN_SENSE_LOW);
                
                // Go to system-off mode (this function will not return; wakeup will cause a reset)
             //     err_code = sd_power_system_off();    
             //   APP_ERROR_CHECK(err_code);
						 advertising_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief       Function for dispatching a S110 SoftDevice event to all modules with a S110
 *              SoftDevice event handler.
 *
 * @details     This function is called from the S110 SoftDevice event interrupt handler after a
 *              S110 SoftDevice event has been received.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief   Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
//协议栈初始化
static void ble_stack_init(void)
{
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, false);
    
    // Subscribe for BLE events.
    uint32_t err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

//static void sd_evt_power_failure()
//{
//	    uint32_t err_code;
//	  err_code = sd_nvic_SetPriority(SD_EVT_IRQn, NRF_APP_PRIORITY_LOW);
//   APP_ERROR_CHECK(err_code);
//   err_code = sd_nvic_EnableIRQ(SD_EVT_IRQn);
//   APP_ERROR_CHECK(err_code);
//    err_code = sd_power_pof_enable(true);
//    APP_ERROR_CHECK(err_code);
//    err_code = sd_power_pof_threshold_set(NRF_POWER_THRESHOLD_V27);
//    APP_ERROR_CHECK(err_code);
//	err_code = softdevice_sys_evt_handler_set();
//}
//static void sd_evt_power_failure_init()
//{
//	uint32_t err_code;
//err_code = sd_nvic_SetPriority(SD_EVT_IRQn, NRF_APP_PRIORITY_LOW);
//APP_ERROR_CHECK(err_code);

//err_code = sd_nvic_EnableIRQ(SD_EVT_IRQn);
//APP_ERROR_CHECK(err_code);

//err_code = sd_power_pof_enable(true);
//APP_ERROR_CHECK(err_code);

//err_code = sd_power_pof_threshold_set(NRF_POWER_THRESHOLD_V27); /**< 2.5 Volts power failure threshold. */ 
//APP_ERROR_CHECK(err_code);
//}
uint32_t p_event;
static void sd_evt_power_failure()
{
uint32_t err_code;
err_code = sd_evt_get(&p_event);    // get any pending events generated by the SoC API

if (err_code == NRF_SUCCESS)
{
    if (p_event == NRF_EVT_POWER_FAILURE_WARNING)   // SD event
    {
        sd_nvic_ClearPendingIRQ(SD_EVT_IRQn);   // clear NRF_POWER->EVENTS_POFWARN after MOTOR test (to clear preceding warnings)
        Red_fastshine(10);
		}
}
}


//static void sys_evt_dispatch(uint32_t sys_evt)
//{
//    if (sys_evt == NRF_EVT_POWER_FAILURE_WARNING)
//    {
//     Red_fastshine(20);
//    }
//}

/**@brief  Application main function.
 */
int main(void)
{
	uint32_t err_code;
  	//sd_evt_power_failure_init();
    //各外设初始化
    Function_Init();        //指示灯震动器初始化  
    timers_init();          //软件定时器初始化
    //uart_init();          //串口初始化
	  nrf_delay_us(100000);
		//AlarmClockInit();
		//新闹钟初始化
		memset(alarmClock,0,sizeof(AlarmClockType)*99);
	  //蓝牙初始化
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();
		err_code = sd_power_pof_enable(true);
    APP_ERROR_CHECK(err_code);
    err_code = sd_power_pof_threshold_set(NRF_POWER_THRESHOLD_V27); /**< 2.5 Volts power failure threshold. */ 
    APP_ERROR_CHECK(err_code);
    advertising_start();      //开始广播
    // Enter main loop	
    for (;;)
    {
////			err_code = sd_power_pof_enable(false);
////			APP_ERROR_CHECK(err_code);
          power_manage();
//      err_code = sd_power_pof_enable(true);
//				APP_ERROR_CHECK(err_code);
        sd_evt_power_failure();			
    }
}
//void SD_EVT_IRQHandler(void)
//{
//    uint32_t err_code;
//    uint32_t sd_event_id;

//    do
//    {
//        // Fetch the event.
//        err_code = sd_evt_get( &sd_event_id);
//        if (err_code == NRF_SUCCESS)
//        {
//            // Handle event.
//            switch (sd_event_id)
//            {
//            case NRF_EVT_POWER_FAILURE_WARNING:
//                Red_fastshine(20);
//                break;
//            default:
//                break;
//            }
//        }
//    } while (err_code == NRF_SUCCESS);
//}
/** 
 * @}
 */
