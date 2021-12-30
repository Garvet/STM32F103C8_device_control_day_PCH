/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "../LoRa_main_cpp/LoRa_main_file.h"
#include <FlashPROM.h>

#include <Device_relay.h>
#include <Device_mosfet.h>
#include <Clock_channel.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#if defined( BUILD_TESTING_CODE_409 )
#define BAND             433375E3
#define PABOOST          true
#define SIGNAL_POWER     14
#define SPREADING_FACTOR 8
#define SIGNAL_BANDWIDTH 150E3
#define SYNC_WORD        0xA5
//  LoRa_begin_result = Begin_lora_module(BAND, true, 14, 8, 250E3, 0x4A);
#else

#define BAND	         43455E4 // basement | object1
//#define BAND             43325E4 // specialized control | test table

#define PABOOST          true
#define SIGNAL_POWER     14
#define SPREADING_FACTOR 8
#define SIGNAL_BANDWIDTH 250E3
#define SYNC_WORD        0x4A
#endif

//#define _TSL2561
//#define _BME280
//#define _CCS811
//#define _WATER_TEMP

#define AMT_CHANNEL 1
//#define CLEAR_TIME
#define USE_LORA
//#define TIME_ADJUSTMENT
#define ADJUSTMENT_SEC 17
#define ADJUSTMENT_SIGN + // +/-

#if defined( USE_LORA )
#define USE_LORA_RTC_SYNC
#define RTC_SYNC_PERIOD_MIN 30
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */


RTC_TimeTypeDef next_time;
struct clock_channel_t all_channel[AMT_CHANNEL];

#if (AMT_CHANNEL > 0)
struct clock_channel_t* channel1 = &all_channel[0];
RTC_TimeTypeDef time_channel1_inclusion = {.Hours=6, .Minutes=0, .Seconds=00};
RTC_TimeTypeDef time_channel1_shutdown = {.Hours=22, .Minutes=00, .Seconds=00};
//RTC_TimeTypeDef time_channel1_inclusion = {.Hours=6, .Minutes=0, .Seconds=00};
//RTC_TimeTypeDef time_channel1_shutdown = {.Hours=0, .Minutes=1, .Seconds=00};
#endif
#if (AMT_CHANNEL > 1)
struct clock_channel_t* channel2 = &all_channel[1];
RTC_TimeTypeDef time_channel2_inclusion = {.Hours=6, .Minutes=0, .Seconds=00};
RTC_TimeTypeDef time_channel2_shutdown = {.Hours=0, .Minutes=1, .Seconds=00};
#endif
#if (AMT_CHANNEL > 2)
struct clock_channel_t* channel3 = &all_channel[2];
RTC_TimeTypeDef time_channel3_inclusion = {.Hours=22, .Minutes=45, .Seconds=0};
RTC_TimeTypeDef time_channel3_shutdown = {.Hours=0, .Minutes=6, .Seconds=0};
#endif
#if (AMT_CHANNEL > 3)
#endif


#if defined (CLEAR_TIME)
RTC_TimeTypeDef clear_time = {.Hours=0, .Minutes=1, .Seconds=0};
#endif

RTC_TimeTypeDef time_synchronization = {.Hours=23, .Minutes=59, .Seconds=30};

struct relay_t relay_led;
bool relay_invert_led;

#if defined( DEBUG )
uint8_t hour=0, min=0, sec=0;
#endif

uint8_t build_hour=0, build_min=0, build_sec=0;


#if defined( TIME_ADJUSTMENT )
RTC_TimeTypeDef time_adjustment;
#endif

#if defined( USE_LORA_RTC_SYNC )
uint32_t deviation_sync_time = 0;
#endif

#if defined( TIME_ADJUSTMENT ) || defined( USE_LORA_RTC_SYNC )
RTC_TimeTypeDef time_subtraction;
#endif

RTC_TimeTypeDef set_sync_time =  {.Hours=0, .Minutes=0, .Seconds=0};
volatile bool receive_sync_time = false;
volatile bool regime_change = false;

volatile int send_type_packet = -1;

												// TSL2561
uint8_t STADY_CONACT_NUMBER = 0;

//SensorsDataTypeDef sensors_data;
DevicesDataTypeDef devices_data;


struct relay_t relay[5];
struct relay_t *relay_start = /** &relay[3]; //*/ &relay[0];
struct relay_t *relay_stop  = /** &relay[2]; //*/ &relay[1];
//struct relay_t *relay_r     = ; // &relay[3];
struct relay_t *relay_valve = /** &relay[0]; //*/ &relay[4]; // ??? (-) ----- (-) ----- (-)
bool relay_invert[5] = {false, false, false, false, false};

struct mosfet_t freq_signal;
// mosfet = mosfet_init(&htim1, TIM_CHANNEL_2, 4095, false);
// mosfet_set_compare(&mosfet, val);

void reset_all_relays() {
	for(int i = 0; i < 5; ++i) {
		relay_set_state(&relay[i], GPIO_PIN_RESET);
	}
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
	//                       RELAY4,    RELAY3,    RELAY2,    RELAY1
}
void set_pwm_pch(uint16_t power) {
	mosfet_set_compare(&freq_signal, power);
}
void start() { // START
	reset_all_relays();
	relay_set_state(relay_start, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // RELAY1
}
void stop() { // STOP
	reset_all_relays();
	relay_set_state(relay_stop, GPIO_PIN_SET);
	set_pwm_pch(0);
	// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_4|GPIO_PIN_3, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // RELAY2
}
void use_valve() { // REVERT
	reset_all_relays();
	relay_set_state(relay_stop, GPIO_PIN_SET);
	relay_set_state(relay_valve, GPIO_PIN_SET);
	set_pwm_pch(0);
	// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // RELAY4
}
//void revert() { // REVERT
//	relay_set_state(relay_r, !relay_r->state);
//	// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);
//	// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // RELAY4
//}

//const uint16_t PUMP_PWM_DAY  = 4095;
const unsigned long PUMP_ON_DAY  = /** (1 * 60 + 00) * 1000; //*/ (0 * 60 + 20) * 1000; // (1 * 60 + 00) * 1000;
const unsigned long PUMP_OFF_DAY = /** (4 * 60 + 00) * 1000; //*/ (0 * 60 + 40) * 1000; // (4 * 60 + 00) * 1000;
const unsigned long VALVE_ON_DAY = /** (0 * 60 + 10) * 1000; //*/ (0 * 60 + 10) * 1000; // (0 * 60 + 10) * 1000;
//const uint16_t PUMP_PWM_NIGHT  = 4095;
const unsigned long PUMP_ON_NIGHT  = /** (1 * 60 + 00) * 1000; //*/ (0 * 60 + 20) * 1000; // (1 * 60 + 00) * 1000;
const unsigned long PUMP_OFF_NIGHT = /** (6 * 60 + 00) * 1000; //*/ (0 * 60 + 60) * 1000; // (6 * 60 + 00) * 1000;
const unsigned long VALVE_ON_NIGHT = /** (0 * 60 + 10) * 1000; //*/ (0 * 60 + 10) * 1000; // (0 * 60 + 10) * 1000;
unsigned long last_devices_change = 0;
uint8_t devices_state = 0;
bool get_stop = false;

volatile bool set_pwm = false;
volatile uint16_t set_pump_pwm = 4095; // set from server
uint16_t pump_pwm = 4095; // set in main

uint8_t LoRa_begin_result;

// If gain = false (0), device is set to low gain (1X)
// If gain = high (1), device is set to high gain (16X)
// If time = 0, integration will be 13.7ms
// If time = 1, integration will be 101ms
// If time = 2, integration will be 402ms
// If time = 3, use manual start / stop (ms = 0)
// ms will be set to integration time
bool gain = 0;
unsigned  int ms = 0;
void user_delay_ms(uint32_t period);


uint8_t tim4 = 0;

int8_t rslt;

uint16_t exti5_10, exti2;
uint16_t cnt_task_1, cnt_task_2, cnt_task_3;

uint16_t water_temp = 0;

volatile bool end_contact = false;
volatile bool change_state = false;

uint32_t control_module_id_and_channel[BUFFSIZE] = {0x00000000, 0x00000000};
int8_t tick;

//buf32_t control_module_id;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void DevicesGetValues();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void alarm_rtc();
void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4) //check if the interrupt comes from TIM1
	{
		//HAL_ResumeTick();
		//HAL_GPIO_TogglePin(LED1_PIN_GPIO_Port, LED1_PIN_Pin);
		tim4++;
		// if end_contact
//			SensorsGetValues();
//			Main_cpp(&sensors_data);
			// wake up
		//HAL_Delay(2000);

	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

//  relay_invert[0] = relay_invert[1] = false;
//  relay_invert[2] = relay_invert[3] = false;
//  relay_invert[4] = false;
#if defined( DEBUG )
	RTC_TimeTypeDef time;
	HAL_StatusTypeDef res_t;
#endif


  Read_control_module_info_from_flash(control_module_id_and_channel);
  Get_control_module_info_from_main(control_module_id_and_channel);
  set_pump_pwm = pump_pwm = Read_PWM_info_from_flash();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  // Подготовка устройств


#if (AMT_CHANNEL > 0)
  relay_led = relay_init(LED1_PIN_GPIO_Port, LED1_PIN_Pin, false);
//  relay[0] = relay_init(RELAY1_PIN_GPIO_Port, RELAY1_PIN_Pin, relay_invert[0]); // {0} -----
  *channel1 = clock_channel_init(&relay_led, time_channel1_inclusion, time_channel1_shutdown);
  channel1->deviation_sec = 2;
#endif
#if (AMT_CHANNEL > 1)
  relay[1] = relay_init(RELAY2_PIN_GPIO_Port, RELAY2_PIN_Pin, relay_invert[1]);
  *channel2 = clock_channel_init(&relay[1], time_channel2_inclusion, time_channel2_shutdown);
  channel2->deviation_sec = 2;
#endif
#if (AMT_CHANNEL > 2)
  relay[2] = relay_init(RELAY_NS_PIN_GPIO_Port, RELAY_NS_PIN_Pin, relay_invert[2]);
  *channel3 = clock_channel_init(&relay[2], time_channel3_inclusion, time_channel3_shutdown);
  channel3->deviation_sec = 2;
#endif
#if (AMT_CHANNEL > 3)
#endif


	relay[0] = relay_init(RELAY1_PIN_GPIO_Port, RELAY1_PIN_Pin, relay_invert[0]);
	relay[1] = relay_init(RELAY2_PIN_GPIO_Port, RELAY2_PIN_Pin, relay_invert[1]);
	relay[2] = relay_init(RELAY3_PIN_GPIO_Port, RELAY3_PIN_Pin, relay_invert[2]);
	relay[3] = relay_init(RELAY4_PIN_GPIO_Port, RELAY4_PIN_Pin, relay_invert[3]);
	relay[4] = relay_init(RELAY5_PIN_GPIO_Port, RELAY5_PIN_Pin, relay_invert[4]);
	freq_signal = mosfet_init(&htim4, TIM_CHANNEL_2, 4095, false);

  //  channel1->state = true;
  //  channel2->state = true;
  //  relay_set_state(channel1->relay, channel1->state);
  //  relay_set_state(channel2->relay, channel2->state);

  // Запуск LoRa
#if defined (USE_LORA)
  Init_lora_module(&hspi1);
  LoRa_begin_result = Begin_lora_module(BAND, PABOOST, SIGNAL_POWER, SPREADING_FACTOR, SIGNAL_BANDWIDTH, SYNC_WORD);
#endif
  for (int i = 0; i < 5; ++i) {
	  HAL_GPIO_TogglePin(LED1_PIN_GPIO_Port, LED1_PIN_Pin);
	  HAL_GPIO_TogglePin(LED2_PIN_GPIO_Port, LED2_PIN_Pin);
	  HAL_GPIO_TogglePin(LED3_PIN_GPIO_Port, LED3_PIN_Pin);
	  HAL_Delay(500);
  }
#if defined (USE_LORA)
  while(LoRa_begin_result != 0) {
	  LoRa_begin_result = Begin_lora_module(BAND, PABOOST, SIGNAL_POWER, SPREADING_FACTOR, SIGNAL_BANDWIDTH, SYNC_WORD);
	  HAL_GPIO_WritePin(LED1_PIN_GPIO_Port, LED1_PIN_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED2_PIN_GPIO_Port, LED2_PIN_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED3_PIN_GPIO_Port, LED3_PIN_Pin, GPIO_PIN_SET);
  }
#endif
  HAL_GPIO_WritePin(LED1_PIN_GPIO_Port, LED1_PIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_PIN_GPIO_Port, LED2_PIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_PIN_GPIO_Port, LED3_PIN_Pin, GPIO_PIN_RESET);


  // Запуск устройств

  // Перенести установку времени в ESP32!!!, как и запуск на регистрацию
  RTC_TimeTypeDef null_time = {.Hours=build_hour, .Minutes=build_min, .Seconds=build_sec};
#if defined( TIME_ADJUSTMENT )
  time_adjustment = null_time;
#endif

  RTC_DateTypeDef null_date = {.Date=0, .Month=0, .Year=0};
  HAL_RTC_SetTime(&hrtc, &null_time, RTC_FORMAT_BIN);
  alarm_rtc();

#ifdef _CCS811
  configureCCS811();
#endif
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(1000);
  //HAL_TIM_Base_Start_IT(&htim4);
  //HAL_PWR_EnableSleepOnExit();
  //end_contact = true;
//  HAL_GPIO_WritePin(RELAY_NS_PIN_GPIO_Port, RELAY_NS_PIN_Pin, GPIO_PIN_SET);
  DevicesGetValues();
#if defined (USE_LORA)
  Main_cpp(&devices_data);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_TogglePin(LED1_PIN_GPIO_Port, LED1_PIN_Pin);
//  while(0) {
//  	volatile static uint16_t set_pwm = 4095;
//	start();
//  	set_pwm_pch(set_pwm);
//  	HAL_Delay(5000);
//  	stop();
//  	HAL_Delay(5000);
//  }
  while (1)
  {
        res_t = HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
        if(res_t == HAL_OK) {
            hour = time.Hours;
            min = time.Minutes;
            sec = time.Seconds;
#if defined ( CLEAR_TIME )
            if(hour == clear_time.Hours && min == clear_time.Minutes && sec == clear_time.Seconds) {
        		HAL_RTC_SetDate(&hrtc, &null_date, RTC_FORMAT_BIN);
        		HAL_RTC_SetTime(&hrtc, &null_time, RTC_FORMAT_BIN);
#if defined( TIME_ADJUSTMENT )
        		time_subtraction = clock_subtraction(&time, &time_adjustment);
        		time = null_time;
        		time_adjustment = time;
        		clock_add_second(&time_adjustment, -clock_get_sec(&time_subtraction));
#endif
        		alarm_rtc();
            }
#endif // CLEAR_TIME

#if defined ( USE_LORA_RTC_SYNC )
            if(sec == 0) {
            	time_subtraction = clock_subtraction(&time, &set_sync_time);
            	if(clock_get_sec(&time_subtraction) > RTC_SYNC_PERIOD_MIN * 60) {
            		if(HAL_GetTick() - deviation_sync_time > 60000) {
            			deviation_sync_time = HAL_GetTick();
            			Set_sync_rtc();
            		}
            	}
            }
#endif // USE_LORA_RTC_SYNC
        }


#if defined( TIME_ADJUSTMENT ) // корректировка времени
        time_subtraction = clock_subtraction(&time, &time_adjustment);
        if(clock_get_sec(&time_subtraction) > ADJUSTMENT_SEC) {
//        	for()
//        		__NOP();
        	clock_add_second(&time, ADJUSTMENT_SIGN 1);
    		HAL_RTC_SetDate(&hrtc, &null_date, RTC_FORMAT_BIN);
    		HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
        	time_adjustment = time;
    		alarm_rtc();
        }
#endif

        // Создаём пакет по включение/выключение устройства или завершении контакта
        if(change_state || end_contact) {
            DevicesGetValues();
            Main_cpp(&devices_data);
            change_state = false;
    		end_contact = false;
        }
        // Принудительное включение/выключение устройства (отключение оборудования)
        if(regime_change) {
			alarm_rtc();
            DevicesGetValues();
            Main_cpp(&devices_data);
            regime_change = false;
        }
        // Установка времени
        if(receive_sync_time) {
        	RTC_TimeTypeDef set_time = set_sync_time;
        	HAL_RTC_SetDate(&hrtc, &null_date, RTC_FORMAT_BIN);
			HAL_RTC_SetTime(&hrtc, &set_time, RTC_FORMAT_BIN);
#if defined( TIME_ADJUSTMENT )
        		time_subtraction = clock_subtraction(&time, &time_adjustment);
        		time = set_time;
        		time_adjustment = time;
        		clock_add_second(&time_adjustment, -clock_get_sec(&time_subtraction));
#endif
			alarm_rtc();
            receive_sync_time = false;
            change_state = true;
        }


        // Управление каналами по дню и ночи
#if defined (USE_LORA)
    	if(Get_stop()) {
    		get_stop = true;
//            relay_set_state(&relay[1], GPIO_PIN_RESET);
//            relay_set_state(&relay[0], GPIO_PIN_RESET);
            stop();
            last_devices_change = HAL_GetTick();
			devices_state = 0;
    	}
    	else
#endif
        {
//    		volatile bool set_pwm = false;
//    		volatile uint16_t set_pump_pwm = 4095; // set from server
//    		uint16_t pump_pwm = 4095; // set in main
    		get_stop = false;
    		static unsigned long pump_on;
    		static unsigned long valve_on;
    		static unsigned long pump_off;
    		if(set_pwm) { // set PWM from server
    			set_pwm = false;
    			pump_pwm = set_pump_pwm;
    			Write_PWM_to_flash(pump_pwm);
    		}
//    		static uint16_t pump_pwm;
            if(all_channel[0].state) {
            	pump_on = PUMP_ON_DAY;
            	pump_off = PUMP_OFF_DAY;
                valve_on = VALVE_ON_DAY;
//				pump_pwm = PUMP_PWM_DAY;
            }
            else {
            	pump_on = PUMP_ON_NIGHT;
            	pump_off = PUMP_OFF_NIGHT;
                valve_on = VALVE_ON_NIGHT;
//        		pump_pwm = PUMP_PWM_NIGHT;
            }
        	switch(devices_state) {
        	case 0: {
        		if(pump_pwm != 0)
        			start();
	            set_pwm_pch(pump_pwm);
        		if(HAL_GetTick() - last_devices_change > pump_on) {
        			last_devices_change = HAL_GetTick();
//					relay_set_state(&relay[1], GPIO_PIN_SET);
//					relay_set_state(&relay[0], GPIO_PIN_RESET);
					devices_state = 1;
					use_valve();
		            DevicesGetValues();
		            Main_cpp(&devices_data);
        		}
        		break;
        	}
        	case 1: {
        		use_valve();
        		if(HAL_GetTick() - last_devices_change > valve_on) {
        			last_devices_change = HAL_GetTick();
//                    relay_set_state(&relay[1], GPIO_PIN_RESET);
//                    relay_set_state(&relay[0], GPIO_PIN_SET);
					devices_state = 2;
		            stop();
		            DevicesGetValues();
		            Main_cpp(&devices_data);
        		}
        		break;
        	}
        	case 2: {
	            stop();
        		if(HAL_GetTick() - last_devices_change > pump_off - valve_on) {
        			last_devices_change = HAL_GetTick();
//                    relay_set_state(&relay[1], GPIO_PIN_RESET);
//                    relay_set_state(&relay[0], GPIO_PIN_RESET);
					devices_state = 0;
		            start();
		            set_pwm_pch(pump_pwm);
		            DevicesGetValues();
		            Main_cpp(&devices_data);
        		}
        		break;
        	}
        	}
        }



//    	volatile static uint16_t set_pwm = 1000;
//		start();
//    	set_pwm_pch(set_pwm);
//    	stop();
////    	HAL_Delay(5000);
//    	set_pwm_pch(2047);
////    	HAL_Delay(5000);
//    	set_pwm_pch(3071);
////    	HAL_Delay(5000);
//    	set_pwm_pch(4095);
////    	HAL_Delay(5000);
//    	stop();
//    	HAL_Delay(10000);



    	if(Get_stop()) {
    	    HAL_GPIO_WritePin(LED1_PIN_GPIO_Port, LED1_PIN_Pin, GPIO_PIN_RESET);
    	    HAL_GPIO_WritePin(LED2_PIN_GPIO_Port, LED2_PIN_Pin, GPIO_PIN_SET);
//    	    HAL_GPIO_WritePin(LED2_PIN_GPIO_Port, LED2_PIN_Pin, GPIO_PIN_RESET);
//    	    HAL_GPIO_WritePin(LED3_PIN_GPIO_Port, LED3_PIN_Pin, GPIO_PIN_SET);
    	}
    	else {
    	    HAL_GPIO_WritePin(LED2_PIN_GPIO_Port, LED2_PIN_Pin, GPIO_PIN_RESET);
    	    HAL_GPIO_WritePin(LED1_PIN_GPIO_Port, LED1_PIN_Pin, GPIO_PIN_SET);
//    	    HAL_GPIO_WritePin(LED3_PIN_GPIO_Port, LED3_PIN_Pin, GPIO_PIN_RESET);
//    	    HAL_GPIO_WritePin(LED2_PIN_GPIO_Port, LED2_PIN_Pin, GPIO_PIN_SET);
	    }


        HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	if (end_contact) { // not use
//		end_contact = false;
//		HAL_Delay(100);
//		HAL_TIM_Base_Start_IT(&htim4);
//		HAL_Delay(100);
//		LoRa_sleep();
//		HAL_SuspendTick();
//		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
//		HAL_ResumeTick();
//		HAL_TIM_Base_Stop_IT(&htim4);
//		DevicesGetValues();
//		Main_cpp(&devices_data);
//	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4095;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LORA_NSS_Pin|LED3_PIN_Pin|LED2_PIN_Pin|LED1_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LORA_RESET_Pin|RELAY4_PIN_Pin|RELAY3_PIN_Pin|RELAY2_PIN_Pin
                          |RELAY1_PIN_Pin|RELAY5_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LORA_DIO2_Pin LORA_DIO4_Pin LORA_DIO5_Pin */
  GPIO_InitStruct.Pin = LORA_DIO2_Pin|LORA_DIO4_Pin|LORA_DIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_NSS_Pin LED3_PIN_Pin LED2_PIN_Pin LED1_PIN_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin|LED3_PIN_Pin|LED2_PIN_Pin|LED1_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_DIO1_Pin LORA_DIO0_Pin */
  GPIO_InitStruct.Pin = LORA_DIO1_Pin|LORA_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_RESET_Pin RELAY4_PIN_Pin RELAY3_PIN_Pin RELAY2_PIN_Pin
                           RELAY1_PIN_Pin RELAY5_PIN_Pin */
  GPIO_InitStruct.Pin = LORA_RESET_Pin|RELAY4_PIN_Pin|RELAY3_PIN_Pin|RELAY2_PIN_Pin
                          |RELAY1_PIN_Pin|RELAY5_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_DIO3_Pin */
  GPIO_InitStruct.Pin = LORA_DIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_DIO3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin== GPIO_PIN_2) {
    exti2++;
    //HAL_ResumeTick();
    Contact_group_control_module();
  } else if(GPIO_Pin== GPIO_PIN_10){
    exti5_10++;
    //HAL_ResumeTick();
    Contact_group_control_module();
  } else{
    __NOP();
  }
}

void alarm_rtc() {
	RTC_AlarmTypeDef alarm = {.Alarm=1};
	alarm.AlarmTime = find_alarm_clock(all_channel, AMT_CHANNEL);

#if defined (USE_LORA)
	if(Get_stop()) {
		for(int i = 0; i < AMT_CHANNEL; ++i) {
			all_channel[i].state = false;
			if(all_channel[i].relay != NULL) // если есть реле, то и его переключаем
				relay_set_state(all_channel[i].relay, all_channel[i].state);
		}
	}
	else
#endif
		for(int i = 0; i < AMT_CHANNEL; ++i)
			check_state_by_RTC(&all_channel[i]);
	HAL_RTC_SetAlarm(&hrtc, &alarm, RTC_FORMAT_BIN);
	HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN);
#if defined(DEBUG)
	next_time = alarm.AlarmTime;
#endif
	change_state = true;
}

void DevicesGetValues() {
//	devices_data.device1 = freq_signal.compare;
	if(relay_start->state)
		devices_data.device1 = 4095;
	else
		devices_data.device1 = 0;


//	if(relay[0].state)
//		devices_data.device1 = 4095;
//	else
//		devices_data.device1 = 0;
//	if(relay[1].state)
//		devices_data.device2 = 4095;
//	else
//		devices_data.device2 = 0;
//	if(relay[2].state)
//		devices_data.device3 = 4095;
//	else
//		devices_data.device3 = 0;


#ifdef _TSL2561
	unsigned int data0, data1;
	if (TSL2561_getData(&data0, &data1))
	{
		double lux;
		TSL2561_getLux(gain, ms, data0, data1, &lux);
		sensors_data.lux = lux;
		//lora_sensor_set_data(&illumination_sensor,(float)lux);
	}
#endif

#ifdef _BME280
	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
	if(rslt == BME280_OK)
	{
		sensors_data.temperature = comp_data.temperature / 100.0;      /* C  */
		sensors_data.humidity = (comp_data.humidity / 1024.0) + offset.humidity_offset;           /* %   */
		sensors_data.pressure = comp_data.pressure / 10000.0 / 1.333;  /* hPa or mmhg */
	}
#endif

#ifdef _CCS811
	uint32_t CO2_tVOC_res = 0;

	CO2_tVOC_res = readAlgorithmResults();
	sensors_data.CO2 = (uint16_t)(CO2_tVOC_res >> 16);
	sensors_data.TVOC = (uint16_t)CO2_tVOC_res;
#endif

#ifdef _WATER_TEMP
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	water_temp = HAL_ADC_GetValue(&hadc1);
	sensors_data.water_temperature = water_temp;
	HAL_ADC_Stop(&hadc1);
#endif
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
