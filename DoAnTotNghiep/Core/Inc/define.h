/******************************************************************************************************************
  * @AUTHOR       :		Nguyen Trong Son
	* @CLASS        : 	69DCCN21
	* @ID           :   69DCCO20164
	* @University   :   University Of Transport Technology
	* @Title        :   Do An Tot Nghiep
	* @Topic        :   May say nong san on dinh nhiet do
	*
	* @Filename     :   define.c
******************************************************************************************************************/

#ifndef __DEFINE_H
#define __DEFINE_H

#include "DS18B20.h"
#include "CLCD_I2C.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "flash.h"
#include "stdbool.h"
#include "stm32f1xx.h" 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#pragma pack(1)
typedef enum
{
	MODE,  // Tat May
	MODE1  // Bat May
} CRR_MODE;
//#pragma pack(0)

// Khai bao co ngat
//#pragma pack(1)
typedef struct
{
	bool flag_1, flag_2, flag_3,flag_4, flag_5, flag_6, flag_7, flag_8, flag_9, flag_10, flag_11, flag_12; 
} FLAG_NAME;
//#pragma pack(0)

// Bo PID
//#pragma pack(1)
typedef struct
{
	float OutPut, Kp, Ki, Kd, Err_now, pPart, iPart, dPart, Err_last;
} PID_CONTROL;
//#pragma pack(0)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Nhiet do cai dat lon nhat va nho nhat
#define TEMP_SET_MAX 								70
#define TEMP_SET_MIN 								20

// Dia chi EEPROOM
#define Address 										0x800FC00
#define ADDRESS_Kp 									0x800F800
#define ADDRESS_Ki 									0x800F400
#define ADDRESS_Kd 									0x800F000

// Nguong che do max
#define MODE_MAX 										1
#define SELEC_MODE_MAX 							3

#define STOP 		  									"STOP"
#define RUN 		   									"RUN"

// Nguong cai dat thong so bo dieu khien PID
#define KP_MAX 											1000
#define KP_MIN											0

#define KI_MAX 											5
#define KI_MIN 											0

#define KD_MAX 											200
#define KD_MIN 											0

// Cai Dat thong so bo dieu khien PID
#define KP_UP   										0.1f
#define KP_DOWN 										0.1f

#define KI_UP   										0.00001f
#define KI_DOWN 										0.00001f

#define KD_UP   										0.0001f
#define KD_DOWN 										0.0001f

// Cai dat nhiet do
#define TEMP_UP											1
#define TEMP_DOWN                   1

// Thoi gian Reload bo dieu khien PID
#define SAMPLING_TIME               1.0f
#define COUNT_PERIOD_MAX            165
#define INV_SAMPLING_TIME_PID       1000

// MAX hoac MIN dau ra bo PID
#define PID_OUTPUT_MAX              165
#define PID_OUTPUT_MIN              0
#define PI_OUTPUT_MAX               165
#define PI_OUTPUT_MIN               -165

// ADC
#define VREF 												3.3f
#define ADC_12BIT_VALUE             4095
#define INV_SAMPLING_TIME_ADC       150

// Nhiet do Max de bat quat lo say
#define TEMP_MAX_START_FAN          27

// Nhiet do Max de bat quat tan nhiet cho van cong suat
#define TEMP_MAX_START_FAN_F       25

#define VAC_OUT_MAX                220.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

DS18B20_Name DS1;
CLCD_I2C_Name LCD1;

// Setup cac thong so dau vao cho bo PID
PID_CONTROL PID1 = 
{
	.Kp = 0,   		// 100
	.Ki = 0, 			// 0.025
	.Kd = 0,    	// 20
	.OutPut = 0,
	.Err_now = 0,
	.pPart = 0,
	.iPart = 0,
	.dPart = 0,
	.Err_last = 0
};


// Setup cac trang thai co ngat
FLAG_NAME flag = 
{
	.flag_1 = 0,
	.flag_2 = 0,
	.flag_3 = 0,
  .flag_4 = 0,
	.flag_5 = 0,
	.flag_6 = 0,
	.flag_8 = 0,
	.flag_9 = 0,
	.flag_10 = 0,
	.flag_11 = 0,
	.flag_12 = 0

};


float temperature = 0, a = 0, last_temperature = 0;
uint8_t Set_Temp = 20, mode=0, mode1 = 0, mode2 = 0 , cnt1 , cnt2, cnt3, cnt4=0, i, mode3=0;
uint16_t cnt = 0, Cnt_Prd_Ctl_Triac = 0;
uint32_t ADC1_CH1_Value = 0, currentMillis = 0, previousMillis = 0;
float time = 0, timePrev = 0, elapsedTime = 0;

// Save data temp, set temp
char data_temp[3];
char data_temp_set[3];
char data_pid_out[3];
char data_Kp[7];
char data_Ki[10];
char data_Kd[10];
char data_duty[7];

//save data to flash
uint32_t last_add = 0;
uint16_t last_save_value = 0;

// Save data Kp Ki Kd to flash
uint32_t last_add_Kp = 0, last_add_Ki = 0, last_add_Kd = 0;
float last_save_Kp=0, last_save_Ki=0, last_save_Kd=0;
float temp_f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_TIM3_Init(void);
//static void MX_I2C1_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

static void Wellcome(void);
static void Display_Save_Parmeter(void);

static void ButtonSetTemperature(uint8_t *set_temp);

static void Display_LCD_Temp(float *temp);
static void Display_LCD_Set_Temp(uint8_t *set_temp);

static void Display_Run(PID_CONTROL *PID1);

static int16_t save_data_temp(uint32_t add, int value)	;

static  void Set_Kp(PID_CONTROL *PID1);
static  void Set_Ki(PID_CONTROL *PID1);
static  void Set_Kd(PID_CONTROL *PID1);

static float Save_Data_Kp(uint32_t add_kp, float Kp);
static float Save_Data_Ki(uint32_t add_ki, float Ki);
static float Save_Data_Kd(uint32_t add_kd, float Kd);

static void PID_Reload_Result(PID_CONTROL *PID1, uint8_t *set_temp, float *temp_sensor);


static void Set_Parmeter(FLAG_NAME *flag, PID_CONTROL *PID1, uint8_t *set_temp);
static void Save_Prameter(PID_CONTROL *PID1, FLAG_NAME *flag, uint8_t *set_temp, float *temp_sensor, float *temp_f);

static void Display_Parameter_KP(PID_CONTROL *PID1);
static void Display_Parameter_Ki(PID_CONTROL *PID1);
static void Display_Parameter_Kd(PID_CONTROL *PID1);
static void Display_SetUp_Temp(uint8_t *temp);

static void Temp_LM35(void);
static void Check_Fan_Temp(float *temp_sensor);

static void Save_KP_KI_KD_To_EEPROM(PID_CONTROL *PID1);

/*

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	// Timer4 interrupt  = 100us 
	// T 220VAC = 20ms => T/2 = 10ms => thoi gian kich mo triac nam trong khoang tu 0->T/2
	// => Cnt_Prd_Ctl_Triac++ == 10000us = 10ms 
	if(htim->Instance == htim4.Instance)
	{
		Cnt_Prd_Ctl_Triac++;
		if(Cnt_Prd_Ctl_Triac == (uint16_t)(COUNT_PERIOD_MAX - PID1.OutPut))
		{
			HAL_GPIO_WritePin(Ctrl_Prd_Triac_GPIO_Port, Ctrl_Prd_Triac_Pin, GPIO_PIN_SET);
		}
		else if(Cnt_Prd_Ctl_Triac == COUNT_PERIOD_MAX)
		{
			HAL_GPIO_WritePin(Ctrl_Prd_Triac_GPIO_Port, Ctrl_Prd_Triac_Pin, GPIO_PIN_RESET);
			Cnt_Prd_Ctl_Triac = 0;
			HAL_TIM_Base_Stop_IT(&htim4);
			__HAL_TIM_SetCounter(&htim4, 0);
		}	
		
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == Zero_Crossing_Pin)
	{
//		HAL_GPIO_WritePin(Fan_Temp_GPIO_Port, Fan_Temp_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Ctrl_Prd_Triac_GPIO_Port, Ctrl_Prd_Triac_Pin, GPIO_PIN_RESET);
		HAL_TIM_Base_Start_IT(&htim4);
	}
}

// IRQ ADC1_CH1
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == hadc1.Instance)
	{
		ADC1_CH1_Value = HAL_ADC_GetValue(&hadc1);
	}
}

*/

#endif