/* USER CODE BEGIN Header */
/**
  ******************************************************************************
	*
  * @AUTHOR       :		Nguyen Trong Son
	* @CLASS        : 	69DCCN21
	* @ID           :   69DCCO20164
	* @University   :   University Of Transport Technology
	* @Title        :   Do An Tot Nghiep
	* @Topic        :   May say nong san on dinh nhiet do
	*
	* @File         :   main.c
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "define.h"
#include "delay_timer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Wellcome(void)
{
	CLCD_I2C_SetCursor(&LCD1,0,0);
	CLCD_I2C_WriteString(&LCD1, "DO AN TOT NGHIEP");
	HAL_Delay(1000);
	CLCD_I2C_Clear(&LCD1);
	CLCD_I2C_SetCursor(&LCD1,0,0);
	CLCD_I2C_WriteString(&LCD1, "LOADDING...");
	HAL_Delay(1000);
	CLCD_I2C_Clear(&LCD1);
}

// Hien thi thong so khi da cai xong kp ki kd
static void Display_Save_Parmeter(void)
{
	CLCD_I2C_SetCursor(&LCD1, 0, 0);
	CLCD_I2C_WriteString(&LCD1, "COPYRIGHT BY SON");
	CLCD_I2C_SetCursor(&LCD1, 0, 1);
	CLCD_I2C_WriteString(&LCD1, "NHAN NEXT =>...");
}

// Cai dat thong so Kp
static void Set_Kp(PID_CONTROL *PID1)
{
	if(HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == 0)
	{
			cnt1++;
			if(cnt1 >= 10)
			{
				if(PID1->Kp < KP_MAX)
				{
					PID1->Kp += KP_UP;
				}
				else
				{
					PID1->Kp = KP_MAX;
				}
				cnt1 = 0;
			}
	}
	
	if(HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == 0)
	{
			cnt1++;
			if(cnt1 >= 10)
			{
				if(PID1->Kp > KP_MIN)
				{
					PID1->Kp -= KP_DOWN;
				}
				else
				{
					PID1->Kp = KP_MIN;
				}
				cnt1 =0;
			}
	}
	Save_Data_Kp(ADDRESS_Kp, PID1->Kp);
}

// Cai Dat Thong So Ki
static  void Set_Ki(PID_CONTROL *PID1)
{
	if(HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == 0)
	{
			cnt2++;
			if(cnt2>=10)
			{
				if(PID1->Ki < KI_MAX)
				{
					PID1->Ki += KI_UP;
				}
				else
				{
					PID1->Ki = KI_MAX;
				}
				cnt2 = 0;
			}
	}
	
	if(HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == 0)
	{
			cnt2++;
			if(cnt2>=10)
			{
				if(PID1->Ki > KI_MIN)
				{
					PID1->Ki -= KI_DOWN;
				}
				else
				{
					PID1->Ki = KI_MIN;
				}
				cnt2 =0;
			}
	}
	Save_Data_Ki(ADDRESS_Ki, PID1->Ki);
}

// Cai Dat Thong So Kd
static  void Set_Kd(PID_CONTROL *PID1)
{
	if(HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == 0)
	{
			cnt3++;
			if(cnt3 >= 10)
			{
				if(PID1->Kd < KD_MAX)
				{
					PID1->Kd += KD_UP;
				}
				else
				{
					PID1->Kd = KD_MAX;
				}
				cnt3 = 0;
			}
	}
	
	if(HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == 0)
	{
			cnt3++;
			if(cnt3>=10)
			{
				if(PID1->Kd > KD_MIN)
				{
					PID1->Kd -= KD_DOWN;
				}
				else
				{
					PID1->Kd = KD_MIN;
				}
				cnt3 = 0;
			}
	}
	Save_Data_Kd(ADDRESS_Kd, PID1->Kd);
}

// Cai Dat nhiet do
static void ButtonSetTemperature(uint8_t *set_temp)
{
	// Tang Nhiet Do
	if(HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == 0)
	{
			cnt4++;
			if(cnt4 >= 5)
			{
				if(*set_temp < TEMP_SET_MAX)
				{
					*set_temp += TEMP_UP;
				}
				else
				{
					*set_temp = TEMP_SET_MAX;
				}
				cnt4 = 0;
		  }
	}
	
	// Giam Nhiet Do
	if(HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == 0)
	{
			cnt4++;
			if(cnt4 >= 5)
			{
				if(*set_temp > TEMP_SET_MIN)
				{
					*set_temp -= TEMP_DOWN;
				}
				else
				{
					*set_temp = TEMP_SET_MIN;
				}
				cnt4 = 0;
		  }
	}
	save_data_temp(Address,*set_temp);
}

// Luu nhiet do cai dat vao eeproom
int16_t save_data_temp(uint32_t add, int value)	
{ 
	// Kiem tra cac bien, dia chi cua bien 
	if((last_save_value != value)|| (last_add != add))	
	{
			Flash_Erase(add);
			Flash_Write_Int(add, value);
			last_save_value = value;
			last_add = add;
			
			if(Flash_Read_Int(add) == value)
				return 1;
			else
				return 0;
	}
	return 0;
}

// Luu gia tri KP
static float Save_Data_Kp(uint32_t add_kp, float Kp)
{
	if((last_save_Kp != Kp) || (last_add_Kp != add_kp))
	{
		Flash_Erase(add_kp);
		Flash_Write_Float(add_kp, Kp);
		last_save_Kp = Kp;
		last_add_Kp = add_kp;
		
		if(Flash_Read_Float(add_kp) == Kp)
		{	
			return 1;
		}
		else 
		{
			return 0;
		}
	}
		return 0;
}

// Luu gia tri Ki
static float Save_Data_Ki(uint32_t add_ki, float Ki)
{
	if((last_save_Ki != Ki) || (last_add_Ki != add_ki))
	{
		Flash_Erase(add_ki);
		Flash_Write_Float(add_ki, Ki);
		last_save_Ki = Ki;
		last_add_Ki = add_ki;
		
		if(Flash_Read_Float(add_ki) == Ki)
		{
			return 1;
		}
		else 
		{
			return 0;
		}
	}
	return 0;
}

// Luu Gia tri Kd
static float Save_Data_Kd(uint32_t add_kd, float Kd)
{
	if((last_save_Kd != Kd) || (last_add_Kd != add_kd))
	{
		Flash_Erase(add_kd);
		Flash_Write_Float(add_kd, Kd);
		last_save_Kd = Kd;
		last_add_Kd = add_kd;
		
		if(Flash_Read_Float(add_kd) == Kd)
		{
			return 1;
		}
		else 
		{
			return 0;
		}
		return 0;
	}
}

// Hien thi thong so Kp
static void Display_Parameter_KP(PID_CONTROL *PID1)
{
		sprintf(data_Kp, "%.2f", PID1->Kp);
		CLCD_I2C_SetCursor(&LCD1, 0,0);
		CLCD_I2C_WriteString(&LCD1, "Cai Dat Kp");
		CLCD_I2C_SetCursor(&LCD1, 0, 1);
		CLCD_I2C_WriteString(&LCD1, data_Kp);
}

// Hien thi thong so Ki
static void Display_Parameter_Ki(PID_CONTROL *PID1)
{
		sprintf(data_Ki, "%.5f", PID1->Ki);
		CLCD_I2C_SetCursor(&LCD1, 0, 0);
		CLCD_I2C_WriteString(&LCD1, "Cai Dat Ki");
		CLCD_I2C_SetCursor(&LCD1, 0, 1);
	  CLCD_I2C_WriteString(&LCD1, data_Ki);
		CLCD_I2C_SetCursor(&LCD1, 7, 1);
		CLCD_I2C_WriteChar(&LCD1, ' ');
}

// Hien thi thong Kd
static void Display_Parameter_Kd(PID_CONTROL *PID1)
{
		sprintf(data_Kd, "%.4f", PID1->Kd);
		CLCD_I2C_SetCursor(&LCD1, 0, 0);
		CLCD_I2C_WriteString(&LCD1, "Cai Dat Kd");
		CLCD_I2C_SetCursor(&LCD1, 0, 1);
		CLCD_I2C_WriteString(&LCD1, data_Kd);
}

// Hien thi nhiet do cai dat
static void Display_SetUp_Temp(uint8_t *temp)
{
		if(*temp<=99)
		{
			sprintf(data_temp, "%d", *temp);
			CLCD_I2C_SetCursor(&LCD1, 0,0);
			CLCD_I2C_WriteString(&LCD1, "Cai Dat Nhiet Do");
			CLCD_I2C_SetCursor(&LCD1, 0,1);
			CLCD_I2C_WriteString(&LCD1, data_temp);
			CLCD_I2C_SetCursor(&LCD1, 2,0);
			CLCD_I2C_WriteChar(&LCD1, 0xDF);
			CLCD_I2C_SetCursor(&LCD1, 3,0);
			CLCD_I2C_WriteChar(&LCD1, 'C');
			CLCD_I2C_SetCursor(&LCD1, 4,0);
			CLCD_I2C_WriteChar(&LCD1, ' ');
		}
		
		// Hien thi nhiet do lon hon 100
		if(*temp>=100)
		{
			sprintf(data_temp, "%d", *temp);
			CLCD_I2C_SetCursor(&LCD1, 0,0);
			CLCD_I2C_WriteString(&LCD1, "Cai Dat Nhiet Do");
			CLCD_I2C_SetCursor(&LCD1, 0,0);
			CLCD_I2C_WriteString(&LCD1, data_temp);
			CLCD_I2C_SetCursor(&LCD1, 3,0);
			CLCD_I2C_WriteChar(&LCD1, 0xDF);
			CLCD_I2C_SetCursor(&LCD1, 4,0);
			CLCD_I2C_WriteChar(&LCD1, 'C');
		}
		
}

// Hien thi nhiet do
static void Display_LCD_Temp(float *temp)
{
		if(*temp<=99)
		{
			sprintf(data_temp, "%0.0f", *temp);
			CLCD_I2C_SetCursor(&LCD1, 0,0);
			CLCD_I2C_WriteString(&LCD1, "T:");
			CLCD_I2C_SetCursor(&LCD1, 2,0);
			CLCD_I2C_WriteString(&LCD1, data_temp);
			CLCD_I2C_SetCursor(&LCD1, 4,0);
			CLCD_I2C_WriteChar(&LCD1, 0xDF);
			CLCD_I2C_SetCursor(&LCD1, 5,0);
			CLCD_I2C_WriteChar(&LCD1, 'C');
			CLCD_I2C_SetCursor(&LCD1, 6,0);
			CLCD_I2C_WriteChar(&LCD1, ' ');
		}
		
		// Hien thi nhiet do lon hon 100
		if(*temp>=100)
		{
			sprintf(data_temp, "%0.0f", *temp);
			CLCD_I2C_SetCursor(&LCD1, 0,0);
			CLCD_I2C_WriteString(&LCD1, "T:");
			CLCD_I2C_SetCursor(&LCD1, 2,0);
			CLCD_I2C_WriteString(&LCD1, data_temp);
			CLCD_I2C_SetCursor(&LCD1, 5,0);
			CLCD_I2C_WriteChar(&LCD1, 0xDF);
			CLCD_I2C_SetCursor(&LCD1, 6,0);
			CLCD_I2C_WriteChar(&LCD1, 'C');
		}
		
}

// Hien thi nhiet do
static void Display_LCD_Set_Temp(uint8_t *set_temp)
{
		// Hien thi nhiet do cai dat be hon 100
		if(*set_temp <= 99)
		{
			sprintf(data_temp_set, "%d", *set_temp);
			CLCD_I2C_SetCursor(&LCD1, 0,1);
			CLCD_I2C_WriteString(&LCD1,  "ST:");
			CLCD_I2C_SetCursor(&LCD1, 3,1);
			CLCD_I2C_WriteString(&LCD1, data_temp_set);
			CLCD_I2C_SetCursor(&LCD1, 5,1);
			CLCD_I2C_WriteChar(&LCD1, 0xDF);
			CLCD_I2C_SetCursor(&LCD1, 6,1);
			CLCD_I2C_WriteChar(&LCD1, 'C');
			CLCD_I2C_SetCursor(&LCD1, 7,1);
			CLCD_I2C_WriteChar(&LCD1, ' ');
		}
		
		// Hien thi nhiet do lon hon 100
		if(*set_temp>=100)
		{
			sprintf(data_temp_set, "%d", *set_temp);
			CLCD_I2C_SetCursor(&LCD1, 0,1);
			CLCD_I2C_WriteString(&LCD1, "ST:");
			CLCD_I2C_WriteChar(&LCD1, ' ');
			CLCD_I2C_SetCursor(&LCD1, 3,1);
			CLCD_I2C_WriteString(&LCD1, data_temp_set);
			CLCD_I2C_SetCursor(&LCD1, 6,1);
			CLCD_I2C_WriteChar(&LCD1, 0xDF);
			CLCD_I2C_SetCursor(&LCD1, 7,1);
			CLCD_I2C_WriteChar(&LCD1, 'C');
		}
}

static void Display_Run(PID_CONTROL *PID1)
{
	  if((uint16_t)PID1->OutPut >= 1000)
		{
			sprintf(data_pid_out, "%d", (uint16_t)PID1->OutPut);
			CLCD_I2C_SetCursor(&LCD1,8, 0);
			CLCD_I2C_WriteString(&LCD1, "PID:");
			CLCD_I2C_SetCursor(&LCD1,12, 0);
			CLCD_I2C_WriteString(&LCD1, data_pid_out);
		}
		
		if((uint16_t)PID1->OutPut <= 999)
		{
			sprintf(data_pid_out, "%d", (uint16_t)PID1->OutPut);
			CLCD_I2C_SetCursor(&LCD1,8, 0);
			CLCD_I2C_WriteString(&LCD1, "PID:");
			CLCD_I2C_SetCursor(&LCD1,12, 0);
			CLCD_I2C_WriteString(&LCD1, data_pid_out);
			CLCD_I2C_SetCursor(&LCD1,15, 0);
			CLCD_I2C_WriteChar(&LCD1, ' ');
		}
		
		if((uint16_t)PID1->OutPut <= 99)
		{
			sprintf(data_pid_out, "%d", (uint16_t)PID1->OutPut);
			CLCD_I2C_SetCursor(&LCD1,8, 0);
			CLCD_I2C_WriteString(&LCD1, "PID:");
			CLCD_I2C_SetCursor(&LCD1,12, 0);
			CLCD_I2C_WriteString(&LCD1, data_pid_out);
			CLCD_I2C_SetCursor(&LCD1,14, 0);
			CLCD_I2C_WriteChar(&LCD1, ' ');
			CLCD_I2C_SetCursor(&LCD1,15, 0);
			CLCD_I2C_WriteChar(&LCD1, ' ');
		}
		
		if((uint16_t)PID1->OutPut <= 9)
		{
			sprintf(data_pid_out, "%d", (uint16_t)PID1->OutPut);
			CLCD_I2C_SetCursor(&LCD1,8, 0);
			CLCD_I2C_WriteString(&LCD1, "PID:");
			CLCD_I2C_SetCursor(&LCD1,12, 0);
			CLCD_I2C_WriteString(&LCD1, data_pid_out);
			CLCD_I2C_SetCursor(&LCD1,13, 0);
			CLCD_I2C_WriteChar(&LCD1, ' ');
			CLCD_I2C_SetCursor(&LCD1,14, 0);
			CLCD_I2C_WriteChar(&LCD1, ' ');
			CLCD_I2C_SetCursor(&LCD1,15, 0);
			CLCD_I2C_WriteChar(&LCD1, ' ');
		}
	  sprintf(data_duty, "%.2f", VAC_OUT_MAX * (PID1->OutPut / 165));
	  CLCD_I2C_SetCursor(&LCD1, 8,1);
		CLCD_I2C_WriteString(&LCD1, "V:");
		CLCD_I2C_SetCursor(&LCD1, 10,1);
		CLCD_I2C_WriteString(&LCD1, data_duty);
}

// Kiem tra ket qua bo PID voi chu ky 1000ms
static void PID_Reload_Result(PID_CONTROL *PID1, uint8_t *set_temp, float *temp_sensor)
{
				currentMillis = HAL_GetTick();
	
				if(currentMillis - previousMillis >= INV_SAMPLING_TIME_PID)
				{
						previousMillis += INV_SAMPLING_TIME_PID;
							
					 *temp_sensor = DS18B20_ReadTemp(&DS1);
			
						PID1->Err_now = (float)(*set_temp - *temp_sensor);
			
						if(PID1->Err_now > 20)
						{
								PID1->iPart = 0;
						}
						
						PID1->pPart = PID1->Kp * PID1->Err_now;
						
						PID1->iPart += PID1->Ki * PID1->Err_now; 
						
						if(PID1->iPart > PI_OUTPUT_MAX)
						{
							PID1->iPart = PI_OUTPUT_MAX;
						}
						
						if(PID1->iPart < PI_OUTPUT_MIN)
						{
							PID1->iPart = PI_OUTPUT_MIN;
						}
						
						timePrev = time;
						time = HAL_GetTick();
						elapsedTime = (time - timePrev) / 1000;
						
						PID1->dPart = PID1->Kd * ((PID1->Err_now - PID1->Err_last) / elapsedTime);
						
		//				PID1->dPart = PID1->Kd * ((float)(PID1->Err_now - PID1->Err_last) / SAMPLING_TIME);
						
						PID1->OutPut = PID1->pPart + PID1->iPart + PID1->dPart;
						
						if(PID1->OutPut > PID_OUTPUT_MAX)
						{
							PID1->OutPut = PID_OUTPUT_MAX;
						}
						
						if(PID1->OutPut < PID_OUTPUT_MIN)
						{
							PID1->OutPut = PID_OUTPUT_MIN; 
						}	
						
						PID1->Err_last = PID1->Err_now; 
			}
}


// Che do cai dat Kp, Ki, Kd
static void Set_Parmeter(FLAG_NAME *flag, PID_CONTROL *PID1, uint8_t *set_temp)
{
	if(flag->flag_7 == 0)
	{
		if(HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin) == 0)
		{
				CLCD_I2C_Clear(&LCD1);
				if(mode1 < 3) mode1++;
				else mode1 = 0;
				while(HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin) == 0);
		}
		if(mode1 == 0)
		{
			Display_Parameter_KP(PID1);
			Set_Kp(PID1);
		}
		else if(mode1 == 1)
		{ 
			Display_Parameter_Ki(PID1);
			Set_Ki(PID1);
		}
		else if(mode1 == 2)
		{
			Display_Parameter_Kd(PID1);
			Set_Kd(PID1);
		}
		else if(mode1 == 3)
		{
			Display_SetUp_Temp(set_temp);
			ButtonSetTemperature(set_temp);
		}
	}
}

// Chay che do hien hanh
static void Save_Prameter(PID_CONTROL *PID1, FLAG_NAME *flag, uint8_t *set_temp, float *temp_sensor, float *temp_f)
{
	if(HAL_GPIO_ReadPin(POWER_GPIO_Port, POWER_Pin) == 0)
	{
//		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(POWER_GPIO_Port, POWER_Pin) == 0)
		{
			CLCD_I2C_Clear(&LCD1);
			mode2++;
			while(HAL_GPIO_ReadPin(POWER_GPIO_Port, POWER_Pin) == 0);
		}
	}
	if(mode2 == 1)
	{
		  flag->flag_7 = 1;
			PID_Reload_Result(PID1, set_temp, temp_sensor);
			Display_LCD_Temp(temp_sensor);
			Display_LCD_Set_Temp(set_temp);
			Display_Run(PID1);
	}
	else if(mode2 == 2)
	{	
		if(PID1->OutPut > 0)
		{
			PID1->OutPut = PID_OUTPUT_MAX;
		}
		
		if(PID1->OutPut < 0)
		{
			PID1->OutPut = PID_OUTPUT_MIN; 
		}	
		
		flag->flag_7 = 0;
		mode2 = 0;
		mode1 = 0;
	}
}

// Save Kp, Ki, Kd, EEPROM
static void Save_KP_KI_KD_To_EEPROM(PID_CONTROL *PID1)
{
	if(HAL_GPIO_ReadPin(SAVE_GPIO_Port, SAVE_Pin) == 0)
	{
		Save_Data_Kp(ADDRESS_Kp, PID1->Kp);
		Save_Data_Kp(ADDRESS_Ki, PID1->Ki);
		Save_Data_Kp(ADDRESS_Kd, PID1->Kd);
		while(HAL_GPIO_ReadPin(SAVE_GPIO_Port, SAVE_Pin) == 0);
	}
	
}

// Doc nhiet do cam bien nhiet do
static void Temp_LM35(void)
{
//	ADC1_CH1_Value = HAL_ADC_GetValue(&hadc1);
	temp_f = (ADC1_CH1_Value * VREF * 100.0f) / ADC_12BIT_VALUE;
	if(temp_f > TEMP_MAX_START_FAN_F)
	{
		HAL_GPIO_WritePin(Fan_F_GPIO_Port, Fan_F_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(Fan_F_GPIO_Port, Fan_F_Pin, GPIO_PIN_RESET);
	}
}

// kiem tra quat lo say
static void Check_Fan_Temp(float *temp_sensor)
{
	if(*temp_sensor >= TEMP_MAX_START_FAN)
	{
		 HAL_GPIO_WritePin(Fan_Temp_GPIO_Port, Fan_Temp_Pin, GPIO_PIN_SET);
	}
	else
	{
		 HAL_GPIO_WritePin(Fan_Temp_GPIO_Port, Fan_Temp_Pin, GPIO_PIN_RESET);
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
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  DS18B20_Init(&DS1, &htim3, DS18B20_GPIO_Port, DS18B20_Pin);
	CLCD_I2C_Init(&LCD1, &hi2c1, 0x4e, 20, 4);
	HAL_ADC_Start(&hadc1);
	Set_Temp = Flash_Read_Int(Address);     										// Doc tu EEPROM 
	PID1.Kp = Flash_Read_Float(ADDRESS_Kp);
	PID1.Ki = Flash_Read_Float(ADDRESS_Ki);
	PID1.Kd = Flash_Read_Float(ADDRESS_Kd);
//  currentMillis = HAL_GetTick();
//  previousMillis = HAL_GetTick();
	Wellcome();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		currentMillis = HAL_GetTick();
////    if(currentMillis - previousMillis >= INV_SAMPLING_TIME_PID)
////		if(HAL_GetTick() - previousMillis >= INV_SAMPLING_TIME_PID)
//	if(currentMillis - previousMillis >= INV_SAMPLING_TIME_PID)
//		{
//			previousMillis += INV_SAMPLING_TIME_PID;
//			PID_Reload_Result(&PID1, &Set_Temp, &temperature);
////			previousMillis = HAL_GetTick();
//		}	
		Check_Fan_Temp(&temperature);
		Set_Parmeter(&flag, &PID1, &Set_Temp);
		Save_Prameter(&PID1,&flag, &Set_Temp, &temperature, &temp_f);
		ADC1_CH1_Value = HAL_ADC_GetValue(&hadc1);
		Temp_LM35();
	}
  /* USER CODE END 3 */
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 36;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Fan_Temp_Pin|Fan_F_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Ctrl_Prd_Triac_Pin|DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : UP_Pin DOWN_Pin POWER_Pin */
  GPIO_InitStruct.Pin = UP_Pin|DOWN_Pin|POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Fan_Temp_Pin Fan_F_Pin */
  GPIO_InitStruct.Pin = Fan_Temp_Pin|Fan_F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MODE_Pin SAVE_Pin */
  GPIO_InitStruct.Pin = MODE_Pin|SAVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Zero_Crossing_Pin */
  GPIO_InitStruct.Pin = Zero_Crossing_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Zero_Crossing_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Ctrl_Prd_Triac_Pin DS18B20_Pin */
  GPIO_InitStruct.Pin = Ctrl_Prd_Triac_Pin|DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

