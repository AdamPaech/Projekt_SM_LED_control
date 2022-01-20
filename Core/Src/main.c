/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bh_1750.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LUXTOMV_RATIO 0.4209
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//zadeklarowanie struktury czujnika
BH1750_HandleTypeDef hbh1750_1 = {
		&hi2c1, BH1750_ADDRESS_L, 200
};

BH1750_HandleTypeDef hbh1750_2 = {
		.I2C = &hi2c1, .Address = BH1750_ADDRESS_H, .Timeout = 0xffff
};
//zmienne dla czujnika
//	char cmd_msg[] = "000000";
//	float control;  //???????????????????
float light;
uint16_t light_calkowite;

//zmienne do przesyłu pomiaru na terminal
uint16_t data_msg[32];
int length;

//wypełnienie PWM
volatile uint16_t pwm = 0;

//bufor do zadawania wypełnienia
char user_val[4];

//zmienne ADC
const uint32_t ADC_REG_MAX = 0xfff; //12-bits
const float ADC_VOLTAGE_MAX = 3.3; //[V]
const uint32_t ADC_TIMEOUT = 100; //[as]

//ADC wyniki konwersji
volatile uint32_t ADC_measurement = 0; //wartość rejestru
volatile float ADC_voltage = 0; //wartośc napięcia w Voltach
volatile uint32_t ADC_voltage_mV = 0; //wartość napięcia w miliVoltach

//zmienna natężenia światła za pomocą potencjometru
volatile float lux_by_ADC = 0;

//zmienna natężenia światła za pomocą UART
volatile uint32_t lux_by_UART = 0;

//param PID
#define PID_PARAM_KP        0.5            /* Proporcional */
#define PID_PARAM_KI        0.1       /* Integral */
#define PID_PARAM_KD        0

//jakies deklaracje nwm
float pid_error = 0;
float duty = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
arm_pid_instance_f32 PID;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	PID.Kp = PID_PARAM_KP;
	PID.Ki = PID_PARAM_KI;
	PID.Kd = PID_PARAM_KD;

	arm_pid_init_f32(&PID, 1);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	//inicjalizacja czujnika światła
	BH1750_Init(&hbh1750_1);

	//wystartowanie timera odpowiadającego za cykliczny pomiar i realizację sterowania
	HAL_TIM_Base_Start_IT(&htim2);
	//wystartowanie PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	//przypisanie wypełnienia do kanału pwm
	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
	//nasłuch na komendę z terminala UART w trybie przerwaniowym
	HAL_UART_Receive_IT(&huart3, (uint8_t*)user_val, 4);

	//Start ADC w trybieprzerwaniowym
		  HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


//przerwanie od portu szeregowego przyjmujące i przypisujące do zmiennej zadaną wartość
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	//obliczenie żądanej wartości natężenia oświetlenia
	lux_by_UART = 1000*((int)user_val[0] - 48) + 100*((int)user_val[1] - 48) + 10*((int)user_val[2] - 48) + 1*((int)user_val[3] - 48);

	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
	HAL_UART_Receive_IT(&huart3, (uint8_t*)user_val, 4);
}

////przerwanie od ADC
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	const uint32_t ADC_REG_MAX = 0xfff; //12-bits
//	const float ADC_VOLTAGE_MAX = 3.3; //[V]
//	const uint32_t ADC_TIMEOUT = 100; //[as]
////	if(hadc->Instance == ADC1)
////	{
//		ADC_measurement = HAL_ADC_GetValue(&hadc1);
//		ADC_voltage = ((float)ADC_measurement / (float)ADC_REG_MAX) * ADC_VOLTAGE_MAX;
//		ADC_voltage_mV = (uint32_t)(1000.0*ADC_voltage);
////	}
//
//	lux_by_ADC = ADC_voltage_mV * LUXTOMV_RATIO;
//}

//przerwanie od timera następujące co odp czest.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// odczyt natężenia światła
	light = BH1750_ReadLux(&hbh1750_1);

	//
	pid_error = lux_by_UART - light;

	duty = arm_pid_f32(&PID, pid_error);

	if (duty > 1000) {
		duty = 1000;
	} else if (duty < 0) {
		duty = 0;
	}

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);


	// odczyt natężenia światła - całkowity
	light_calkowite = (uint16_t)light;

	//transmisja całkowitej wartości pomiaru na port szeregowy
//	length = sprintf(data_msg, "%6d\n\r", (int)light);
//	HAL_UART_Transmit(&huart3, (uint8_t*)data_msg, length, 0xffff);

	length = sprintf(data_msg, " POM: %d \[lux\] , REF: %d \[lux\] , STER: %d \r\n", (int)light,  lux_by_UART, (int)duty);
	HAL_UART_Transmit(&huart3, data_msg, length, 0xffff);
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
