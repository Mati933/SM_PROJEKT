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
#include "stdio.h"
#include "bh1750_config.h"
#include "PID.h"
#include "lcd_config.h"
#include <stdlib.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Sp5Kp2Kd0Ki0.5Z0 komenda UART

int value_PV;
int value_SP=10;
int value_U;
pid_str*PID;
float kp_init=4.38127169718089; //1.5
float ki_init=0.355762323613465; //0.5
float kd_init=0;
float zak=0;
float elo=0;
int anti_windup_limit_init=20;
int PWM=0;


//UART
#define LINE_MAX_LENGTH	80
static char line_buffer[LINE_MAX_LENGTH + 1];
char tablica[6]="STOP\0";
static uint32_t line_length;
uint8_t value;
int rand2;
uint32_t adc,adc2;

int __io_putchar(int ch)
{
	if(ch=='\n')
	{
		int ch2='\r';
		HAL_UART_Transmit(&huart3, (uint8_t*)&ch2, 1, HAL_MAX_DELAY);

	}
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
int a=5;
int c=10;
int m=89;
int los=0;
float x=0;
float losowa()
{


	    los=(a*los+c)%m;
	    x=(float)los/(float)m-0.5;
	    x=abs(x*400);

	    return x;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc==&hadc1)
	{
		adc=HAL_ADC_GetValue(&hadc1);
		if(abs(adc-adc2)>50)
		{

			value_SP=70*adc/4096;
			adc2=adc;
		}

	}

}

//PID
int j=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {

	 //PV
	 value_PV=BH1750_ReadLux(&hbh1750_1);


	 value_U=pid_calculate(PID, value_SP, value_PV);
	 PWM=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1);
	 PWM=PWM+value_U;
	 if(PWM<0)
	 {
		 PWM=0;
	 }
	 else if(PWM>1000)
	 {
		 PWM=1000;
	 }
	 printf("PV%dSP%dCV%d\n",value_PV,value_SP,PWM);
	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM);
	 HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	 float uchyb=((float)value_SP-(float)value_PV)/(float)value_SP*100;
	 LCD_SetCursor(&hlcd1, 0, 0);
	 LCD_printf(&hlcd1, "PWM: %d",PWM);
	 LCD_SetCursor(&hlcd1, 1, 0);
	 LCD_printf(&hlcd1, "Uchyb: %.2f",uchyb);


	  }

  if (htim == &htim10)
  {
	  if(zak==1)
	  {
	  rand2=losowa();
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, rand2);
	  j=1;
	  }
	  else if(zak==0&&j==1)
	  {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		j=0;
	  }

  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
	{
		if (value == '\r' || value == '\n')
			{

				if (line_length > 0)
				{

					line_buffer[line_length] = '\0';
					sscanf(line_buffer,"Sp%dKp%fKd%fKi%fZ%f",&value_SP,&kp_init,&kd_init,&ki_init,&zak);
					pid_reset(&PID);
					pid_init(PID,kp_init,ki_init,kd_init,anti_windup_limit_init);
					/*PWM=0;
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM);*/



					line_length = 0;
				}
			}
			else
			{
				if (line_length >= LINE_MAX_LENGTH)
				{
					// za dużo danych, usuwamy wszystko co odebraliśmy dotychczas
					line_length = 0;
				}
				// dopisujemy wartość do bufora
				line_buffer[line_length++] = value;
			}
	}
	  HAL_UART_Receive_IT(&huart3, &value,1);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  BH1750_Init(&hbh1750_1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_UART_Receive_IT(&huart3, &value,1);
  //pid
  pid_init(PID,kp_init,ki_init,kd_init,anti_windup_limit_init);
  LCD_Init(&hlcd1);







  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 HAL_ADC_Start_IT(&hadc1);



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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
