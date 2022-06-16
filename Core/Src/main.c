/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
unsigned char f_timer_10ms=0;
unsigned char f_timer_30ms=0;
unsigned char d_timer_30ms;
unsigned char key_value;
unsigned char curr_event;
unsigned char bufferEvent[64];

int digit;
char seven_segment_table[17] = {	0b1111110,	// '0'
		                            	0b0110000,	// '1'
		   	                          0b1101101,	// '2'
			                            0b1111001,	// '3'
			                            0b0110011,	// '4'
			                            0b1011011,	// '5'
			                            0b1011111,	// '6'
			                            0b1110000,	// '7'
			                            0b1111111,	// '8'
			                            0b1111011,	// '9'
			                            0b1111101,	// 'a'  --10
			                            0b0011111,	// 'b'  --11
			                            0b0001101,	// 'c'  --12
			                            0b0111101,	// 'd'  --13
			                            0b1101111,	// 'e'	--14
			                            0b1000111,	// 'f'  --15
			                            0b0000001 	// '-'  --16
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void task_timer(void);
void seven_segment_display(char input);
void key_read_task(void);
void main_task(void);
void setEvent(unsigned char event);
void getevent(unsigned char* event);
void uart_TX_task(void);
void uart_RX_task(void);
void Set_Transmitter_RS485(void);
void Set_Receiver_RS485(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_Base_Start_IT(&htim1);  

  //HAL_UART_Receive_IT(&huart1,&rx_buffer1[rx1_wp], 1);
  /* USER CODE END 2 */
  digit=0;
  seven_segment_display(seven_segment_table[digit]);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void task_timer(void)
{
  if (!f_timer_10ms) return;
  f_timer_10ms=0;

  d_timer_30ms++;

  if(d_timer_30ms==3){
    d_timer_30ms =0;
    f_timer_10ms=1;
  }
  

}

void seven_segment_display(char input)
{
   uint32_t mask =  a_Pin|b_Pin|c_Pin|d_Pin|e_Pin|f_Pin|g_Pin;
   uint32_t val = (uint32_t)input & mask;
   SEGMENT_PORT->ODR = val;
}

void key_read_task(void)
{
  if(!f_timer_30ms) return;
  unsigned char key_pindata = (uint8_t)(key_GPIO_Port->IDR & key_Pin);

  key_value = key_value<<1;
  key_value &= 0b00001110;
  key_value |= (key_pindata>>key_Pin)&0x1;

  if (key_value==KEY_PRESSED){

  }else if(key_value==KEY_RELEASED){

  }

}

void main_task(void)
{

}

void setEvent(unsigned char event)
{

}

void getevent(unsigned char* event)
{

}

void uart_TX_task(void)
{

}

void uart_RX_task(void)
{

}

void Set_Transmitter_RS485(void)
{

}

void Set_Receiver_RS485(void)
{

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim1)
  {
	  f_timer_10ms=1;
  }
  
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	if (huart == &huart1)
	{
	/*	rx1_wp++;
		HAL_UART_Receive_IT(&huart1, &rx_buffer1[rx1_wp], 1);
		 if(rx1_wp>63){
		    	rx1_wp=0;
		    }*/
	}


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
