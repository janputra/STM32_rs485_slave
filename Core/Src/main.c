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
#include "message.h"
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
uint8_t f_busy;
uint8_t f_timer_10ms=0;
uint8_t f_timer_30ms=0;
uint8_t d_timer_30ms;
uint8_t key_value;
uint8_t curr_event;
//uint8_t bufferEvent[64];
uint8_t read_res;
uint8_t rx_temp;
uint8_t transmission_f;
uint8_t start_cmd=0x2;
uint8_t stop_cmd=0x3;
circular_buffer rx_buffer;
circular_buffer event_buffer;
uint8_t ID;
uint8_t TX_msg[4];
uint8_t RX_msg[4];
uint8_t state,event;
int digit;
uint8_t seven_segment_table[17] = {	0b0111111,	// '0'
		                            	 0b0000110,	// '1'
		   	                           0b1011011,	// '2'
			                             0b1001111,	// '3'
			                             0b1100110,	// '4'
                                   0b1101101,	// '5'
                                   0b1111101,	// '6'
                                   0b0000111,	// '7'
			                             0b1111111,	// '8'
                                   0b1101111,	// '9'
                                   0b1011111,	// 'a'  --10
                                   0b1111100,	// 'b'  --11
			                             0b1011000,	// 'c'  --14
                                   0b1011110,	// 'd'  --13
                                   0b1111011,	// 'e'	--14
                                   0b1110001,	// 'f'  --15
                                   0b1000000 	// '-'  --16
};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void task_timer(void);
void seven_segment_display(char input);
void key_read_task(void);
void main_task(void);
void RS485_Send_Message(void);
uint8_t RS485_Read_Message(void);
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

 	HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#ifdef SLAVE_1
                
  ID =0x10;
#endif
#ifdef SLAVE_2
                
  ID=0x20;
#endif
  state = STATE_WAITING_REQUEST;
  event = EVENT_RESET;
  digit=16;
  seven_segment_display(seven_segment_table[digit]);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
     
      
      task_timer();
      key_read_task();
      main_task();
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
    f_timer_30ms=1;
    
    
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
  f_timer_30ms =0; 
   
 
  unsigned char key_pindata = (uint8_t)(key_GPIO_Port->IDR & key_Pin);
 
  key_value = key_value<<1;
  key_value &= 0b00001110;
  key_value |= key_pindata&0X1;
 
  if (key_value==KEY_PRESSED){
      buffer_push(&event_buffer,EVENT_KEY_PRESSED);
      

  }else if(key_value==KEY_RELEASED){
      buffer_push(&event_buffer,EVENT_KEY_RELEASED);
  }

}

void main_task(void)
{
  	if (event_buffer.head!=event_buffer.tail){

		  event = buffer_pop(&event_buffer);   // if there is event then get the event from buffer

  }

    if(event == EVENT_KEY_PRESSED){
          digit++;
#ifdef SLAVE_1
                
          if (digit>9){
               digit=0;
             }
#endif
#ifdef SLAVE_2
                
          if (digit>15){
               digit=10;
            }
#endif
               
            seven_segment_display(seven_segment_table[digit]);
            RS485_Send_Message();
            event = EVENT_RESET;
          
          }
        else if(event == EVENT_RX_COMPLETE){
            
                  read_res= RS485_Read_Message();
                  if (read_res==MSG_READ_REQ){
                    
                    state=STATE_SENDING_RESPOND;
                }
                else if(read_res==MSG_WRITE_REQ){
                    f_busy=0;
                    digit = RX_msg[2]-'0';
                    seven_segment_display(seven_segment_table[digit]);
                    state= STATE_WAITING_REQUEST;
                    
                }/*else if(read_res==MSG_ERROR){
                      f_busy=0;
                      digit=16;
                      seven_segment_display(seven_segment_table[digit]);
                      state= STATE_WAITING_REQUEST;;
                    
                }
                */else{
                  f_busy=0;
                  state= STATE_WAITING_REQUEST;
      
                }
      
            event = EVENT_RESET;
        }


}

uint8_t RS485_Read_Message(void){

  if (rx_buffer.tail==rx_buffer.head) return MSG_NO_PROCESS;

  buffer_to_message(&rx_buffer, RX_msg);

  if (check_checksum(RX_msg)==CHECKSUM_ERROR) return MSG_ERROR;
  
  if (RX_msg[0]!= ID) return MSG_NO_PROCESS;

  if (RX_msg[1] == FUNC_WRITE)
  {
       return MSG_WRITE_REQ;

  }else if (RX_msg[1] == FUNC_READ){

      return MSG_READ_REQ;
  }
    
    return MSG_NO_PROCESS;

}

void RS485_Send_Message(void)
{
   TX_msg[0]= ID;
   TX_msg[1]= RX_msg[1];
   TX_msg[2]= digit+'0';

   cal_checksum(TX_msg);

   HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, 1); /// Enable Transmitter Mode
  
   HAL_UART_Transmit(&huart1,&start_cmd,1,10);
  
   HAL_UART_Transmit(&huart1,TX_msg,sizeof(TX_msg),10);

   HAL_UART_Transmit(&huart1,&stop_cmd,1,10);
  
   HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, 0); /// Enable Receiver Mode


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
    if(!transmission_f){
			if (rx_temp==0x2){
				transmission_f=1;
			}

		}else{
			if (rx_temp==0x3)
			{		//digit1= 2;
				transmission_f=0;
				buffer_push(&event_buffer,EVENT_RX_COMPLETE);
				HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
				return;
			}

 			buffer_push(&rx_buffer,rx_temp);
		}


		HAL_UART_Receive_IT(&huart1, &rx_temp, 1);
    
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
