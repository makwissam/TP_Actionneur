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
#define UART_RX_BUFFER_SIZE 32
#define UART_TX_BUFFER_SIZE 32
#define CMD_BUFFER_SIZE 32

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int idxCmd = 0;

int val=50;
uint8_t bouton = 0; //Variable servant au bouton
int valeur_ancienne = 0;
int valeur_nouvelle = 0;
int vitesse = 0;
int current = 0;

//Variables console
char uart_rx_buffer[UART_RX_BUFFER_SIZE];
char uart_tx_buffer[UART_TX_BUFFER_SIZE];
int it_uart_rx_ready=0;
int valCmd[4];

const uint8_t starting[32] = {"\r\n\r\n---Jour---\r\n"};
const uint8_t prompt[32] = {"<user>@Nucleo-G431 >> "};
char cmd[CMD_BUFFER_SIZE];

const uint8_t help[128] = {"\r\npinout:contient la liste des pin utilisées\r\nstart: démarre le moteur \r\nstop: Arrête le moteur\r\n"};
const uint8_t pinout[];
const uint8_t power_on[CMD_BUFFER_SIZE]={"\r\nLancement du Moteur\r\n"};
const uint8_t power_off[CMD_BUFFER_SIZE]={"\r\nArret du Moteur\r\n"};
const uint8_t not_found[CMD_BUFFER_SIZE]={"\r\nCommande inconnue\r\n"};

char backspace[] = " \b";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

//Début code console
void Uartprint(char * s){
	sprintf((char *)uart_tx_buffer,"%s",s);
	HAL_UART_Transmit(&huart2, uart_tx_buffer, strlen(s), HAL_MAX_DELAY);
	HAL_Delay(200);
}

void Uartreceive(){
	HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  it_uart_rx_ready=1;
}

void echo(){
	if(it_uart_rx_ready==1){
		uart_tx_buffer[0]=uart_rx_buffer[0];
		HAL_UART_Transmit(&huart2, uart_tx_buffer, 1, HAL_MAX_DELAY);
		it_uart_rx_ready=0;
	}
}

void Console(){
	Uartreceive();
	if (it_uart_rx_ready==1) {
		echo();
		cmd[idxCmd++]=uart_rx_buffer[0];
		if (uart_rx_buffer[0]=='\r') {
			cmd[idxCmd-1]='\0';
			if(CprCommande(cmd)){
				idxCmd=0;
			}
		}
		else if (uart_rx_buffer[0]=='\b') {
			if (idxCmd>0) {      //is there a char to delete?
				idxCmd=idxCmd-2;          //remove it in buffer
				Uartprint(backspace);
			}
		}
	}
}

int CprCommande(char * c){
	if(strcmp(c,"help")==0) {
		Uartprint((char *) help);
		return 1;
	}
	else if(strcmp(c,"on")==0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,0);
		for (int i=0;i<25;i++);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,1);
		Uartprint((char *) power_on);
		//Uartprint((char *) prompt);

		return 1;
	}
	else if(strcmp(c,"stop")==0) {
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,1);
		Uartprint((char *) power_off);
		return 1;
	}
	else if(strcmp(c,"pinout")==0) {
		Uartprint((char *) pinout);
		Uartprint((char *) prompt);
		return 1;
	}

	else if(strcmp(c,"start")==0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_Delay(1);

		Uartprint((char *) power_on);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		TIM1->CCR1 = val;
		TIM1->CCR2 = TIM1->ARR - val;
	}

	else{
		Uartprint((char *) not_found);
		Uartprint((char *) prompt);
		return 1;
	}

}
//fin code console

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  valeur_ancienne = TIM4->CNT;
  HAL_TIM_Base_Start_IT(&htim8);
  printf("%d\r\n", TIM4->CNT/4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Console();
	  /*if (1 == bouton) {
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		  TIM1->CCR1 = val;
		  TIM1->CCR2 = TIM1->ARR - val;

		  bouton=0;
	  }*/
	  valeur_nouvelle = TIM4->CNT;
	  //printf("%d\r\n", TIM4->CNT/4);
	  //TIM4->CNT = 0;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void TIM8_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_IRQn 0 */
	vitesse = (valeur_nouvelle-valeur_ancienne)/1024;
	valeur_ancienne = valeur_nouvelle;
	printf("Vitesse = %d tr/s\r\n", vitesse);

  /* USER CODE END TIM8_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);

  /* USER CODE BEGIN TIM8_UP_IRQn 1 */

  /* USER CODE END TIM8_UP_IRQn 1 */
}

/*void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	bouton=1;
	printf("entré dans callback");
}*/
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
