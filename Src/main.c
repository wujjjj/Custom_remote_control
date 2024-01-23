/* USER CODE BEGIN Header */
/**ICBK-wj
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "struct_typedef.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
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

//�м����
float ADC_value[4];//ADCת��ֵ����
bool_t key[4];//io�ڵ�ƽ��ȡֵ����
//�������ݴ���
uint16_t send_value[5];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//����������,��4��bool����ֵ���Ϊuint16_t���͵�ֵ
uint16_t PackBoolData(bool_t data1, bool_t data2, bool_t data3, bool_t data4)
{
    uint16_t packedData = 0;
    
    packedData |= (data1 ? 1 : 0) << 0;
    packedData |= (data2 ? 1 : 0) << 1;
    packedData |= (data3 ? 1 : 0) << 2;
    packedData |= (data4 ? 1 : 0) << 3;
    
    return packedData;
}

//�������ݺ���
uint8_t txBuffer[3];
void send_data(uint8_t id,uint16_t sdata){
    txBuffer[0] = id;//�����һλ��������Ϊ��ʶ��
	txBuffer[1] = (sdata >> 8) & 0xFF;//��8λ���ݴ��浽�ڶ�λ����������
	txBuffer[2] = sdata & 0xFF;//��8λ���ݴ��浽����λ����������

    HAL_UART_Transmit(&huart6, txBuffer, 3, HAL_MAX_DELAY);
}

//��ȡADCת��ֵ����
static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(ADCx);

    HAL_ADC_PollForConversion(ADCx, 10);
    return (uint16_t)HAL_ADC_GetValue(ADCx);

}
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
	uint8_t i;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	key[0] = !HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12);//����״̬Ϊ1,��������Ϊ1,
	key[1] = !HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13);//��ô����ʱ���͵�1ͬΪ����״̬,�ᵼ�»���ʧ��,
	key[2] = !HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14);//���ȡ��,����״̬Ϊ0,����״̬ҲΪ0
	key[3] = !HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15);
	
	ADC_value[0] = adcx_get_chx_value(&hadc1,ADC_CHANNEL_10);
	ADC_value[1] = adcx_get_chx_value(&hadc1,ADC_CHANNEL_11);
	ADC_value[2] = adcx_get_chx_value(&hadc1,ADC_CHANNEL_12);
	ADC_value[3] = adcx_get_chx_value(&hadc1,ADC_CHANNEL_13);
	
	send_value[0] = (uint16_t)(((float)(ADC_value[0] * 1320)) / 4095);//����ȡ��ADCֵ��4095��дΪ0~1����ı���
	send_value[1] = (uint16_t)(((float)(ADC_value[1] * 1320)) / 4095);//Ȼ���2*660=1320���0~1320����ı���
	send_value[2] = (uint16_t)(((float)(ADC_value[2] * 1320)) / 4095);//���ն˼�660������ң�������͵�����-660~660���Ӧ
	send_value[3] = (uint16_t)(((float)(ADC_value[3] * 1320)) / 4095);//�������ն˲����޸�̫��,�ȳ˺����С���
	
	//��4����ť����ֵ�����1��uint16_t����ֵ��,ͳһ���͸�ʽ,���ӷ������ݵ�������
	send_value[4] = PackBoolData(key[0],key[1],key[2],key[3]);
	  
	for(i=0;i<5;i++){
		send_data(i,send_value[i]);//����5��ң��������
	}
	
	HAL_Delay(50);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics ICBK-wj *****END OF FILE****/
