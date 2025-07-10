/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "u8g2.h"

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ----------------- SH1106 Functions ------------------- */
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	  switch(msg)
	  {
	  case U8X8_MSG_DELAY_MILLI:
		  HAL_Delay(arg_int);
		  break;
	  case U8X8_MSG_GPIO_CS:
		  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, arg_int);
		  break;
	  case U8X8_MSG_GPIO_DC:
		  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, arg_int);
		  break;
	  case U8X8_MSG_GPIO_RESET:
		  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, arg_int);
		  break;
	  }
	  return 1;
}

// Normal SPI
//uint8_t u8x8_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
//{
//	  switch(msg)
//	  {
//	  case U8X8_MSG_BYTE_SET_DC:
//		  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, arg_int);
//		  break;
//	  case U8X8_MSG_BYTE_SEND:
//		  HAL_SPI_Transmit(&hspi1, (uint8_t *)arg_ptr, arg_int, 1000);
//		  break;
//	  case U8X8_MSG_BYTE_START_TRANSFER:
//		  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
//		  break;
//	  case U8X8_MSG_BYTE_END_TRANSFER:
//		  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
//		  break;
//	  }
//	  return 1;
//}

//DMA SPI
int txDone = 1;
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	txDone = 1;
}

uint8_t dma_buffer[256]; /* required for DMA transfer */

uint8_t u8x8_byte_dma_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
  uint16_t i;
  switch(msg) {
    case U8X8_MSG_BYTE_SEND:
      /* wait for DMA completion */
      while (txDone == 0);
      txDone = 0;
      /* create a copy of the input data */
      for( i = 0; i < arg_int; i++ )
        dma_buffer[i] = ((uint8_t *)arg_ptr)[i];
      /* create DMA SPI Transfer for arg_int bytes, located at dma_buffer */
      HAL_SPI_Transmit_DMA(&hspi1, dma_buffer, arg_int);
      break;
    case U8X8_MSG_BYTE_INIT:
      /* setup SPI & DMA */
      /* ... */
      HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
      break;
    case U8X8_MSG_BYTE_SET_DC:
      /* Wait for DMA SPI transfer completion */
      HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, arg_int);
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
    	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
      /* Wait for DMA SPI transfer completion */
    	while (txDone == 0);
    	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
      break;
    default:
      return 0;
  }
  return 1;
}

/* ----------------- ST7920 Functions ------------------- */
//uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
//{
//	  switch(msg)
//	  {
//	  case U8X8_MSG_DELAY_MILLI:
//		  HAL_Delay(arg_int);
//		  break;
//	  case U8X8_MSG_GPIO_CS:
//		  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, arg_int);
//		  break;
//	  case U8X8_MSG_GPIO_DC:
////		  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, arg_int);
//		  break;
//	  case U8X8_MSG_GPIO_RESET:
//		  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, arg_int);
//		  break;
//	  }
//	  return 1;
//}
//
//
//uint8_t u8x8_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
//{
//	  switch(msg)
//	  {
//	  case U8X8_MSG_BYTE_SET_DC:
////		  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, arg_int);
//		  break;
//	  case U8X8_MSG_BYTE_SEND:
//		  HAL_SPI_Transmit(&hspi1, (uint8_t *)arg_ptr, arg_int, 1000);
//		  break;
//	  case U8X8_MSG_BYTE_START_TRANSFER:
//		  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);  // The CS pin is inverted
//		  break;
//	  case U8X8_MSG_BYTE_END_TRANSFER:
//		  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
//		  break;
//	  }
//	  return 1;
//}

/* ----------------- SSD1306 Functions I2C ------------------- */
//uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
//{
//	  switch(msg)
//	  {
//	  case U8X8_MSG_DELAY_MILLI:
//		  HAL_Delay(arg_int);
//		  break;
//	  }
//	  return 1;
//}
//
//
//uint8_t u8x8_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
//{
//  static uint8_t buffer[32];		/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
//  static uint8_t buf_idx;
//  uint8_t *data;
//
//  switch(msg)
//  {
//    case U8X8_MSG_BYTE_SEND:
//      data = (uint8_t *)arg_ptr;
//      while( arg_int > 0 )
//      {
//	buffer[buf_idx++] = *data;
//	data++;
//	arg_int--;
//      }
//      break;
//    case U8X8_MSG_BYTE_INIT:
//      /* add your custom code to init i2c subsystem */
//      break;
//    case U8X8_MSG_BYTE_SET_DC:
//      /* ignored for i2c */
//      break;
//    case U8X8_MSG_BYTE_START_TRANSFER:
//      buf_idx = 0;
//      break;
//    case U8X8_MSG_BYTE_END_TRANSFER:
//    	HAL_I2C_Master_Transmit(&hi2c1, 0x78, buffer, buf_idx, 1000);
//      break;
//    default:
//      return 0;
//  }
//  return 1;
//}

/* ----------------- HX1230 Functions SW_SPI ------------------- */
//uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
//{
//	  switch(msg)
//	  {
//	    case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
//	    	asm("NOP");
//	      break;
//	    case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
//	    	for (int i=0; i<30; i++)asm("NOP");
//	      break;
//	  case U8X8_MSG_DELAY_MILLI:
//		  HAL_Delay(arg_int);
//		  break;
//	  case U8X8_MSG_GPIO_SPI_DATA:
//		  HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, arg_int);
//	        break;
//	  case U8X8_MSG_GPIO_SPI_CLOCK:
//		  HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, arg_int);
//	        break;
//	  case U8X8_MSG_GPIO_CS:
//		  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, arg_int);
//		  break;
//	  case U8X8_MSG_GPIO_RESET:
//		  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, arg_int);
//		  break;
//	  }
//	  return 1;
//}

u8g2_t myDisplay;

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // SH1106 Function
  u8g2_Setup_sh1106_128x64_noname_f(&myDisplay, U8G2_R0, u8x8_byte_dma_spi, u8x8_gpio_and_delay);  // init u8g2 structure

  // ST7920 Function
//  u8g2_Setup_st7920_s_128x64_f(&myDisplay, U8G2_R2, u8x8_spi, u8x8_gpio_and_delay);  // init u8g2 structure

  // SSD1306 Function
//  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&myDisplay, U8G2_R0, u8x8_i2c, u8x8_gpio_and_delay);  // init u8g2 structure

  //HX1230 Function
//  u8g2_Setup_hx1230_96x68_f(&myDisplay, U8G2_R2, u8x8_byte_3wire_sw_spi, u8x8_gpio_and_delay);  // init u8g2 structure

  u8g2_InitDisplay(&myDisplay); // send init sequence to the display, display is in sleep mode after this,
  u8g2_SetPowerSave(&myDisplay, 0); // wake up display
  u8g2_ClearDisplay(&myDisplay);

  // Full Display (**_f variant)
  u8g2_SetFont(&myDisplay, u8g2_font_ncenB14_tr);
  u8g2_DrawStr(&myDisplay, 0,15,"Hello 123");
  u8g2_DrawCircle(&myDisplay, 60, 30, 10, U8G2_DRAW_ALL);
  u8g2_SendBuffer(&myDisplay);

  // Partial Page (**_1/2 variant)
//  u8g2_FirstPage(&myDisplay);
//  do
//  {
//    u8g2_SetFont(&myDisplay, u8g2_font_ncenB14_tr);
//    u8g2_DrawStr(&myDisplay, 0, 15, "hello world");
//   u8g2_DrawCircle(&myDisplay, 64, 40, 10, U8G2_DRAW_ALL);
//  } while (u8g2_NextPage(&myDisplay));

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DC_Pin */
  GPIO_InitStruct.Pin = DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_Pin */
  GPIO_InitStruct.Pin = RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
