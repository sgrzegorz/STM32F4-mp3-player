
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#define READ_BUFFER_SIZE	2 * MAINBUF_SIZE  //4096
#define DECODED_MP3_FRAME_SIZE	MAX_NGRAN * MAX_NCHAN * MAX_NSAMP
//#define OUT_BUFFER_SIZE			2 * DECODED_MP3_FRAME_SIZE//
#define OUT_BUFFER_SIZE			2 * DECODED_MP3_FRAME_SIZE

#define END_OF_FILE	-1
#define READ_ERROR	-2
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_host.h"
#include "mp3dec.h"

/* USER CODE BEGIN Includes */

#include "stm32f4_discovery_audio.h"
#include "ansi.h"
#include "term_io.h"
#include "dbgu.h"

HMP3Decoder hMP3Decoder;
static uint8_t *read_pointer;
static uint8_t read_buffer[READ_BUFFER_SIZE];
static int offset;
static int result;
short out_buffer[OUT_BUFFER_SIZE];
static int underflows = 0;
static int bytes_left = 0;
static int bytes_left_before_decoding = 0;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
int refill_inbuffer(FIL *in_file);
int mp3_proccess(FIL *mp3_file);
void StartDefaultTask(void const *argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_RNG_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  debug_init(&huart2);
  xprintf("F4 player test!\n");
  xprintf("printf test!\n");
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  // 
hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* RNG init function */
static void MX_RNG_Init(void)
{

  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PB10   ------> I2S2_CK
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
FIL file;

const char *FNAME = "haltmich.wav";
extern ApplicationTypeDef Appli_state;
extern USBH_HandleTypeDef hUsbHostHS;
enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
};
#define AUDIO_BUFFER_SIZE 4096
uint8_t buff[AUDIO_BUFFER_SIZE];
static uint8_t player_state = 0;
static uint8_t buf_offs = BUFFER_OFFSET_NONE;
static uint32_t fpos = 0;

static void f_disp_res(FRESULT r)
{
  switch (r)
  {
  case FR_OK:
    printf("FR_OK\n");
    break;
  case FR_DISK_ERR:
    printf("FR_DISK_ERR\n");
    break;
  case FR_INT_ERR:
    printf("FR_INT_ERR\n");
    break;
  case FR_NOT_READY:
    printf("FR_NOT_READY\n");
    break;
  case FR_NO_FILE:
    printf("FR_NO_FILE\n");
    break;
  case FR_NO_PATH:
    printf("FR_NO_PATH\n");
    break;
  case FR_INVALID_NAME:
    printf("FR_INVALID_NAME\n");
    break;
  case FR_DENIED:
    printf("FR_DENIED\n");
    break;
  case FR_EXIST:
    printf("FR_EXIST\n");
    break;
  case FR_INVALID_OBJECT:
    printf("FR_INVALID_OBJECT\n");
    break;
  case FR_WRITE_PROTECTED:
    printf("FR_WRITE_PROTECTED\n");
    break;
  case FR_INVALID_DRIVE:
    printf("FR_INVALID_DRIVE\n");
    break;
  case FR_NOT_ENABLED:
    printf("FR_NOT_ENABLED\n");
    break;
  case FR_NO_FILESYSTEM:
    printf("FR_NO_FILESYSTEM\n");
    break;
  case FR_MKFS_ABORTED:
    printf("FR_MKFS_ABORTED\n");
    break;
  case FR_TIMEOUT:
    printf("FR_TIMEOUT\n");
    break;
  case FR_LOCKED:
    printf("FR_LOCKED\n");
    break;
  case FR_NOT_ENOUGH_CORE:
    printf("FR_NOT_ENOUGH_CORE\n");
    break;
  case FR_TOO_MANY_OPEN_FILES:
    printf("FR_TOO_MANY_OPEN_FILES\n");
    break;
  case FR_INVALID_PARAMETER:
    printf("FR_INVALID_PARAMETER\n");
    break;
  default:
    printf("result code unknown (%d = 0x%X)\n", r, r);
  }
}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
  buf_offs = BUFFER_OFFSET_HALF;
   if(mp3_proccess(&file)!=0){
      BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);

   }

}

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  buf_offs = BUFFER_OFFSET_FULL;
  // BSP_AUDIO_OUT_ChangeBuffer((uint16_t *)&out_buffer[0], OUT_BUFFER_SIZE / 2);
     if(mp3_proccess(&file)!=0){
      BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);

   }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();

  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN 5 */

  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  vTaskDelay(1000);

  xprintf("waiting for USB mass storage\n");

  do
  {
    xprintf(".");
    vTaskDelay(250);
  } while (Appli_state != APPLICATION_READY);

  FRESULT res;

  res = f_open(&file, "0:/sound.mp3", FA_READ);

  if (res == FR_OK)
  {
    xprintf("wave file open OK\n");
  }
  else
  {
    xprintf("wave file open ERROR, res = %d\n", res);
    f_disp_res(res);
    while (1)
      ;
  }

  // if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 70, 44100) == 0)
  // {
  //   xprintf("audio init OK\n");
  // }
  // else
  // {
  //   xprintf("audio init ERROR\n");
  // }

  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_BOTH, 100, AUDIO_FREQUENCY_44K) == 0)
	{
	  // BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
		xprintf("Audio Init Ok\n");
	}
	else
	{
		xprintf("Audio Init Error\n");
	}

  hMP3Decoder = MP3InitDecoder();
  read_pointer = NULL;

  if(mp3_proccess(&file)){
    BSP_AUDIO_OUT_Play((uint16_t*)&out_buffer[0],OUT_BUFFER_SIZE*2);
    while(1){

    }
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
		buf_offs = BUFFER_OFFSET_NONE;


  }


  /* Infinite loop */

  // for (;;)
  // {
	  
  //   char key = debug_inkey();
    
  //   switch(key)
  //   {
  //     case 'p':
  //     {
  //       xprintf("play command...\n");
  //       if(player_state) {xprintf("already playing\n"); break;}
  //       player_state = 1;
  //       BSP_AUDIO_OUT_Play((uint16_t*)&out_buffer[0],OUT_BUFFER_SIZE*2);
  //       read_pointer = read_buffer;
  //       buf_offs = BUFFER_OFFSET_NONE;

  //       break;
  //     }
  //   }
    
  //   if(player_state)
  //   {
  //     uint32_t br;
  //     mp3_proccess(&file);
  //     vTaskDelay(2);  
  //   }
  
  // /* USER CODE END 5 */ 
  // }
  
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

int mp3_proccess(FIL *mp3_file){
  	MP3FrameInfo mp3FrameInfo;

	if (read_pointer == NULL) {
		if(refill_inbuffer(mp3_file) != 0){
			return END_OF_FILE;
		}
	}

	offset = MP3FindSyncWord(read_pointer, bytes_left);
	while(offset < 0) {
         xprintf("!");

		if(refill_inbuffer(mp3_file) != 0)
			return END_OF_FILE;
		if(bytes_left > 0){
			bytes_left -= 1;
			read_pointer += 1;
		}
		offset = MP3FindSyncWord(read_pointer, bytes_left);
	}
	read_pointer += offset;
	bytes_left -= offset;

	if (MP3GetNextFrameInfo(hMP3Decoder, &mp3FrameInfo, read_pointer) == 0 &&
			mp3FrameInfo.nChans == 2 &&
			mp3FrameInfo.version == 0) {
		//debug_printf("Found a frame at offset %x\n", offset + read_ptr - mp3buf + mp3file->fptr);
	} else {
    //nigdy tu nie wpada
		// advance data pointer
		// TODO: handle bytes_left == 0
		if(bytes_left > 0){
			bytes_left -= 1;
			read_pointer += 1;
      
		}
		return 0;
	}

	if (bytes_left < MAINBUF_SIZE) {

		if(refill_inbuffer(mp3_file) != 0)
			return END_OF_FILE;
	}


	if(buf_offs == (BUFFER_OFFSET_HALF)){
    xprintf("%d---\n",bytes_left);
		result = MP3Decode(hMP3Decoder, &read_pointer, &bytes_left, out_buffer, 0);
		buf_offs = BUFFER_OFFSET_NONE;
	}
	if(buf_offs == (BUFFER_OFFSET_FULL)){
        xprintf("%d+++\n",bytes_left);

		result = MP3Decode(hMP3Decoder, &read_pointer, &bytes_left, &out_buffer[DECODED_MP3_FRAME_SIZE], 0);
		buf_offs = BUFFER_OFFSET_NONE;
	}

	//DAC_DMA_enable();
	if(result != ERR_MP3_NONE){
		switch(result){
		case ERR_MP3_INDATA_UNDERFLOW:
			bytes_left = 0;
			if(refill_inbuffer(mp3_file) != 0)
				return END_OF_FILE;
			break;
		case ERR_MP3_MAINDATA_UNDERFLOW:
			//do nothing, next call to MP3Decode will provide more data
			break;
		default:
			return 0; //skip this frame if error
			//return END_OF_FILE; //skip this file if error
		}
	}
	return 0;

}


int refill_inbuffer(FIL *in_file)
{
	unsigned int bytes_read;
	unsigned int bytes_to_read;
	FRESULT result;

	if (bytes_left > 0) {

		//copy remaining data to beginning of buffer
    //    copy to  / copy from   /count
    //xprintf("%d,",bytes_left);
		memcpy(read_buffer, read_pointer, bytes_left);
	}

	bytes_to_read = READ_BUFFER_SIZE - bytes_left;
     // xprintf("%d,",bytes_to_read );

// * [IN] File object */
//   void* buff,  /* [OUT] Buffer to store read data */
//   UINT btr,    /* [IN] Number of bytes to read */
//   UINT* br     /* [OUT] Number of bytes read */
	result = f_read(in_file, (BYTE *)read_buffer + bytes_left, bytes_to_read, &bytes_read);
       //xprintf(":%d,\n",bytes_read );

	if(result != FR_OK)
		return READ_ERROR;

	if (bytes_read == bytes_to_read){
		read_pointer = read_buffer;
		//offset = 0;
		bytes_left = READ_BUFFER_SIZE;
		return 0;
	}
	else{
		return END_OF_FILE;
	}

	return 0;  //should never reach this point
}

#ifdef USE_FULL_ASSERT
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

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
