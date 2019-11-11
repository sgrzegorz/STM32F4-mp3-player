
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
#define BUTTON			(GPIOA->IDR & GPIO_Pin_0)


#define AUDIO_BUFFER_SIZE             4096
#define FILE_READ_BUFFER_SIZE			8192

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"

#define IS_GPIO_ALL_PERIPH(PERIPH) (((PERIPH) == GPIOA) || \
                                ((PERIPH) == GPIOB) || \
                                ((PERIPH) == GPIOC) || \
                                ((PERIPH) == GPIOD) || \
                                ((PERIPH) == GPIOE) || \
                                ((PERIPH) == GPIOF) || \
                                ((PERIPH) == GPIOG) || \
                                ((PERIPH) == GPIOH) || \
                                ((PERIPH) == GPIOI))
                                FIL file;

#include "usb_host.h"
#include "mp3dec.h"

/* USER CODE BEGIN Includes */
#include "stm32f4_discovery_audio.h"
#include "ansi.h"
#include "term_io.h"
#include "dbgu.h"

const char* FNAME = "haltmich.wav";
extern ApplicationTypeDef Appli_state;
extern USBH_HandleTypeDef hUsbHostHS;
enum
{
  BUFFER_OFFSET_NONE = 0,  
  BUFFER_OFFSET_HALF,  
  BUFFER_OFFSET_FULL,     
};
char					file_read_buffer[FILE_READ_BUFFER_SIZE];
MP3FrameInfo			mp3FrameInfo;
HMP3Decoder				hMP3Decoder;
uint8_t buff[AUDIO_BUFFER_SIZE];
static uint8_t player_state = 0;
static uint8_t buf_offs = BUFFER_OFFSET_NONE;
static uint32_t fpos = 0;


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
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
static void playMP3();

 
static void AudioCallback(void *context, int buffer) ;
void StartDefaultTask(void const * argument);


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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
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



static void f_disp_res(FRESULT r)
{
	switch(r)
	{
		case FR_OK: printf("FR_OK\n"); break;
		case FR_DISK_ERR: printf("FR_DISK_ERR\n"); break;
		case FR_INT_ERR: printf("FR_INT_ERR\n"); break;
		case FR_NOT_READY: printf("FR_NOT_READY\n"); break;
		case FR_NO_FILE: printf("FR_NO_FILE\n"); break;
		case FR_NO_PATH: printf("FR_NO_PATH\n"); break;
		case FR_INVALID_NAME: printf("FR_INVALID_NAME\n"); break;
		case FR_DENIED: printf("FR_DENIED\n"); break;
		case FR_EXIST: printf("FR_EXIST\n"); break;
		case FR_INVALID_OBJECT: printf("FR_INVALID_OBJECT\n"); break;
		case FR_WRITE_PROTECTED: printf("FR_WRITE_PROTECTED\n"); break;
		case FR_INVALID_DRIVE: printf("FR_INVALID_DRIVE\n"); break;
		case FR_NOT_ENABLED: printf("FR_NOT_ENABLED\n"); break;
		case FR_NO_FILESYSTEM: printf("FR_NO_FILESYSTEM\n"); break;
		case FR_MKFS_ABORTED: printf("FR_MKFS_ABORTED\n"); break;
		case FR_TIMEOUT: printf("FR_TIMEOUT\n"); break;
		case FR_LOCKED: printf("FR_LOCKED\n"); break;
		case FR_NOT_ENOUGH_CORE: printf("FR_NOT_ENOUGH_CORE\n"); break;
		case FR_TOO_MANY_OPEN_FILES: printf("FR_TOO_MANY_OPEN_FILES\n"); break;
		case FR_INVALID_PARAMETER: printf("FR_INVALID_PARAMETER\n"); break;
		default: printf("result code unknown (%d = 0x%X)\n",r,r);
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
}

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  buf_offs = BUFFER_OFFSET_FULL;
  BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&buff[0], AUDIO_BUFFER_SIZE / 2);
}

// //funkcja przeklejona z biblioteki
// void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
// {
//   /* Check the parameters */
//   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//   assert_param(IS_GPIO_PIN(GPIO_Pin));

//   GPIOx->BSRRL = GPIO_Pin;
// }

// void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
// {
//   /* Check the parameters */
//   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//   assert_param(IS_GPIO_PIN(GPIO_Pin));

//   GPIOx->BSRRH = GPIO_Pin;
// }

static void AudioCallback(void *context, int buffer) {
	static int16_t audio_buffer0[4096];
	static int16_t audio_buffer1[4096];

	int offset, err;
	int outOfData = 0;

	int16_t *samples;
	if (buffer) {
		samples = audio_buffer0;
	//	GPIO_SetBits(GPIOD, GPIO_Pin_13);
	//	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	} else {
		samples = audio_buffer1;
	//	GPIO_SetBits(GPIOD, GPIO_Pin_14);
	//	GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	}

	offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
	bytes_left -= offset;
	read_ptr += offset;

	err = MP3Decode(hMP3Decoder, (unsigned char**)&read_ptr, (int*)&bytes_left, samples, 0);

	if (err) {
		/* error occurred */
		switch (err) {
		case ERR_MP3_INDATA_UNDERFLOW:
			outOfData = 1;
			break;
		case ERR_MP3_MAINDATA_UNDERFLOW:
			/* do nothing - next call to decode will provide more mainData */
			break;
		case ERR_MP3_FREE_BITRATE_SYNC:
		default:
			outOfData = 1;
			break;
		}
	} else {
		// no error
		MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);

		// Duplicate data in case of mono to maintain playback speed
		if (mp3FrameInfo.nChans == 1) {
			for(int i = mp3FrameInfo.outputSamps;i >= 0;i--) 	{
				samples[2 * i]=samples[i];
				samples[2 * i + 1]=samples[i];
			}
			mp3FrameInfo.outputSamps *= 2;
		}
	}

	if (!outOfData) {
		ProvideAudioBuffer(samples, mp3FrameInfo.outputSamps);
	}
}





static void playMP3(){
  char* filename = "0:/sound.mp3";
  unsigned int br, btr;
	FRESULT res;

	bytes_left = FILE_READ_BUFFER_SIZE;
	read_ptr = file_read_buffer;

	if (FR_OK == f_open(&file, filename, FA_OPEN_EXISTING | FA_READ)) {

		//Read ID3v2 Tag
		/*
		char szArtist[120];
		char szTitle[120];
		Mp3ReadId3V2Tag(&file, szArtist, sizeof(szArtist), szTitle, sizeof(szTitle));
		*/

		Fill buffer
		f_read(&file, file_read_buffer, FILE_READ_BUFFER_SIZE, &br);

		// Play mp3
		hMP3Decoder = MP3InitDecoder();

     if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO,70,44100) == 0)
  {
	  xprintf("audio init OK\n");
  }


		//InitializeAudio(Audio44100HzSettings);
		//SetAudioVolume(0xAF);
		PlayAudioWithCallback(AudioCallback, 0);

		for(;;) {
			/*
			 * If past half of buffer, refill...
			 *
			 * When bytes_left changes, the audio callback has just been executed. This
			 * means that there should be enough time to copy the end of the buffer
			 * to the beginning and update the pointer before the next audio callback.
			 * Getting audio callbacks while the next part of the file is read from the
			 * file system should not cause problems.
			 */
			if (bytes_left < (FILE_READ_BUFFER_SIZE / 2)) {
				// Copy rest of data to beginning of read buffer
				memcpy(file_read_buffer, read_ptr, bytes_left);

				// Update read pointer for audio sampling
				read_ptr = file_read_buffer;

				// Read next part of file
				btr = FILE_READ_BUFFER_SIZE - bytes_left;
				res = f_read(&file, file_read_buffer + bytes_left, btr, &br);

				// Update the bytes left variable
				bytes_left = FILE_READ_BUFFER_SIZE;

				// Out of data or error or user button... Stop playback!
				if (br < btr || res != FR_OK || BUTTON) {
					StopAudio();

					// Re-initialize and set volume to avoid noise
					InitializeAudio(Audio44100HzSettings);
					SetAudioVolume(0);

					// Close currently open file
					f_close(&file);

					// Wait for user button release
					while(BUTTON){};

					// Return to previous function
					return;
				}
			}
		}
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
void StartDefaultTask(void const * argument)
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
  }while(Appli_state != APPLICATION_READY);
  
  playMP3();
  
  /*
  FRESULT res;
  
  res = f_open(&file,"0:/test_1k.wav",FA_READ);
  
  if(res==FR_OK)
  {
	  xprintf("wave file open OK\n");
  }
  else
  {
	  xprintf("wave file open ERROR, res = %d\n",res);
	  f_disp_res(res);
	  while(1);
  }
  
  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO,70,44100) == 0)
  {
	  xprintf("audio init OK\n");
  }
  else
  {
	  xprintf("audio init ERROR\n");
  }
  /*
  
  /* Infinite loop */
  /*
  for(;;)
  {
	  
	char key = debug_inkey();
	
	switch(key)
	{
		case 'p':
		{
			xprintf("play command...\n");
			if(player_state) {xprintf("already playing\n"); break;}
			player_state = 1;
			playMP3();
			BSP_AUDIO_OUT_Play((uint16_t*)&buff[0],AUDIO_BUFFER_SIZE);
			fpos = 0;
			buf_offs = BUFFER_OFFSET_NONE;
			break;
		}
	}
	
	if(player_state)
	{
		uint32_t br;
		
		if(buf_offs == BUFFER_OFFSET_HALF)
		{
		  if(f_read(&file, 
					&buff[0], 
					AUDIO_BUFFER_SIZE/2,
					(void *)&br) != FR_OK)
		  { 
			BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW); 
			xprintf("f_read error on half\n");
		  }
		  buf_offs = BUFFER_OFFSET_NONE;
		  fpos += br;
		  
		}
		
		if(buf_offs == BUFFER_OFFSET_FULL)
		{
			if(f_read(&file, 
					&buff[AUDIO_BUFFER_SIZE /2], 
					AUDIO_BUFFER_SIZE/2, 
					(void *)&br) != FR_OK)
			{ 
				BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW); 
				xprintf("f_read error on full\n");
			} 

			buf_offs = BUFFER_OFFSET_NONE;
			fpos += br; 
		}

		if( br < AUDIO_BUFFER_SIZE/2 )
		{
			xprintf("stop at eof\n");
			BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW); 
			player_state = 0;
		}
	}  //if(player_state)
	  
	vTaskDelay(2);
	  
  }
  */
  /* USER CODE END 5 */ 
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
  if (htim->Instance == TIM7) {
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
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

//OUR FUNCTIONS













#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
