/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "can.h"
#include "dac.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
FMC_SDRAM_CommandTypeDef SDRAM_Command;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	CAN_TxHeaderTypeDef   TxMessage;
	CAN_RxHeaderTypeDef   RxMessage;
	CAN_FilterTypeDef     sFilterConfig;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Can_Filter(CAN_HandleTypeDef *hcan);
void Can_Set_ID(CAN_HandleTypeDef *hcan,uint16_t ID);
void Can_Send_Msg(CAN_HandleTypeDef *hcan);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  //I2C
uint8_t TxI2C1[29]= "\r\n**SEND I2C_2 CUBESAT2020**";
uint8_t RxI2C1[30];

uint8_t count;
	//sdram
	uint8_t   sdram_buffer_8bit[11];
	//can
	uint8_t   TxData[8]={'C','U','B','E',' ','S','A','T'};
	uint8_t		RxData[8];
	uint32_t  TxMailbox;		
  //RTC
  RTC_TimeTypeDef  sTime  = {0};
  RTC_DateTypeDef  sDate  = {0};
  RTC_AlarmTypeDef sAlarm = {0};

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



	//SPI3 Buffer TX
	uint8_t 	aTXBuffer[7]="CUBESAT";
	
	//uSD CARD Private variables
	UINT 			TestByte;
	char 			uSD_Data[]="Write Text To uSD CARD Board salam";
	char 			uSD_Path[]="/CUBE/RcvData.TXT\0";
	//RTC
	char 			RTC_Buffer[60];
	
	//ADC Private variables
	char 			ADC_RANK1[30],ADC_RANK2[30],ADC_RANK3[30],ADC_RANK4[30];
	volatile 	uint16_t ADC_DATA[4];
	int  			ADC_READ;
	//DAC
		int  Value=0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//	HAL_Delay(5000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_FMC_Init();
  MX_USART1_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_SPI6_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	/****************************************************************
	*                           SDRAM			                          *
	*****************************************************************/
	SDRAM_Initialization_Sequence(&hsdram1, &SDRAM_Command);
	#if SDRAM_WRITE
	HAL_SDRAM_Write_8b (&hsdram1,( uint32_t *)0xD0000000,( uint8_t  *)"test 8 bit",10);
	HAL_SDRAM_Write_16b(&hsdram1,( uint32_t *)0xD0000010,( uint16_t *)"test 16 bit",11);	
	HAL_SDRAM_Write_32b(&hsdram1,( uint32_t *)0xD0000020,( uint32_t *)"test 32 bit",11);
	
	HAL_SDRAM_Read_8b  (&hsdram1,( uint32_t *)0xD0000000,( uint8_t  *)sdram_buffer_8bit,10);
	HAL_UART_Transmit(&huart6,( uint8_t *)sdram_buffer_8bit,10,10);
	#endif
	/****************************************************************
	*                         NAND FLASH                            *
	*****************************************************************/
	#if		ENABLE_NAND_FLASH
	NAND_RESET();
	NAND_ReadID();
	Erase_First_Block();
	Write_Nand_Page();
	Read_Nand_Page();
	
	#endif
	/****************************************************************
	*                         uSD CARD SDIO		   	                  *
	*****************************************************************/
	#if  ENABLE_uSD_CARD
	if (f_mount(&SDFatFS,SDPath,1)==FR_OK){
	
		f_mkdir("/CUBE\0"); 																//Creat Folder to uSd Card
		f_open(&SDFile,uSD_Path,FA_WRITE|FA_CREATE_ALWAYS); 	//Open  Folder & Create File Text To uSD Card
		f_write(&SDFile,uSD_Data,sizeof(uSD_Data),&TestByte); //Write Data To File uSD Card
		f_close(&SDFile);																			//Close uSD Card
	
	}
	#endif
	/****************************************************************
	*                        USART[6] TO USB			                  *
	*****************************************************************/
	#if ENABLE_USART6
	HAL_UART_Transmit(&huart6,( uint8_t *)"\n\rCUBESAT2020 [OBC]\n\r",21,10);
	#endif
	/****************************************************************
	*                        USART[6] TO USB			                  *
	*****************************************************************/
	#if ENABLE_USART1
	HAL_UART_Transmit(&huart1,( uint8_t *)"\n\rCUBESAT [OBC]",15,10);
	#endif
	/****************************************************************
	*                        SPI3 SEND & RECIVE DATA                *
	*****************************************************************/	
	#if SPI3_SEND_DATA
		SPI3_CS_SET();
	#endif
	/****************************************************************
	*                        SPI6 SEND & RECIVE DATA                *
	*****************************************************************/	
	#if SPI6_SEND_DATA
	 SPI6_CS_SET();
  #endif	
	/****************************************************************
	*                        RTC DATE & TIME                        *
	*****************************************************************/	
	#if RTC_ENABLE
	sTime.Hours = 15;
	sTime.Minutes = 35;
	sTime.Seconds = 0;
	HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
	
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 30;
  sDate.Year = 20;
	HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
	#endif
	/****************************************************************
	*                        CAN2 Send Data to CAN1                 *
	*****************************************************************/		
  #if CAN2_MASTER_CAN1_SLAVE
	Can_Filter(&hcan1);
	/*This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */
	Can_Set_ID(&hcan2,0x119); 
	#endif	
	/****************************************************************
	*                        CAN1 Send Data to CAN2                 *
	*****************************************************************/	
  #if CAN1_MASTER_CAN2_SLAVE
	Can_Filter(&hcan2);
	/*This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */
	Can_Set_ID(&hcan1,0x220);
	#endif
  #if DAC1_init	
	HAL_TIM_Base_Start(&htim2);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	#endif
  #if DAC2_init	
	HAL_TIM_Base_Start(&htim2);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	#endif	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	#if DAC1_init	
		for(char cnt=0;cnt<=20;cnt++){
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Value);
			Value=Value+100;	
			HAL_Delay(500);
		}
	#endif
	#if DAC2_init	
		for(char cnt=0;cnt<=20;cnt++){
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Value);
			Value=Value+100;	
			HAL_Delay(500);
		}
	#endif		
	#if I2C1_SEND_DATA	
		do
			{
				if(HAL_I2C_Master_Transmit_IT(&hi2c1, (uint16_t)0x28F, (uint8_t*)TxI2C1, sizeof(TxI2C1))!= HAL_OK)
				{
					Error_Handler();
				}
				while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
				{
				}
			}
			while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF); 
		do
			{
				if(HAL_I2C_Master_Receive_IT(&hi2c1, (uint16_t)0x28F, (uint8_t *)RxI2C1, sizeof(RxI2C1)) != HAL_OK)
				{
					Error_Handler();
				}
			}
			while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);
			HAL_UART_Transmit(&huart6,RxI2C1,sizeof(RxI2C1),HAL_MAX_DELAY);
      HAL_Delay(100);		
   #endif
		
		
	/****************************************************************
	*                        CAN1 AND CAN2            		          *
	*****************************************************************/		
  #if CAN1_MASTER_CAN2_SLAVE
    Can_Send_Msg(&hcan1);
		HAL_Delay(500);
	#endif
	
	#if CAN2_MASTER_CAN1_SLAVE		
	  Can_Send_Msg(&hcan2);
		HAL_Delay(500);
	#endif
	/****************************************************************
	*                              RTC		                          *
	*****************************************************************/		
	#if RTC_ENABLE	
		HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
		sprintf(RTC_Buffer,"\n\r%i : %i : %i",sTime.Hours,sTime.Minutes,sTime.Seconds);
		HAL_UART_Transmit(&huart6,(uint8_t*)RTC_Buffer,14,15);	
		HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
		sprintf(RTC_Buffer,"\n\r%i / %i / %i -%i",sDate.Year,sDate.Month,sDate.WeekDay,sDate.Date);
		HAL_UART_Transmit(&huart6,(uint8_t*)RTC_Buffer,17,15);		
		HAL_Delay(1000);	
	#endif	
	/****************************************************************
	*                        SPI3 & SPI6 READ  Channel		          *
	*****************************************************************/		
	#if	SPI3_SEND_DATA
		HAL_SPI_Transmit(&hspi3,aTXBuffer,sizeof(aTXBuffer),1000);
		SPI3_STROB();
		HAL_Delay(1000);
	#endif
	
	#if	SPI6_SEND_DATA	
		HAL_SPI_Transmit(&hspi6,aTXBuffer,sizeof(aTXBuffer),1000);
		SPI6_STROB();
		HAL_Delay(1000);
	#endif		
	/****************************************************************
	*                        ADC READ 1 Channel			                *
	*****************************************************************/			
	#if	ADC_READ_None
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		ADC_READ=HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		sprintf(ADC_RANK1,"\r\nVoltage:%dmV",(ADC_READ*3300)/4095);
		HAL_UART_Transmit(&huart6,(uint8_t *)ADC_RANK1,16,10);
		HAL_Delay(500);
	#endif
		
	#if  ADC_READ_MULTI
		HAL_ADCEx_InjectedStart(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		ADC_DATA[0]=(HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1)*3300/4095);
		ADC_DATA[1]=(HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2)*3300/4095);
		ADC_DATA[2]=(HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3)*3300/4095);
		ADC_DATA[3]=(HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_4)*3300/4095);	
		HAL_ADCEx_InjectedStop(&hadc1);
		sprintf(ADC_RANK1,"\r\nVoltage ADC1_IN6:%dmV" ,ADC_DATA[0]);
		HAL_UART_Transmit(&huart6,(uint8_t *)ADC_RANK1,25,10);

		sprintf(ADC_RANK2,"\r\nVoltage ADC1_IN7:%dmV" ,ADC_DATA[1]);
		HAL_UART_Transmit(&huart6,(uint8_t *)ADC_RANK2,25,10);

		sprintf(ADC_RANK3,"\r\nVoltage ADC1_IN14:%dmV",ADC_DATA[2]);
		HAL_UART_Transmit(&huart6,(uint8_t *)ADC_RANK3,26,10);

		sprintf(ADC_RANK4,"\r\nVoltage ADC1_IN15:%dmV",ADC_DATA[3]);
		HAL_UART_Transmit(&huart6,(uint8_t *)ADC_RANK4,26,10);	

    HAL_Delay(500);		
   #endif
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//CAN
void Can_Filter(CAN_HandleTypeDef *hcan){

	sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 0;	

	if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(hcan) != HAL_OK){
    /* Start Error */
    Error_Handler();
  }

  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
    /* Notification Error */
    Error_Handler();
  }
}
void Can_Set_ID(CAN_HandleTypeDef *hcan,uint16_t ID){

	TxMessage.StdId							  = ID;
	TxMessage.RTR       					  = CAN_RTR_DATA;
	TxMessage.IDE       					  = CAN_ID_STD;
	TxMessage.ExtId     					  = 0;
	TxMessage.DLC       					  = 8;
	TxMessage.TransmitGlobalTime 	=	DISABLE;	
  /*##-2- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(hcan) != HAL_OK){
    /* Start Error */
    Error_Handler();
  }
}
void Can_Send_Msg(CAN_HandleTypeDef *hcan){
	
			HAL_CAN_AddTxMessage(hcan,&TxMessage,TxData,&TxMailbox);
}
//I2C
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Toggle LED3: Transfer in transmission process is correct */
  count++;
	if(count==20)
		count=0;
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Toggle LED3: Transfer in reception process is correct */
  count++;
	if(count==20)
		count=0;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
