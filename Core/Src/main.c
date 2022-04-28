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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AESK_CAN_Library.h"
#include "AESK_Electronic_Differential.h"
#include "AESK_Data_Pack_lib.h"
#include "Can_Lyra_Header.h"
#include "AESK_VCU_General.h"
#include "AESK_GL.h"
#include "AESK_Ring_Buffer.h"
#include "AESK_UART_STM32.h"
#include "AESK_comm_pro.h"
#include "AESK_VCU_Nextion.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART6_UART_Init(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  AESK_Electronic_Dif_Init(&electronic_dif_param, SP_KP, SP_KI, SP_KD, REGEN_RAMP_COEF);
  Nextion_Init();

  //CAN1STDBY Set
  HAL_GPIO_WritePin(CAN1_STDBY_GPIO_Port, CAN1_STDBY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAN2_STDBY_GPIO_Port, CAN2_STDBY_Pin, GPIO_PIN_SET);

  //Compro Init
#if COM_MODE == COMPRO || COM_MODE == MCU_OLD
  AESK_Compro_Init(&compro_set_can_mcu_rx, Electromobile, VCU, &Solver);
  AESK_Compro_Init(&compro_set_can_bms_rx, Electromobile, VCU, &Solver);
  AESK_Compro_Init(&compro_set_uart_rx, Electromobile, VCU, &Solver);
  AESK_Compro_Init(&compro_set_can_telemetry_rx, Electromobile, VCU, &Solver);
  AESK_GL_Init(&aesk_gl);
  aesk_gl.aesk_can_1.hcan = &hcan1;
  AESK_CAN_ExtIDMaskFilterConfiguration(&aesk_gl.aesk_can_1, 0, 0, CAN_FILTER_FIFO0, 0);



  Uart_DMA_Receive_Start(&RS485_Serial, &uart_buf);
#elif COM_MODE == OLD_COM
  AESK_GL_Init(&aesk_gl);
  aesk_gl.aesk_can_1.hcan = &hcan1;

  AESK_CAN_ExtIDMaskFilterConfiguration(&aesk_gl.aesk_can_1, 0, 0, CAN_FILTER_FIFO0, 0);
#endif

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&sensor_data.adc_val_u16, 1);
  HAL_UART_Receive_DMA(&huart1, &screen_data.rx_u8, 1);

  AESK_CAN_Init(&aesk_gl.aesk_can_1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(sys_timer._1ms_flag == TRUE)
	{
		//ADC Median Filter
		/*static uint8_t ctr = 0;
		static uint64_t adc_sum = 0;
		adc_sum += sensor_data.adc_val_u16;
		ctr++;

		if(ctr == 10)
		{
			sensor_data.filtered_adc_val_u16 = adc_sum / ctr;
			adc_sum = 0;
			ctr = 0;
		}*/
		sys_timer._1ms_flag = FALSE;
	}


	if(sys_timer._20ms_flag == TRUE)
	{

		sys_timer._20ms_flag = FALSE;
	}

	if(sys_timer._10ms_flag == 1)
	{
		//Direksiyon Acisi Hesaplamasi
		/*
		if(sensor_data.filtered_adc_val_u16 > MIDDLE_POINT_ADC_VAL)
		{
			lyra_can_data.tx.telemetry.steering_angle_u8 = ((sensor_data.filtered_adc_val_u16 - MIDDLE_POINT_ADC_VAL) / (MAX_STEERING_ADC_VAL - MIDDLE_POINT_ADC_VAL) * MAX_STEERING_TURN) + MAX_STEERING_TURN;
		}

		else
		{
			lyra_can_data.tx.telemetry.steering_angle_u8 = ((-sensor_data.filtered_adc_val_u16 + MIDDLE_POINT_ADC_VAL) / (MAX_STEERING_ADC_VAL - MIDDLE_POINT_ADC_VAL) * -MAX_STEERING_TURN) + MAX_STEERING_TURN;
		}*/

		sys_timer._10ms_flag = 0;
	}

	if(sys_timer._100ms_flag == TRUE)
	{
		static float old_rpm_f32;
		//RPM to KM/h
		vcu_data.actual_velocity_i8 = lyra_can_data.rx.mcu.motor_rpm_f32 * RPM_TO_KMH_COEF;

		//KM/h to RPM
		lyra_can_data.tx.mcu.speed_set_rpm_i16 = vcu_data.set_velocity_u8 / RPM_TO_KMH_COEF;
		vcu_data.set_rpm_f32 = vcu_data.set_velocity_u8 / RPM_TO_KMH_COEF;

		switch(lyra_can_data.tx.mcu.states_union.states_t.mode)
		{
			case Torque_Control:
			{
				if(lyra_can_data.tx.mcu.states_union.states_t.reverse == ON)
				{
					lyra_can_data.tx.mcu.torque_set_i16 = (vcu_data.set_torque_u8 * -1);
				}

				else
				{
					lyra_can_data.tx.mcu.torque_set_i16 = vcu_data.set_torque_u8;
				}
				break;
			}

			case Speed_Control:
			{
				if(lyra_can_data.tx.mcu.states_union.states_t.reverse == 0)
				{
					if(lyra_can_data.tx.mcu.states_union.states_t.ignition != 0 &&
					   lyra_can_data.tx.mcu.states_union.states_t.brake != 1 )

					{
						//Acceleration Regulation,
//						if()
//						{
//
//						}
						lyra_can_data.tx.mcu.torque_set_i16 = AESK_Speed_PI(electronic_dif_param.min_neg_out, electronic_dif_param.max_pos_out, vcu_data.set_rpm_f32, lyra_can_data.rx.mcu.motor_rpm_f32, DT);
					}

					else
					{
						electronic_dif_param.integral_sum_f32 = 0.0f;
						electronic_dif_param.integral_f32 = 0.0f;
					}
				}

				else
				{
					lyra_can_data.tx.mcu.torque_set_i16 = -1 * (int16_t)vcu_data.set_velocity_u8;
				}
				break;
			}
			default: break;
		}

		//Pack 1
		uint16_t mcu_idx = 0;
		uint8_t mcu_data[8];

		AESK_UINT8toUINT8CODE(&lyra_can_data.tx.mcu.states_union.states_u8, mcu_data, &mcu_idx);
		AESK_INT16toUINT8_LE(&lyra_can_data.tx.mcu.speed_set_rpm_i16, mcu_data, &mcu_idx);
		AESK_INT16toUINT8_LE(&lyra_can_data.tx.mcu.torque_set_i16, mcu_data, &mcu_idx);
		AESK_INT16toUINT8_LE(&lyra_can_data.tx.mcu.torque_set_2_i16, mcu_data, &mcu_idx);
		AESK_UINT8toUINT8CODE(&lyra_can_data.tx.mcu.torque_limit_u8, mcu_data, &mcu_idx);
		AESK_CAN_SendExtIDMessage(&aesk_gl.aesk_can_1, VCU_CAN_ID_1, (uint8_t *)mcu_data, sizeof(mcu_data));

		//Pack 2
		uint16_t packet2_idx = 0;
		uint8_t packet2_data[8];

		AESK_UINT8toUINT8CODE(&lyra_can_data.tx.telemetry.steering_angle_u8, packet2_data, &packet2_idx);
		AESK_INT16toUINT8_LE(&vcu_data.l_rpm_i16, packet2_data, &packet2_idx);
		AESK_INT16toUINT8_LE(&vcu_data.r_rpm_i16, packet2_data, &packet2_idx);
		AESK_CAN_SendExtIDMessage(&aesk_gl.aesk_can_1, VCU_CAN_ID_2, (uint8_t *)packet2_data, sizeof(packet2_data));

		sys_timer._100ms_flag = FALSE;
	}

	 //Buton kontrolu 120ms'de bir yapilarak debounce engellenmiş olunur.
	 if(sys_timer._120ms_flag == TRUE)
	 {
		  AESK_Read_All_Signals();
		  sys_timer._120ms_flag = FALSE;
	 }

 	  //Buffer'a toplanan Nextion komutlari iletilir.
 	 if(sys_timer._200ms_flag == TRUE)
 	 {
 		 if(vcu_data.bms_on_bus == 1)
		 {
			//Tepe Lambasi Kontrolü
			Beacon_Control();
		 }

 		 //Nextion Buffer TX
 		 Send_Buf_2_Nextion();
 		 sys_timer._200ms_flag = 0;
 	  }

 	  if(sys_timer._1000ms_flag == TRUE)
 	  {
#if COM_MODE == COMPRO || COM_MODE == MCU_OLD
 		 static uint64_t anlik_sayac_MCU = 0;
 		 sayac_MCU_1sn = sayac_MCU - anlik_sayac_MCU;
 		 anlik_sayac_MCU  = sayac_MCU;
 		 static uint64_t anlik_sayac_Telemetry = 0;
		 sayac_Telemetry_1sn = sayac_Telemetry - anlik_sayac_Telemetry;
		 anlik_sayac_Telemetry = sayac_Telemetry;
#endif

 		 //On tekerlerin RPM Hesabi
		 if(lyra_can_data.tx.mcu.states_union.states_t.reverse == ON)
		 {
			 vcu_data.l_rpm_i16 = -(sensor_data.l_rim_hole_ctr / 24.0f) * 60.0f;
			 vcu_data.r_rpm_i16 = -(sensor_data.r_rim_hole_ctr / 15.0f) * 60.0f;
		 }

		 else
		 {
			 vcu_data.l_rpm_i16 = (sensor_data.l_rim_hole_ctr / 24.0f) * 60.0f;
			 vcu_data.r_rpm_i16 = (sensor_data.r_rim_hole_ctr / 15.0f) * 60.0f;
		 }

		 sensor_data.l_rim_hole_ctr = 0;
		 sensor_data.r_rim_hole_ctr = 0;


		 //Telemetri Sorgu Cevabi
		 Telemetry_Query_Answer();

 		 //Odometer Hesabi
 		 vcu_data.odometer_u32 += abs(vcu_data.m1_actual_velocity_i8 + vcu_data.m2_actual_velocity_i8) / 2;

 		 sys_timer._1000ms_flag = FALSE;
 	  }
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 129;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Beacon_Trigger_GPIO_Port, Beacon_Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAN1_STDBY_Pin|CAN2_STDBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Brake_Switch_Pin */
  GPIO_InitStruct.Pin = Brake_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Brake_Switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Reverse_Switch_Pin Fake_Brake_Pin Ignition_Button_2_Pin Reset_Vel_Btn_Pin
                           Decrease_Vel_Btn_Pin */
  GPIO_InitStruct.Pin = Reverse_Switch_Pin|Fake_Brake_Pin|Ignition_Button_2_Pin|Reset_Vel_Btn_Pin
                          |Decrease_Vel_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Trq_Spd_Btn_Pin */
  GPIO_InitStruct.Pin = Trq_Spd_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Trq_Spd_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Increase_Vel_Btn_Pin */
  GPIO_InitStruct.Pin = Increase_Vel_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Increase_Vel_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : System_On_Off_Pin Ignition_Button_Pin */
  GPIO_InitStruct.Pin = System_On_Off_Pin|Ignition_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Beacon_Trigger_Pin */
  GPIO_InitStruct.Pin = Beacon_Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Beacon_Trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_Sensor_R_Pin */
  GPIO_InitStruct.Pin = RPM_Sensor_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RPM_Sensor_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_Sensor_L_Pin */
  GPIO_InitStruct.Pin = RPM_Sensor_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_Sensor_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN1_STDBY_Pin CAN2_STDBY_Pin */
  GPIO_InitStruct.Pin = CAN1_STDBY_Pin|CAN2_STDBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
