/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#define					SET								1
	#define					NULL							0

	#define					HEARTBEAT					0x01
	#define					CMD_MASK					0x01f		
	#define					BT_READ						HAL_GPIO_ReadPin(UART5_State_GPIO_Port,UART5_State_Pin);

	#define				VELOCITY					0
	#define				TORQUE						1
	#define				VEL_LIMIT					2
	#define				MOT_ERROR					3
	#define				ENC_ERROR					4
	#define				IQ								5
	#define				ENC_EST						6
	#define				T_RAMP						7
	#define				REQ_STATE					8
	#define				SNL_ERROR					9
	#define				SENSL_EST					10
	
	
	#define				VEL_ID						0x0D
	#define				TRQ_ID						0x0E
	#define				VLMT_ID						0x0F
	#define				MERR_ID						0x03
	#define				ENERR_ID					0x04
	#define				SNERR_ID					0x05
	#define				IQM_ID						0x14
	#define				ENEST_ID					0x009
	#define				T_RAMP_ID					0x1C
	#define				REQ_STATE_ID			0x07
	#define				SENS_EST					0x015
	#define				VOLTAGE					  0x017
	#define				TOP_SENSR					0x261

	#define				REMOTE						1
	#define				DATA							2
	#define				Def_Size					4
	#define				CAN_LOOP					2
	#define				LSB								0
	#define				MSB	 							1
	#define				ALL								2
	#define 			Z									7//3
	
	#define					BUZZER_ON						HAL_GPIO_WritePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin, SET);			HAL_GPIO_WritePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin, SET);	  
	#define					BUZZER_OFF					HAL_GPIO_WritePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin, NULL);			HAL_GPIO_WritePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin, NULL);	
	#define					BUZZER_TOGGLE				HAL_GPIO_TogglePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin );					HAL_GPIO_TogglePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin);

	#define					DRIVE_TORQUE				5
	#define					Jump_Time						10
	#define					ARM_MIN							1
	#define					ARM_MAX							25 //25//15
	#define					A_Kp								2//3
	#define					A_LIMIT							50
	#define					ARM_HOMING_SPEED	 	15
	
	
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
float print=40;
/*			CAN Variables      */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8],  RxBuff[8];
uint32_t TxMailbox;
uint16_t Received_Node_Id=0, Received_Command_Id=0,Node_Id[20];
float Motor_Velocity[20];
uint8_t Motor_Error[20], Encoder_Error[20] , Motor_Current[20];
uint8_t Axis_State[20] , Controller_Status[20], Error_Status[20];
uint8_t Voltage[4];
/*			CAN Variables      */



uint8_t BT_Rx[8], BT_State=0, Enc_Rx[8];
uint8_t Bt_Data[1]={1};
uint8_t Mode=0,Speed=0,Shearing=0,Joystick=0,Steering_Mode=0, Pot_Angle=0, Joystick_Temp=0, Shearing_Temp=0;
float Vel_Limit=1, Vel_Limit_Temp=1, Torque=0, Torque_Temp=0 , Prev_Torque=0, Prev_Vel_Limit=-0.2, Joy_New_Temp=0;
uint8_t cantest=0,BT_Count=0,ux_fail=0;
float Turn_Ratio=0,Reduced_Speed=0, Turn_Temp=0, Reduced_Speed_Temp=0; 
uint8_t Mod[8], Idle_Wheels=1;
float Current=0;
/*																								ARM DEFINITIONS																							*/		
float L_Arm_Speed=0, R_Arm_Speed=0, L_Arm_Speed_Temp=0, R_Arm_Speed_Temp=0, Pitch_Arm_Speed=0, Pitch_Arm_Speed_Temp=0;
uint8_t L_Arm=12, R_Arm=13, P_Arm=14;
float Left_Arm_Error=0, Right_Arm_Error=0, Pitch_Arms_Error=0;
//int Left_Target=18+Z-5, Right_Target=15+Z-5, Pitch_Target=15+Z, Flap_Target=30;//p=20+Z, l=18+z, r=20+z
int Left_Target=10+Z, Right_Target=13+Z-5, Pitch_Target=8+Z, Flap_Target=30;//p=20+Z, l=18+z, r=20+z
//int Left_Target=5, Right_Target=2, Pitch_Target=2, Flap_Target=30;//p=20+Z, l=18+z, r=20+z
float Left_Arm=0, Right_Arm =0 , Pitch_Arm=0;
uint16_t Left_Arm_Raw=0, Right_Arm_Raw =0 , Pitch_Arm_Raw=0;
uint16_t Left_Arm_Limit = 0 ,	Right_Arm_Limit =0 , Pitch_Arm_Limit=0; 
_Bool Flap_Sensed = 1 , Switch_Shearing = 0, Sensing_Switch=1, Shearing_Sensed = 0;
uint8_t ARM_BOUNDARY =2 , Arm_Prop_Factor=1;
uint16_t LA_Home_Pos = 69-30, RA_Home_Pos=400-30, PA_Home_Pos = 428-30;
/*																								ARM DEFINITIONS																							*/	
/*																TOP SENSOR DEFINITIONS																							*/
uint16_t Left_Enc_Zero_Pos=289, Right_Enc_Zero_Pos=445, Pitch_Enc_Zero_Pos=703, Flap_Enc_Zero_Pos=125;
uint8_t TOP_SENS=0;
float Left_Encoder,Right_Encoder, Pitch_Encoder, Flap_Encoder;
uint16_t Left_Encoder_Conv,Right_Encoder_Conv, Pitch_Encoder_Conv, Flap_Encoder_Conv;
float Left_Encoder_Conv_Angle,Right_Encoder_Conv_Angle, Pitch_Encoder_Conv_Angle, Flap_Encoder_Conv_Angle;
	uint8_t Flap_Sensed_Temp = 1;
	uint16_t Volt=0;
	float New=0;
/*																TOP SENSOR DEFINITIONS																							*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void Joystick_Reception(void);
void Drive_Wheel_Controls(void);
void Set_Motor_Torque ( uint8_t Axis , float Torque );
void CAN_Transmit ( uint8_t NODE, uint8_t Command, float Tx_Data,	uint8_t Data_Size, uint8_t Frame_Format);
void Set_Motor_Velocity ( uint8_t Axis , float Velocity );
void Shearing_Motors (void);
void Start_Calibration_For (uint8_t Axis, int Command, uint8_t loop_times);
void Skid_Turning ( void );
void UART_Reception ( void );
void Arm_Controls (void);
void Flap_Sensing(void);
void Reboot (int Axis);
void Clear_Errors(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float New_Sensor_Pos(float sensorvalue, float zero_pos)
{
float output;

output=((sensorvalue-zero_pos)>360.0)? ((sensorvalue-zero_pos)-720.0) : (sensorvalue-zero_pos);
output = (output<-359.0)?(output+720.0):(output);
return (output/2);
}
void TOP_SENS_READ( void )
{ 
				Left_Encoder_Conv = RxData[0];
				Left_Encoder_Conv_Angle = Left_Encoder_Conv << 8 | RxData[1];	
				Left_Encoder = fabs(New_Sensor_Pos ( Left_Encoder_Conv_Angle , Left_Enc_Zero_Pos));
	
								Right_Encoder_Conv = RxData[2];
								Right_Encoder_Conv_Angle = Right_Encoder_Conv << 8 | RxData[3];
								Right_Encoder = fabs(New_Sensor_Pos ( Right_Encoder_Conv_Angle , Right_Enc_Zero_Pos));
	
												Pitch_Encoder_Conv = RxData[4];
												Pitch_Encoder_Conv_Angle = Pitch_Encoder_Conv << 8 | RxData[5];
												Pitch_Encoder = fabs(New_Sensor_Pos ( Pitch_Encoder_Conv_Angle , Pitch_Enc_Zero_Pos));
												
	
																Flap_Encoder_Conv = RxData[6];
																Flap_Encoder_Conv_Angle = Flap_Encoder_Conv << 8 | RxData[7];
																Flap_Encoder= (New_Sensor_Pos ( Flap_Encoder_Conv_Angle , Flap_Enc_Zero_Pos));																									
}	
float CAN_Reception(uint8_t byte_choice)
{
		float Can_Temp;
	
	if ( byte_choice == LSB )
	{
		for (int k=0; k<=3; k++)
		{
			RxBuff[k]= RxData[k];
		}
		memcpy(&Can_Temp, RxBuff,4);
		
	}
	
	else if ( byte_choice == MSB )
	{
		for (int k=0; k<=3; k++)
		{
			RxBuff[k]= RxData[k+4];
		}
		memcpy(&Can_Temp, RxBuff,4);
		
	}
	
	else { }

	return(Can_Temp);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ( huart->Instance == UART5 )
	{
			HAL_UART_Receive_IT(&huart5,BT_Rx ,sizeof(BT_Rx));
			//BT_Count++;
	}
	
	else if ( huart->Instance == UART4 )
	{
			HAL_UART_Receive_IT(&huart4,Enc_Rx ,sizeof(Enc_Rx));
			BT_Count++;
	}
	
	else {}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData);
	cantest++;
	
	Received_Node_Id = RxHeader.StdId >> 5;
	Received_Command_Id = RxHeader.StdId & CMD_MASK;
		
	if (RxHeader.StdId == 0x051 )
	{
//		TOP_SENS_READ();										
//		TOP_SENS++;
	}
	else
	{
		switch( Received_Command_Id )	
		{
			case HEARTBEAT:  							Node_Id[Received_Node_Id]++;    Axis_State[Received_Node_Id] = RxData[4]; break;//memcpy(&Error_Status[Received_Node_Id], RxData2, 4); Axis_State[Received_Node_Id] = RxData2[4];
			
			case ENEST_ID:  							Motor_Velocity[Received_Node_Id]	= CAN_Reception(MSB); 							break;		

			case SENS_EST:  							Motor_Velocity[Received_Node_Id]	= CAN_Reception(MSB); 				  		break;
			
			case MERR_ID:  								Motor_Error[Received_Node_Id]			= CAN_Reception(MSB); 							break;
			
			case ENERR_ID:  							Encoder_Error[Received_Node_Id]		= CAN_Reception(MSB); 					 		break;
			
			case SNERR_ID:  							Encoder_Error[Received_Node_Id]		= CAN_Reception(LSB); 					  	break;
			
			case IQM_ID:  								Motor_Current[Received_Node_Id]		= CAN_Reception(MSB); 					  	break;
			
			case VOLTAGE: 								memcpy(&Volt, RxData, 4);	  break;
			
//			case 0x261: Node_Id[19]++; break;
//		  case 0x281: Node_Id[20]++; break;
		
			default: 																																													  break;

		}
	}
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
  MX_CAN1_Init();
  MX_CAN2_Init();
//  MX_UART4_Init();
//  MX_UART5_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
	BUZZER_ON;
	
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2 , CAN_IT_RX_FIFO0_MSG_PENDING);
	
	

	HAL_Delay(3000);
	
	MX_UART4_Init();
  MX_UART5_Init();
	//	HAL_Delay(3000);

	HAL_UART_Receive_IT(&huart5,BT_Rx ,sizeof(BT_Rx));
//	HAL_UART_Receive_IT(&huart4,Enc_Rx ,sizeof(Enc_Rx));
//	Start_Calibration_For( 2 , 3 , 1); HAL_Delay(10000);
	HAL_Delay(2000);
//	for(uint8_t i=1; i < 5; i++){Start_Calibration_For( i , 8 , 5); HAL_Delay(100);}
	for(uint8_t i=12; i < 19; i++){Start_Calibration_For( i , 8 , 5); HAL_Delay(100);}
	
	for ( uint8_t i=1 ; i < 5 ; i++ )
		{
			Clear_Errors();
			HAL_Delay(2000);
//			clr++;
		}
	
	
	
	
	
//	HAL_TIM_Base_Start_IT(&htim14); 
//	Start_Calibration_For( 18 , 8 , 5);
	BUZZER_OFF;
//	Voltage[0]=0;
//		Voltage[1]=0;
//		Voltage[2]=0x48;
//		Voltage[3]=0x42;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		BT_State = BT_READ;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		UART_Reception();
		Joystick_Reception();
//		Drive_Wheel_Controls();
//		Skid_Turning();
		//Arm_Controls();
//		Shearing_Motors();
		
//		memcpy(&New, Voltage,4);
//		HAL_Delay(2000);
	
		
//		if( R_Arm_Speed_Temp != R_Arm_Speed )
//		{
//			Set_Motor_Velocity (R_Arm , -R_Arm_Speed );	
//			R_Arm_Speed_Temp = R_Arm_Speed ;
//		}
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
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
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	
	
	CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 14;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x000<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x000<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14;  // doesn't matter in single can controllers

  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);
	

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 9000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|Buzzer_1_Pin|Buzzer_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART4_State_Pin */
  GPIO_InitStruct.Pin = UART4_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UART4_State_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_1_Pin Buzzer_2_Pin */
  GPIO_InitStruct.Pin = Buzzer_1_Pin|Buzzer_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : UART5_State_Pin */
  GPIO_InitStruct.Pin = UART5_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UART5_State_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void UART_Reception ( void )
{
	//HAL_UART_Receive(&huart4,Enc_Rx ,sizeof(Enc_Rx), 1);
	
	if ( ( Enc_Rx[0] == 0x11 ) &&	( Enc_Rx[7] == 0xEE ) )
	{	
		Left_Arm_Raw = Enc_Rx[5];
		Left_Arm_Raw = Left_Arm_Raw << 8 | Enc_Rx[6];
		Left_Arm = New_Sensor_Pos (Left_Arm_Raw, LA_Home_Pos );
		
		Right_Arm_Raw = Enc_Rx[1];
		Right_Arm_Raw = Right_Arm_Raw << 8 | Enc_Rx[2];
		Right_Arm = New_Sensor_Pos (Right_Arm_Raw, RA_Home_Pos );
		
		Pitch_Arm_Raw = Enc_Rx[3];
		Pitch_Arm_Raw = Pitch_Arm_Raw << 8 | Enc_Rx[4];
		Pitch_Arm = New_Sensor_Pos (Pitch_Arm_Raw, PA_Home_Pos );
		
	}
	else 
		{	
		//HAL_UART_AbortReceive(&huart4);
		//for(uint8_t i=0 ; i<8;i++)Enc_Rx[i] = 0;
			ux_fail++;
			BUZZER_ON;
//			MX_UART4_Init();HAL_Delay(200);
//			HAL_UART_AbortReceive(&huart4); HAL_Delay(1000);
		}


}
void Joystick_Reception(void)
{
/* 1. Turn On buzzer if the Bt is not connected 
	 2. Assign Buffer values to the appropriate functions	*/

	/*				JOYSTICK VALUES ASSIGNING								*/
	
	
	if (( BT_Rx[0] == 0xAA ) &&	( BT_Rx[7] == 0xFF ))
	{	
		Mode 						 = BT_Rx[1];
		Speed 					 = BT_Rx[2]  != 0 ? BT_Rx[2] : Speed ;	
		Steering_Mode 	 = BT_Rx[3];
		Pot_Angle 			 = BT_Rx[4]; 
		Joystick 				 = BT_Rx[5];
		Shearing				 = BT_Rx[6];
	}	

	/*				JOYSTICK VALUES ASSIGNING								*/
///////////////////////////////////////////////////////////////////////////////////	
	/*				Machine Speed Alotter							*/
	
/*	 Vel_Limit = 20 + Speed * 20;

	//if(Vel_Limit > 40 ) Vel_Limit = 40;
	if( Prev_Vel_Limit > Vel_Limit ) 	
	{
			 for ( float v = Prev_Vel_Limit-2; v >= Vel_Limit  ; v=v-2 )
			 {	
				 for(int i=1 ; i < 5 ; i++) 
				{
					CAN_Transmit(i,VEL_LIMIT,v,4,DATA);
			  }
			 }
		HAL_Delay(Jump_Time);
		Prev_Vel_Limit = Vel_Limit ;
	}
	
	else// if( Prev_Vel_Limit < Vel_Limit ) 	
	{
			 for ( float v = Prev_Vel_Limit+2; v <= Vel_Limit  ; v=v+2 )
			 {	
				 for(int i=1 ; i < 5 ; i++) 
				{
					CAN_Transmit(i,VEL_LIMIT,v,4,DATA);
			  }
			 }
		HAL_Delay(200);
		Prev_Vel_Limit = Vel_Limit ;
	}
	*/
//	else if ( Vel_Limit_Temp != Vel_Limit && Vel_Limit != 0 )
//	{
//		for( uint8_t i=1 ; i < 5 ; i++ )
//		{
//		 CAN_Transmit(i,VEL_LIMIT,Vel_Limit,4,DATA);HAL_Delay(1);
//		} 
////		for(uint8_t i=1 ; i < 5 ; i++) {Set_Motor_Torque(i, Torque);}
//				
//	Prev_Vel_Limit = Vel_Limit ;
//	Vel_Limit_Temp = Vel_Limit;
//	}
	
	/*				Machine Speed Alotter								*/

}

void Drive_Wheel_Controls(void)
{
	/* Actuates the Drive Wheels only when the Steering Resets have been completed and the Bluetooth is in Connection. */

	if ( (Speed!= 0) && (BT_State))
	{
		if ( Joystick_Temp != Joystick )
		{
			switch (Joystick)
			{
				case 0 :   Torque =  NULL;  							break;								
				case 1 :   Torque =	-DRIVE_TORQUE; 				break; 
				case 2 :   Torque = DRIVE_TORQUE;					break; 
				case 3 :   Torque =	DRIVE_TORQUE;					break; 
				case 4 :   Torque = DRIVE_TORQUE;					break;
				default :																	break;
			}
//			for ( uint8_t i = 1 ; i < 5 ; i++ ){ Set_Motor_Torque ( i , Torque );}
			
			if ( Torque != 0 && Prev_Torque == 0 )
			{
				if ( Idle_Wheels) 
				{
					for ( uint8_t i = 1 ; i < 5 ; i++ ){ Set_Motor_Torque ( i , Torque );}
					Joystick_Temp = Joystick;
					Prev_Torque = Torque;
				}
			}
			else 
			{
			for ( uint8_t i = 1 ; i < 5 ; i++ ){ Set_Motor_Torque ( i , Torque );}
			Joystick_Temp = Joystick;
			Prev_Torque = Torque;
			}
//			Joystick_Temp = Joystick;
			
		//	Pot_Angle = Pot_Angle > 170 ? Pot_Angle-10 : Pot_Angle < 10 ? Pot_Angle+ 10 : Pot_Angle+5; // dummy line
		}
	}
	
	if ( Motor_Velocity[1] < 2 && Motor_Velocity[1] > -2 && Motor_Velocity[2] < 2 && Motor_Velocity[2] > -2 && Motor_Velocity[3] < 2 && Motor_Velocity[3] > -2 && Motor_Velocity[4] < 2 && Motor_Velocity[4] > -2 ) Idle_Wheels = SET;
	else Idle_Wheels = NULL;
	
	if ( Idle_Wheels ) Vel_Limit = 10;
	else if (  !Idle_Wheels )  Vel_Limit = 20 + Speed * 20;
	//Vel_Limit = 20 + Speed * 20;
	if ( Vel_Limit_Temp != Vel_Limit )
	{
		if( Prev_Vel_Limit > Vel_Limit ) 	
		{
				 for ( float v = Prev_Vel_Limit-2; v >= Vel_Limit  ; v=v-2 )
				 {	
					 for(int i=1 ; i < 5 ; i++) 
					{
						CAN_Transmit(i,VEL_LIMIT,v,4,DATA);
					}
					HAL_Delay(Jump_Time);
				 }
			
			Prev_Vel_Limit = Vel_Limit ;
			
		}
		
		else
		{
				 for ( float v = Prev_Vel_Limit+2; v <= Vel_Limit  ; v=v+2 )
				 {	
					 for(int i=1 ; i < 5 ; i++) 
					{
						CAN_Transmit(i,VEL_LIMIT,v,4,DATA);
					}
					HAL_Delay(Jump_Time);
				 }
			
			Prev_Vel_Limit = Vel_Limit ;
		}
	
	Vel_Limit_Temp = Vel_Limit;
	}
	
	if ( (!BT_State ) && Joystick != 0 )
	{
		for ( uint8_t i = 1 ; i < 5 ; i++ ){Set_Motor_Torque ( i , NULL );}
		Joystick = 0 ;
	}
	
}
void Shearing_Motors (void)
{
		if ( Shearing_Temp != Shearing )
	{
		if ( Shearing == 2 )
		{
			for ( int i=0; i < 1 ; i++)
			{
				Set_Motor_Velocity( 18 , -50);  // Cutter
				HAL_Delay(3000);
				Set_Motor_Velocity( 17 , 45 );// Side Belt
				HAL_Delay(3000);
				Set_Motor_Velocity( 15 , 35 ); // Paddle
				HAL_Delay(2000);
				Set_Motor_Velocity( 16 , 30 );  // Sel
				
				
			}
		}
		else 
		{	
			for ( int i=0; i < 4 ; i++)
			{
				Set_Motor_Velocity( 15 , 0 ); // cutter 
				Set_Motor_Velocity( 16 , 0 ); // side paddle
				Set_Motor_Velocity( 17 , 0 ); // selective 
				Set_Motor_Velocity( 18 , 0 ); // paddle
			}
		}
		 Shearing_Temp = Shearing ;
	}
}

void Skid_Turning ( void )
{
	Turn_Ratio = Pot_Angle - 90;
	Reduced_Speed = Vel_Limit - ((fabs(Turn_Ratio)) * (Vel_Limit / 100 ));
	
	if ( Turn_Ratio != Turn_Temp || Reduced_Speed != Reduced_Speed_Temp|| Joy_New_Temp != Joystick )// fabs(Motor_Velocity[4]) < Reduced_Speed - 2 || fabs(Motor_Velocity[4]) < Reduced_Speed + 2 || fabs(Motor_Velocity[1]) < Reduced_Speed - 2 || fabs(Motor_Velocity[1]) > Reduced_Speed + 2    )
	{
		if ( Reduced_Speed > 20 )
		{
			if ( Turn_Ratio < -3 )
			{
				if ( Turn_Temp >= Turn_Ratio)
				{
					for ( float i = fabs(Motor_Velocity[4])-1 ; i >= Reduced_Speed ; i=i-1)
					{		
						CAN_Transmit(3,VEL_LIMIT,i,4,DATA);
						CAN_Transmit(4,VEL_LIMIT,i,4,DATA);
						HAL_Delay (Jump_Time);
						print = i;
					}
				}
				
				else 
				{
					for ( float i = fabs(Motor_Velocity[4])+2 ; i <= Reduced_Speed ; i=i+2)
					{		
						CAN_Transmit(3,VEL_LIMIT,i,4,DATA);
						CAN_Transmit(4,VEL_LIMIT,i,4,DATA);
						HAL_Delay (Jump_Time);
						print = i;
					}
				}
			}
			
			else if ( Turn_Ratio > 3 )
			{
				if ( Turn_Temp <= Turn_Ratio)
				{
					for ( float i = fabs(Motor_Velocity[1])-1 ; i >= Reduced_Speed ; i=i-1)
					{		
						CAN_Transmit(1,VEL_LIMIT,i,4,DATA);
						CAN_Transmit(2,VEL_LIMIT,i,4,DATA);
						HAL_Delay (50);
						print = i;
					}
				}
				
				else 
				{
					for ( float i = fabs(Motor_Velocity[1])+2 ; i <= Reduced_Speed ; i=i+2)
					{		
						CAN_Transmit(1,VEL_LIMIT,i,4,DATA);
						CAN_Transmit(2,VEL_LIMIT,i,4,DATA);
						HAL_Delay (Jump_Time);
						print = i;
					}
				}
			}
			
			else 
			{
				for( uint8_t i=1 ; i < 5 ; i++ ){CAN_Transmit(i,VEL_LIMIT,Vel_Limit,4,DATA);}
			}
		}
	Turn_Temp = Turn_Ratio;
	Reduced_Speed_Temp = Reduced_Speed;
		Joy_New_Temp = Joystick ;
	}
	
	
	
	
	
}
void Set_Motor_Torque ( uint8_t Axis , float Torque )
{
	
	Torque = (Axis==4) || (Axis==2)  ? -Torque : Torque ;

	
	if ( Joystick == 4 ) // Zero Turn
	{
//	Torque = (Axis==4) || (Axis==2)  ? -Torque : Torque ;	
	//Torque = (Axis==1) || (Axis==2) ? -Torque : Torque ;	
		//Torque = -Torque;
		Torque = (Axis==3) || (Axis==4) ? Torque : -Torque ;	
//		Torque = -Torque;
	}
	
	else if ( Joystick == 3 )
	{
//	Torque = (Axis==4) || (Axis==2)  ? -Torque : Torque ;	
	Torque = (Axis==3) || (Axis==4) ? -Torque : Torque ;	
	
	}
	/*
	if(Joystick > 2)
	{
	if(Axis != 1) CAN_Transmit(Axis,TORQUE,Torque,4,DATA);
	}
	else */
		
	CAN_Transmit(Axis,TORQUE,Torque,4,DATA);
	
}
void Set_Motor_Velocity ( uint8_t Axis , float Velocity )
{
	CAN_Transmit(Axis,VELOCITY,Velocity,4,DATA);
}
void Start_Calibration_For (uint8_t Axis, int Command, uint8_t loop_times)
{
//	for ( uint8_t i=0; i < loop_times; i++)
//	{	
//	CAN_Transmit ( Axis, REQ_STATE, Command,	4 ,  DATA);
//	}
	memcpy(TxData, &Command, 4);		
				TxHeader.DLC = 4;	
				TxHeader.IDE = CAN_ID_STD;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.StdId = ( Axis <<5) | 0x007 ;	
		

		for (int i=0 ; i<loop_times ; i++ )	{	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);			HAL_Delay(20);			}
}

void CAN_Transmit ( uint8_t NODE, uint8_t Command, float Tx_Data,	uint8_t Data_Size, uint8_t Frame_Format)
{

	TxHeader.ExtId = NULL;
	
	TxHeader.TransmitGlobalTime = DISABLE;
	
	TxHeader.IDE = CAN_ID_STD;
	
	TxHeader.DLC	= Data_Size;
	
	TxHeader.RTR = (Frame_Format == REMOTE) ? (CAN_RTR_REMOTE) : (Frame_Format == DATA) ? (CAN_RTR_DATA):(CAN_RTR_REMOTE);
	
	switch (Command)
	{
		case VELOCITY:	
									memcpy (TxData, &Tx_Data, Data_Size);					
									TxHeader.StdId = (NODE << 5)| VEL_ID;
									break;
		
		case TORQUE:	
									memcpy (TxData, &Tx_Data, Data_Size);					
									TxHeader.StdId = (NODE << 5)| TRQ_ID;
									break;
		
		case VEL_LIMIT:	
									TxData[6] = 0x48;//0x20
									TxData[7] = 0x42;
									memcpy (TxData, &Tx_Data, 4);	
									TxHeader.DLC	= 8;
									TxHeader.StdId = (NODE << 5)| 0x00F;
									break;
		
		case MOT_ERROR: 	
									TxHeader.StdId = (NODE << 5)| MERR_ID;
									break;
		
		case ENC_ERROR:					
									TxHeader.StdId = (NODE << 5)| ENERR_ID;
									break;
		
		case SNL_ERROR:					
									TxHeader.StdId = (NODE << 5)| SNERR_ID;
									break;
		
		case IQ:						
									TxHeader.StdId = (NODE << 5)| IQM_ID;
									break;
		
		case ENC_EST:					
									TxHeader.StdId = (NODE << 5)| ENEST_ID;
									break;
									
		case T_RAMP:					
									memcpy (TxData, &Tx_Data, Data_Size);	
									TxHeader.StdId = (NODE << 5)| T_RAMP_ID;
									break;
		
		case SENSL_EST:					
									memcpy (TxData, &Tx_Data, Data_Size);	
									TxHeader.StdId = (NODE << 5)| 0x015;
									break;
		
		case REQ_STATE:					
									memcpy (TxData, &Tx_Data, 4 );	
									TxHeader.DLC = 4;	
									TxHeader.IDE = CAN_ID_STD;
									TxHeader.RTR = CAN_RTR_DATA;
									TxHeader.StdId = ( NODE << 5) | 0x007 ; 
									break;
		
		case 0x017:
									TxHeader.StdId = (NODE << 5)| 0x017;
									break;
		
		default: break;
		

	}

		for ( uint8_t i=0 ; i<3; i++ ) //3
	{
				HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox); 
				HAL_Delay(1);
	}
	for (uint8_t i=0 ; i <8 ; i++)
	{
		TxData[i] = 0;
	}
}

void Arm_Controls (void)
{ //Flap_Sensing();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
		/* 															SEMI-AUTO HOMING													*/
	if ( Mode == 1 )
	{

		L_Arm_Speed 		= (( Left_Arm <= ARM_BOUNDARY )  && ( Left_Arm >= -ARM_BOUNDARY ))  ? 0 : ( Left_Arm > ARM_BOUNDARY ) ? -ARM_HOMING_SPEED  : ARM_HOMING_SPEED;					//Left_Arm*Arm_Prop_Factor ;	
	
		R_Arm_Speed 		= (( Right_Arm <= ARM_BOUNDARY ) && ( Right_Arm >= -ARM_BOUNDARY )) ? 0 : ( Right_Arm > ARM_BOUNDARY ) ? ARM_HOMING_SPEED : -ARM_HOMING_SPEED;					//Right_Arm*Arm_Prop_Factor ;

		Pitch_Arm_Speed = (( Pitch_Arm <= ARM_BOUNDARY+2 ) && ( Pitch_Arm >= -ARM_BOUNDARY+2 )) ? 0 : ( Pitch_Arm > ARM_BOUNDARY )? -ARM_HOMING_SPEED :  ARM_HOMING_SPEED;					//Pitch_Arm_Speed*Arm_Prop_Factor ;
		Sensing_Switch = SET;
		Shearing_Sensed = NULL;
	}
		/* 															SEMI-AUTO HOMING													*/			
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
			

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	/*								TOP SENSING CONTROLS ------- PID						*/
	else if (	Mode == 2 && Sensing_Switch ) // auto
	{ 
	
		
		
		Left_Arm_Error = Left_Target - Left_Encoder;
		if ( Left_Arm_Error > 2 || Left_Arm_Error < -2 ) L_Arm_Speed = Left_Arm_Error*A_Kp;
		else L_Arm_Speed =0;
		L_Arm_Speed = L_Arm_Speed > A_LIMIT ? A_LIMIT : L_Arm_Speed < -A_LIMIT ? -A_LIMIT : L_Arm_Speed;


		Right_Arm_Error = Right_Target - Right_Encoder;
		if ( Right_Arm_Error > 2 || Right_Arm_Error < -2 ) R_Arm_Speed = Right_Arm_Error*A_Kp;
		else R_Arm_Speed =0;
		R_Arm_Speed = R_Arm_Speed > A_LIMIT ? A_LIMIT : R_Arm_Speed < -A_LIMIT ? -A_LIMIT : R_Arm_Speed;
	

		Pitch_Arms_Error = Pitch_Target - Pitch_Encoder;  //----> new
		if ( Pitch_Arms_Error > 2 || Pitch_Arms_Error < -2) Pitch_Arm_Speed = -Pitch_Arms_Error*A_Kp; //----> new
		else Pitch_Arm_Speed =0;
		Pitch_Arm_Speed = Pitch_Arm_Speed > A_LIMIT ? A_LIMIT : Pitch_Arm_Speed < -A_LIMIT ? -A_LIMIT : Pitch_Arm_Speed;

		if ( L_Arm_Speed == R_Arm_Speed == Pitch_Arm_Speed == 0   ) Shearing_Sensed = SET;

	}
	
	else if ( Mode==2 && !Sensing_Switch)
	{
		
		L_Arm_Speed = R_Arm_Speed = Pitch_Arm_Speed =0;
	
	}
	
	

	/*								TOP SENSING CONTROLS ------- PID						*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	
	else { for ( uint8_t i =12 ; i < 15 ; i++) { Set_Motor_Velocity( i, 0 ); } }
	 
		
		if ( L_Arm_Speed > 0 && Left_Arm > ARM_MAX) { L_Arm_Speed = 0; }// ---> LEFT ARM BOTTOM LIMIT 
		if ( L_Arm_Speed < 0 && Left_Arm < -ARM_MIN ) 	{ L_Arm_Speed = 0; }// ---> LEFT ARM TOP LIMIT 
//		
		if( L_Arm_Speed_Temp != L_Arm_Speed )
			{
		Set_Motor_Velocity (L_Arm , -L_Arm_Speed );	
		L_Arm_Speed_Temp = L_Arm_Speed ;
			}
		
		if ( R_Arm_Speed > 0 && Right_Arm < -ARM_MAX) 	{ R_Arm_Speed = 0; }// ---> RIGHT ARM BOTTOM LIMIT 
		if ( R_Arm_Speed < 0 && Right_Arm > ARM_MIN ) 	{ R_Arm_Speed = 0; }// ---> RIGHT ARM TOP LIMIT 
		
		if( R_Arm_Speed_Temp != R_Arm_Speed )
		{
			Set_Motor_Velocity (R_Arm , -R_Arm_Speed );	
			R_Arm_Speed_Temp = R_Arm_Speed ;
		}
		
		if ( Pitch_Arm_Speed < 0 && Pitch_Arm < -ARM_MAX+5) 	{ Pitch_Arm_Speed = 0; }// ---> PITCH ARM BOTTOM LIMIT 
		if ( Pitch_Arm_Speed > 0 && Pitch_Arm > ARM_MIN ) 	{ Pitch_Arm_Speed = 0; }// ---> PITCH ARM TOP LIMIT
		
		if( Pitch_Arm_Speed_Temp != Pitch_Arm_Speed )    // 35 deg Range
		{
			Set_Motor_Velocity (P_Arm , -Pitch_Arm_Speed );	
			Pitch_Arm_Speed_Temp = Pitch_Arm_Speed ;
		}
		
}

void Flap_Sensing(void)
{
	if ( Shearing_Sensed )
	{
		Flap_Sensed = Flap_Encoder < -50   ? SET : NULL ;  
		
		if ( Flap_Sensed != Flap_Sensed_Temp )
		{
			Switch_Shearing = SET;
			Flap_Sensed_Temp = Flap_Sensed;
		}
	}
}

void Reboot (int Axis)
{
	
	
			//	memcpy(TxData, &command_id, 4);		
				TxHeader.DLC = 4;	
				TxHeader.IDE = CAN_ID_STD;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.StdId = ( Axis <<5) | 0x016 ;	
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
	HAL_Delay(1); 

	//	if( axis_id < 19) {for (int i=0 ; i<loop_times ; i++ )	{	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);HAL_Delay(20); } }
	//	else { for (int i=0 ; i<loop_times ; i++ )	{	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);			HAL_Delay(20);		}	}
				
}

void Clear_Errors(void)
{
	for ( uint8_t i = 1 ; i < 12 ; i++)
	{
		if ( i != 5 ){ if ( Axis_State[i] != 8 ){Reboot(i);} }
	}
	HAL_Delay(2000);
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
