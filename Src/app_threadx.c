/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motorcontrol.h"
#include "mc_tasks.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define THREAD_STACK_SIZE 1024
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t mc_thread_stack[THREAD_STACK_SIZE];
TX_THREAD mc_thread_ptr;
extern FDCAN_HandleTypeDef hfdcan1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void mc_thread(ULONG initial_input);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
  (void)byte_pool;
  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  tx_thread_create(&mc_thread_ptr, "mc_thread", mc_thread, 0U, mc_thread_stack, sizeof(mc_thread_stack), 15, 15, 1, TX_AUTO_START);
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
bool sendCan(uint32_t aIdentifier, uint32_t aDataLength, uint8_t* aTxDataPointer) {

	  // Send via FDCAN
	  FDCAN_TxHeaderTypeDef fdCanHeader;
	  fdCanHeader.BitRateSwitch = FDCAN_BRS_OFF;
	  switch(aDataLength) {
	    case 0u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_0;
	      break;
	    case 1u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_1;
	      break;
	    case 2u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_2;
	      break;
	    case 3u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_3;
	      break;
	    case 4u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_4;
	      break;
	    case 5u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_5;
	      break;
	    case 6u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_6;
	      break;
	    case 7u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_7;
	      break;
	    case 8u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_8;
	      break;
	    case 12u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_12;
	      break;
	    case 16u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_16;
	      break;
	    case 20u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_20;
	      break;
	    case 24u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_24;
	      break;
	    case 32u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_32;
	      break;
	    case 48u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_48;
	      break;
	    case 64u: fdCanHeader.DataLength = FDCAN_DLC_BYTES_64;
	      break;
	  default:
	    //Invalid data length set
	    return false;

	  }

	  fdCanHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	  fdCanHeader.FDFormat = FDCAN_CLASSIC_CAN;
	  fdCanHeader.Identifier = aIdentifier;
	  fdCanHeader.IdType = FDCAN_STANDARD_ID;
	  fdCanHeader.MessageMarker = 0u;
	  fdCanHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	  fdCanHeader.TxFrameType = FDCAN_DATA_FRAME;

	  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &fdCanHeader, aTxDataPointer) == HAL_OK) {
	    return true;
	  }
	  else{
	    return false;
	  }
	}

void mc_thread(ULONG initial_input)
{
	MX_MotorControl_Init();
	while(1)
	{
        MC_RunMotorControlTasks();
        sendCan(0x123, 1, "s");
        tx_thread_sleep(1U);
	}
}
/* USER CODE END 1 */
