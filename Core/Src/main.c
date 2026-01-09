/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void DbgIO(int set){
	if( set ==0 || set ==1 )
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, set);
	else
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
int LightOn=0;
struct UartRcv_t {
	char RxByte; // recv byte
	volatile uint8_t nRx;   // Data cnt in Rx Buf
	uint8_t RxMax; // Size of burst

	char HdrByte; // first byte expected value

	int RxBufSz;
	char RxBuf[16]; //

	int nErr;
	volatile int TimedOut;
	uint32_t LastRcvGood; // tick last good rx occured
	uint32_t BadRx;
	UART_HandleTypeDef *huart;
	TIM_HandleTypeDef  *htim;
	void  (*Process)(struct UartRcv_t *Rx); //call when burst data received

};

uint32_t SumSerie(uint8_t *Data, int n){
	uint32_t sum;
	int i;
	for( i=0, sum=0; i<n; i++)
		sum+=Data[i];
	return sum;
}

//Rx shall have at least RaMax data this is not check here
/**
 * VAldiate buffer start byet and sum
 * Rx shall have at least RaMax data this is not check here
 * @param Rx The ex
 * @return 0 f validate non 0 otherwise
 */
int RxValidate(struct UartRcv_t *Rx) {
	uint32_t sum;
	if (Rx->RxBuf[0] == Rx->HdrByte ) {
		sum = SumSerie((uint8_t*) Rx->RxBuf, Rx->RxMax);
		if ( (sum & 0xFF) == Rx->RxBuf[Rx->RxMax - 1]) {
			return 0;//ok packet is valid
		}
	}
	return -1;
}

void  EdkProcess(struct UartRcv_t *Rx){
	if( RxValidate(Rx)==0 ){
		Rx->LastRcvGood = HAL_GetTick();
		LightOn =Rx->RxBuf[1]&0x01; // bit 1 of control motot flasg
	}else {
		// shit rx 1 byte ?
		Rx->BadRx++;
	}
}


struct UartRcv_t   EdkRx = {
		.RxMax = 7, // edk sent 7 byte per packet
		.HdrByte = 0x59,
		.RxBufSz = sizeof(EdkRx.RxBuf),
		.huart = &huart1, //     PA10     ------> USART1_RX
		.htim = &htim2,
		.Process = EdkProcess,
};
struct UartRcv_t   MotRx = {
		.RxMax = 8, // tsdz sent 8 byte per packet
		.HdrByte = 0x43,
		.RxBufSz = sizeof(MotRx.RxBuf),
		.huart = &huart2,
		// PA3     ------> USART2_RX
	    // PA2     ------> USART2_TX
		.htim = &htim3,
};

void KickRx(struct UartRcv_t *Rx){
	Rx->nRx=0;
	__HAL_TIM_DISABLE(Rx->htim);
	HAL_UART_Receive_IT(Rx->huart, (void*)Rx->RxBuf,1);
}

void Rcv_ReamTimeOut(struct UartRcv_t *Rx){
	Rx->htim->Instance->CNT=0;
	__HAL_TIM_CLEAR_FLAG(Rx->htim, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(Rx->htim, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE(Rx->htim);
	DbgIO(-1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	struct UartRcv_t  *Rx = huart == EdkRx.huart ? &EdkRx : &MotRx;
	if(Rx->nRx < Rx->RxBufSz ){
		Rx->RxBuf[Rx->nRx]=Rx->RxByte;
		Rx->nRx++;
	}
	else {
		//how idle is dead not picking ?
	}
	HAL_UART_Receive_IT(Rx->huart, (void*)&Rx->RxByte,1);

	if( Rx->nRx < Rx->RxMax ) // when full packet rcv no need to use iddle time for eod of burst reset
		Rcv_ReamTimeOut(Rx);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	struct UartRcv_t  *Rx = huart == EdkRx.huart ? &EdkRx : &MotRx;
	Rx->nErr++;
	//Rx->nRx = 0;
	//We could continue on the end crc +  timeout with not enough data  will says if goog
	// has if error is due to some glitch it can  be good on the end
	HAL_UART_Receive_IT(Rx->huart, (void*)&Rx->RxByte,1);
	Rcv_ReamTimeOut(Rx);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	struct UartRcv_t  *Rx = htim == EdkRx.htim ? &EdkRx : &MotRx;
	__HAL_TIM_DISABLE_IT(Rx->htim, TIM_IT_UPDATE);
	__HAL_TIM_DISABLE(htim);
	DbgIO(-1);
	htim->Instance->CNT = 0 ;
	Rx->TimedOut = 1;
}
/**
 * reset reception
 */
void RxReset( struct UartRcv_t  *Rx){
	 Rx->nRx = 0 ;
	 Rx->TimedOut = 0;
}
char DbgTx[16]={0x01, 0x02};

void DoDbgTx(int *Cnt){
	if( Cnt && MotRx.huart->gState ==  HAL_UART_STATE_READY ){
		HAL_UART_Transmit_DMA(MotRx.huart, (void*)DbgTx,*Cnt);
		*Cnt= 0;
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
	  struct UartRcv_t  *Rx;
	  int i;
	  volatile int DbgTxCnt=0;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  KickRx(&EdkRx);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	  //Check for 2 Rx time out ou Rx liit
	  for( i=0; i<1; i ++ ) { //dbgg only run the displauy rx
		 Rx = i==0 ? &EdkRx : &MotRx;

		 if ( Rx->nRx >=  Rx->RxMax ){
			 if(Rx->Process )
					Rx->Process(Rx);
			 RxReset(Rx);
		 }else {
			 if( Rx->TimedOut && Rx->nRx < Rx->RxMax ){ // repeat check minimize unlikely race effect
				 RxReset(Rx);
			 }
		 }
	  }
	  if( DbgTxCnt ){
		 // DbgTxCnt
		  DoDbgTx((void*)&DbgTxCnt);
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
