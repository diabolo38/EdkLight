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
#include <stdio.h>
#include <string.h>
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
void SetLight();
void MotDbg( char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SOFT_TIMEOUT	3 // ifdef use rx end of  message by soft time  time out else h/w timer used
//the 3  define the  time out in tick so between 2 and 3  ms given  1 ms error  on start and check
struct UartRcv_t {
	char RxByte; // recv byte
	volatile uint8_t nRx;   // Data cnt in Rx Buf
	uint8_t RxMax; // Size of burst

	char HdrByte; // first byte expected value

	int RxBufSz;
	char RxBuf[16]; //

	int nErr;
	volatile int TimedOut;
	uint32_t LastRcvGood; // tick last full msg received
	uint32_t LastRxTick; // last char received tick for soft time out handling
	uint32_t BadRx;
	UART_HandleTypeDef *huart;
	TIM_HandleTypeDef  *htim;
	void  (*Process)(struct UartRcv_t *Rx); //call when burst data received
};

// led set ticking in idle with low/high leelv state  fm light status => don' give info on live rx+f/w status
// Led set by process and then  toggle once by idle after short time is better
//   led blink until rx is ok ,if not bug f/W stuck ?
//   "static" led level high/low is the light sniffed level
int LedTogleTick=5; // 1/33 of edk repeat period
uint32_t LedSetTick; //we may use last rx good too
int ToggleLed=0;
volatile int light_upd; // set when reception done active light value updated (reset once done)
int LightOn=0; // set by process to sniffed light  value
int LightOn_p=1; // set by process to sniffed light  value
volatile int OnLvl=512;
int OnLvl_p=0;


void DbgIO(int set){
	if( set ==0 || set ==1 )
		HAL_GPIO_WritePin(DBG_IO_GPIO_Port, DBG_IO_Pin, set);
	else
		HAL_GPIO_TogglePin(DBG_IO_GPIO_Port, DBG_IO_Pin);
}


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
		sum = SumSerie((uint8_t*) Rx->RxBuf, Rx->RxMax-1);
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
		light_upd=1;
	}else {
		// shit rx 1 byte ?
		Rx->BadRx++;
	}
}

int MotRcvCnt=0;
char MotInfo[64];
int MotSpd;
int MotTorque;
uint8_t MotStatus;
//from https://github.com/PetteriAimonen/STM32_Trace_Example/blob/master/trace_example.c
void ITM_Print(int port, const char *p)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port)))
    {
        while (*p)
        {
            while (ITM->PORT[port].u32 == 0);
            ITM->PORT[port].u8 = *p++;
        }
    }
}

void configure_tracing()
{
    /* STM32 specific configuration to enable the TRACESWO IO pin */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR |= (2 << 24); // Disable JTAG to release TRACESWO
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins

    if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
    {
        // Some (all?) STM32s don't allow writes to DBGMCU register until
        // C_DEBUGEN in CoreDebug->DHCSR is set. This cannot be set by the
        // CPU itself, so in practice you need to connect to the CPU with
        // a debugger once before resetting it.
        return;
    }

    /* Configure Trace Port Interface Unit */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable access to registers
    TPI->ACPR = 0; // Trace clock = HCLK/(x+1) = 8MHz = UART 's baudrate
                   // The HCLK of F105 is 8MHz so x is 0, and the F103 is 72MHz so x is 8
    TPI->SPPR = 2; // Pin protocol = NRZ/USART
    TPI->FFCR = 0x102; // TPIU packet framing enabled when bit 2 is set.
                       // You can use 0x100 if you only need DWT/ITM and not ETM.

    /* Configure PC sampling and exception trace  */
    DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos) // Prescaler for PC sampling
                                           // 0 = x64, 1 = x1024
              | (0 << DWT_CTRL_POSTPRESET_Pos) // Postscaler for PC sampling
                                                // Divider = value + 1
              | (1 << DWT_CTRL_PCSAMPLENA_Pos) // Enable PC sampling
              | (2 << DWT_CTRL_SYNCTAP_Pos)    // Sync packet interval
                                               // 0 = Off, 1 = Every 2^23 cycles,
                                               // 2 = Every 2^25, 3 = Every 2^27
              | (1 << DWT_CTRL_EXCTRCENA_Pos)  // Enable exception trace
              | (1 << DWT_CTRL_CYCCNTENA_Pos); // Enable cycle counter

    /* Configure instrumentation trace macroblock */
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = (1 << ITM_TCR_TraceBusID_Pos) // Trace bus ID for TPIU
             | (1 << ITM_TCR_DWTENA_Pos) // Enable events from DWT
             | (1 << ITM_TCR_SYNCENA_Pos) // Enable sync packets
             | (1 << ITM_TCR_ITMENA_Pos); // Main enable for ITM
    ITM->TER = 0xFFFFFFFF; // Enable all stimulus ports

    /* Configure embedded trace macroblock */

}


void  MotProcess(struct UartRcv_t *Rx){
	if( RxValidate(Rx)==0 ){
		Rx->LastRcvGood = HAL_GetTick();

		MotStatus=Rx->RxBuf[2];
		if(Rx->RxBuf[4]> Rx->RxBuf[3])
			MotTorque=Rx->RxBuf[4]-Rx->RxBuf[3]; //only if 4 > 3 or motot on else vive negative number or status say mot on
		else
			MotTorque=0;
		MotSpd=Rx->RxBuf[7]+((uint32_t)Rx->RxBuf[7]<<8); // max 0x0707 / 1799 when stoped of very slow
		//once every sec trace
		if( MotRcvCnt++ > 15){
			sprintf(MotInfo,"St %02X S %d C %d",MotStatus, MotSpd, MotTorque );
			MotDbg(MotInfo);
			//ITM_Print(0, MotInfo);
			MotRcvCnt=0;
		}
	}else {
		// shit rx 1 byte ?
		Rx->BadRx++;
	}
}

struct UartRcv_t   EdkRx = {
		.RxMax = 7, // edk sent 7 byte per packet
		.HdrByte = 0x59,
		.RxBufSz = sizeof(EdkRx.RxBuf),
		.huart = &huart1,
		// PA10     ------> USART1_RX alternate PB7 (PA10 usb uart via CH340)
		// PA9     ------> USART1_TX alternate PB6  (PA9  usb uart via CH340)
		// Mini103 USART1 PA10/PA9 connect to usb via CH340 we coul yet use it on alt mapping PB7/6  but losing capability to use serial/usb
		//         has one more usart3 vs F103c6 black/blue pill than can map the edk whille keeping usb on usrt1
		.htim = &htim2,
		.Process = EdkProcess,
};
struct UartRcv_t   MotRx = {
		.RxMax = 9, // tsdz sent 8 byte per packet
		.HdrByte = 0x43,
		.RxBufSz = sizeof(MotRx.RxBuf),
		.huart = &huart2,
		// PA3     ------> USART2_RX
	    // PA2     ------> USART2_TX
		.htim = &htim3,
		.Process = MotProcess,
};

void KickRx(struct UartRcv_t *Rx){
	Rx->nRx=0;
#ifdef SOFT_TIMEOUT
	Rx->LastRxTick= HAL_GetTick();
#else
	__HAL_TIM_DISABLE(Rx->htim);
#endif
	HAL_UART_Receive_IT(Rx->huart, (void*)Rx->RxBuf,1);
}

void Rcv_ReamTimeOut(struct UartRcv_t *Rx){
#ifdef SOFT_TIMEOUT
	Rx->LastRxTick = HAL_GetTick();
#else
	Rx->htim->Instance->CNT=0;
	__HAL_TIM_CLEAR_FLAG(Rx->htim, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(Rx->htim, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE(Rx->htim);
	DbgIO(-1);
#endif
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

	if( Rx->nRx < Rx->RxMax ) // when full packet received no need to use idle time for end of burst detection
		Rcv_ReamTimeOut(Rx);  // save cpu time and minimize race timer isr/idle rx check
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
#ifdef SOFT_TIMEOUT
	 Rx->LastRxTick = HAL_GetTick();
#endif
}



void LedCheck(){
	if( ToggleLed  ){
		if( HAL_GetTick() - LedSetTick > LedTogleTick ){
			ToggleLed=0;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
	}
}

void SetLight(){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LightOn); // led is on when writing 0 (connect from vcc to port)
	LedSetTick= HAL_GetTick();
	ToggleLed=-1;
	if( LightOn != LightOn_p ||OnLvl != OnLvl_p ){
		LightOn_p = LightOn;
		OnLvl_p = OnLvl;
		htim1.Instance->CCR1 = LightOn ? OnLvl : 0; //OnLvl 1 pulse low all over hight 1025 (arr+1) for full on
	}
}

// test sequence
static uint8_t Pas1_Ligh0[] ={ 0x59, 0x80, 0x00, 0x1A, 0x00, 0x3C, 0x2F }; //EDK sent PAS 1 light off
static uint8_t Pas2_Ligh1[] ={ 0x59, 0x41, 0x00, 0x1A, 0x00, 0x3C, 0xF0 }; //EDK sent PAS 12 light on
uint8_t *DbgTx= Pas2_Ligh1; //change during debug session  to one  above array or data to send ptr

/**
 * send actual DbgTx data array on Mot Tx (ext wired to Edk Rx ofr debu)
 * @param Cnt [in/out] byte count to send ,  cleared at output
 */
void DoDbgTx(int *Cnt){
	if( Cnt && MotRx.huart->gState ==  HAL_UART_STATE_READY ){
		HAL_UART_Transmit_DMA(MotRx.huart, DbgTx,*Cnt);
		*Cnt= 0;
	}
}

void MotDbg( char *str){
	int n;
	if ( EdkRx.huart->gState ==  HAL_UART_STATE_READY ){
		n=strlen(str);
		HAL_UART_Transmit_DMA(EdkRx.huart, (void*)str, n);
	}
}

extern uint8_t _eedata_start; /* Symbol defined in the linker script */
extern uint8_t _eedata_end;
struct EeData_t {
	uint8_t EeUSed_Res; //for free/use next management 0 deleted not use anymore 0xFF free any non 0xFF
	// user data below
	uint8_t FrontLevel;
	unsigned RearBlink:1;
	uint8_t RearPer;
};
void ee_check(){
	uint8_t *pEE = &_eedata_start;
	while( pEE < &_eedata_end){
		pEE+=sizeof(struct EeData_t);
	}
}

//  state     cond/event -> next {action}; * rep cond act ...
// rest state = high (pull up ) button short capacitor to gnd
//  Idle      :   lowEdge -> SHortWait {SetLigh(full) , TimeEdge=now} ;
//  ShortWait :  HighEdge -> Idle {LightOff, if time fm last short < tdbl -> dbl click ()} ;
//               TFromEdge > Long  -> LongWait { SetLigh(lvl) }
// LongWait   :  HighEdge ->Idle { Store new lvl if chg}

// short  from ligh off et on stop on release "short beam warn"
// short when on => nothing of set lvl max , short off befoe one "high beam warm"
// long turn on/off as time > no action on release
// very long  ? => enter  lvl setting level  increase level rotating in aprox  1 sec step and set new lev on release
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
	  volatile int RepDbg=0;
	  int TickNext=0;
	  volatile int trace_en=0; /// for dynamaic when dbg attach to enable trace
	  volatile int trace_test=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DbgTx= RepDbg ? Pas2_Ligh1 : Pas1_Ligh0; //Trick to avoid  data removed at link cos unused
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //configure_tracing();
  ee_check();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  SetLight(); // init prev
  KickRx(&EdkRx);
  KickRx(&MotRx);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	  //Check for 2 Rx time out ou Rx liit
	  for( i=0; i<2; i ++ ) { //dbgg only run the displauy rx
		 Rx = i==0 ? &EdkRx : &MotRx;

		 if ( Rx->nRx >=  Rx->RxMax ){
			 if(Rx->Process )
					Rx->Process(Rx);
			 RxReset(Rx);
		 }else {
#ifdef SOFT_TIMEOUT
			 Rx->TimedOut = HAL_GetTick()  - Rx->LastRxTick > SOFT_TIMEOUT;
#endif
			 if( Rx->TimedOut && Rx->nRx < Rx->RxMax ){ // repeat check minimize unlikely race effect
				 RxReset(Rx);
			 }
		 }
	  }
	  if( DbgTxCnt || RepDbg ){
		  if( RepDbg && HAL_GetTick() > TickNext ) {
			  DbgTxCnt=7;
			  RepDbg--;
			  TickNext = HAL_GetTick()+ 66; //66  ms 15Hz as lcd rate
			  DoDbgTx((void*)&DbgTxCnt);
		  }
	  }
	  LedCheck();
	  if( light_upd){
		  SetLight();
		  light_upd=0;
	  }
	  if( trace_en){
		  configure_tracing();
		  trace_en = 0;
	  }
	  if( trace_test){
		  sprintf(MotInfo, "Test %d", (int)HAL_GetTick());
		  ITM_Print(0,MotInfo);
		  trace_test=0;
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
