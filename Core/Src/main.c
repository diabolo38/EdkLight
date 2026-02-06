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
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ARRAY_SIZE(x)  (sizeof(x)/sizeof(x[0]))
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
	const char *Name; //human readable nick naleuse in log etc ..
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
volatile int light_upd; // flags set when reception done to activate  light value updated task (reset once done)
int LightOn=0; // set by process to sniffed light  value
int LightOn_p=1; // set by process to sniffed light  value
volatile int OnLvl=512;
int OnLvl_p=0;
volatile int BrakeActive=0;
int FlightPwm = 0; //may init by ee or chg by button combo

void DbgIO(int set){
	if( set ==0 || set ==1 )
		HAL_GPIO_WritePin(DBG_IO_GPIO_Port, DBG_IO_Pin, set);
	else
		HAL_GPIO_TogglePin(DBG_IO_GPIO_Port, DBG_IO_Pin);
}

void DbgIO2(int set){
	if( set ==0 || set ==1 )
		HAL_GPIO_WritePin(DBG2_GPIO_Port, DBG2_Pin, set);
	else
		HAL_GPIO_TogglePin(DBG2_GPIO_Port, DBG2_Pin);
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
		LightOn =Rx->RxBuf[1]&0x01; // bit 0 of control motor flags
		light_upd=1;
	}else {
		Rx->BadRx++;// shit rx 1 byte ?
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

/**USART1 GPIO Configuration
   PA9     ------> USART1_TX
   PA10     ------> USART1_RX to usb
   */

struct UartRcv_t   EdkRx = {
		.Name ="Edk",
		.RxMax = 7, // edk sent 7 byte per packet
		.HdrByte = 0x59,
		.RxBufSz = sizeof(EdkRx.RxBuf),
		.huart = &huart2,
		/**USART2 GPIO Configuration
		PA2     ------> USART2_TX
		PA3     ------> USART2_RX
		*/
		.htim = &htim2,
		.Process = EdkProcess,
};

struct UartRcv_t   MotRx = {
		.Name ="Mot",
		.RxMax = 9, // tsdz2b sent 8 byte per packet
		.HdrByte = 0x43,
		.RxBufSz = sizeof(MotRx.RxBuf),
		.huart = &huart3,
		/**USART3 GPIO Configuration
		  PB10     ------> USART3_TX
		  PB11     ------> USART3_RX
		  */
		.Process = MotProcess,
};


struct UartRcv_t UsbRx = {
	.Name ="Usb",
	.RxMax =  sizeof(UsbRx.RxBuf) -1 , // keep one to add 0 en f=to use string function
	.RxBufSz = sizeof(UsbRx.RxBuf),
	.huart = &huart1,
	// no process not header
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
	if( RxRx->htim != NULL ){
		Rx->htim->Instance->CNT=0;
		__HAL_TIM_CLEAR_FLAG(Rx->htim, TIM_FLAG_UPDATE);
		__HAL_TIM_ENABLE_IT(Rx->htim, TIM_IT_UPDATE);
		__HAL_TIM_ENABLE(Rx->htim);
		DbgIO(-1);
	}else
		Rx->LastRxTick = HAL_GetTick();
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	struct UartRcv_t  *Rx = huart == EdkRx.huart ? &EdkRx : huart == MotRx.huart  ? &MotRx : &UsbRx;
	if(Rx->nRx < Rx->RxBufSz ){
		Rx->RxBuf[Rx->nRx]=Rx->RxByte;
		Rx->nRx++;
	}
	else {
		//how can ? idle is dead not picking ?
	}
	HAL_UART_Receive_IT(Rx->huart, (void*)&Rx->RxByte,1);

	if( Rx->nRx < Rx->RxMax ) // when full packet received no need to use idle time for end of burst detection
		Rcv_ReamTimeOut(Rx);  // save cpu time and minimize race timer isr/idle rx check
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	struct UartRcv_t  *Rx = huart == EdkRx.huart ? &EdkRx : huart == MotRx.huart  ? &MotRx : &UsbRx;
	Rx->nErr++;
	//Rx->nRx = 0;
	//We could continue on the end crc +  timeout with not enough data  will says if good anyway
	//  if error is due to some glitch extra rcv , it can  be good on the end
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

struct Lbut_t {
	uint8_t  State;
	uint32_t TLastChg;
} Lbut;

void SetLight(){
	uint8_t ButState =  HAL_GPIO_ReadPin(BUT_LIGHT_GPIO_Port, BUT_LIGHT_Pin);

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LightOn); // led is on when writing 0 (connect from vcc to port)
	LedSetTick= HAL_GetTick();
	ToggleLed=-1;

	if( LightOn != LightOn_p ||OnLvl != OnLvl_p ||ButState != Lbut.State ){
		int IsOn, OnLv;
		LightOn_p = LightOn;
		OnLvl_p = OnLvl;
		IsOn = LightOn | ButState;
		if( FlightPwm ){
			OnLv = ButState ==0 ? OnLvl : 1025; //full on when but pressed
			htim1.Instance->CCR1 = IsOn ? OnLv  : 0; //OnLvl 1 pulse low all over hight 1025 (arr+1) for full on
		}
		else {
			HAL_GPIO_WritePin(FLIGHT_PWM_GPIO_Port,FLIGHT_PWM_Pin, IsOn);
		}
		HAL_GPIO_WritePin(RLIGHT_ON_GPIO_Port, RLIGHT_ON_Pin, LightOn); // real ligh no pww not chg on flash by button
	}
	HAL_GPIO_WritePin(RLIGHT_ON_GPIO_Port, RLIGHT_ON_Pin, LightOn );
	if( Lbut.State != ButState ){
		Lbut.TLastChg = HAL_GetTick();
		Lbut.State = ButState;
	}
}

void BrakeCheck(){
	int BrakeIn = HAL_GPIO_ReadPin(BRAKE_IN_GPIO_Port, BRAKE_IN_Pin);
	BrakeActive = BrakeIn;
	HAL_GPIO_WritePin(BRAKE_ON_GPIO_Port, BRAKE_ON_Pin, BrakeActive);
}

void HornCheck(){
	int HornIn = HAL_GPIO_ReadPin(BUT_HORN_GPIO_Port, BUT_HORN_Pin);
	HAL_GPIO_WritePin(HORN_ON_GPIO_Port, HORN_ON_Pin, !HornIn); // io  is pull-up with button short to gnd
}

// test sequence we send to ourself
static uint8_t Pas1_Ligh0[] ={ 0x59, 0x80, 0x00, 0x1A, 0x00, 0x3C, 0x2F }; //EDK sent PAS 1 light off
static uint8_t Pas2_Ligh1[] ={ 0x59, 0x41, 0x00, 0x1A, 0x00, 0x3C, 0xF0 }; //EDK sent PAS 12 light on
uint8_t *DbgTx= Pas2_Ligh1; //change during debug session  to one  above array or data to send ptr

/**
 * send actual DbgTx data array on Mot Tx (externaly  wired to Edk Rx for dev and debug)
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
/* Symbols defined in the linker script to mark the non volatile "eeprom" flash area start and end
 * we'll use this to save settings such as light intensity level, brake with no speed time max  */
extern uint8_t _eedata_start;
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

void InitLight(){
	// tim1 ch2n
	FlightPwm=1; //get from ee
	htim1.Instance->CCR1 = 0; //Set off default before first  update done
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

struct UCmd_t {
	const char * cmd ;
	const char * help;
	int len;
	void (*validate)(struct UCmd_t *, char*);
};

int CmdL;
#define IsBlank(x) (x==' ' || x == '\t' || x == '\r' || x == '\n')
void ValidateLight(struct UCmd_t *cmd, char *str){
	int v,n;
	if( !IsBlank(str[cmd->len]) ){
		//we need white space and then only value
		return;
	}
	n = sscanf(str+cmd->len ,"%d", &v);
	if( n < 1)
		return ;

	if( v<0 ){
		v =0;
	}
	else if ( v>= 100 ){
		v= 100;
	}
	OnLvl = v*1024/100;
}

void CmdStat(struct UCmd_t *cmd, char *str);


struct UCmd_t Cmds[]= {
		/*warning parser is over simple not too command can start the same or if they do
		 * the longest one must be place first so longest match/cmp  is done first
		 *
		 */
		{ .cmd = "lvl", .validate = ValidateLight , .help="% d 0-100 %% pwm"},
		{ .cmd = "stat", .validate = CmdStat , .help="do log status"},
};

struct UsbTask_t {
	int ToSend; // shall eb set when we have stuff waiting to eb sent
	int LastNRx; //use for usb parser to detect  new char arrival
	uint32_t StatusErr;

	char LogBuf[1024];
}UsbTh;

int UsbIsDone(){
	//We sent echo w/o waiting writing in dr but we too send via dma what affect state
	return (UsbRx.huart->Instance->SR & UART_FLAG_TXE ) &&(	UsbRx.huart->gState ==  HAL_UART_STATE_READY ) ;
}

void DoHelp(){
	UsbTh.ToSend = 1;
	UsbRx.huart->Instance->DR = '?';
}

struct UartRcv_t *RxLut[] = {
		&EdkRx,
		&MotRx,
		&UsbRx
};

int UsbRxWaitTxrdy(int MsMax){
	uint32_t t0 = HAL_GetTick();
	do {
		if ( UsbRx.huart->gState ==  HAL_UART_STATE_READY ){
				return 0;
		}
	}while(HAL_GetTick() - t0 < MsMax);
	return -1;
}

void DoStatus(){
	char *buf=UsbTh.LogBuf;
	int i,n;
	struct UartRcv_t *Rx;
	n= sprintf(buf,"Now %u\n", (unsigned)HAL_GetTick());
	for( i=0 ; i < ARRAY_SIZE(RxLut); i++ ){
		Rx=RxLut[i];
		n += sprintf(buf+n, "%s\nBadRX %u Err %u\n", Rx->Name, (unsigned)Rx->BadRx, (unsigned)Rx->nErr);
		n += sprintf(buf+n, "Lg %u Lrx %u\n", (unsigned)Rx->LastRcvGood, (unsigned)Rx->LastRxTick);
	}
	n+= sprintf(buf+n,"Th stat err %u\n", (unsigned)UsbTh.StatusErr);
	if( UsbRxWaitTxrdy(5) == 0 ){
		UsbTh.ToSend =n;
		HAL_UART_Transmit_DMA(UsbRx.huart, (void*)buf, n);
	}else {
		//too bad simply keep track
		UsbTh.StatusErr++;
	}
}
void CmdStat(struct UCmd_t *cmd, char *str){
	DoStatus();
}

#define IsBlank(x) (x==' ' || x == '\t' || x == '\r' || x == '\n')
void UCmdCheck(){
	int i,c, match,n;
	char cmd[32];
	if( UsbTh.ToSend ){
		//check sending thing waiting for prev done
		//ix TXE and state rdy we can clear
		if( UsbIsDone() )
			UsbTh.ToSend=0;
	}

	if( UsbRx.nRx <=  UsbTh.LastNRx )
		return;
	if( UsbRx.nRx >=  UsbRx.RxMax ){
		//shirt ? trash
	}
	//local ehco if no data to send direct handlign else put to what to send or discard ;
	if( UsbTh.ToSend  == 0 ){
		UsbTh.ToSend = 1;
		UsbRx.huart->Instance->DR = UsbRx.RxBuf[UsbTh.LastNRx]; //send echo no wait don't care error etc
	}
	//Trim the input discard any blank and non alpha likely junk
	i=0;
	while( ( !isalpha( ( (int)UsbRx.RxBuf[i]) ) || IsBlank(UsbRx.RxBuf[i] ) ) && i < UsbRx.nRx ){
		i++;
	}
	// trim and update ptr must be done int clr to avoid race with uuart handler
	if( i != 0 ){
		__disable_irq();
		if( UsbRx.nRx  -i > 1 ){
			//else mean nothing left no need to move
			memmove(UsbRx.RxBuf, UsbRx.RxBuf+i, UsbRx.nRx-i );
		}
		n= (UsbRx.nRx-=i); //may be 0 now
		__enable_irq();
	}
	else
		n = UsbTh.LastNRx+1; //use all no trime

	// check if we have \n if not not evben check stop using nRx racy but n from now
	if( n <1 )
		goto done;
	memcpy(cmd,UsbRx.RxBuf,  n); //content can change in irq handle best cpy avosi any issue
	cmd[n]=0;
	match=0;
	if( cmd[n-1]== '\r'  || cmd[n-1]== '\n' ){
		for( c=0; c<ARRAY_SIZE(Cmds );c++){
			if( Cmds[c].len <= UsbRx.nRx ){ // we have at least cmd len data
				if( strncmp(cmd, Cmds[c].cmd, Cmds[c].len ) == 0 ){
					match = 1;
					break;
				}
			}
		}

		if( match ){
			Cmds[c].validate(Cmds+c, cmd);
			UsbRx.nRx = n = 0; //fixme Remove the cmd or reset all ?
		}
		else
		{
			UsbRx.BadRx++; // reset or not
			UsbRx.nRx = n = 0; // full no command reset
			DoHelp();
		}
	}
	else{
		//todod check we're full trash UsbRx.nRx = n =
	}
done:
	UsbTh.LastNRx=n;
}

void UCmdInit(){
	int i;
	for( i=0; i<ARRAY_SIZE(Cmds );i++){
		Cmds[i].len = strlen(Cmds[i].cmd);
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //configure_tracing();

  ee_check();
  UCmdInit();
  InitLight();
  SetLight(); // init prev
  KickRx(&EdkRx);
  KickRx(&MotRx);
  KickRx(&UsbRx);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	  //Check for 2 Rx time out out Rx limit
	  for( i=0; i<2; i ++ ) { //dbgg only run the displauy rx
		 Rx = i==0 ? &EdkRx : &MotRx;

		 if ( Rx->nRx >=  Rx->RxMax ){
			 if(Rx->Process )
					Rx->Process(Rx);
			 RxReset(Rx);
			 //When process/validate is bad shift data out rather than reset any better ?
			 // this can cope with case we had data from prev or junk + current packet still  entering
			 // this shall not occur with no rx time out detection any partial packet is droped.

		 }else {
#ifdef SOFT_TIMEOUT
			 if( Rx->nRx > 0){ //no time handlign until rcv anything
				 Rx->TimedOut = HAL_GetTick()  - Rx->LastRxTick > SOFT_TIMEOUT;
			 }
#endif
			 if( Rx->TimedOut && Rx->nRx < Rx->RxMax ){ // repeat check minimize unlikely race effect
				 RxReset(Rx);
			 }
		 }
	  }
	  //todo handle usb rx
	  UCmdCheck();
	  if( DbgTxCnt || RepDbg ){
		  // do debug tx
		  if( RepDbg && HAL_GetTick() > TickNext ) {
			  DbgTxCnt=7;
			  RepDbg--;
			  TickNext = HAL_GetTick()+ 66; //66  ms 15Hz as lcd rate
			  DoDbgTx((void*)&DbgTxCnt);
		  }
	  }
	  BrakeCheck();
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
