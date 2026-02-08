#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"

//must be mltiple of 2 at leaste as we can only write by 2 byte in flash

_Static_assert(sizeof(struct EeData_t) == 8 );

//Recover non ee data value used
const struct EeData_t EeDefault = {
		.FrontLevel = 66,
		.LongPressSec = 5,
};

struct LogEntry_t {
	uint8_t sz; // 0x00 reserved for back to start
	char msg[];
};
#define LOG_BUF_SZ	2048
char LogMem[LOG_BUF_SZ];
#define LogEnd  (LogMem+sizeof(LogMem))
struct LogEntry_t *LogTop=(struct LogEntry_t *)LogMem; //next to fill
struct LogEntry_t *LogOut=(struct LogEntry_t *)LogMem; // head of  be sent via uart
struct LogDaata_t {
	uint32_t InCnt;
	volatile uint32_t OutCnt; //Settign int cnt  =0 may prevent any self continue from complete
}LogData;
/*
 top & out  ptr with wrap and overtake roll over is hell to handle track of entry  enyterign and leaving make it simpler
yet this do not prevent we full  and make a mess
*/

char _LogPrBuf[256]; //interlediate format before locked cpy to log LogMem  shall be less than 256 -4 as

UART_HandleTypeDef *huart=& huart1;
void LogCheckRewind(){
	//irq must eb disable or use in context uart irq can't fire
	if( LogData.OutCnt >= LogData.InCnt ){
		LogOut= LogTop= (struct LogEntry_t *)LogMem;
		LogData.OutCnt = LogData.InCnt =0;
	}
}
void LogTxComplete(){
	if( LogData.OutCnt< LogData.InCnt ) { //over safe if come from false irq
		void * next;
		next = LogOut->msg+LogOut->sz;
		LogData.OutCnt+= next-(void*)LogOut;
		LogCheckRewind();
		if(  LogData.OutCnt< LogData.InCnt ){
			LogOut=next;
			HAL_UART_Transmit_DMA(huart, (void*)LogOut->msg, LogOut->sz);
		}
		//Else we're done
	}
}

void KickLog(struct LogEntry_t *e){
	int n;
	void *next;
	uint32_t flags = __get_PRIMASK();
	__disable_irq();
	LogCheckRewind();
	//we may check in == out now and reste
	next=LogTop->msg+e->sz;
	n = next- (void*)LogTop;
	if( next  > (void*)LogEnd ){
		LogTop->sz=0; //Wrap before end mark
		LogTop=(struct LogEntry_t *)LogMem;
		next=LogTop->msg+e->sz; //changed
	}
	memcpy(LogTop,e, n);
	LogData.InCnt+=n;
	LogTop=(struct LogEntry_t *)next;

	if(huart->gState ==  HAL_UART_STATE_READY ){
		if( LogOut->sz == 0 ){
			LogOut=(struct LogEntry_t *)LogMem; //  end  split send avoid
		}
		HAL_UART_Transmit_DMA(huart, (void*)LogOut->msg, LogOut->sz);
	}
	else {
		//will be picked up on  loop or complete
	}
	__set_PRIMASK(flags);
}

void Log(const char * fmt, ...){
	va_list ap;
	struct LogEntry_t *e =(struct LogEntry_t *)_LogPrBuf;
	int n,waited;
	uint32_t t0;
	va_start(ap,fmt);

	e->sz = vsnprintf(e->msg, 254-sizeof(*e), fmt, ap);
	if( e->sz == 0 ){
		Stats(EmptyLog++);
		return ;
	}
	n = offsetof(struct LogEntry_t, msg[e->sz]);

	t0 = HAL_GetTick();
	waited=0;
	while( LogData.InCnt + n  > sizeof(LogMem)-256 ){
		//We must wait if ther is not at least  full message left after puting this one this shall cover case of wrap hardly countable
		if(  HAL_GetTick()-t0 > 5){
			Stats(DropLog++);
			goto done ;
		}
		__WFI();
		waited++;
	}
	if( waited )
		Stats(WaitLog++);
	KickLog(e);
done:
	va_end(ap);
}



/* Symbols defined in the linker script to mark the non volatile "eeprom" flash area start and end
 * we'll use this to save settings such as light intensity level, brake with no speed time max  */
extern struct EeData_t  _eedata_start;
extern struct EeData_t  _eedata_end;

/**
 * flashs may  be lock after so write ca be done w/o unlock again
 */
void EeFullErase(){
	int rc;
	FLASH_EraseInitTypeDef Erase = {
			.TypeErase = FLASH_TYPEERASE_PAGES,
			.Banks= FLASH_BANK_1,
			.NbPages = ((void*)&_eedata_end - (void*)&_eedata_start )/FLASH_PAGE_SIZE,
			.PageAddress = (uint32_t)&_eedata_start,
	};

	rc = HAL_FLASH_Unlock();
	if( rc == 0 ){
		uint32_t Err;
		rc = HAL_FLASHEx_Erase(&Erase, &Err);
		HAL_FLASH_Lock();
	}
}
// ee start end in address sense , bottom of flash area but end of the nvm
// using stack like top to bottom is prefered vs bot top . It enable extension/grow on f/W upgrade
//  (bot top don't enbale it) yet shrink space is not direct. The eedata if to be shrink nned to be move on top
// before flashing the new f/w.
//if not and ee data content is robust the trashed  nvm entry (now code) shall be detect bad and f/w will restart
//clean default , basiclay ok but previous stored settings lost.

/**
 *
 * @return NULL if none required erase
 */
struct EeData_t * EeFirstFree(){
	struct EeData_t *pEE;
	pEE = &_eedata_end-1;//topmost last/first entry
	while( pEE > &_eedata_start  && pEE->EeCtrl != 0xFF  ){
		pEE--;
	}
	// we could check the full entry if to be oversafeQ
	if( pEE->EeCtrl !=  0xFF ){
		//reach end w/o free entry need to full erase the area and start again
		pEE=NULL;
	}
	return pEE;
}

const struct EeData_t * EeActiveEntry(){
	struct EeData_t *pEE;
	pEE = &_eedata_end-1;//topmost last/first entry
	while( pEE > &_eedata_start  && pEE->EeCtrl != EE_CTRL_VALID  && pEE->EeCtrl!=0xFF  ){
		pEE--;
	}
	// we could check the full entry if to be oversafeQ
	if( pEE->EeCtrl !=  EE_CTRL_VALID ){
		//reach end w/o free entry need to full erase the area and start again
		pEE=NULL;
	}
	return pEE;
}
/**
 * @return true if valid
 */
int EeIsValidEntry(const struct EeData_t *pEe){
	return (SumSerie((void*)pEe, sizeof(*pEe)-1)&0xFF) == pEe->Sum;
}
/**
 *
 * @param pSet [in] all value [out]  ctrl+checksum udpated
 * @return
 */
int EeSetEntry(struct EeData_t *pSet){
	struct EeData_t *pFree;
	struct EeData_t *pPrev=NULL;
	uint16_t  *psw, *pdw; //Src dest word
	int rc,left;

	pSet->EeCtrl = 0xA5;
	pSet->Sum=SumSerie((void*)pSet, sizeof(*pSet)-1);

	pFree = EeFirstFree();
	if( pFree == NULL ){
		Log("Eerase all\n");
		EeFullErase();
		pFree=(&_eedata_end)-1;
	}
	if( pFree+1 < &_eedata_end ){
		pPrev=pFree+1;
	}
	rc = HAL_FLASH_Unlock();
	for( left = sizeof(*pSet), psw=(void*)pSet, pdw=(void*)pFree;
			left >0;
			left-=2, psw++, pdw++ ){

		rc = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)pdw, *psw);
		if( rc ){
			Log("EeW %d\n", left);
			goto done_l;
		}
	}
	//invbaldiate orevious now we set new (woudl be find last anyway in seek first using free)
	if( pPrev ){
		rc = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)pPrev, 0);
		Log("ERel %p\n", pPrev);
		//what to do ? can it happen
	}
done_l:
	HAL_FLASH_Lock();
	return rc;

}
//if run twice a full erase shall occur
void ee_test(){
	const struct EeData_t * Activ=NULL;
	struct EeData_t Ref;
	uint8_t *b=(void*)&Ref;
	int i,rc, ok;
	//setup ref data with adress pattern
	for( i=1; b<&Ref.Sum; b++,i++ )
		*b=i;

	Activ = EeFirstFree();
	Log("ee test org free %p\n",Activ);

	i=0;
	do{
		rc = EeSetEntry(&Ref);
		Activ=EeActiveEntry();
		ok=memcmp(Activ, &Ref, sizeof(Ref));
		Log("Set %3d rc %d cmp %d\n",i,rc,ok);

		i++;
		Ref.FrontLevel=i; //some chg in pattern
	}while( EeFirstFree()!=NULL );

}

void ee_check(){
	volatile int x=0;
	if( x )
		ee_test();
}
