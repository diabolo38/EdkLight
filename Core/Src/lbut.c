/*
 * lbut.c
 *
 *  Created on: Feb 11, 2026
 *      Author: diabo
 */
#include "main.h"
#include <string.h>

__weak void LbutSetup(){
}

// on release
__weak void Lbut_EventCb(int){

}


#define inrange( x , min , max ) ( ((x)>= (min)) && ((x)<=(max)) )
#define MIN(a,b) ((a)<(b) ? (a):(b))

int LongPressMs=3000;
int ShortPressMs=350;


enum ButState_e {
	BStWaitPress=0,
	BStLongWait = 1,
	BStLongDone = 2 , // wait long done action now wait release or else
};

struct Lbut_t {
	uint8_t  Lvl; //Actiev level
	uint8_t  State; // state for long shot detect
	uint8_t  ShortCnt,ShortCnt_p;
	uint8_t  LastEvent; // get clear when read
	uint32_t TLastChg;
	uint32_t LastShortTrain;
	uint32_t LastShortTrain_p; //prev n -1 train
} Lbut;

//is reporte while button is till low/pressed
void OnLongPressLight(){
	//TODO
	Log("Long\n");
}

#define LbutDbg(...)  (void)0


void LbutInit(){
	memset(&Lbut, 0 , sizeof(Lbut));
	Lbut.Lvl=1;
}

void LbutTrainEnd(){
	LbutDbg("train %d at %d\n", Lbut.ShortCnt, Lbut.LastShortTrain);
	Lbut.ShortCnt_p = Lbut.ShortCnt;
	Lbut.LastShortTrain_p=  Lbut.LastShortTrain;
	Lbut.ShortCnt =0;


}

void LbutTrainKIll(){
	Lbut.ShortCnt_p=Lbut.ShortCnt=0;
}

int Lbut_GetLastEvent(){
	int ev = Lbut.LastEvent;
	Lbut.LastEvent= 0;
	return ev;
}
static void Lbut_DoEvent(int ev){
	Lbut.LastEvent=ev;
	Lbut_EventCb(ev);
}

int  task_Lbut(uint32_t now, int ButLvl){
	int prev=Lbut.Lvl;
	if (Lbut.Lvl != ButLvl) {
		if( ButLvl == 0 ) { //press
			Lbut.State = BStLongWait;
		}
		else {//Release
			 Lbut.State = BStWaitPress;
			 if( now -Lbut.TLastChg < ShortPressMs ){
				 //short pressed
				 Lbut.ShortCnt = MIN(10, Lbut.ShortCnt+1); // clip avodi 255 loop
				 Lbut.LastShortTrain=now;
				 Lbut_DoEvent(Lbut_EvShort);
			 }else {
				 if( now -Lbut.TLastChg > LongPressMs){
					 Lbut_DoEvent(Lbut_EvLong);
				 }
				 else {
					 Lbut_DoEvent(Lbut_EvLong);
				 }
				 LbutTrainKIll();
			 }
		}
		Lbut.TLastChg = now;
		Lbut.Lvl = ButLvl;
	}
	else {
		//no chg
		if( ButLvl ==  0 ){
			if( now -Lbut.TLastChg > LongPressMs  &&  Lbut.State == BStLongWait){
				OnLongPressLight(); //eray notice before release
				Lbut.State = BStLongDone; //Wai
			}
		}
	}
	if (ButLvl == 1) { // off
		if( now - Lbut.TLastChg  > ShortPressMs*2 && Lbut.ShortCnt ){
			// idle unpress from long time follow  train of short pulse
			//this train and last check enter setup
			if( Lbut.ShortCnt == 2 && Lbut.ShortCnt_p==3 && inrange(Lbut.LastShortTrain -Lbut.LastShortTrain_p, 500 , 4000)   ){
				//Setup mode
				LbutSetup();
			}
			LbutTrainEnd(); //short cnt willc ome 0  and we want chcek this any more before press
		}
	}
	return prev != ButLvl;
}


