#include "main.h"
#include "adc.h"
#include "run.h"
#include "config.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_iwdg.h"
#include "misc.h"
#include <math.h>
#include "serial.h"
#include "tripod.h"
digitalPin *m2Out1,*m2Out2,*m1Out1,*m1Out2;
static uint32_t tripodStartTime,tripodTotalTime,tripodInitTime;
static int tripodActionReq,tripodActionPoint;
static uint16_t firstpoint = 0;
static void tripodOperationTime(uint16_t totalX00mSec){
	tripodStartTime = 0;
	tripodTotalTime = totalX00mSec;
}

static int tripodStatusGet(){
	uint8_t m1o1,m1o2,m2o1,m2o2;
	m1o1 = digitalGet(m1Out1);m1o2 = digitalGet(m1Out2);
	m2o1 = digitalGet(m2Out1);m2o2 = digitalGet(m2Out2);
	return m1o1|m1o2<<8|m2o1<<16|m2o2<<24;
}

static void tripodOperationOpen(uint16_t arm){// 1 tripod1 2 tripod2 3 tripod1&2
	if(arm == TRIPOD_ARMBOTH){
		digitalHi(m1Out1);digitalLo(m1Out2);
		digitalHi(m2Out1);digitalLo(m2Out2);
	}else if(arm == TRIPOD_ARM1){
		digitalHi(m1Out1);digitalLo(m1Out2);
	}else{
		digitalHi(m2Out1);digitalLo(m2Out2);
	}
}
static void tripodOperationClose(uint16_t arm){
	if(arm == TRIPOD_ARMBOTH){
		digitalLo(m1Out1);digitalHi(m1Out2);
		digitalLo(m2Out1);digitalHi(m2Out2);
	}else if(arm == TRIPOD_ARM1){
		digitalLo(m1Out1);digitalHi(m1Out2);
	}else{
		digitalLo(m2Out1);digitalHi(m2Out2);
	}

}

static void tripodOperationStop(uint16_t arm){
	if(arm == TRIPOD_ARMBOTH){
		digitalLo(m1Out1);digitalLo(m1Out2);
		digitalLo(m2Out1);digitalLo(m2Out2);
	}else if(arm == TRIPOD_ARM1){
		digitalLo(m1Out1);digitalLo(m1Out2);
	}else{
		digitalLo(m2Out1);digitalLo(m2Out2);
	}
}

static void tripodStop(){
	if((tripodState== TRIPOD_STATE_CLOSING)||(tripodState == TRIPOD_STATE_OPENING)){
		tripodOperationStop(TRIPOD_ARMBOTH);
		ADC_SoftwareStartConvCmd(ADC1, DISABLE);
		if(tripodState== TRIPOD_STATE_CLOSING){
			tripodState = TRIPOD_STATE_CLOSED;
		}else if(tripodState == TRIPOD_STATE_OPENING){
			tripodState = TRIPOD_STATE_OPENED;
		}
	}
	sprintf(printBuf, "tripodStop,tripodState:%d\r\n",tripodState);
	serialPrint(printBuf);
}

static void tripodPwmAction(int action,uint16_t setpoint){//action opening,closeing
	if((action == TRIPOD_STATE_OPENING)||(action == TRIPOD_STATE_CLOSING)){
		tripodOperationTime(DEFAULT_TRIPOD_X00MSEC);
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		if(action == TRIPOD_STATE_CLOSING){
			tripodState = TRIPOD_STATE_CLOSING;
			tripodOperationClose(TRIPOD_ARMBOTH);
		}else if(action == TRIPOD_STATE_OPENING){
			tripodState = TRIPOD_STATE_OPENING;
			tripodOperationOpen(TRIPOD_ARMBOTH);
		}
		sprintf(printBuf, "tripodPwmAction setpoint:%d tripodState:%d\r\n",setpoint,tripodState);
		serialPrint(printBuf);
	}
}
static void tripodPwmActionRequest(int action,uint16_t setpoint){//action opening,closeing
	if(action == tripodActionReq)
		return;//already requested same action
	tripodInitTime =0;
	tripodActionReq = action;
	tripodActionPoint = setpoint;
	sprintf(printBuf, "tripodPwmActionRequest setpoint:%d action:%d tripodState:%d\r\n",setpoint,action,tripodState);
	serialPrint(printBuf);

}

//call by main
void tripodInit(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
    // GPIOA_5 button
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	m2Out2 = digitalInit(GPIOA, GPIO_Pin_6);	// GPIOA_6 M2_OUT2
	m2Out1 = digitalInit(GPIOA, GPIO_Pin_7);	// GPIOA_7 M2_OUT1
	m1Out2 = digitalInit(GPIOB, GPIO_Pin_0);	// GPIOB_0 M1_OUT2
	m1Out1 = digitalInit(GPIOB, GPIO_Pin_1);	// GPIOB_1 M1_OUT1
	tripodInitTime = 0;
	tripodActionReq = TRIPOD_STATE_INIT;
}

//call by adc when current over flow
//arm set TRIPOD_ARM1 or TRIPOD_ARM2
void tripodAdcAction(int arm){// 1 adcval1 2 adcval2
	tripodOperationStop(arm);
	if(tripodStatusGet()==0){
		ADC_SoftwareStartConvCmd(ADC1, DISABLE);
		if(tripodState== TRIPOD_STATE_CLOSING){
			tripodState = TRIPOD_STATE_CLOSED;
		}else if(tripodState == TRIPOD_STATE_OPENING){
			tripodState = TRIPOD_STATE_OPENED;
		}
	}
	sprintf(printBuf, "tripodAction,tripodState:%d\r\n",tripodState);
	serialPrint(printBuf);

}

//call by timer 
void tripodTimerAction(){
	if((tripodActionReq == TRIPOD_STATE_OPENING)||(tripodActionReq == TRIPOD_STATE_CLOSING)){
		if(tripodInitTime++ > 5){//500ms
			tripodPwmAction(tripodActionReq,tripodActionPoint);
			tripodActionReq = TRIPOD_STATE_INIT;//set init for done
		}
	}

	if((tripodState == TRIPOD_STATE_OPENING)||(tripodState == TRIPOD_STATE_CLOSING)){
		tripodStartTime++;
		if(tripodStartTime >= tripodTotalTime){
			sprintf(printBuf, "tripodCheckTimeout,tripodState:%d time out,ACTION\r\n",tripodState);
			serialPrint(printBuf);
			tripodStop();
		}
	}
}

//call by pwm,setpoint init closing ,than H-->L closing,L-->H opening
void pwmNewInput(uint16_t setpoint) {
//		sprintf(printBuf, "pwmNewInput setpoint:%d tripodState:%d\r\n",setpoint,tripodState);
//		serialPrint(printBuf);
	if(tripodState == TRIPOD_STATE_INIT){//init status
		if((firstpoint == 0)){
			firstpoint = setpoint;
			tripodPwmActionRequest(TRIPOD_STATE_CLOSING,setpoint);
		}
	}else if((tripodState == TRIPOD_STATE_OPENED)||(tripodState == TRIPOD_STATE_CLOSED)){//opened, closed status
		if(setpoint < 1200){
			if(tripodState == TRIPOD_STATE_OPENED ){
				tripodPwmActionRequest(TRIPOD_STATE_CLOSING,setpoint);
			}else if((tripodState == TRIPOD_STATE_CLOSED)&&(firstpoint > 1700)){// means pwm from H to L,so we can let arm action if pwm to H
				firstpoint = 0;
			}
		}else if(setpoint > 1700){
			if(tripodState == TRIPOD_STATE_CLOSED){
				if(firstpoint > 1700)
					return;
				tripodPwmActionRequest(TRIPOD_STATE_OPENING,setpoint);
			}
		}
	}else{//opening,closing status
		if((tripodState == TRIPOD_STATE_OPENING)&&(setpoint < 1200)){
			tripodStop();//stop first
			tripodPwmActionRequest(TRIPOD_STATE_CLOSING,setpoint);
		}
		if((tripodState == TRIPOD_STATE_CLOSING)&&(setpoint > 1700)
			&&(firstpoint == 0)){//means not first closing,so we will take action
			tripodStop();//stop first
			tripodPwmActionRequest(TRIPOD_STATE_OPENING,setpoint);
		}
	}
}

