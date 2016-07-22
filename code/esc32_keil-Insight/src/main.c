/*
    This file is part of AutoQuad ESC32.

    AutoQuad ESC32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad ESC32 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad ESC32.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012  Bill Nesbitt
*/

#include "main.h"
#include "rcc.h"
#include "config.h"
#include "serial.h"
#include "pwm.h"
#include "adc.h"
#include "run.h"
#include "cli.h"
#include "ow.h"
#include "tripod.h"

digitalPin *errorLed, *statusLed;
#ifdef ESC_DEBUG
digitalPin *tp;
#endif

volatile uint32_t minCycles, idleCounter,      totalCycles;
//                ��С����   mainѭ�����д���  �ܹ�����     �⼸�����������������ʱ��ٷֱ�(Ҳ����û���ж�,����ѭ����ʲô����Ҳ����)
volatile uint8_t state,    inputMode;
//               ����״̬  ����ģʽ(���Ʒ�ʽ ���� iic can pwm 1wire)
volatile uint8_t tripodState;//0 init 1 opening 2 opened 3 closing 4 closed
char printBuf[64];
__asm void nop(void);

int main(void) 
{
	tripodState = TRIPOD_STATE_INIT;
	rccInit();     //����CPUĬ�ϵ�һЩʱ�ӵ�

	statusLed = digitalInit(GPIO_STATUS_LED_PORT, GPIO_STATUS_LED_PIN);
	errorLed = digitalInit(GPIO_ERROR_LED_PORT, GPIO_ERROR_LED_PIN);
#ifdef ESC_DEBUG
	tp = digitalInit(GPIO_TP_PORT, GPIO_TP_PIN);
	digitalLo(tp);
#endif

	//timerInit();   //timer2�ڲ�������
	configInit();  //����Ĭ�ϵ�����
	serialInit();  //���ڳ�ʼ��
	adcInit();     //ADC��������
	//fetInit();
	tripodInit();

	runInit();
	cliInit();     //�򴮿ڷ���һЩ�汾��Ϣ,�������ݴ���Ҳ��������ʵ��
	//owInit();      //��ʼ��1wireЭ��

	//runDisarm(REASON_STARTUP);
	inputMode = ESC_INPUT_PWM;

	pwmInit(); //PWM IN��ʼ��

	//LED��ʼ��
	digitalHi(statusLed);
	digitalHi(errorLed);
	tripodFirstAction();
	// self calibrating idle timer loop
	{
		volatile unsigned long cycles;
		volatile unsigned int *DWT_CYCCNT  = (volatile unsigned int *)0xE0001004;//��ǰPC���������ڼ����Ĵ���
		volatile unsigned int *DWT_CONTROL = (volatile unsigned int *)0xE0001000;
		volatile unsigned int *SCB_DEMCR   = (volatile unsigned int *)0xE000EDFC;

		*SCB_DEMCR = *SCB_DEMCR | 0x01000000;
		*DWT_CONTROL = *DWT_CONTROL | 1;	// enable the counter

		minCycles = 0xffff;

		while (1) 
		{
			idleCounter++;

			//NOPS_4;
			nop();

			cycles = *DWT_CYCCNT;
			*DWT_CYCCNT = 0;		    // reset the counter

			//�����������ʱ��ٷֱ�
			// record shortest number of instructions for loop
			totalCycles += cycles;
			if (cycles < minCycles)
				minCycles = cycles;
		}
	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1) {
    NOP;
  }
}
#endif

void HardFault_Handler(void) {
    //FET_PANIC;
    while (1)
	;
}

void MemManage_Handler(void) {
    //FET_PANIC;
    while (1)
	;
}

void BusFault_Handler(void) {
    //FET_PANIC;
    while (1)
	;
}

void UsageFault_Handler(void) {
    //FET_PANIC;
    while (1)
	;
}

void reset_wait(void) {
    //FET_PANIC;
    while (1)
	;
}

#if 0
//__disable_irq()
__asm void CPSID_I(void)
{
	PUSH {lr}
	cpsid i
	POP {PC}
}

//__disable_fault_irq()
__asm void CPSID_F(void)
{
	PUSH {lr}
	cpsid f
	POP {PC}
}

//__enable_irq()
__asm void CPSIE_I(void)
{
	PUSH {lr}
	cpsie i 
	POP {PC}
}
#endif

__asm void nop(void)
{
	PUSH {lr}
	nop
	nop
	nop
	nop
	POP {PC}
}
