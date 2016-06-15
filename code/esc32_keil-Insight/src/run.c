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

    Copyright ? 2011, 2012, 2013  Bill Nesbitt
*/

#include "run.h"
#include "main.h"
#include "adc.h"
#include "pwm.h"
#include "cli.h"
#include "binary.h"
#include "config.h"
#include "misc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_dbgmcu.h"
#include <math.h>
#include "tripod.h"

uint32_t runMilis;   //systick中断中自加.没有什么控制用途
static uint32_t oldIdleCounter;  //上次main函数中,死循环次数.
float idlePercent;   //空闲时间百分比(在main循环里,什么事情也不做.main死循环运行的时间)
float avgAmps, maxAmps; //平均电流, 最大电流
float avgVolts;      //当前ADC采集转换后的电池电压(也就是12v)

float rpm;           //当前转速(1分钟多少转) 测量值 在runRpm函数中计算出来.在runThrotLim中还要继续使用.
float targetRpm;     //目标转速 设定值(只在闭环 或 闭环推力模式下使用此变量)

static float rpmI;
static float maxCurrentSQRT;  //最大电流 平方根 后
uint8_t disarmReason;//此变量没啥作用.只用于给上位机显示当前的 调试代码(或者说停止电机的原因)
uint8_t commandMode; //串口通讯的模式, cli是ascii模式, binary是二进制通讯模式
static uint8_t runArmCount;
volatile uint8_t runMode;//运行模式 (开环模式, RPM模式, 推力模式, 伺服模式)
static float maxThrust;

//执行看门狗喂狗
void runFeedIWDG(void) {
#ifdef RUN_ENABLE_IWDG
    IWDG_ReloadCounter();
#endif
}

// setup the hardware independent watchdog
// 初始化并开启独立看门狗
uint16_t runIWDGInit(int ms) 
{
#ifndef RUN_ENABLE_IWDG
    return 0;
#else
	uint16_t prevReloadVal;
	int reloadVal;

	IWDG_ReloadCounter();//喂狗

	DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);//当在jtag调试的时候.停止看门狗

	// IWDG timeout equal to 10 ms (the timeout may varies due to LSI frequency dispersion)
	// Enable write access to IWDG_PR and IWDG_RLR registers
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);//允许访问IWDG_PR和IWDG_RLR寄存器

	// IWDG counter clock: LSI/4
	IWDG_SetPrescaler(IWDG_Prescaler_4);

	// Set counter reload value to obtain 10ms IWDG TimeOut.
	//  Counter Reload Value	= 10ms/IWDG counter clock period
	//				= 10ms / (RUN_LSI_FREQ/4)
	//				= 0.01s / (RUN_LSI_FREQ/4)
	//				= RUN_LSI_FREQ/(4 * 100)
	//				= RUN_LSI_FREQ/400
	reloadVal = RUN_LSI_FREQ*ms/4000;

	if (reloadVal < 1)
		reloadVal = 1;
	else if (reloadVal > 0xfff)
		reloadVal = 0xfff;

	prevReloadVal = IWDG->RLR;

	IWDG_SetReload(reloadVal);

	// Reload IWDG counter
	IWDG_ReloadCounter();

	// Enable IWDG (the LSI oscillator will be enabled by hardware)
	IWDG_Enable();

	return (prevReloadVal*4000/RUN_LSI_FREQ);
#endif
}

//电调运行看门狗. 主要是判断电调的当前一些状态.做出停机等处理
static void runWatchDog(void) 
{/*
	register uint32_t t, d, p;

	//__asm volatile ("cpsid i");
	//CPSID_I();
	__disable_irq();
	t = timerMicros;      //当前的系统tick时间
	d = detectedCrossing;
	p = pwmValidMicros;   //在PWM输入模式下.把timerMicros的时间赋值给此变量
	//__asm volatile ("cpsie i");
	//CPSIE_I();
	__enable_irq();

	if (state == ESC_STATE_STARTING && fetGoodDetects > fetStartDetects) //这里要检测到fetStartDetects好的检测,才允许切换电机状态
	{
		//是启动状态.切换到 运行状态
		state = ESC_STATE_RUNNING;
		digitalHi(statusLed);   // turn off
	}
	else if (state >= ESC_STATE_STOPPED) 
	{
		//运行模式状态下.会一直在这里检测状态.如果状态不对出错.会调用runDisarm函数停止

		// running or starting
		d = (t >= d) ? (t - d) : (TIMER_MASK - d + t);

		// timeout if PWM signal disappears
		if (inputMode == ESC_INPUT_PWM) 
		{
			//PWM模式 判断PWM输入是否超时
			p = (t >= p) ? (t - p) : (TIMER_MASK - p + t);

			if (p > PWM_TIMEOUT)
				runDisarm(REASON_PWM_TIMEOUT);//pwm输入超时
		}

		if (state >= ESC_STATE_STARTING && d > ADC_CROSSING_TIMEOUT) 
		{
			if (fetDutyCycle > 0) {
				runDisarm(REASON_CROSSING_TIMEOUT);//错误停止
			}
			else 
			{
				runArm();//手动运行起来
				pwmIsrRunOn();//PWM开启输入比较
			}
		}
		else if (state >= ESC_STATE_STARTING && fetBadDetects > fetDisarmDetects)  //运行状态中  检测到错误的个数后.进入这个判断
		{
			//在运行过程中,出现错误.停止运行
			if (fetDutyCycle > 0)
				runDisarm(REASON_BAD_DETECTS);//错误停止
		}
		else if (state == ESC_STATE_STOPPED) 
		{
			//停止模式
			adcAmpsOffset = adcAvgAmps;	// record current amperage offset
		}
	}
	else if (state == ESC_STATE_DISARMED && !(runMilis % 100)) 
	{
		//停止模式下
		adcAmpsOffset = adcAvgAmps;	// record current amperage offset
		digitalTogg(errorLed);
	}
	*/
}

/*
static void runSetupPVD(void) {
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Configure EXTI Line16(PVD Output) to generate an interrupt on rising and falling edges
    EXTI_ClearITPendingBit(EXTI_Line16);
    EXTI_InitStructure.EXTI_Line = EXTI_Line16;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable the PVD Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Configure the PVD Level to 2.2V
    PWR_PVDLevelConfig(PWR_PVDLevel_2V2);//配置pvd电压等级.当电压小于2.2V的时候产生中断

    // Enable the PVD Output
    PWR_PVDCmd(ENABLE);
}
*/
void runInit(void) {
    //runSetupPVD();
    runSetConstants();
    runMode = p[STARTUP_MODE];//启动 运行模式

	//系统tickcount时钟
    SysTick_Config(SystemCoreClock / 1000); // 1ms
    NVIC_SetPriority(SysTick_IRQn, 2);	    // lower priority

    // setup hardware watchdog
    runIWDGInit(20);
}

#define RUN_CURRENT_ITERM	1.0f
#define RUN_CURRENT_PTERM	10.0f
#define RUN_MAX_DUTY_INCREASE	1.0f

float currentIState;

//根据PID计算出PWM占空比的值

//系统tickcount中断
void SysTick_Handler(void) {
    // reload the hardware watchdog
    runFeedIWDG();


    avgVolts = adcAvgVolts * ADC_TO_VOLTS;                     //转换后的电池电压(一般是12V) = ADC采集电压原始值 * 电压算法
    avgAmps = (adcAvgAmps - adcAmpsOffset) * adcToAmps;        //平均电流 = (当前电流 - 停止时候的电流) * 转换公式
    maxAmps = (adcMaxAmps - adcAmpsOffset) * adcToAmps;        //最大电流 = (最大电流 - 停止时候的电流) * 转换公式

	if(runMilis%100 ==0){//100ms
		tripodTimerAction();
		adcTimerAction();
	}

	//计算空闲时间百分比 通过串口发送给上位机  没什么用途
    idlePercent = 100.0f * (idleCounter-oldIdleCounter) * minCycles / totalCycles;
//  空闲时间百分比 = 100 * (本次循环次数 - 上次循环次数) * 最小周期 / 总共周期
    oldIdleCounter = idleCounter;
    totalCycles = 0;


	//处理串口数据 和串口交互使用的
    if (commandMode == CLI_MODE)
		cliCheck();    //ascii模式
    else
		binaryCheck(); //二进制模式

    runMilis++;
}

//低电压中断
/*
void PVD_IRQHandler(void) {
    // voltage dropping too low
    if (EXTI_GetITStatus(EXTI_Line16) != RESET) {
		// shut everything down
		runDisarm(REASON_LOW_VOLTAGE);

		// turn on both LEDs
		digitalLo(statusLed);
		digitalLo(errorLed);

		EXTI_ClearITPendingBit(EXTI_Line16);
    }
}
*/
void runSetConstants(void) {
    int32_t startupMode = (int)p[STARTUP_MODE];
    float maxCurrent = p[MAX_CURRENT];

	//运行模式
    if (startupMode < 0 || startupMode >= NUM_RUN_MODES)
		startupMode = 0;

    if (maxCurrent > RUN_MAX_MAX_CURRENT)
		maxCurrent = RUN_MAX_MAX_CURRENT;
    else if (maxCurrent < RUN_MIN_MAX_CURRENT)
		maxCurrent = RUN_MIN_MAX_CURRENT;

    //runRPMFactor = (1e6f * (float)TIMER_MULT * 120.0f) / (p[MOTOR_POLES] * 6.0f);
    maxCurrentSQRT = sqrtf(maxCurrent);

    p[MOTOR_POLES] = (int)p[MOTOR_POLES];
    p[STARTUP_MODE] = startupMode;
    p[MAX_CURRENT] = maxCurrent;

    // Calculate MAX_THRUST from PWM_RPM_SCALE (which is MAX_RPM) and THRxTERMs
    // Based on "thrust = rpm * a1 + rpm^2 * a2"
    maxThrust = p[PWM_RPM_SCALE] * p[THR1TERM] + p[PWM_RPM_SCALE] * p[PWM_RPM_SCALE] * p[THR2TERM];
}
