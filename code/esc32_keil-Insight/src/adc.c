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

#include "main.h"
#include "adc.h"
#include "run.h"
#include "digital.h"
#include "config.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "tripod.h"
#include "misc.h"

static uint16_t adcRawData[ADC_CHANNELS];
/*ADC_CH2,ADC_CH1*/

float adcToAmps;    //ADC的电流测量值 转换成 实际的电流公式
int32_t adcMaxPeriod;//ADC最大的周期


static int16_t histIndex;  //数组的索引值
int16_t histSize;          //数组的大小
static uint16_t histA[ADC_HIST_SIZE];//adc转换后的值,保准在这个数组里
static uint16_t histB[ADC_HIST_SIZE];
static uint16_t histC[ADC_HIST_SIZE];
uint32_t avgA, avgB, avgC;    //电机A B C相的电压ADC采集.平均值
int32_t adcAmpsOffset;        //当前ADC采集转换后的 传感器电流 电流偏移 (在停止运行模式下,电流的值.做偏移值)
volatile int32_t adcAvgAmps;  //当前ADC采集转换后的 传感器电流
volatile int32_t adcMaxAmps;  //运行中最大 传感器电流
volatile int32_t adcAvgVolts; //运行的电压
volatile int32_t adcVal1,adcVal2;
volatile int32_t adcVal1HighX00ms,adcVal2HighX00ms;
volatile int8_t adcVal1HighFlag,adcVal2HighFlag;
static uint8_t adcStateA, adcStateB, adcStateC;//当前的状态

volatile uint32_t detectedCrossing;
volatile uint32_t crossingPeriod;
volatile int32_t adcCrossingPeriod;
static uint32_t nextCrossingDetect;  //ADC换相时间(在中断中会计算出来,下一个换相的时间)

//重新对ADC内部校准
static void adcCalibrateADC(ADC_TypeDef *ADCx) 
{
    // Enable ADC reset calibration register
    ADC_ResetCalibration(ADCx);

    // Check the end of ADC reset calibration register
    while(ADC_GetResetCalibrationStatus(ADCx))
		;

    // Start ADC calibration
    ADC_StartCalibration(ADCx);

    // Check the end of ADC calibration
    while(ADC_GetCalibrationStatus(ADCx))
		;
}

void adcInit(void) 
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    adcSetConstants();
    histSize = ADC_HIST_SIZE;
		adcVal1HighFlag = 0;
		adcVal2HighFlag = 0;
    // Use STM32's Dual Regular Simultaneous Mode capable of ~ 1.7M samples per second

    // NOTE: assume that RCC code has already placed all pins into Analog In mode during startup

    // DMA1 channel1 configuration (ADC1)
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1 + 0x4c;   //从这个寄存器读
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adcRawData[0];    //写入到这个内存
	DMA_InitStructure.DMA_BufferSize = ADC_CHANNELS;            //传输数据量

	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                     //从外设读
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;       //外设地址不递加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                //存储器地址递加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据宽度16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;        //存储器数据宽度16位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                        //循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                //通道优先级最高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                           //非存储器到存储器模式
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    DMA_ClearITPendingBit(DMA1_IT_GL1 | DMA1_IT_TC1);
    DMA_Cmd(DMA1_Channel1, ENABLE);


    // Enable the DMA1_Channel1 global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    // ADC1 configuration
//    ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//混合的同步规则+注入同步模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;         //使用扫描模式

	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                  //连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //SWSTART 软件触发模式
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              //数据右对齐

	ADC_InitStructure.ADC_NbrOfChannel = ADC_CHANNELS;//规则通道序列长度 有8个转换通道
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SAMPLE_TIME);	// ADC_CH2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SAMPLE_TIME);	// ADC_CH1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SAMPLE_TIME);	// ADC_CH2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 4, ADC_SAMPLE_TIME);	// ADC_CH1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 5, ADC_SAMPLE_TIME);	// ADC_CH2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 6, ADC_SAMPLE_TIME);	// ADC_CH1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 7, ADC_SAMPLE_TIME);	// ADC_CH2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 8, ADC_SAMPLE_TIME);	// ADC_CH1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 9, ADC_SAMPLE_TIME);	// ADC_CH2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 10, ADC_SAMPLE_TIME);	// ADC_CH1
    ADC_DMACmd(ADC1, ENABLE);//ADC1开启DMA模式

    // enable and calibrate
    ADC_Cmd(ADC1, ENABLE);
    adcCalibrateADC(ADC1);

    nextCrossingDetect = adcMaxPeriod;

    // setup injection sequence
	// 设置注入序列
    //ADC_InjectedSequencerLengthConfig(ADC1, 1);//注入序列只有1个转换
    //ADC_InjectedChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SAMPLE_TIME);//设置注入序列转换的通道
    //ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);//注入序列 使用外部事件启动转换
    //ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);//软件触发

    // Start ADC1 / ADC2 Conversions
    //ADC_SoftwareStartConvCmd(ADC1, ENABLE);//开始转换.并设置好外部触发模式
}

void adcTimerAction(){
	if(adcVal2HighFlag){
		adcVal2HighX00ms++;
	}
	if(adcVal1HighFlag){
		adcVal1HighX00ms++;
	}
}

//dma1 ad采样完成中断
//#pragma GCC optimize ("-O1")
#pragma O1
void DMA1_Channel1_IRQHandler(void) 
{

	if ((DMA1->ISR & DMA1_FLAG_TC1) != RESET) {
		adcVal2= (adcRawData[0] + adcRawData[2] + adcRawData[4] + adcRawData[6] + adcRawData[8])*2/ADC_CHANNELS;
		adcVal1= (adcRawData[1] + adcRawData[3] + adcRawData[5] + adcRawData[7] + adcRawData[9])*2/ADC_CHANNELS;
	}

	DMA1->IFCR = DMA1_IT_GL1 | DMA1_IT_TC1;


	if(tripodState == TRIPOD_STATE_INIT || tripodState == TRIPOD_STATE_OPENED|| tripodState == TRIPOD_STATE_CLOSED){
		adcVal1HighFlag = 0;
		adcVal2HighFlag = 0;
		ADC_SoftwareStartConvCmd(ADC1, DISABLE);
		return;
	}

	//sprintf(printBuf, "DMA1_Channel1_IRQHandler adcVal1:%d,adcVal1:%d\r\n",adcVal2,adcVal1);
	//serialPrint(printBuf);
	if(adcVal2 > ADC_THRESHOLD_VALUE){
		if(adcVal2HighFlag){
			if(adcVal2HighX00ms > 5){//> 500ms
				adcVal2HighFlag = 0;
				tripodAdcAction(TRIPOD_ARM2);
			}
		}else{
			adcVal2HighX00ms = 0;
			adcVal2HighFlag = 1;
		}
	}
	if(adcVal1 > ADC_THRESHOLD_VALUE){
		if(adcVal1HighFlag){
			if(adcVal1HighX00ms> 5){//> 500ms
				adcVal1HighFlag = 0;
				tripodAdcAction(TRIPOD_ARM1);
			}
		}else{
			adcVal1HighX00ms= 0;
			adcVal1HighFlag = 1;
		}
	}
}


void adcSetConstants(void) 
{
    float shuntResistance = p[SHUNT_RESISTANCE];
    float advance = p[ADVANCE];
    float blankingMicros = p[BLANKING_MICROS];
    float minPeriod = p[MIN_PERIOD];
    float maxPeriod = p[MAX_PERIOD];

    // bounds checking
	// 边界检查.不能超出一定的范围
    if (shuntResistance > ADC_MAX_SHUNT)
		shuntResistance = ADC_MAX_SHUNT;
    else if (shuntResistance < ADC_MIN_SHUNT)
		shuntResistance = ADC_MIN_SHUNT;

    if (advance > ADC_MAX_ADVANCE)
		advance = ADC_MAX_ADVANCE;
    else if (advance < ADC_MIN_ADVANCE)
		advance = ADC_MIN_ADVANCE;

    if (blankingMicros > ADC_MAX_BLANKING_MICROS)
		blankingMicros = ADC_MAX_BLANKING_MICROS;
    else if (blankingMicros < ADC_MIN_BLANKING_MICROS)
		blankingMicros = ADC_MIN_BLANKING_MICROS;

    if (minPeriod > ADC_MAX_MIN_PERIOD)
		minPeriod = ADC_MAX_MIN_PERIOD;
    else if (minPeriod < ADC_MIN_MIN_PERIOD)
		minPeriod = ADC_MIN_MIN_PERIOD;

    if (maxPeriod > ADC_MAX_MAX_PERIOD)
		maxPeriod = ADC_MAX_MAX_PERIOD;
    else if (maxPeriod < ADC_MIN_MAX_PERIOD)
		maxPeriod = ADC_MIN_MAX_PERIOD;

	//计算出几个参数
  //  adcMinPeriod = minPeriod * TIMER_MULT;//adc的最小采样周期(时间us)
  //  adcMaxPeriod = maxPeriod * TIMER_MULT;//adc的最大采样周期

	//写回数组里面
    p[SHUNT_RESISTANCE] = shuntResistance;
    p[MIN_PERIOD] = minPeriod;
    p[MAX_PERIOD] = maxPeriod;
}
