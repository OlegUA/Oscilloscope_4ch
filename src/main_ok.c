#include "main.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_dma.h"
#include "stm32f30x_adc.h"
#include "stm32f30x_tim.h"


// STM32 ADC Quad Mode STM32F3-Discovery - sourcer32@gmail.com

/**************************************************************************************/

//       ADC1    ADC2    ADC3    ADC4
//  IN1   PA0*    PA4     PB1     PE14*
//  IN2   PA1     PA5*    PE9*    PE15*
//  IN3   PA2     PA6*    PE13*   PB12
//  IN4   PA3     PA7*    ---     PB14
//  IN5   PF4     PC4     PB13    PB15
//
//  *Used on STM32F3-Discovery
// Free pins located in STM32F3-Discovery manual, UM1570

// ADC1/2 Trigger EXT3 TIM2_CC2
// ADC3/4 Trigger EXT1 TIM2_CC3

// Pins for this example PA1, PA4, PB1, PB12

/**************************************************************************************/

//#include "stm32f30x.h"
#include "stm32f3_discovery.h"

/**************************************************************************************/

#define SAMPLES 25

#define ADC12_CDR_ADDRESS    ((uint32_t)0x5000030C)
#define ADC34_CDR_ADDRESS    ((uint32_t)0x5000070C)

volatile uint16_t ADC12DualConvertedValue[SAMPLES * 2];
volatile uint16_t ADC34DualConvertedValue[SAMPLES * 2];

/**************************************************************************************/

void ADC_GPIO_Configuration(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOA and GPIOB Periph clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

	/* ADC Channels configuration */
	/* Configure  as analog input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**************************************************************************************/

void ADC12_DMA_Configuration(void) {
	DMA_InitTypeDef DMA_InitStructure;

	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* DMA configuration */
	/* DMA1 Channel1 Init Test */
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC12_CDR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC12DualConvertedValue[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = SAMPLES;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1, &DMA_InitStructure); // ADC1
}

/**************************************************************************************/

void ADC34_DMA_Configuration(void) {
	DMA_InitTypeDef DMA_InitStructure;

	/* Enable DMA2 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

	/* DMA configuration */
	/* DMA2 Channel5 Init Test */
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC34_CDR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC34DualConvertedValue[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = SAMPLES;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA2_Channel5, &DMA_InitStructure); // ADC3
}

/**************************************************************************************/

void ADC_Configuration(void) {
	ADC_InitTypeDef        ADC_InitStructure;
	ADC_CommonInitTypeDef  ADC_CommonInitStructure;
	volatile int i;
	uint16_t CalibrationValue[4];

	/* Configure the ADC clocks */
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div1);

	/* Enable ADC1/2/3/4 clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12 | RCC_AHBPeriph_ADC34, ENABLE);

	/* ADC GPIO configuration */
	ADC_GPIO_Configuration();

	/* ADC DMA Channel configuration */
	ADC12_DMA_Configuration();
	ADC34_DMA_Configuration();

	/* ADC Calibration procedure */
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	ADC_VoltageRegulatorCmd(ADC2, ENABLE);
	ADC_VoltageRegulatorCmd(ADC3, ENABLE);
	ADC_VoltageRegulatorCmd(ADC4, ENABLE);

	/* Insert delay equal to 10 µs */
	//Delay(10);
	for(i=0; i<10000; i++);

	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);

	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC2);

	ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC3);

	ADC_SelectCalibrationMode(ADC4, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC4);

	while(ADC_GetCalibrationStatus(ADC1) != RESET );
	CalibrationValue[0] = ADC_GetCalibrationValue(ADC1);

	while(ADC_GetCalibrationStatus(ADC2) != RESET );
	CalibrationValue[1] = ADC_GetCalibrationValue(ADC2);

	while(ADC_GetCalibrationStatus(ADC3) != RESET );
	CalibrationValue[2] = ADC_GetCalibrationValue(ADC3);

	while(ADC_GetCalibrationStatus(ADC4) != RESET );
	CalibrationValue[3] = ADC_GetCalibrationValue(ADC4);

	/* ADC Dual mode configuration */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_RegSimul;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; // 12-bit
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 10;

	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC3, &ADC_CommonInitStructure);

	/* */
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable; // Triggered
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;

	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_3; // ADC1/2 EXT3 TIM2_CC2

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC2, &ADC_InitStructure);

	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_1; // ADC3/4 EXT1 TIM2_CC3

	ADC_Init(ADC3, &ADC_InitStructure);
	ADC_Init(ADC4, &ADC_InitStructure);

	/* ADC1 regular configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_7Cycles5); // PA1
	/* ADC2 regular configuration */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5); // PA4
	/* ADC3 regular configuration */
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5); // PB1
	/* ADC4 regular configuration */
	ADC_RegularChannelConfig(ADC4, ADC_Channel_3, 1, ADC_SampleTime_7Cycles5); // PB12

	/* Configures the ADC DMA */
	ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMAConfig(ADC3, ADC_DMAMode_Circular);

	/* Enable the ADC DMA */
	ADC_DMACmd(ADC1, ENABLE);
	ADC_DMACmd(ADC3, ENABLE);

	/* Enable ADC[1..4] */
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_Cmd(ADC4, ENABLE);

	/* wait for ADC1 ADRDY */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));

	/* wait for ADC2 ADRDY */
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));

	/* wait for ADC3 ADRDY */
	while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY));

	/* wait for ADC4 ADRDY */
	while(!ADC_GetFlagStatus(ADC4, ADC_FLAG_RDY));

	/* Enable the DMA channel */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	DMA_Cmd(DMA2_Channel5, ENABLE);

	/* Start ADC1 Software Conversion */
	ADC_StartConversion(ADC1);
	ADC_StartConversion(ADC3);
}

/**************************************************************************************/

void TIM2_Configuration(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* Enable ADC1/2/3/4 clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / 50) - 1; // 50 Hz Sample Rate
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel2 & 3 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 10; // Some arbitary width
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_Cmd(TIM2, ENABLE);
}

/**************************************************************************************/

void NVIC_Configuration(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	/* Enable DMA1 channel1 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable DMA2 channel5 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel5_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

/**************************************************************************************/

void DMA1_Channel1_IRQHandler(void) { // 2 Hz
	/* Test on DMA1 Channel1 Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA1_IT_TC1))
	{
		/* Clear DMA1 Channel1 Half Transfer, Transfer Complete and Global interrupt pending bits */
		DMA_ClearITPendingBit(DMA1_IT_GL1);

		STM_EVAL_LEDToggle(LED3); // 1 Hz
	}
}

/**************************************************************************************/

void DMA2_Channel5_IRQHandler(void) { // 2 Hz
	/* Test on DMA2 Channel5 Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA2_IT_TC5))
	{
		/* Clear DMA2 Channel5 Half Transfer, Transfer Complete and Global interrupt pending bits */
		DMA_ClearITPendingBit(DMA2_IT_GL5);

		STM_EVAL_LEDToggle(LED4); // 1 Hz
	}
}

/**************************************************************************************/

int main(void) {
	/* Initialize LEDs available on STM32F3-Discovery board */
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);

	/* Turn on LD3 */
	STM_EVAL_LEDOn(LED3);
	/* Turn off LD4 */
	STM_EVAL_LEDOff(LED4);

	TIM2_Configuration();

	ADC_Configuration();

	/* Use LED's to confirm sample periodicity */
	NVIC_Configuration();

	/* Enable DMA1 Channel1 Transfer Complete interrupt */
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

	/* Enable DMA2 Channel5 Transfer Complete interrupt */
	DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);

	while(1); // Don't want to exit
}

