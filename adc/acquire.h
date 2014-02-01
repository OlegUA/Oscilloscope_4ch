#ifndef __ACQUIRE_H__
#define __ACQUIRE_H__

#include "stm32f30x.h"

//
//   Hardware configuration
//
/**************************************************************************************/
//
//        ADC1    ADC2    ADC3    ADC4
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

// ADC1/2 only
#define OSC_CHANNEL1	GPIO_Pin_1	// PA1
#define OSC_CHANNEL2	GPIO_Pin_4	// PA4
// ADC2/3 only
#define OSC_CHANNEL3	GPIO_Pin_1	// PB1
#define OSC_CHANNEL4	GPIO_Pin_12	// PB12

// num_channels = 12 or 34 or 1234 only
extern void Acquire_Init(uint16_t acq_length, uint32_t acq_frequency, int num_channels);

extern void Acquire_Start(void);
extern void Acquire_Stop(void);
extern void Acquire_SetSampleLength(uint16_t acq_length);
extern void Acquire_SetFrequency(uint32_t acq_frequency);

extern uint16_t Acquire_GetSampleLength(void);
extern uint32_t Acquire_GetFrequency(void);
extern int Acquire_GetChannels(void);
extern int Acquire_IsDone(void);

//extern volatile uint16_t *ADC12DualConvertedValue;
//extern volatile uint16_t *ADC34DualConvertedValue;

extern volatile uint16_t ADC12DualConvertedValue[];
extern volatile uint16_t ADC34DualConvertedValue[];

#endif  /* __ACQUIRE_H__ */
