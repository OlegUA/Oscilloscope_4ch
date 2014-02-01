
// Rough SWV support in Keil, STM32F3-Discovery, Make SB10 to connect PB3/SWO
#include <stdio.h>
#include <rt_misc.h>

#include "platform_config.h"
#include "stm32f30x_usart.h"

	

void USART1_Init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
    /* enable usart clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

 
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 38400;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);

    NVIC_EnableIRQ(USART1_IRQn);
}


void USART1putc(const char ch) {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, ch);
}


#pragma import(__use_no_semihosting_swi)
 
struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;
 
int fputc(int ch, FILE *f)
{
    USART1putc(ch);
 
  return(ch);
}
 
int fgetc(FILE *f)
{
    char ch;
 
ch = 1;
 
  return((int)ch);
}
 
int ferror(FILE *f)
{
  /* Your implementation of ferror */
  return EOF;
}
 
void _ttywrch(int ch)
{
    USART1putc(ch);
}
 
void _sys_exit(int return_code)
{
label:  goto label;  /* endless loop */
}
 
void redirect_init(void) {
	USART1_Init();
}
