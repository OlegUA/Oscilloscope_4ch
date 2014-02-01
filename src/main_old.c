
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
uint8_t Send_Buffer[64];
uint32_t packet_sent=1;
uint32_t packet_receive=1;

__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0, calibration_value = 0;

ADC_InitTypeDef       ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
GPIO_InitTypeDef      GPIO_InitStructure;

/* Private function prototypes -----------------------------------------------*/
#define START_ACQ	"S"
#define LOOP_ACQ	"L"
#define STOP_ACQ	"P"
#define RECALIBRATE	"C"
#define VERSION		"V"

typedef enum acq_state { 
	UNDEFINE = -1, 
	IDLE, 
	ACQUIRE, 
	ADC_RETRIVE_DATA, 
	CALIBRATE, 
	RETRIVE_CMD, 
	PROCESS_CMD, 
	TRANSMIT_DATA 
} ACQ_State_Type;

typedef enum command { 
	CMD_UNKNOWN = -1, 
	CMD_START,
	CMD_LOOP,
	CMD_STOP, 
	CMD_CALIBR,
	CMD_VERSION
} Command_Type;

ACQ_State_Type ACQ_State = UNDEFINE;
Command_Type OscCmd = CMD_UNKNOWN;
int loop_mode = 0;

void Delay(__IO uint32_t nTime);
void ADCInit(void);
Command_Type ParseCommand(void);

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : main.
* Descriptioan    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  char str[sizeof(Receive_Buffer)+1];
	
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
  ADCInit();

  /* Infinite loop */
  while (1) {
	switch(ACQ_State) {
		case	IDLE:
		  if (bDeviceState == CONFIGURED) {
			  CDC_Receive_DATA();
			  /*Check to see if we have data yet */
			  if (Receive_length  != 0) {
				 OscCmd = ParseCommand();
				 Receive_length = 0;
				 ACQ_State = PROCESS_CMD;
			  } else {
				 if( loop_mode == 1 ) ACQ_State = ACQUIRE;
			  }
		  } else {
			 if( loop_mode == 1 ) ACQ_State = ACQUIRE;
		  }
		break;
		case 	ACQUIRE:
			/* Test EOC flag */
			while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
			ACQ_State = ADC_RETRIVE_DATA;
		break;
		case 	ADC_RETRIVE_DATA:
			/* Get ADC1 converted data */
			ADC1ConvertedValue =ADC_GetConversionValue(ADC1);
			/* Compute the voltage */
			ADC1ConvertedVoltage = (ADC1ConvertedValue *3300)/0xFFF;
			sprintf(str,"%d",ADC1ConvertedValue);
			ACQ_State = TRANSMIT_DATA;
		break;
		case 	CALIBRATE:
			/* not implemented yet */
			  strcpy(str,"ERR: NOT IMPLEMENTED");
			  ACQ_State = TRANSMIT_DATA;
		break;
		case 	RETRIVE_CMD:
			/* unused */
			ACQ_State = IDLE;
		break;
		case	PROCESS_CMD:
		  if( OscCmd == CMD_START || OscCmd == CMD_LOOP ) {
			  /* Start ADC1 Software Conversion */ 
			  ADC_StartConversion(ADC1);   
			  ACQ_State = ACQUIRE;
			  if( OscCmd == CMD_LOOP ) 
				  loop_mode = 1;
		  } else if( OscCmd == CMD_STOP ) {
			  /* Stop ADC1 Software Conversion */ 
			  ADC_StopConversion(ADC1);
			  loop_mode = 0;
			  ACQ_State = IDLE;
		  } else if( OscCmd == CMD_CALIBR ) {
			  /* not implemented yet */
			  ACQ_State = CALIBRATE;
		  } else if( OscCmd == CMD_VERSION ) {
			  sprintf(str,"VERSION=%d.%d (%s %s)",0,1,__DATE__, __TIME__);
			  ACQ_State = TRANSMIT_DATA;
		  } else {
			  sprintf(str,"ERR: <%s>:%d - UNKNOWN CMD",Receive_Buffer,OscCmd);
			  ACQ_State = TRANSMIT_DATA;
		  }
		break;
		case 	TRANSMIT_DATA:
			if (bDeviceState == CONFIGURED) {
			  strcat(str,"\n");
			  if (packet_sent == 1)
				CDC_Send_DATA ((uint8_t *)str,strlen(str));
			}
			ACQ_State = IDLE;
		break;
		default:
		break;
	}
  }
} 

Command_Type ParseCommand() {
  char *ptr = (char*)Receive_Buffer;
  Command_Type cmd = CMD_UNKNOWN;
	
  ptr[Receive_length] = '\0';
  while(*ptr) {
	*ptr = toupper((int)*ptr);
	ptr++;
  }
  ptr = (char*)Receive_Buffer;
  if( strcmp(ptr,START_ACQ) == 0 ) {
	  cmd = CMD_START;
  } else if( strcmp(ptr,LOOP_ACQ) == 0 ) {
	  cmd = CMD_LOOP;
  } else if(strcmp(ptr,STOP_ACQ) == 0 ) {
	  cmd = CMD_STOP;
  } else if(strcmp(ptr,RECALIBRATE) == 0 ) {
	  cmd = CMD_CALIBR;
  } else if(strcmp(ptr,VERSION) == 0 ) {
	  cmd = CMD_VERSION;
  }
  return(cmd);
}

void ADCInit() {
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f30x.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f30x.c file
     */ 

  /* Configure the ADC clock */
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
  
  /* Enable ADC1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
      
  /* Setup SysTick Timer for 1 µsec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000000))
  { 
    /* Capture error */ 
    while (1)
    {}
  }
  
  /* ADC Channel configuration */
  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

  /* Configure ADC Channel7 as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  ADC_StructInit(&ADC_InitStructure);

  /* Calibration procedure */  
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  
  /* Insert delay equal to 10 µs */
  Delay(10);
  
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
  
  while(ADC_GetCalibrationStatus(ADC1) != RESET );
  calibration_value = ADC_GetCalibrationValue(ADC1);
     
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* ADC1 regular channel7 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_7Cycles5);
   
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* wait for ADRDY */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
  
  ACQ_State = IDLE;
}


#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
