
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <string.h>
#include "main.h"
#include "acquire.h"

void redirect_init(void);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
__IO uint32_t UserButtonPressed = 0;

extern __IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
uint8_t Send_Buffer[64];
uint32_t packet_sent=1;
uint32_t packet_receive=1;

__IO uint16_t  calibration_value = 0;
static uint16_t _acq_length = 200;
static uint32_t _acq_req = 100000;
static uint8_t	_fcm = 0;	// ******************* set to 2 for debug purposes
static uint8_t	_bin_out = 0;
static uint8_t 	_echo = 0;

void send_string(char *str);
char *read_string(void);
void transfer_samples(void);
void print_prompt(void);

/* Private function prototypes -----------------------------------------------*/
#define START_ACQ	"START"
#define LOOP_ACQ	"LOOP"
#define STOP_ACQ	"STOP"
#define RECALIBRATE	"CALIB"
#define VERSION		"VERS"
#define PARAMETERS	"PAR"
#define NEW_FRQ		"FRQ"
#define ASCII_OUT	"ASCII"
#define BINARY_OUT	"BIN"

typedef enum acq_state { 
	UNDEFINE = -1, 
	IDLE, 
	ACQUIRE, 
	ADC_RETRIVE_DATA, 
	CALIBRATE, 
	RETRIVE_CMD, 
	PROCESS_CMD, 
	TRANSMIT_DATA,
	SEND_RESPONSE,
	UPDATE_FRQ,
} ACQ_State_Type;

typedef enum command { 
	CMD_UNKNOWN = -1, 
	NOP,
	CMD_START,
	CMD_LOOP,
	CMD_STOP, 
	CMD_CALIBR,
	CMD_VERSION,
	CMD_PARAMETERS,
	CMD_ASCII,
	CMD_BINARY,
	CMD_NEW_FRQ
} Command_Type;

ACQ_State_Type ACQ_State = UNDEFINE;
Command_Type OscCmd = CMD_UNKNOWN;
int loop_mode = 0;

Command_Type ParseCommand(char *);

/* Private functions ---------------------------------------------------------*/
static void test_vcp_loopback() {
	int i;
  while (1) {
    if (bDeviceState == CONFIGURED) {
      CDC_Receive_DATA();
      /*Check to see if we have data yet */
      if (Receive_length  != 0) {
        if (packet_sent == 1)
          CDC_Send_DATA ((unsigned char*)Receive_Buffer,Receive_length);
		for(i=0;i<Receive_length;i++)
			printf("%c",Receive_Buffer[i]);
        Receive_length = 0;
      }
    }
  }
}

static void test_adc() {
	int i,b;
	
	Acquire_Start(); 
	b = UserButtonPressed;
	while(1) {
		while(!Acquire_IsDone());
		STM_EVAL_LEDOn(LED9);
		for(i=0;i<_acq_length;i+=2) {
			if( _fcm ) {
				printf("%d\n%d\n%d\n%d\n",
						ADC12DualConvertedValue[i],
						ADC12DualConvertedValue[i+1],
						ADC34DualConvertedValue[i],
						ADC34DualConvertedValue[i+1]);
			} else {
				printf("%d\n%d\n", 
						ADC12DualConvertedValue[i],
						ADC12DualConvertedValue[i+1]);
			}
		}
		while(b==UserButtonPressed);
		b = UserButtonPressed;
		Acquire_Start();
	}
}

/*******************************************************************************
* Function Name  : main.
* Descriptioan    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void) {
	char str[sizeof(Receive_Buffer)+1],*ptr;
	int i;
	redirect_init();	
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI); 

	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED7);
	STM_EVAL_LEDInit(LED9);
	STM_EVAL_LEDInit(LED10);
	STM_EVAL_LEDInit(LED8);
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED4);

	VCP_Set_System();	
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();

	Acquire_Init(200, 1000000, _fcm?1234:12);
	STM_EVAL_LEDOn(LED3);
	printf("Initialized\n");

	/* Turn on LD3 */
	STM_EVAL_LEDOn(LED6);
	/* Turn off LD4 */
	STM_EVAL_LEDOff(LED4);

//	test_adc();
//	test_vcp_loopback();
	ACQ_State = IDLE;
	print_prompt();
  /* Infinite loop */
  while (1) {
	char cmd_save[16];
	switch(ACQ_State) {
		case	IDLE:
		    if( loop_mode == 1 ) {
				ACQ_State = ACQUIRE;
				if (bDeviceState == CONFIGURED) {
					CDC_Receive_DATA();
					if (Receive_length  != 0) {
						Receive_Buffer[Receive_length] = '\0';
						//printf("<%s> %d",Receive_Buffer,Receive_length);
						loop_mode = 0;
						Receive_length = 0;
						ACQ_State = IDLE;
					}
				}
		    } else if ( (ptr = read_string()) ) {
				//printf("got=\'%s\'\n",ptr);
				strcpy(cmd_save,ptr);
				cmd_save[strlen(cmd_save)-1] = '\0';
				OscCmd = ParseCommand(ptr);
				Receive_length = 0;
				ACQ_State = PROCESS_CMD;
		    }
		break;
		case 	ACQUIRE:
			Acquire_Start(); 
			while(!Acquire_IsDone());
			ACQ_State = ADC_RETRIVE_DATA;
		break;
		case 	ADC_RETRIVE_DATA:
			/* Get ADC1 converted data */
			/* Compute the voltage */
			ACQ_State = TRANSMIT_DATA;
		break;
		case 	CALIBRATE:
			// full restart ADC here, but not work yet :(
//			Acquire_Init(_acq_length, _acq_req, _fcm?1234:12);
			sprintf(str,"OK %d %d %d",\
				Acquire_GetSampleLength(),
				Acquire_GetFrequency(),
				Acquire_GetChannels()==1234?4:2);
			ACQ_State = SEND_RESPONSE;
		break;
		case	UPDATE_FRQ:
			Acquire_SetFrequency(_acq_req);
			sprintf(str,"OK %d",Acquire_GetFrequency());
			ACQ_State = SEND_RESPONSE;
		break;
		case 	RETRIVE_CMD:
			/* unused */
			ACQ_State = IDLE;
		break;
		case	PROCESS_CMD:
		  if( OscCmd == NOP ) {
			  str[0] = '\0';
			  ACQ_State = SEND_RESPONSE;			
		  } else if( OscCmd == CMD_START ) {
			  ACQ_State = ACQUIRE;
		  } else if( OscCmd == CMD_LOOP ) {
			  ACQ_State = ACQUIRE;
			  loop_mode = 1;
	      } else if( OscCmd == CMD_STOP ) {
			  loop_mode = 0;
			  ACQ_State = IDLE;
		  } else if( OscCmd == CMD_CALIBR ) {
			  ACQ_State = CALIBRATE;
		  } else if( OscCmd == CMD_VERSION ) {
			  sprintf(str,"VERSION=%d.%d (%s %s)",0,1,__DATE__, __TIME__);
			  ACQ_State = SEND_RESPONSE;
		  } else if( OscCmd == CMD_PARAMETERS ) {
			sprintf(str,"%d %d %d",\
				Acquire_GetSampleLength(),
				Acquire_GetFrequency(),
				Acquire_GetChannels()==1234?4:2);
			  ACQ_State = SEND_RESPONSE;
		  } else if( OscCmd == CMD_NEW_FRQ ) {
			  ACQ_State = UPDATE_FRQ;
		  } else {
			  sprintf(str,"ERR <%s> %d UNKNOWN CMD",cmd_save,OscCmd);
			  ACQ_State = SEND_RESPONSE;
		  }
		break;
		case 	TRANSMIT_DATA:
			transfer_samples();
			ACQ_State = IDLE;
		break;
		case	SEND_RESPONSE:
			if( str[0] != 0 )
				send_string(str);
			print_prompt();
			ACQ_State = IDLE;
		break;
		default:
		break;
	}
  }
} 

void print_prompt(void) {
	if (bDeviceState == CONFIGURED) {
		if (packet_sent == 1)
		CDC_Send_DATA ((uint8_t *)"*",1);
	}
}

static char line[sizeof(Receive_Buffer)];
static int lp = 0;
char *read_string() {
	uint8_t *dst = (uint8_t *)&line[lp];
	uint8_t *src;
	if (bDeviceState == CONFIGURED) {
		CDC_Receive_DATA();
		if (Receive_length  == 0) return(0);
		//printf("!");
		do {
			if (Receive_length  != 0) {
				if( _echo ) {
					while(packet_sent != 1);
					CDC_Send_DATA ((uint8_t *)Receive_Buffer,Receive_length);
				}
				Receive_Buffer[Receive_length] = '\0';
				//printf("<%s> %d",Receive_Buffer,Receive_length);
				src = (uint8_t *)&Receive_Buffer[0];
				while(Receive_length) {
					*dst++ = *src++;
					Receive_length--;
					lp++;
				}
			}
			line[lp] = '\0';
			CDC_Receive_DATA();
		} while(!strchr(line, '\n') && !strchr(line, '\r')); 
		lp = 0;
		return(line);
	} else
		return(0);
}

void send_string(char *str) {
	if (bDeviceState == CONFIGURED) {
		strcat(str,"\n");
		while(packet_sent != 1);
		CDC_Send_DATA ((uint8_t *)str,strlen(str));
	}
}

void transfer_samples(void) {
	register int i,n;
	char str[64];
	if (bDeviceState == CONFIGURED) {
		for(n=0;n<2;n++) {
			for(i=0;i<Acquire_GetSampleLength()*2;i+=2) {
				sprintf(str,"%d ",ADC12DualConvertedValue[i+n]*3300/0xFFF);
				while(packet_sent != 1);
				CDC_Send_DATA ((uint8_t *)str,strlen(str));
			}
			str[0] = '\n';
			str[1] = '\0';
			while(packet_sent != 1);
			CDC_Send_DATA ((uint8_t *)str,1);
		}
		// adc34
		if( Acquire_GetChannels() == 1234 ) {
			for(n=0;n<2;n++) {
				for(i=0;i<Acquire_GetSampleLength()*2;i+=2) {
					sprintf(str,"%d ",ADC34DualConvertedValue[i+n]*3300/0xFFF);
					while(packet_sent != 1);
					CDC_Send_DATA ((uint8_t *)str,strlen(str));
				}
				str[0] = '\n';
				str[1] = '\0';
				while(packet_sent != 1);
				CDC_Send_DATA ((uint8_t *)str,1);
			}
		}
	}
}

static int parse_params(char *ptr, int *p, int num) {
	int i,n;
	char *next = strchr(ptr,';');
	if( !next ) return(0);
	*next++ = '\0';
	for(n=i=0;i<num;i++) {
		printf("<%s>\n",ptr);
		sscanf(ptr,"%d",&p[i]);
		n++;
		printf("%d\n",p[i]);
		ptr = next;
		if( !(next = strchr(ptr,';')) ) break;
		*next++ = '\0';
	}
	return(n);
}

Command_Type ParseCommand(char *ptr) {
	Command_Type cmd = CMD_UNKNOWN;
	
	printf("cmd=%s\n",ptr);

	if( *ptr == '\r' || *ptr == '\n' )
		return(NOP);
		
	if( strncasecmp(ptr,START_ACQ,strlen(START_ACQ)) == 0 ) {
		cmd = CMD_START;
	} else if( strncasecmp(ptr,LOOP_ACQ,strlen(LOOP_ACQ)) == 0 ) {
		cmd = CMD_LOOP;
	} else if(strncasecmp(ptr,STOP_ACQ,strlen(STOP_ACQ)) == 0 ) {
		cmd = CMD_STOP;
	} else if(strncasecmp(ptr,RECALIBRATE,strlen(RECALIBRATE)) == 0 ) {
		int p[3],r;
		if( (r=parse_params(ptr+strlen(RECALIBRATE),p,3)) == 3 ) {
			_acq_length = p[0];
			_acq_req = p[1];
			_fcm = p[2];
		}
		printf("calib %d\n",r);
		cmd = CMD_CALIBR;
	} else if(strncasecmp(ptr,VERSION,strlen(VERSION)) == 0 ) {
		cmd = CMD_VERSION;
	} else if(strncasecmp(ptr,PARAMETERS,strlen(PARAMETERS)) == 0 ) {
		cmd = CMD_PARAMETERS;
	} else if(strncasecmp(ptr,NEW_FRQ,strlen(NEW_FRQ)) == 0 ) {
		int p[1],r;
		if( (r=parse_params(ptr+strlen(NEW_FRQ),p,1)) == 1 ) {
			_acq_req = p[0];
		}
		printf("freq %d\n",r);
		cmd = CMD_NEW_FRQ;
	}
	return(cmd);
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
