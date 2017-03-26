#ifndef __UZ__
#define __UZ__


#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>

#define MAX_UZ_HATS			4

#define UZ_LAST_PORT	-1

#define INTREF				2500
#define AREF				3300

#define DAC_READBACK_ENABLE			0b11
#define DAC_READBACK_ENABLE_SHIFT	3



#define LDAC_READBACK_ENABLE 		6
#define LDAC_READBACK_REGISTER		2

#define PWR_REF_INTREF_ENABLE		1
#define PWR_REF_POWER_DOWN_ALL		2

#define GPCR_DAC_GAIN				4
#define GPCR_ADC_GAIN				5
#define GPCR_ALL_DACS				6
#define GPCR_LOCK					7
#define GPCR_ADC_BUFFER_ENABLE		0     	//byte 0
#define GPCR_ADC_BUFFER_PRECHARGE	1		//byte 0

#define TXBUFFSIZE					2
#define RXBUFFSIZE					TXBUFFSIZE

typedef struct __attribute__((packed)){
	unsigned int PIN_0  	: 1;
	unsigned int PIN_1  	: 1;
	unsigned int PIN_2  	: 1;
	unsigned int PIN_3  	: 1;
	unsigned int PIN_4  	: 1;
	unsigned int PIN_5  	: 1;
	unsigned int PIN_6  	: 1;
	unsigned int PIN_7  	: 1;
}PINS;

typedef enum {
	DAC_WRITE_IMMEDIATELY	=	0,
	DAC_WRITE_SYNCHRONIZED	=	1,
	DAC_UPDATE				=	2,
}DAC_MODES;



typedef enum {
	NOP = 							0b00000000,
	DAC_READ_BACK = 				0b00001000,
	ADC_SEQ_REGISTER = 				0b00010000,
	DAC_ADC_CR = 					0b00011000,
	ADC_PIN_CFG = 					0b00100000,
	DAC_PIN_CFG = 					0b00101000,
	PULL_DOWN_CFG = 				0b00110000,
	READBACK_LDAC = 				0b00111000,
	GPIO_WRITE_CFG = 				0b01000000,
	GPIO_WRITE_DATA = 				0b01001000,
	GPIO_READ_CFG = 				0b01010000,
	PWR_REF_CFG = 					0b01011000,
	GPIO_OD_CFG = 					0b01100000,
	THREE_STATE_CFG = 				0b01101000,
	RES = 							0b01110000,
	SOFT_RESETHI = 					0b01111101,
	SOFT_RESETLO = 					0b10101100,

	DAC_WRITE = 					0b10000000,
}UZ_Commands;

extern uint8_t SPI_buff_tx[TXBUFFSIZE];
extern uint8_t SPI_buff_rx[RXBUFFSIZE];

void SetVRef(int UZ, int ReferenceVoltage) ;
void UZ_Close(int UZ) ;
int UZ_Start(int UZ, int SPI_Dev, int SPI_Port, int speed) ;
int UZ_Reset(int UZ) ;
int UZ_ADCBuffer(int UZ, int enable) ;
int UZ_ADCBufferPrecharge(int UZ, int enable) ;
int UZ_Lock(int UZ, int lock) ;
int UZ_AllDAC(int UZ, int alldac) ;
int UZ_DACSetGain(int UZ, int gain) ;
int UZ_ADCSetGain(int UZ, int gain) ;
int UZ_SetIntRef(int UZ, int intref) ;
int UZ_DACPowerDownAll(int UZ, int powerdown) ;
int UZ_GPIO_OutputCfg(int UZ, PINS *pins, int busy) ;
int UZ_GPIO_InputCfgRead(int UZ, PINS *pins, int readback) ;
int UZ_GPIO_InputCfg(int UZ, PINS *pins) ;
int UZ_GPIO_InputRead(int UZ, PINS *pins) ;
int UZ_GPIO_Write(int UZ, PINS *pins) ;
int UZ_DAC_Cfg(int UZ, PINS *pins) ;
int UZ_DAC_Write(int UZ, int pin, int value) ;
int UZ_ADC_Cfg(int UZ, PINS *pins) ;
int UZ_ADC_ReadStart(int UZ, PINS *pins, int *ADC_Results, int Repeat, int Temperature) ;
int UZ_ADC_Read(int UZ, PINS *pins, int *ADC_Results, int Temperature) ;
void ConvertResults(int UZ, int *ADCResults, int *ConvertedResults) ;
int UZ_DAC_Readback(int UZ, int DAC);
int UZ_DAC_Mode(int UZ, int DACmode) ;
int UZ_Readback(int UZ, int reg) ;
int UZ_ThreeState(int UZ, PINS *pins) ;
int UZ_PullDown(int UZ, PINS *pins) ;


#endif
