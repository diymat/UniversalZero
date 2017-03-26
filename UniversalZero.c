
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

#include "UniversalZeroSPIlib.h"
#include "UniversalZero.h"


static struct {
	int 	SPI_Num;
	int 	ResultData[9];
	uint8_t PWR_REF_Register[2];
	uint8_t GPCR_Register[2];
	uint8_t LDAC_Register[2];
	uint8_t InputPins;
	int 	Vref;
	int 	gain, ADCgain;
} UZ_Data[MAX_UZ_HATS] = {	{.PWR_REF_Register = {PWR_REF_CFG, 0}, .GPCR_Register = {DAC_ADC_CR, 0}, .LDAC_Register = {READBACK_LDAC, 0}, .SPI_Num = -1, .Vref = 3300, .gain = 1, .ADCgain = 1},};

uint8_t SPI_buff_tx[TXBUFFSIZE] = {0,};
uint8_t SPI_buff_rx[RXBUFFSIZE] = {0, };

static inline int inarray(int *haystack, int needle) {
	int i;

	for(i = 0; i < 9; i++)
		if(needle == haystack[i]) return 1;
	return 0;
}

void SetVRef(int UZ, int ReferenceVoltage) {
	UZ_Data[UZ].Vref = ReferenceVoltage;
}

void UZ_Close(int UZ) {
	SPI_close(UZ_Data[UZ].SPI_Num);
}

int UZ_Start(int UZ, int SPI_Dev, int SPI_Port, int speed) {
	int SPI;

	if(UZ_Data[MAX_UZ_HATS - 1].Vref == 0) {								//first run
		for(SPI = 1; SPI < MAX_UZ_HATS; SPI++)
			memcpy(&UZ_Data[SPI], &UZ_Data[0], sizeof(UZ_Data[0]));
	}
	if(UZ_Data[UZ].SPI_Num != -1) return -1;
	SPI = GetFreeSPI();
	if(SPI == -1) return -1;
	if(SPI_set(SPI, SPI_Dev, SPI_Port, speed, 0,0,1,0) == -1) return -1;
	UZ_Data[UZ].SPI_Num = SPI;
	return 0;
}

int UZ_Reset(int UZ) {
	SPI_buff_tx[0] = SOFT_RESETHI;
	SPI_buff_tx[1] = SOFT_RESETLO;
	return transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2);
}

int UZ_ADCBuffer(int UZ, int enable) {
	if(enable) UZ_Data[UZ].GPCR_Register[0] |= (1 << GPCR_ADC_BUFFER_ENABLE);
		else UZ_Data[UZ].GPCR_Register[0] &= ~(1 << GPCR_ADC_BUFFER_ENABLE);
	SPI_buff_tx[0] = UZ_Data[UZ].GPCR_Register[0];
	SPI_buff_tx[1] = UZ_Data[UZ].GPCR_Register[1];
	if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
	return 0;
}

int UZ_ADCBufferPrecharge(int UZ, int enable) {
	if(enable) UZ_Data[UZ].GPCR_Register[0] |= (1 << GPCR_ADC_BUFFER_PRECHARGE);
		else UZ_Data[UZ].GPCR_Register[0] &= ~(1 << GPCR_ADC_BUFFER_PRECHARGE);
	SPI_buff_tx[0] = UZ_Data[UZ].GPCR_Register[0];
	SPI_buff_tx[1] = UZ_Data[UZ].GPCR_Register[1];
	if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
	return 0;
}

int UZ_Lock(int UZ, int lock) {
	if(lock) UZ_Data[UZ].GPCR_Register[1] |= (1 << GPCR_LOCK);
		else UZ_Data[UZ].GPCR_Register[1] &= ~(1 << GPCR_LOCK);
	SPI_buff_tx[0] = UZ_Data[UZ].GPCR_Register[0];
	SPI_buff_tx[1] = UZ_Data[UZ].GPCR_Register[1];
	if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
	return 0;
}

int UZ_AllDAC(int UZ, int alldac) {
	if(alldac) UZ_Data[UZ].GPCR_Register[1] |= (1 << GPCR_ALL_DACS);
		else UZ_Data[UZ].GPCR_Register[1] &= ~(1 << GPCR_ALL_DACS);
	SPI_buff_tx[0] = UZ_Data[UZ].GPCR_Register[0];
	SPI_buff_tx[1] = UZ_Data[UZ].GPCR_Register[1];
	if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
	return 0;
}


int UZ_DACSetGain(int UZ, int gain) {
	if(gain == UZ_Data[UZ].gain) return 0;											// no change
	if(gain == 2) UZ_Data[UZ].GPCR_Register[1] |= (1 << GPCR_DAC_GAIN);
		else if (gain == 1) UZ_Data[UZ].GPCR_Register[1] &= ~(1 << GPCR_DAC_GAIN);
				else return 0;
	SPI_buff_tx[0] = UZ_Data[UZ].GPCR_Register[0];
	SPI_buff_tx[1] = UZ_Data[UZ].GPCR_Register[1];
	if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
	UZ_Data[UZ].gain = gain;
	return 0;
}

int UZ_ADCSetGain(int UZ, int gain) {
	if(gain == UZ_Data[UZ].ADCgain) return 0;											// no change
	if(gain == 2) UZ_Data[UZ].PWR_REF_Register[0] |= (1 << GPCR_ADC_GAIN);
		else if (gain == 1) UZ_Data[UZ].PWR_REF_Register[0] &= ~(1 << GPCR_ADC_GAIN);
				else return -1;
	SPI_buff_tx[0] = UZ_Data[UZ].PWR_REF_Register[0];
	SPI_buff_tx[1] = UZ_Data[UZ].PWR_REF_Register[1];
	if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
	UZ_Data[UZ].ADCgain = gain;
	return 0;
}


int UZ_SetIntRef(int UZ, int intref) {
	switch(intref) {
		case 1: 				//disable internal voltage reference
			UZ_Data[UZ].PWR_REF_Register[0] |= (1 << PWR_REF_INTREF_ENABLE);
			SetVRef(UZ, INTREF);
			break;
		case 0:
			UZ_Data[UZ].PWR_REF_Register[0] &= ~(1 << PWR_REF_INTREF_ENABLE);
			SetVRef(UZ, AREF);
			break;
		default:
			return -1;
			break;
	}
	SPI_buff_tx[0] = UZ_Data[UZ].PWR_REF_Register[0];
	SPI_buff_tx[1] = UZ_Data[UZ].PWR_REF_Register[1];
	if (transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
	return 0;
}

int UZ_DACPowerDownAll(int UZ, int powerdown) {
	if (powerdown) UZ_Data[UZ].PWR_REF_Register[0] |= (1 << PWR_REF_POWER_DOWN_ALL);
		else UZ_Data[UZ].PWR_REF_Register[0] &= ~(1 << PWR_REF_POWER_DOWN_ALL);
	SPI_buff_tx[0] = UZ_Data[UZ].PWR_REF_Register[0];
	SPI_buff_tx[1] = UZ_Data[UZ].PWR_REF_Register[1];
	if (transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
	return 0;
}

int UZ_GPIO_OutputCfg(int UZ, PINS *pins, int busy) {
	uint8_t *pindata = (uint8_t *)pins;

	SPI_buff_tx[0] = GPIO_WRITE_CFG + !!busy;
	SPI_buff_tx[1] = *pindata;
	return transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2);
}

int UZ_GPIO_InputCfgRead(int UZ, PINS *pins, int readback) {
	uint8_t *pindata = (uint8_t *)pins;

	SPI_buff_tx[0] = GPIO_READ_CFG | ((!!readback) << 2);
	SPI_buff_tx[1] = *pindata;

	if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
	if(readback) {
		SPI_buff_tx[0] = SPI_buff_tx[1] = 0;
		if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
		*pindata = SPI_buff_rx[1];
		}
	return 0;
}

int UZ_GPIO_InputCfg(int UZ, PINS *pins) {
	return UZ_GPIO_InputCfgRead(UZ, pins, 0);
}

int UZ_GPIO_InputRead(int UZ, PINS *pins) {
	return UZ_GPIO_InputCfgRead(UZ, pins, 1);
}

int UZ_GPIO_Write(int UZ, PINS *pins) {
	uint8_t *pindata = (uint8_t *)pins;

	SPI_buff_tx[0] = GPIO_WRITE_DATA;
	SPI_buff_tx[1] = *pindata;

	return transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2);
}

int UZ_DAC_Cfg(int UZ, PINS *pins) {
	uint8_t *data = (uint8_t *)pins;

	SPI_buff_tx[0] = DAC_PIN_CFG;
	SPI_buff_tx[1] = *data;

	return transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2);
}


int UZ_DAC_Write(int UZ, int pin, int value) {

	SPI_buff_tx[0] = DAC_WRITE | (pin << 4) | ((value & 0xf00) >> 8);
	SPI_buff_tx[1] = value & 0xff;
	return transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2);
}

int UZ_ADC_Cfg(int UZ, PINS *pins) {

	uint8_t *data = (uint8_t *)pins;

	SPI_buff_tx[0] = ADC_PIN_CFG;
	SPI_buff_tx[1] = *data;

	return transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2);
}

int UZ_ADC_ReadStart(int UZ, PINS *pins, int *ADC_Results, int Repeat, int Temperature) {
	uint8_t *data = (uint8_t *)pins;

	int NumberOfConversions, index;
	int ConversionTable[9] = {0xffff,};
	int port;
	int found = 0;

	NumberOfConversions = 0;
	if(*data) {
		for(index = 0; index < 8; index ++) {
			if(*data & (1 << index)) {
				ConversionTable[index] = index;
				NumberOfConversions++;
			}
			else
				ConversionTable[index] = 0;
		}
		//if(NumberOfConversions) NumberOfConversions++; 			// If there is anything to convert add a dummy one (as stated in the datasheet)


		SPI_buff_tx[0] = ADC_SEQ_REGISTER;
		SPI_buff_tx[1] = *data;
		if(Temperature) {
			SPI_buff_tx[0] |= (1 << 0);
			NumberOfConversions++;
			ConversionTable[8] = 8;
		}
		if(Repeat) SPI_buff_tx[0] |= (1 << 1);

		if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
		SPI_buff_tx[0] = NOP;
		SPI_buff_tx[1] = NOP;
		for(index = 0; index < NumberOfConversions; index += found) {
			if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
			port = (SPI_buff_rx[0] & 0xf0) >> 4;
			if((found = inarray(ConversionTable, port))) {
				ADC_Results[port] = ((SPI_buff_rx[0] & 0x0f) << 8) + SPI_buff_rx[1];
				UZ_Data[UZ].ResultData[port] = ADC_Results[port];
			}
		}
	}
	return 0;
}

int UZ_ADC_Read(int UZ, PINS *pins, int *ADC_Results, int Temperature) {
	uint8_t *data = (uint8_t *)pins;

	int NumberOfConversions, index;
	int ConversionTable[9] = {0xffff,};
	int port;
	int found = 0;

	NumberOfConversions = 0;
	if(*data) {
		for(index = 0; index < 8; index ++) {
			if(*data & (1 << index)) {
				ConversionTable[index] = index;
				NumberOfConversions++;
			}
			else
				ConversionTable[index] = 0;
		}
		//if(NumberOfConversions) NumberOfConversions++; 			// If there is anything to convert add a dummy one (as stated in the datasheet)


		SPI_buff_tx[0] = ADC_SEQ_REGISTER;
		SPI_buff_tx[1] = *data;
		if(Temperature) {
			NumberOfConversions++;
			ConversionTable[8] = 8;
		}

		SPI_buff_tx[0] = NOP;
		SPI_buff_tx[1] = NOP;
		for(index = 0; index < NumberOfConversions; index += found) {
			if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
			port = (SPI_buff_rx[0] & 0xf0) >> 4;
			if((found = inarray(ConversionTable, port))) {
				ADC_Results[port] = ((SPI_buff_rx[0] & 0x0f) << 8) + SPI_buff_rx[1];
				UZ_Data[UZ].ResultData[port] = ADC_Results[port];
			}
		}
	}
	return 0;
}

int UZ_DAC_Readback(int UZ, int DAC) {
	uint16_t *result = (uint16_t *)SPI_buff_rx;

	DAC &= 0b111;
	SPI_buff_tx[0] = DAC_READ_BACK;
	SPI_buff_tx[1] = (DAC_READBACK_ENABLE << DAC_READBACK_ENABLE_SHIFT) | DAC;
	do {
		if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
	}while((SPI_buff_rx[0] & 0b111) >> 4 != DAC);
	return *result & 0b0000111111111111;
}

int UZ_DAC_Mode(int UZ, int DACmode) {
	DACmode &= 0b11;
	SPI_buff_tx[0] = READBACK_LDAC;
	SPI_buff_tx[1] = DACmode;
	return transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2);
}

int UZ_DAC_Update(int UZ) {
	return UZ_DAC_Mode(UZ, DAC_UPDATE);
}

int UZ_Readback(int UZ, int reg) {
	uint16_t *result = (uint16_t *)SPI_buff_rx;

	reg &= 0b1111;
	SPI_buff_tx[0] = READBACK_LDAC;
	SPI_buff_tx[1] = (reg << LDAC_READBACK_REGISTER) | (1 << LDAC_READBACK_ENABLE);
	if(transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2) == -1) return -1;
	return *result;
}

int UZ_ThreeState(int UZ, PINS *pins) {
	uint8_t *bpins = (uint8_t *)pins;

	SPI_buff_tx[0] = THREE_STATE_CFG;
	SPI_buff_tx[1] = *bpins;
	return transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2);
}

int UZ_PullDown(int UZ, PINS *pins) {
	uint8_t *bpins = (uint8_t *)pins;

	SPI_buff_tx[0] = PULL_DOWN_CFG;
	SPI_buff_tx[1] = *bpins;
	return transfer(UZ_Data[UZ].SPI_Num, SPI_buff_tx, SPI_buff_rx,2);
}


void ConvertResults(int UZ, int *ADCResults, int *ConvertedResults) {
	int index;

	for(index = 0; index < 8; index++) {
		ConvertedResults[index] = ((ADCResults[index] * (UZ_Data[UZ].Vref * UZ_Data[UZ].ADCgain)) / (1 << 12));
	}
	if(ADCResults[8] != 0xffff) ConvertedResults[8] = 2500 - ((ADCResults[8] - (UZ_Data[UZ].ADCgain == 2 ? 410 : 820)) * 1000) / 2654 ;
}
