
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>


#include "UniversalZeroSPIlib.h"
#include "UniversalZero.h"

#define NSAMPLES	32

char *strtolower(char *str){
    char *ptr = str;

    do{
    *ptr = tolower(*ptr);
    } while (*++ptr);
    return str;
}

int main(int argc, char *argv[])
{


	PINS ADCpins = {0,}, DACpins = {0,}, GPIO_OUTpins = {0, }, GPIO_INpins = {0, };
	int ADCResults[9], index;
	uint16_t Voltage = 0;

	if(UZ_Start(0,0,0,500000UL) == -1) {
		fprintf(stderr, "Error opening SPI port\n\r");
		return -1;
	}

	UZ_Reset(0);
    ADCpins.PIN_3 = 1;
    ADCpins.PIN_5 = 1;
    ADCpins.PIN_7 = 1;
    UZ_ADC_Cfg(0, &ADCpins);

    UZ_SetIntRef(0,1);
    DACpins.PIN_6 = 1;
    DACpins.PIN_4 = 1;
    UZ_DAC_Cfg(0, &DACpins);

	GPIO_OUTpins.PIN_0 = 1;
	GPIO_OUTpins.PIN_2 = 1;
	UZ_GPIO_OutputCfg(0, &GPIO_OUTpins, 0);

	GPIO_INpins.PIN_1 = 1;
	UZ_GPIO_InputCfgRead(0, &GPIO_INpins, 0);

    if(argc == 1)
        while(1) {
            UZ_DAC_Write(0,4, (40950 - Voltage++) / 10);
            Voltage %= 40950;
            UZ_ADC_ReadStart(0, &ADCpins, ADCResults, 0, 1);
            ConvertResults(0, ADCResults, ADCResults);
            for(index =  0; index < 9; index++) printf("%04d ", ADCResults[index]);
            printf("\r");
            Voltage++;
        }


    while(1) {
        UZ_DAC_Write(0,4,(Voltage & 1) * 4095);
        UZ_DAC_Write(0,6,(int)(2047 * (sin((2 * M_PI * (Voltage++)) / NSAMPLES) + 1)));
        Voltage %= NSAMPLES;
    }
}
