
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "UniversalZeroSPIlib.h"


#define SPI_DEV_THUMB						"/dev/spidev"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

inline char *SPIName(char *buff, int SPI_Dev, int SPI_Port) {
	int dvlen __attribute__((unused)) = strlen(SPI_DEV_THUMB) ;
	if(SPI_Dev < 0 && SPI_Dev > 9) return NULL;
	if(SPI_Port < 0 && SPI_Port > 9) return NULL;
	sprintf(buff,"/dev/spidev%01d.%01d", SPI_Dev, SPI_Port);
	return buff;
}

static struct {
	int SPI_Num;
	int SPI_Dev;
	char *device;
	uint32_t mode;
	uint8_t bits;
	uint32_t speed;
	uint16_t delay;
	int fd;
	char notfree;
}SPI_devices[MAX_SPI_DEV] = {	{.device = NULL, .mode = 0, .bits = 8, .speed = 1200000, .delay = 0, .fd = -1}, };


void SPI_close(int SPI_Num) {
	if(SPI_devices[SPI_Num].fd != -1) close(SPI_devices[SPI_Num].fd);
	if(SPI_devices[SPI_Num].device != NULL) free(SPI_devices[SPI_Num].device);
	SPI_devices[SPI_Num].notfree = 0;
	SPI_devices[SPI_Num].fd = -1;
}

int GetFreeSPI(void) {
	int i;

	for(i = 0; i < MAX_SPI_DEV; i++) {
		if(!SPI_devices[i].notfree) return i;
	}
	return -1;
}

int transfer(int SPI_num, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;

	if(SPI_num < 0 || SPI_num >= MAX_SPI_DEV) return -1; 					// wrong SPI device number (0 - MAX_SPI_DEV)
	if(SPI_devices[SPI_num].fd == -1) return -1;							// device not opened or device error

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = SPI_devices[SPI_num].delay,
		.speed_hz = SPI_devices[SPI_num].speed,
		.bits_per_word = SPI_devices[SPI_num].bits,
	};

	if (SPI_devices[SPI_num].mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (SPI_devices[SPI_num].mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (SPI_devices[SPI_num].mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (SPI_devices[SPI_num].mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(SPI_devices[SPI_num].mode & SPI_LOOP)) {
		if (SPI_devices[SPI_num].mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (SPI_devices[SPI_num].mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}

	ret = ioctl(SPI_devices[SPI_num].fd, SPI_IOC_MESSAGE(1), &tr);
	return ret;
}

int SPI_set(int SPI_Num, int SPI_Dev, int SPI_Port, uint32_t SPI_Speed, int CS_ActiveHigh, int SCK_IdleHigh, int SCK_Pol, uint32_t delay_us)
{
	int ret;

	if(SPI_devices[MAX_SPI_DEV - 1].speed == 0) {								//first time run
		for(ret = 1; ret < MAX_SPI_DEV; ret++)
			memcpy(&SPI_devices[ret], &SPI_devices[0], sizeof(SPI_devices[0]));
	}
	ret = 0;

	if(SPI_Num < 0 || SPI_Num >= MAX_SPI_DEV) return -1; 					// wrong SPI device number (0 - MAX_SPI_DEV)

	if(SPI_devices[SPI_Num].device == NULL) {
		SPI_devices[SPI_Num].device = malloc(0x20);
		SPIName(SPI_devices[SPI_Num].device, SPI_Dev, SPI_Port);
	}
	SPI_devices[SPI_Num].speed = SPI_Speed;
	SPI_devices[SPI_Num].delay = delay_us;

	if(CS_ActiveHigh) SPI_devices[SPI_Num].mode |= SPI_CS_HIGH;
	if(SCK_IdleHigh) SPI_devices[SPI_Num].mode |= SPI_CPHA;
	if(SCK_Pol) SPI_devices[SPI_Num].mode |= SPI_CPOL;

	if (SPI_devices[SPI_Num].fd < 0) {
		SPI_devices[SPI_Num].fd = open(SPI_devices[SPI_Num].device, O_RDWR);
		if(SPI_devices[SPI_Num].fd == -1) return -1;
	}
	SPI_devices[SPI_Num].notfree = 1;
	ret = ioctl(SPI_devices[SPI_Num].fd, SPI_IOC_WR_MODE32, &SPI_devices[SPI_Num].mode);
	if (ret == -1) return -1;
/*
	ret = ioctl(fd[SPI_Num], SPI_IOC_RD_MODE32, &mode[SPI_Num]);
	if (ret == -1) return -1;
*/
	ret = ioctl(SPI_devices[SPI_Num].fd, SPI_IOC_WR_BITS_PER_WORD, &SPI_devices[SPI_Num].bits);
	if (ret == -1) return -1;
/*
	ret = ioctl(fd[SPI_Num], SPI_IOC_RD_BITS_PER_WORD, &bits[SPI_Num]);
	if (ret == -1) return -1;
*/
	ret = ioctl(SPI_devices[SPI_Num].fd, SPI_IOC_WR_MAX_SPEED_HZ, &SPI_devices[SPI_Num].speed);
	if (ret == -1) return -1;
/*
	ret = ioctl(fd[SPI_Num], SPI_IOC_RD_MAX_SPEED_HZ, &speed[SPI_Num]);
	if (ret == -1) return -1;
*/
	SPI_devices[SPI_Num].notfree = 1;
	return 0;
}
