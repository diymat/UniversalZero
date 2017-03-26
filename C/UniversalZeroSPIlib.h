#ifndef __SPILIB__
#define __SPILIB__

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

#define MAX_SPI_DEV					4

int transfer(int SPI_dev, uint8_t const *tx, uint8_t const *rx, size_t len);
int GetFreeSPI(void);

int SPI_set(int SPI_Num, int SPI_Dev, int SPI_Port, uint32_t SPI_Speed, int CS_ActiveHigh, int SCK_IdleHigh, int SCK_Pol, uint32_t delay_us);
void SPI_close(int SPI_Num);

#endif
