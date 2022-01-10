#include "w25qxx.h"

extern SPI_HandleTypeDef W25QXX_SPI;

uint8_t rx_buf[1025];
uint8_t tx_buf[10];

void W25QXX_Reset (void)
{
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_RESET);
	tx_buf[0] = W25QXX_ENABLE_RESET;
	tx_buf[1] = W25QXX_RESET;
	HAL_SPI_Transmit (&W25QXX_SPI, tx_buf, 2, 1000);
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_SET);
}

void W25QXX_Read_Data(uint32_t addr, uint8_t* data, uint32_t sz)
{
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_RESET);
	tx_buf[0] = W25QXX_READ;
	tx_buf[1] = (addr >> 16) & 0xFF;
	tx_buf[2] = (addr >> 8) & 0xFF;
	tx_buf[3] = addr & 0xFF;
	HAL_SPI_Transmit (&W25QXX_SPI, tx_buf, 4, 1000);
	HAL_SPI_Receive (&W25QXX_SPI, data, sz, 1000);
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_SET);
}

uint32_t W25QXX_Read_ID(void)
{
	uint8_t dt[4];
	tx_buf[0] = W25QXX_GET_JEDEC_ID;
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit (&W25QXX_SPI, tx_buf, 1, 1000);
	HAL_SPI_Receive (&W25QXX_SPI, dt, 3, 1000);
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_SET);
	return (dt[0] << 16) | (dt[1] << 8) | dt[2];
}

void W25QXX_Init(void)
{
	HAL_Delay(100);
	W25QXX_Reset();
	HAL_Delay(100);
}

