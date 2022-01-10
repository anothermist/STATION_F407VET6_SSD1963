#include "w25qxx.h"

extern SPI_HandleTypeDef W25QXX_SPI;

uint8_t rx_buf[1025];
uint8_t tx_buf[10];

void SPI1_Send (uint8_t *dt, uint16_t cnt)
{
	HAL_SPI_Transmit (&W25QXX_SPI, dt, cnt, 5000);
}

void SPI1_Recv (uint8_t *dt, uint16_t cnt)
{
	HAL_SPI_Receive (&W25QXX_SPI, dt, cnt, 5000);
}

void W25QXX_Reset (void)
{
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_RESET);
	tx_buf[0] = W25QXX_ENABLE_RESET;
	tx_buf[1] = W25QXX_RESET;
	SPI1_Send(tx_buf, 2);
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_SET);
}

void W25QXX_Read_Data(uint32_t addr, uint8_t* data, uint32_t sz)
{
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_RESET);
	tx_buf[0] = W25QXX_READ;
	tx_buf[1] = (addr >> 16) & 0xFF;
	tx_buf[2] = (addr >> 8) & 0xFF;
	tx_buf[3] = addr & 0xFF;
	SPI1_Send(tx_buf, 4);
	SPI1_Recv(data, sz);
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_SET);
}

uint32_t W25QXX_Read_ID(void)
{
	uint8_t dt[4];
	tx_buf[0] = W25QXX_GET_JEDEC_ID;
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_RESET);
	SPI1_Send(tx_buf, 1);
	SPI1_Recv(dt,3);
	HAL_GPIO_WritePin(W25QXX_NSS_PORT, W25QXX_NSS_PIN, GPIO_PIN_SET);
	return (dt[0] << 16) | (dt[1] << 8) | dt[2];
}

void W25QXX_Init(void)
{
	HAL_Delay(100);
	W25QXX_Reset();
	HAL_Delay(100);
}

