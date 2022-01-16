#include "w25q.h"

extern SPI_HandleTypeDef W25Q_SPI;

#if (INIT_DEBUG == 1)
extern UART_HandleTypeDef huart1;
#endif

w25_info_t  w25_info;
uint8_t buf[64] = {0};

void W25Q_Reset (void) {
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_ENABLE_RESET;
	buf[1] = W25Q_RESET;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 2, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Write_Enable(void) {
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_WRITE_ENABLE;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Write_Disable(void) {
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_WRITE_DISABLE;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Set_Block_Protect(uint8_t val) {
	buf[0] = 0x50;
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	buf[0] = W25Q_WRITE_STATUS_1;
	buf[1] = ((val & 0x0F) << 2);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit (&W25Q_SPI, buf, 2, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Wait_Write_End(void) {
	HAL_Delay(1);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_READ_STATUS_1;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	do {
		HAL_SPI_Receive (&W25Q_SPI, buf, 1, 1000);
		w25_info.StatusRegister1 = buf[0];
		HAL_Delay(1);
	}
	while((w25_info.StatusRegister1 & 0x01) == 0x01);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Erase_Sector(uint32_t addr) {
	W25Q_Wait_Write_End();
	W25Q_Set_Block_Protect(0x00);
	addr = addr * w25_info.SectorSize;
	W25Q_Write_Enable();
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_SECTOR_ERASE;
	if(w25_info.high_cap)
	{
		buf[1] = (addr >> 24) & 0xFF;
		buf[2] = (addr >> 16) & 0xFF;
		buf[3] = (addr >> 8) & 0xFF;
		buf[4] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 5, 1000);
	}
	else
	{
		buf[1] = (addr >> 16) & 0xFF;
		buf[2] = (addr >> 8) & 0xFF;
		buf[3] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 4, 1000);
	}
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	W25Q_Wait_Write_End();
	W25Q_Write_Disable();
	W25Q_Set_Block_Protect(0x0F);
}

void W25Q_Erase_Block(uint32_t addr) {
	W25Q_Wait_Write_End();
	W25Q_Set_Block_Protect(0x00);
	addr = addr * w25_info.BlockSize;
	W25Q_Write_Enable();
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_BLOCK_ERASE;
	if(w25_info.high_cap)
	{
		buf[1] = (addr >> 24) & 0xFF;
		buf[2] = (addr >> 16) & 0xFF;
		buf[3] = (addr >> 8) & 0xFF;
		buf[4] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 5, 1000);
	}
	else
	{
		buf[1] = (addr >> 16) & 0xFF;
		buf[2] = (addr >> 8) & 0xFF;
		buf[3] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 4, 1000);
	}
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	W25Q_Wait_Write_End();
	W25Q_Write_Disable();
	W25Q_Set_Block_Protect(0x0F);
}

void W25Q_Erase_Chip(void) {
	W25Q_Wait_Write_End();
	W25Q_Set_Block_Protect(0x00);
	W25Q_Write_Enable();
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_CHIP_ERASE;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	W25Q_Wait_Write_End();
	W25Q_Write_Disable();
	W25Q_Set_Block_Protect(0x0F);
}

void W25Q_Write_Data(uint32_t addr, uint8_t* data, uint32_t sz) {
	W25Q_Wait_Write_End();
	W25Q_Set_Block_Protect(0x00);
	W25Q_Write_Enable();
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_PAGE_PROGRAMM;
	if(w25_info.high_cap)
	{
		buf[1] = (addr >> 24) & 0xFF;
		buf[2] = (addr >> 16) & 0xFF;
		buf[3] = (addr >> 8) & 0xFF;
		buf[4] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 5, 1000);
	}
	else
	{
		buf[1] = (addr >> 16) & 0xFF;
		buf[2] = (addr >> 8) & 0xFF;
		buf[3] = addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 4, 1000);
	}
	HAL_SPI_Transmit (&W25Q_SPI, data, sz, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	W25Q_Wait_Write_End();
	W25Q_Write_Disable();
	W25Q_Set_Block_Protect(0x0F);
}

void W25Q_Write_Page(uint8_t* data, uint32_t page_addr, uint32_t offset, uint32_t sz) {
	if(sz > w25_info.PageSize)
		sz=w25_info.PageSize;
	if((offset+sz) > w25_info.PageSize)
		sz = w25_info.PageSize - offset;
	page_addr = page_addr * w25_info.PageSize + offset;

	W25Q_Wait_Write_End();
	W25Q_Set_Block_Protect(0x00);
	W25Q_Write_Enable();
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_PAGE_PROGRAMM;
	if(w25_info.high_cap)
	{
		buf[1] = (page_addr >> 24) & 0xFF;
		buf[2] = (page_addr >> 16) & 0xFF;
		buf[3] = (page_addr >> 8) & 0xFF;
		buf[4] = page_addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 5, 1000);
	}
	else
	{
		buf[1] = (page_addr >> 16) & 0xFF;
		buf[2] = (page_addr >> 8) & 0xFF;
		buf[3] = page_addr & 0xFF;
		HAL_SPI_Transmit (&W25Q_SPI, buf, 4, 1000);
	}
	HAL_SPI_Transmit (&W25Q_SPI, data, sz, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	W25Q_Wait_Write_End();
	W25Q_Write_Disable();
	W25Q_Set_Block_Protect(0x0F);
}

void W25Q_Read_Data(uint32_t addr, uint8_t* data, uint32_t sz) {
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	buf[0] = W25Q_READ;
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = addr & 0xFF;
	HAL_SPI_Transmit (&W25Q_SPI, buf, 4, 1000);
	HAL_SPI_Receive (&W25Q_SPI, data, sz, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

void W25Q_Read_Page(uint8_t* data, uint32_t page_addr, uint32_t offset, uint32_t sz) {
	if(sz > w25_info.PageSize)
		sz=w25_info.PageSize;
	if((offset+sz) > w25_info.PageSize)
		sz = w25_info.PageSize - offset;
	page_addr = page_addr * w25_info.PageSize + offset;
	buf[0] = W25Q_FAST_READ;
	if(w25_info.high_cap)
	{
		buf[1] = (page_addr >> 24) & 0xFF;
		buf[2] = (page_addr >> 16) & 0xFF;
		buf[3] = (page_addr >> 8) & 0xFF;
		buf[4] = page_addr & 0xFF;
		buf[5] = 0;
		HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit (&W25Q_SPI, buf, 6, 1000);
	}
	else
	{
		buf[1] = (page_addr >> 16) & 0xFF;
		buf[2] = (page_addr >> 8) & 0xFF;
		buf[3] = page_addr & 0xFF;
		buf[4] = 0;
		HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit (&W25Q_SPI, buf, 5, 1000);
	}
	HAL_SPI_Receive (&W25Q_SPI, data, sz, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
}

uint32_t W25Q_Read_ID(void) {
	uint8_t dt[4];
	buf[0] = W25Q_GET_JEDEC_ID;
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit (&W25Q_SPI, buf, 1, 1000);
	HAL_SPI_Receive (&W25Q_SPI, dt, 3, 1000);
	HAL_GPIO_WritePin(W25Q_NSS_PORT, W25Q_NSS_PIN, GPIO_PIN_SET);
	return (dt[0] << 16) | (dt[1] << 8) | dt[2];
}

void W25Q_Init(void) {
	unsigned int id = W25Q_Read_ID();
	HAL_Delay(100);
	W25Q_Reset();
	HAL_Delay(100);
	id &= 0x0000FFFF;
	w25_info.high_cap = 0;
	switch(id)
	{
	case 0x401A:
		w25_info.high_cap=1;
		w25_info.BlockCount=1024;
		break;
	case 0x4019:
		w25_info.high_cap=1;
		w25_info.BlockCount=512;
		break;
	case 0x4018:
		w25_info.BlockCount=256;
		break;
	case 0x4017:
		w25_info.BlockCount=128;
		break;
	case 0x4016:
		w25_info.BlockCount=64;
		break;
	case 0x4015:
		w25_info.BlockCount=32;
		break;
	case 0x4014:
		w25_info.BlockCount=16;
		break;
	case 0x4013:
		w25_info.BlockCount=8;
		break;
	case 0x4012:
		w25_info.BlockCount=4;
		break;
	case 0x4011:
		w25_info.BlockCount=2;
		break;
	default:
		return;
	}
	w25_info.PageSize=256;
	w25_info.SectorSize=0x1000;
	w25_info.SectorCount=w25_info.BlockCount*16;
	w25_info.PageCount=(w25_info.SectorCount*w25_info.SectorSize)/w25_info.PageSize;
	w25_info.BlockSize=w25_info.SectorSize*16;
	w25_info.NumKB=(w25_info.SectorCount*w25_info.SectorSize)/1024;

#if (INIT_DEBUG == 1)
	char str1[30];
	sprintf(str1,"FLASH ID: 0x%X \r\n",id);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1, strlen(str1), 0x1000);

	w25_info.high_cap = 0;

	switch(id)
	{
	case 0x401A:
		w25_info.BlockCount=1024;
		sprintf(str1,"FLASH CHIP: W25Q512 \r\n");
		break;
	case 0x4019:
		w25_info.BlockCount=512;
		sprintf(str1,"FLASH CHIP: W25Q256 \r\n");
		break;
	case 0x4018:
		w25_info.BlockCount=256;
		sprintf(str1,"FLASH CHIP: W25Q128 \r\n");
		break;
	case 0x4017:
		w25_info.BlockCount=128;
		sprintf(str1,"FLASH CHIP: W25Q64 \r\n");
		break;
	case 0x4016:
		w25_info.BlockCount=64;
		sprintf(str1,"FLASH CHIP: W25Q32 \r\n");
		break;
	case 0x4015:
		w25_info.BlockCount=32;
		sprintf(str1,"FLASH CHIP: W25Q16 \r\n");
		break;
	case 0x4014:
		w25_info.BlockCount=16;
		sprintf(str1,"FLASH CHIP: W25Q80 \r\n");
		break;
	case 0x4013:
		w25_info.BlockCount=8;
		sprintf(str1,"FLASH CHIP: W25Q40 \r\n");
		break;
	case 0x4012:
		w25_info.BlockCount=4;
		sprintf(str1,"FLASH CHIP: W25Q20 \r\n");
		break;
	case 0x4011:
		w25_info.BlockCount=2;
		sprintf(str1,"FLASH CHIP: W25Q10 \r\n");
		break;
	default:
		sprintf(str1,"FLASH CHIP: UNKNOWN ID \r\n");
		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
		break;
	}

	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	w25_info.PageSize=256;
	w25_info.SectorSize=0x1000;
	w25_info.SectorCount=w25_info.BlockCount*16;
	w25_info.PageCount=(w25_info.SectorCount*w25_info.SectorSize)/w25_info.PageSize;
	w25_info.BlockSize=w25_info.SectorSize*16;
	w25_info.NumKB=(w25_info.SectorCount*w25_info.SectorSize)/1024;
	sprintf(str1,"FLASH PAGE SIZE: %d Bytes \r\n",(unsigned int)w25_info.PageSize);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	sprintf(str1,"FLASH PAGE COUNT: %u \r\n",(unsigned int)w25_info.PageCount);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	sprintf(str1,"FLASH SECTOR SIZE: %u Bytes \r\n",(unsigned int)w25_info.SectorSize);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	sprintf(str1,"FLASH SECTOR COUNT: %u \r\n",(unsigned int)w25_info.SectorCount);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	sprintf(str1,"FLASH BLOCK SIZE: %u Bytes \r\n",(unsigned int)w25_info.BlockSize);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	sprintf(str1,"FLASH BLOCK COUNT: %u \r\n",(unsigned int)w25_info.BlockCount);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	sprintf(str1,"FLASH CAPACITY: %u KB \r\n",(unsigned int)w25_info.NumKB);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
#endif
}

void W25Q_Save_Page(uint32_t pagenum, uint8_t* data, uint32_t sz) {
	W25Q_Erase_Sector(pagenum);
	W25Q_Write_Data(pagenum * 4096, data, sz);
}

void W25Q_Load_Page(uint32_t pagenum, uint8_t* data, uint32_t sz) {
	W25Q_Read_Data(pagenum * 4096, data, sz);
}
