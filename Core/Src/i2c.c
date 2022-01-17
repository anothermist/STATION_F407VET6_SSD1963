#include "i2c.h"

extern I2C_HandleTypeDef I2C;

#if (INIT_DEBUG == 1)
extern UART_HandleTypeDef &DEBUG_UART_I2C;
#endif

void I2C_Scan_Bus(I2C_HandleTypeDef *hi2c) {
    char info[] = "SCANNING I2C BUS... \r\n";
    HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)info, strlen(info), 1000);

    for(uint16_t i = 0; i < 128; i++) {
        if(HAL_I2C_IsDeviceReady(hi2c, i << 1, 1, 100) == HAL_OK)
        {
        	char msg[64] = {0,};
            snprintf(msg, 64, "I2C DEVICE: 0x%02X \r\n", i);
            HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)msg, strlen(msg), 1000);
        }
    }
}

void I2C_Error(char *er, uint32_t status) {
	char str[64] = {0,};

	switch(status) {
		case HAL_ERROR:
			snprintf(str, 64, "%s - HAL_ERROR \r\n", er);
			HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)str, strlen(str), 1000);
		break;

		case HAL_BUSY:
			snprintf(str, 64, "%s - HAL_BUSY \r\n", er);
			HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)str, strlen(str), 1000);
		break;

		case HAL_TIMEOUT:
			snprintf(str, 64, "%s - HAL_TIMEOUT \r\n", er);
			HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)str, strlen(str), 1000);
		break;

		default:
		break;
	}

	uint32_t err = HAL_I2C_GetError(&hi2c1);

	switch(err) {
		case HAL_I2C_ERROR_NONE:
			snprintf(str, 64, "HAL_I2C_ERROR_NONE \r\n");
			HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)str, strlen(str), 1000);
		break;

		case HAL_I2C_ERROR_BERR:
			snprintf(str, 64, "HAL_I2C_ERROR_BERR \r\n");
			HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)str, strlen(str), 1000);
		break;

		case HAL_I2C_ERROR_ARLO:
			snprintf(str, 64, "HAL_I2C_ERROR_ARLO \r\n");
			HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)str, strlen(str), 1000);
		break;

		case HAL_I2C_ERROR_AF:
			snprintf(str, 64, "HAL_I2C_ERROR_AF \r\n");
			HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)str, strlen(str), 1000);
		break;

		case HAL_I2C_ERROR_OVR:
			snprintf(str, 64, "HAL_I2C_ERROR_OVR \r\n");
			HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)str, strlen(str), 1000);
		break;

		case HAL_I2C_ERROR_DMA:
			snprintf(str, 64, "HAL_I2C_ERROR_DMA \r\n");
			HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)str, strlen(str), 1000);
		break;

		case HAL_I2C_ERROR_TIMEOUT:
			snprintf(str, 64, "HAL_I2C_ERROR_TIMEOUT \r\n");
			HAL_UART_Transmit(&DEBUG_UART_I2C, (uint8_t*)str, strlen(str), 1000);
		break;

		default:
		break;
	}

	while(1){};
}

static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout) {
    uint32_t Tickstart = HAL_GetTick();
    uint8_t ret = 1;

    for(;(state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret);) {
        if(timeout != HAL_MAX_DELAY) {
            if((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout)) ret = 0;
        }
        asm("nop");
    }
    return ret;
}

void I2C_Clear(I2C_HandleTypeDef *hi2c) {

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_PE);

    HAL_I2C_DeInit(hi2c);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = SCL_PIN;
    HAL_GPIO_Init(SCL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = SDA_PIN;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStructure);

    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(SCL_PORT, SCL_PIN, GPIO_PIN_SET, 1000);
    wait_for_gpio_state_timeout(SDA_PORT, SDA_PIN, GPIO_PIN_SET, 1000);

    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET);

    wait_for_gpio_state_timeout(SDA_PORT, SDA_PIN, GPIO_PIN_RESET, 1000);

    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);

    wait_for_gpio_state_timeout(SCL_PORT, SCL_PIN, GPIO_PIN_RESET, 1000);

    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(SCL_PORT, SCL_PIN, GPIO_PIN_SET, 1000);

    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(SDA_PORT, SDA_PIN, GPIO_PIN_SET, 1000);

    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;

    GPIO_InitStructure.Pin = SCL_PIN;
    HAL_GPIO_Init(SCL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = SDA_PIN;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStructure);

    SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    SET_BIT(hi2c->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    HAL_I2C_Init(hi2c);
}
