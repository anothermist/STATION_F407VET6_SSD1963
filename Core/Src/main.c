/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "string.h"
#include "ssd1963.h"
#include "xpt2046.h"
#include "bme280.h"
#include "ds3231.h"
#include "at24xx.h"
#include "fonts/DejaVu_Sans/008_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/009_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/010_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/011_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/012_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/014_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/016_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/018_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/020_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/022_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/024_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/026_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/028_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/036_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/048_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/072_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/096_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/112_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/128_DejaVu_Sans.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_TEMPERATURE_X10 226
#define MAX_TEMPERATURE_X10 290

#define MIN_HUMIDITY_X10 80
#define MAX_HUMIDITY_X10 720

#define MIN_PRESSURE 937
#define MAX_PRESSURE 1065

#define UART_RX_BUFFER_SIZE 16
#define UART_TX_BUFFER_SIZE 10000

#define WAIT_REMOTE_SENSOR_SEC 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
double map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t byteL(uint16_t val) {
	return (val & 0xFF);
}

uint8_t byteH(uint16_t val) {
	return ((val >> 8) & 0xFF);
}

uint16_t byteS(uint8_t byteL, uint8_t byteH) {
	return (byteH << 8) | byteL;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint8_t viewGraphs;
uint8_t rtcSec, rtcMin, rtcHrs, rtcDay, rtcDate, rtcMonth, rtcYear;
uint8_t rtcSecLast = 61, rtcMinLast = 61, rtcHrsLast = 25, rtcDayLast, rtcDateLast, rtcMonthLast, rtcYearLast;
double temperature, temperatureLast, humidity, humidityLast, temperatureRemote, temperatureRemoteLast, humidityRemote, humidityRemoteLast;
uint16_t pressure, pressureLast, remoteSensorLastUpdate = WAIT_REMOTE_SENSOR_SEC + 1;
int16_t hT[155], hH[155], hP[155];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void delay_us (uint16_t us)
//{
//	__HAL_TIM_SET_COUNTER(&htim2,0);
//	while (__HAL_TIM_GET_COUNTER(&htim2) < us);
//}

uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t rx_index = 0;
uint8_t rx_data;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		rx_buffer[rx_index++] = rx_data;
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
//		HAL_UART_Receive_DMA(&huart1, &rx_data, 1);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	XPT2046_Init();
	BME280_Init();

//	temperature = BME280_getTemperature(-1);
//	humidity = BME280_getHumidity(-1);
//	pressure = (uint16_t)BME280_getPressure();

	LCD_Rect_Fill(0, 0, 800, 480, BLUE);
	LCD_Rect_Fill(1, 1, 798, 478, BLACK);

	for (uint16_t i = 0; i < 155; i++) hT[i] = byteS(AT24XX_Read(i * 2 + 1000), AT24XX_Read(i * 2 + 1 + 1000));
	for (uint16_t i = 0; i < 155; i++) hH[i] = byteS(AT24XX_Read(i * 2 + 2000), AT24XX_Read(i * 2 + 1 + 2000));
	for (uint16_t i = 0; i < 155; i++) hP[i] = byteS(AT24XX_Read(i * 2 + 3000), AT24XX_Read(i * 2 + 1 + 3000));


//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	uint8_t uartTransmit[] = "UART OK\r\n";
	HAL_UART_Transmit(&huart1, uartTransmit, sizeof(uartTransmit), 100);

	uint8_t uartTransmit_IT[] = "UART INTERRUPT OK\r\n";
	HAL_UART_Transmit_IT(&huart1, uartTransmit_IT, sizeof(uartTransmit_IT));

//	uint8_t uartTransmit_DMA[] = "UART DMA OK\r\n";
//	HAL_UART_Transmit_DMA(&huart1, uartTransmit_DMA, sizeof(uartTransmit_DMA));

	HAL_UART_Receive_IT(&huart1, &rx_data, UART_RX_BUFFER_SIZE);
//	HAL_UART_Receive_DMA (&huart1, rx_buffer, UART_RX_BUFFER_SIZE);

//	for (uint32_t i = 0; i <= 65536; i++) TIM1->CCR1 = i;

//	DS3231_setHrs(10);
//	DS3231_setMin(15);
//	DS3231_setSec(0);
//	DS3231_setDate(07);
//	DS3231_setMonth(1);
//	DS3231_setYear(22);
//	DS3231_setDay(5);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) {

			uint16_t touchX = getX();
			uint16_t touchY = getY();
			if (touchX && touchY && touchX != 0x0DB)
			{
				LCD_Rect_Fill(touchX, touchY, 1, 1, WHITE);
			}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
			touchX = 0;
			touchY = 0;
		} else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);



		//		if (millis / 1000 % 2 == 0)
		////		else
		//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		rtcSec = DS3231_getSec();

		char clockPrint[13];

		if (rtcSecLast != rtcSec) {

			rtcMin = DS3231_getMin();

			LCD_Circle(170, 35, 8, 0, 1, ORANGE);
			LCD_Circle(170, 75, 8, 0, 1, ORANGE);

			if (rtcSec % 2 != 0) {
				LCD_Circle(170, 35, 7, 1, 1, ORANGE);
				LCD_Circle(170, 75, 7, 1, 1, ORANGE);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			}
			else {
				LCD_Circle(170, 35, 7, 1, 1, BLACK);
				LCD_Circle(170, 75, 7, 1, 1, BLACK);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}

			if (rtcMinLast != rtcMin) {

				rtcHrs = DS3231_getHrs();

				sprintf(clockPrint, "%02d", rtcMinLast);
				LCD_Font(178, 100, clockPrint, &DejaVu_Sans_128, 1, BLACK);
				sprintf(clockPrint, "%02d", rtcMin);
				LCD_Font(178, 100, clockPrint, &DejaVu_Sans_128, 1, ORANGE);

				if (rtcHrsLast != rtcHrs) {

					rtcDay = DS3231_getDay();
					rtcDate = DS3231_getDate();
					rtcMonth = DS3231_getMonth();
					rtcYear = DS3231_getYear();

					sprintf(clockPrint, "%02d", rtcHrsLast);
					LCD_Font(0, 100, clockPrint, &DejaVu_Sans_128, 1, BLACK);
					sprintf(clockPrint, "%02d", rtcHrs);
					LCD_Font(0, 100, clockPrint, &DejaVu_Sans_128, 1, ORANGE);

					if (rtcDayLast != rtcDay) {

						static const char* days[7] = { "MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN" };
						LCD_Font(5, 140, days[(7 + rtcDay - 2) % 7], &DejaVu_Sans_48, 1, BLACK);
						LCD_Font(5, 140, days[(7 + rtcDay - 1) % 7], &DejaVu_Sans_48, 1, BLUE);

						static const char* months[12] = { "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC" };

						LCD_Font(150, 140, months[(12 + rtcMonth - 2) % 12], &DejaVu_Sans_48, 1, BLACK);
						LCD_Font(150, 140, months[(12 + rtcMonth - 1) % 12], &DejaVu_Sans_48, 1, CYAN);

						sprintf(clockPrint, "%02d-%02d-%02d", rtcDateLast, rtcMonthLast, rtcYearLast);
						LCD_Font(250, 140, clockPrint, &DejaVu_Sans_48, 1, BLACK);
						sprintf(clockPrint, "%02d-%02d-%02d", rtcDate, rtcMonth, rtcYear);
						LCD_Font(250, 140, clockPrint, &DejaVu_Sans_48, 1, CYAN);

						rtcDayLast = rtcDay;
						rtcDateLast = rtcDate;
					}
					rtcMonthLast = rtcMonth;
					rtcYearLast = rtcYear;
					rtcHrsLast = rtcHrs;
				}
				rtcMinLast = rtcMin;
				temperature = BME280_getTemperature(-1);
				humidity = BME280_getHumidity(-1);
				pressure = (uint16_t)BME280_getPressure();

				if (pressure > 900 && pressure < 1100 && temperature < 85 && temperature > -40 && humidity > 0 && humidity < 100) {

					if (temperature != temperatureLast) {

						char weatherPrintT[8];

						if (temperatureLast >= 10 || (temperatureLast < 0 && temperatureLast > -10)) {
							sprintf(weatherPrintT, "%.1f'C", temperatureLast);
							LCD_Font(1, 187, weatherPrintT, &DejaVu_Sans_48, 1, BLACK);
						}
						else if (temperatureLast < 10 && temperatureLast > 0) {
							sprintf(weatherPrintT, "%.1f'C", temperatureLast);
							LCD_Font(27, 187, weatherPrintT, &DejaVu_Sans_48, 1, BLACK);
						}
						else if (temperatureLast <= -10) {
							sprintf(weatherPrintT, "%2d'C", (int8_t)temperatureLast);
							LCD_Font(1, 187, weatherPrintT, &DejaVu_Sans_48, 1, BLACK);
						}

						if (temperature >= 10 || (temperature < 0 && temperature > -10)) {
							sprintf(weatherPrintT, "%.1f'C", temperature);
							LCD_Font(1, 187, weatherPrintT, &DejaVu_Sans_48, 1, ORANGE);
						}
						else if (temperature < 10 && temperature > 0) {
							sprintf(weatherPrintT, "%.1f'C", temperature);
							LCD_Font(27, 187, weatherPrintT, &DejaVu_Sans_48, 1, ORANGE);
						}
						else if (temperature <= -10) {
							sprintf(weatherPrintT, "%2d'C", (int8_t)temperature);
							LCD_Font(1, 187, weatherPrintT, &DejaVu_Sans_48, 1, ORANGE);
						}

						temperatureLast = temperature;
					}

					if (humidity != humidityLast) {

						char weatherPrintH[7];

						sprintf(weatherPrintH, "%.1f'H", humidityLast);
						if (humidityLast >= 10)
							LCD_Font(160, 187, weatherPrintH, &DejaVu_Sans_48, 1, BLACK);
						else LCD_Font(186, 187, weatherPrintH, &DejaVu_Sans_48, 1, BLACK);

						sprintf(weatherPrintH, "%.1f'H", humidity);
						if (humidity >= 10)
							LCD_Font(160, 187, weatherPrintH, &DejaVu_Sans_48, 1, CYAN);
						else LCD_Font(186, 187, weatherPrintH, &DejaVu_Sans_48, 1, CYAN);

						humidityLast = humidity;
					}

					if (pressureLast != pressure) {

						char weatherPrintP[11];

						if (pressureLast >= 1000) sprintf(weatherPrintP, "%02dP", pressureLast);
						LCD_Font(320, 187, weatherPrintP, &DejaVu_Sans_48, 1, BLACK);

						if (pressureLast < 1000) sprintf(weatherPrintP, " %02dP", pressureLast);
						LCD_Font(320, 187, weatherPrintP, &DejaVu_Sans_48, 1, BLACK);

						pressureLast = pressure;

						if (pressureLast >= 1000) sprintf(weatherPrintP, "%02dP", pressureLast);
						LCD_Font(320, 187, weatherPrintP, &DejaVu_Sans_48, 1, GREEN);

						if (pressureLast < 1000) sprintf(weatherPrintP, " %02dP", pressureLast);
						LCD_Font(320, 187, weatherPrintP, &DejaVu_Sans_48, 1, GREEN);
					}

					if (AT24XX_Read(0) != rtcHrs) {

						AT24XX_Update(0, rtcHrs);

						for (uint16_t i = 0; i < 155; i++) hT[i] = byteS(AT24XX_Read(i * 2 + 1000), AT24XX_Read(i * 2 + 1 + 1000));
						for (uint16_t i = 154; i > 0; i--) hT[i] = hT[i - 1];
						hT[0] = (uint16_t) (temperature * 10);

						for (uint16_t i = 0; i < 155; i++) {
							AT24XX_Update(i * 2 + 1000, byteL(hT[i]));
							AT24XX_Update(i * 2 + 1 + 1000, byteH(hT[i]));
						}

						for (uint16_t i = 0; i < 155; i++) hH[i] = byteS(AT24XX_Read(i * 2 + 2000), AT24XX_Read(i * 2 + 1 + 2000));
						for (uint16_t i = 154; i > 0; i--) hH[i] = hH[i - 1];
						hH[0] = (uint16_t) (humidity * 10);

						for (uint16_t i = 0; i < 155; i++) {
							AT24XX_Update(i * 2 + 2000, byteL(hH[i]));
							AT24XX_Update(i * 2 + 1 + 2000, byteH(hH[i]));
						}

						for (uint16_t i = 0; i < 155; i++) hP[i] = byteS(AT24XX_Read(i * 2 + 3000), AT24XX_Read(i * 2 + 1 + 3000));
						for (uint16_t i = 154; i > 0; i--) hP[i] = hP[i - 1];
						hP[0] = (uint16_t)pressure;

						for (uint16_t i = 0; i < 155; i++) {
							AT24XX_Update(i * 2 + 3000, byteL(hP[i]));
							AT24XX_Update(i * 2 + 1 + 3000, byteH(hP[i]));
						}

						viewGraphs = 0;
					}

					LCD_Rect(2, 189, 157, 129, 1, BLUE);
					int16_t valMap = map(((int16_t)(temperature * 10)), MIN_TEMPERATURE_X10, MAX_TEMPERATURE_X10, 0, 128);
					if (valMap < 0) valMap = 0;
					if (valMap > 127) valMap = 127;
					LCD_Line(3 + 155, 191, 3 + 155, 317, 1, BLACK);
					if (valMap) LCD_Line(3 + 155, 191 + (127 - valMap), 3 + 155, 317,
							1, RGB(255 - ((127 - valMap) * 2), 0, 255 - (255 - ((127 - valMap) * 2))));

					LCD_Rect(161, 189, 157, 129, 1, BLUE);
					valMap = map(((int16_t)(humidity * 10)), MIN_HUMIDITY_X10, MAX_HUMIDITY_X10, 0, 128);
					if (valMap < 0) valMap = 0;
					if (valMap > 127) valMap = 127;
					LCD_Line(162 + 155, 191, 162 + 155, 317, 1, BLACK);
					if (valMap) LCD_Line(162 + 155, 191 + (127 - valMap), 162 + 155, 317,
							1, RGB(255 - ((127 - valMap) * 2), 0, 255 - (255 - ((127 - valMap) * 2))));

					LCD_Rect(320, 189, 157, 129, 1, BLUE);
					valMap = map(((int16_t)(pressure)), MIN_PRESSURE, MAX_PRESSURE, 0, 128);
					if (valMap < 0) valMap = 0;
					if (valMap > 127) valMap = 127;
					LCD_Line(321 + 155, 191, 321 + 155, 317, 1, BLACK);
					if (valMap) LCD_Line(321 + 155, 191 + (127 - valMap), 321 + 155, 317,
							1, RGB(255 - ((127 - valMap) * 2), 0, 255 - (255 - ((127 - valMap) * 2))));

					if (!viewGraphs) {

						for (uint16_t i = 0; i < 155 ; i++) {
							valMap = map(((int16_t)hT[i]), MIN_TEMPERATURE_X10, MAX_TEMPERATURE_X10, 0, 128);
							if (valMap < 0) valMap = 0;
							if (valMap > 127) valMap = 127;
							LCD_Line(3 + (154-i), 191, 3 + (154-i), 317, 1, BLACK);
							if (valMap) LCD_Line(3 + (154-i), 191 + (127 - valMap), 3 + (154-i), 317,
									1, RGB(255 - ((127 - valMap) * 2), 0, 255 - (255 - ((127 - valMap) * 2))));
						}

						for (uint16_t i = 0; i < 155 ; i++) {
							valMap = map(((int16_t)hH[i]), MIN_HUMIDITY_X10, MAX_HUMIDITY_X10, 0, 128);
							if (valMap < 0) valMap = 0;
							if (valMap > 127) valMap = 127;
							LCD_Line(162 + (154-i), 191, 162 + (154-i), 317, 1, BLACK);
							if (valMap) LCD_Line(162 + (154-i), 191 + (127 - valMap), 162 + (154-i), 317,
									1, RGB(255 - ((127 - valMap) * 2), 0, 255 - (255 - ((127 - valMap) * 2))));
						}

						for (uint16_t i = 0; i < 155 ; i++) {
							valMap = map(((int16_t)hP[i]), MIN_PRESSURE, MAX_PRESSURE, 0, 128);
							if (valMap < 0) valMap = 0;
							if (valMap > 127) valMap = 127;
							LCD_Line(321 + (154-i), 191, 321 + (154-i), 317, 1, BLACK);
							if (valMap) LCD_Line(321 + (154-i), 191 + (127 - valMap), 321 + (154-i), 317,
									1, RGB(255 - ((127 - valMap) * 2), 0, 255 - (255 - ((127 - valMap) * 2))));
						}
						for (uint32_t i = 0; i <= 65536; i++) TIM1->CCR1 = i;
						viewGraphs = 1;
					}
				}
			}

			if (rx_index != 0) {
				if (memcmp(rx_buffer, "TS", 2) == 0) {

					char val[2];

					val[0] = rx_buffer[2];
					val[1] = rx_buffer[3];
					DS3231_setHrs(atoi(val));

					val[0] = rx_buffer[4];
					val[1] = rx_buffer[5];
					DS3231_setMin(atoi(val));

					val[0] = 0;
					val[1] = 0;
					DS3231_setSec(atoi(val));

					val[0] = rx_buffer[6];
					val[1] = rx_buffer[7];
					DS3231_setDate(atoi(val));

					val[0] = rx_buffer[8];
					val[1] = rx_buffer[9];
					DS3231_setMonth(atoi(val));

					val[0] = rx_buffer[10];
					val[1] = rx_buffer[11];
					DS3231_setYear(atoi(val));

					val[1] = rx_buffer[12];
					DS3231_setDay(atoi(val));

					for (uint32_t i = 0; i <= 65536; i++) TIM1->CCR1 = i;
				}

				if (memcmp(rx_buffer, "CE", 2) == 0) {
					for (uint16_t i = 0; i < 4096; i++) AT24XX_Update(i, 0);
					uint8_t uartTransmit[] = "EEPROM IS CLEANED\r\n";
					HAL_UART_Transmit(&huart1, uartTransmit, sizeof(uartTransmit), 100);
				}

				if (memcmp(rx_buffer, "RS", 2) == 0) {

					char valT[4] = { 0 };

					for (uint8_t i = 0; i < 4; i++) valT[i] = rx_buffer[2 + i];

					temperatureRemote = atoi(valT);
					temperatureRemote = temperatureRemote / 10;

					char valH[3] = { 0 };

					for (uint8_t i = 0; i < 3; i++) valH[i] = rx_buffer[6 + i];

					humidityRemote = atoi(valH);
					humidityRemote = humidityRemote / 10;

					if ((temperatureRemote != temperatureRemoteLast && temperatureRemote < 85 && temperatureRemote > -40) ||
							(humidityRemote != humidityRemoteLast && humidityRemote > 0)) {

						remoteSensorLastUpdate = 0;

						sprintf(clockPrint, "%02d", rtcSecLast);
						LCD_Font(375, 40, clockPrint, &DejaVu_Sans_48, 1, BLACK);

						char weatherPrintRemoteT[5];
						if (temperatureRemoteLast <= -10) {
							sprintf(weatherPrintRemoteT, "%.1f", temperatureRemoteLast);
							LCD_Font(353, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, BLACK);
						}
						else if (temperatureRemoteLast < 0 && temperatureRemoteLast > -10) {
							sprintf(weatherPrintRemoteT, "%.1f", temperatureRemoteLast);
							LCD_Font(385, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, BLACK);
						}
						else if (temperatureRemoteLast > 0 && temperatureRemoteLast < 10) {
							sprintf(weatherPrintRemoteT, "+%.1f", temperatureRemoteLast);
							LCD_Font(362, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, BLACK);
						}
						else if (temperatureRemoteLast >= 10) {
							sprintf(weatherPrintRemoteT, "+%.1f", temperatureRemoteLast);
							LCD_Font(330, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, BLACK);
						}
						temperatureRemoteLast = temperatureRemote;
						if (temperatureRemoteLast <= -10) {
							sprintf(weatherPrintRemoteT, "%.1f", temperatureRemoteLast);
							LCD_Font(353, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, CYAN);
						}
						else if (temperatureRemoteLast < 0 && temperatureRemoteLast > -10) {
							sprintf(weatherPrintRemoteT, "%.1f", temperatureRemoteLast);
							LCD_Font(385, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, CYAN);
						}
						else if (temperatureRemoteLast > 0 && temperatureRemoteLast < 10) {
							sprintf(weatherPrintRemoteT, "+%.1f", temperatureRemoteLast);
							LCD_Font(362, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, CYAN);
						}
						else if (temperatureRemoteLast >= 10) {
							sprintf(weatherPrintRemoteT, "+%.1f", temperatureRemoteLast);
							LCD_Font(330, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, CYAN);
						}


						char weatherPrintRemoteH[4];

						if (humidityRemoteLast >= 10) {
							sprintf(weatherPrintRemoteH, "%.1f", humidityRemoteLast);
							LCD_Font(371, 40, weatherPrintRemoteH, &DejaVu_Sans_48, 1, BLACK);
						}
						else if (humidityRemoteLast < 10) {
							sprintf(weatherPrintRemoteH, "%.1f", humidityRemoteLast);
							LCD_Font(403, 40, weatherPrintRemoteH, &DejaVu_Sans_48, 1, BLACK);
						}

						humidityRemoteLast = humidityRemote;

						if (humidityRemoteLast >= 10) {
							sprintf(weatherPrintRemoteH, "%.1f", humidityRemoteLast);
							LCD_Font(371, 40, weatherPrintRemoteH, &DejaVu_Sans_48, 1, CYAN);
						}
						else if (humidityRemoteLast < 10) {
							sprintf(weatherPrintRemoteH, "%.1f", humidityRemoteLast);
							LCD_Font(403, 40, weatherPrintRemoteH, &DejaVu_Sans_48, 1, CYAN);
						}
					}
				}
				rx_index = 0;
				for (uint8_t i = 0; i < UART_RX_BUFFER_SIZE; i++) rx_buffer[i] = 0;
			}

			remoteSensorLastUpdate++;

			if (remoteSensorLastUpdate > WAIT_REMOTE_SENSOR_SEC) {

				if (temperatureRemoteLast && humidityRemoteLast) {

					char weatherPrintRemoteT[5];
					if (temperatureRemoteLast <= -10) {
						sprintf(weatherPrintRemoteT, "%.1f", temperatureRemoteLast);
						LCD_Font(353, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, BLACK);
					}
					else if (temperatureRemoteLast < 0 && temperatureRemoteLast > -10) {
						sprintf(weatherPrintRemoteT, "%.1f", temperatureRemoteLast);
						LCD_Font(385, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, BLACK);
					}
					else if (temperatureRemoteLast > 0 && temperatureRemoteLast < 10) {
						sprintf(weatherPrintRemoteT, "+%.1f", temperatureRemoteLast);
						LCD_Font(362, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, BLACK);
					}
					else if (temperatureRemoteLast >= 10) {
						sprintf(weatherPrintRemoteT, "+%.1f", temperatureRemoteLast);
						LCD_Font(330, 90, weatherPrintRemoteT, &DejaVu_Sans_48, 1, BLACK);
					}

					char weatherPrintRemoteH[4];

					if (humidityRemoteLast >= 10) {
						sprintf(weatherPrintRemoteH, "%.1f", humidityRemoteLast);
						LCD_Font(371, 40, weatherPrintRemoteH, &DejaVu_Sans_48, 1, BLACK);
					}
					else if (humidityRemoteLast < 10) {
						sprintf(weatherPrintRemoteH, "%.1f", humidityRemoteLast);
						LCD_Font(403, 40, weatherPrintRemoteH, &DejaVu_Sans_48, 1, BLACK);
					}

					temperatureRemoteLast = 0;
					humidityRemoteLast = 0;
				}

				sprintf(clockPrint, "%02d", rtcSecLast);
				LCD_Font(375, 40, clockPrint, &DejaVu_Sans_48, 1, BLACK);
				sprintf(clockPrint, "%02d", rtcSec);
				LCD_Font(375, 40, clockPrint, &DejaVu_Sans_48, 1, ORANGE);
			}
			rtcSecLast = rtcSec;
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SD_D0_Pin|SD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CMD_GPIO_Port, SD_CMD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY_1_Pin KEY_0_Pin */
  GPIO_InitStruct.Pin = KEY_1_Pin|KEY_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_A_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_A_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_IRQ_Pin */
  GPIO_InitStruct.Pin = TOUCH_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TOUCH_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_D0_Pin SD_D1_Pin */
  GPIO_InitStruct.Pin = SD_D0_Pin|SD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CMD_Pin */
  GPIO_InitStruct.Pin = SD_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CMD_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 2;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 5;
  Timing.BusTurnAroundDuration = 2;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
//{
//    micros++;
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

