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
#define DWT_CYCCNT *(volatile unsigned long *)0xE0001004
#define DWT_CONTROL *(volatile unsigned long *)0xE0001000
#define SCB_DEMCR *(volatile unsigned long *)0xE000EDFC

#include <stdio.h>
#include <math.h>
#include "string.h"
#include "ssd1963.h"
#include "xpt2046.h"
#include "i2c.h"
#include "bme280.h"
#include "ds3231.h"
#include "at24.h"
#include "w25q.h"
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
#define MIN_TEMPERATURE_X10 200
#define MAX_TEMPERATURE_X10 328

#define MIN_HUMIDITY_X10 100
#define MAX_HUMIDITY_X10 868

#define MIN_PRESSURE_X10  9300
#define MAX_PRESSURE_X10 10580

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
DMA_HandleTypeDef hdma_dac1;
DMA_HandleTypeDef hdma_dac2;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

RTC_TimeTypeDef clockTime;
RTC_DateTypeDef clockDate;

uint8_t viewGraphs;
uint8_t rtcSec, rtcMin, rtcHrs, rtcDate, rtcMonth, rtcWeekD, rtcYear;
uint8_t rtcSecLast = 61, rtcMinLast = 61, rtcHrsLast = 25, rtcWeekDLast, rtcDateLast, rtcMonthLast, rtcYearLast;
double temperature, temperatureLast, humidity, humidityLast, temperatureRemote, temperatureRemoteLast, humidityRemote, humidityRemoteLast;
uint16_t pressure, pressureLast, remoteSensorLastUpdate = WAIT_REMOTE_SENSOR_SEC + 1;
int16_t hT[500], hH[500], hP[500];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI2_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float value = 0.2;

uint32_t var;

uint32_t sine_val[100];

#define PI 3.1415926

void get_sineval () {
	for (int i = 0; i<100; i++) sine_val[i] = ((sin(i*2*PI/100) + 1)* (4096/2));
}

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
	//	__HAL_RCC_I2C1_CLK_ENABLE();
	//	HAL_Delay(100);
	//	__HAL_RCC_I2C1_FORCE_RESET();
	//	HAL_Delay(100);
	//	__HAL_RCC_I2C1_RELEASE_RESET();
	//	HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_SPI2_Init();
  MX_DAC_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim6);
    HAL_TIM_Base_Start(&htim7);
    get_sineval();
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_val, 100, DAC_ALIGN_12B_R);
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, sine_val, 100, DAC_ALIGN_12B_R);
    HAL_Delay(2000);
    HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Stop(&hdac, DAC_CHANNEL_2);

	HAL_RTC_Init(&hrtc);
	uint8_t uart_tx_new[] = "\r\n";
	HAL_UART_Transmit(&huart1, uart_tx_new, sizeof(uart_tx_new), 100);
	I2C_Init(&hi2c1);
	I2C_Scan_Bus(&hi2c1);
	HAL_UART_Transmit(&huart1, uart_tx_new, sizeof(uart_tx_new), 100);
	W25Q_Init();
	HAL_UART_Transmit(&huart1, uart_tx_new, sizeof(uart_tx_new), 100);
	BME280_Init();
	LCD_Init();
	XPT2046_Init();

	DS3231_Update(); rtcSec = DS3231_getSec(); rtcMin = DS3231_getMin(); rtcHrs = DS3231_getHrs();
	rtcDate = DS3231_getDate(); rtcMonth = DS3231_getMonth(); rtcYear = DS3231_getYear(); rtcWeekD = DS3231_getWeekDay();

	HAL_RTC_GetTime(&hrtc, &clockTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &clockDate, RTC_FORMAT_BIN);

	if (!clockDate.Year) {
	clockTime.Hours = rtcHrs;
	clockTime.Minutes = rtcMin;
	clockTime.Seconds = rtcSec +1;
	HAL_RTC_SetTime(&hrtc, &clockTime, RTC_FORMAT_BIN);
	clockDate.Date = rtcDate;
	clockDate.Month = rtcMonth;
	clockDate.Year = rtcYear;
	clockDate.WeekDay = rtcWeekD;
	HAL_RTC_SetDate(&hrtc, &clockDate, RTC_FORMAT_BIN);
	}

	char uart_tx[45];
	static const char* weekdays[7] = { "MO", "TU", "WE", "TH", "FR", "SA", "SU" };
	HAL_RTC_GetTime(&hrtc, &clockTime, RTC_FORMAT_BIN);
	snprintf(uart_tx, 45, "INTRTC Time: %02d:%02d:%02d ", clockTime.Hours, clockTime.Minutes, clockTime.Seconds);
	HAL_RTC_GetDate(&hrtc, &clockDate, RTC_FORMAT_BIN);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx, strlen(uart_tx), 100);
	snprintf(uart_tx, 45, "Date: %02d.%02d.20%02d ", clockDate.Date, clockDate.Month, clockDate.Year);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx, strlen(uart_tx), 100);
	snprintf(uart_tx, 45, "%s \r\n", weekdays[(7 + clockDate.WeekDay - 1) % 7]);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx, strlen(uart_tx), 100);

	if (!rtcYear) {
	DS3231_Update();
	DS3231_setHrs(clockTime.Hours);
	DS3231_setMin(clockTime.Minutes);
	DS3231_setSec(clockTime.Seconds);
	DS3231_setDate(clockDate.Date);
	DS3231_setMonth(clockDate.Month);
	DS3231_setYear(clockDate.Year);
	DS3231_setWeekDay(clockDate.WeekDay);
	}

	DS3231_Update(); rtcSec = DS3231_getSec(); rtcMin = DS3231_getMin(); rtcHrs = DS3231_getHrs();
	rtcDate = DS3231_getDate(); rtcMonth = DS3231_getMonth(); rtcYear = DS3231_getYear(); rtcWeekD = DS3231_getWeekDay();

	snprintf(uart_tx, 45, "DS3231 Time: %02d:%02d:%02d ", rtcHrs, rtcMin, rtcSec);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx, strlen(uart_tx), 100);
	snprintf(uart_tx, 45, "Date: %02d.%02d.20%02d ", rtcDate, rtcMonth, rtcYear);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx, strlen(uart_tx), 100);
	snprintf(uart_tx, 45, "%s \r\n", weekdays[(7 + rtcWeekD - 1) % 7]);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx, strlen(uart_tx), 100);

	temperature = BME280_getTemperature(-1);
	humidity = BME280_getHumidity(-1);
	pressure = (uint16_t)BME280_getPressure();
	snprintf(uart_tx, 45, "BME280 T: %.1f 'C | H: %.1f %% | P: %04d HPa \r\n", temperature, humidity, pressure);
	HAL_UART_Transmit(&huart1, uart_tx_new, sizeof(uart_tx_new), 100);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx, strlen(uart_tx), 100);
	HAL_UART_Transmit(&huart1, uart_tx_new, sizeof(uart_tx_new), 100);

	LCD_Rect_Fill(0, 0, 800, 480, BLUE);
	LCD_Rect_Fill(1, 1, 798, 478, BLACK);

	//	W25Q_Erase_Chip;
	//	for (uint16_t i = 0; i < 4096; i++) AT24XX_Update(i, 0);

	//	uint8_t flashIN[] = "W25Q IS OK \r\n";
	//	W25Q_Save_Page(15, flashIN, 10);

	//	uint8_t flashOUT[10] = {0};
	//	W25Q_Load_Page(15, flashOUT, 10);
	//	HAL_UART_Transmit(&huart1, flashOUT, sizeof(flashOUT), 100);

	for (uint16_t i = 0; i < 499; i++) hT[i] =  byteS(AT24XX_Read(i * 2 + 1000), AT24XX_Read(i * 2 + 1 + 1000));
	for (uint16_t i = 0; i < 499; i++) hH[i] =  byteS(AT24XX_Read(i * 2 + 2000), AT24XX_Read(i * 2 + 1 + 2000));
	for (uint16_t i = 0; i < 499; i++) hP[i] =  byteS(AT24XX_Read(i * 2 + 3000), AT24XX_Read(i * 2 + 1 + 3000));

	//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	//	uint8_t uartTransmit[] = "UART OK\r\n";
	//	HAL_UART_Transmit(&huart1, uartTransmit, sizeof(uartTransmit), 100);

	//	uint8_t uartTransmit_IT[] = "UART INTERRUPT OK\r\n";
	//	HAL_UART_Transmit_IT(&huart1, uartTransmit_IT, sizeof(uartTransmit_IT));
	//
	//	uint8_t uartTransmit_DMA[] = "UART DMA OK\r\n";
	//	HAL_UART_Transmit_DMA(&huart1, uartTransmit_DMA, sizeof(uartTransmit_DMA));
	//
	//	HAL_UART_Receive_IT(&huart1, &rx_data, UART_RX_BUFFER_SIZE);
	//	HAL_UART_Receive_DMA (&huart1, rx_buffer, UART_RX_BUFFER_SIZE);

	for (uint32_t i = 0; i <= 65536; i++) TIM1->CCR1 = i;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		HAL_IWDG_Refresh(&hiwdg); //IWDG->KR = 0x0000AAAAU;
		if (HAL_I2C_Init(&hi2c1) != HAL_OK) I2C_Init(&hi2c1);

//		HAL_RTC_GetTime(&hrtc, &clockTime, RTC_FORMAT_BIN);
//		rtcSec = clockTime.Seconds;
//		rtcMin = clockTime.Minutes;
//		rtcHrs = clockTime.Hours;
//		HAL_RTC_GetDate(&hrtc, &clockDate, RTC_FORMAT_BIN);
//		rtcDate = clockDate.Date;
//		rtcMonth = clockDate.Month;
//		rtcYear = clockDate.Year;
//		rtcWeekD = clockDate.WeekDay;

		DS3231_Update(); rtcSec = DS3231_getSec(); rtcMin = DS3231_getMin(); rtcHrs = DS3231_getHrs();
		rtcDate = DS3231_getDate(); rtcMonth = DS3231_getMonth(); rtcYear = DS3231_getYear(); rtcWeekD = DS3231_getWeekDay();

		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) {

			uint16_t touchX = getX();
			uint16_t touchY = getY();
			if (touchX && touchY && touchX != 0x0DB) {
				LCD_Rect_Fill(touchX, touchY, 1, 1, WHITE);
			}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
			touchX = 0;
			touchY = 0;
		} else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

		if (rtcSec % 2 == 0)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

		char clockPrint[13];

		if (rtcSecLast != rtcSec) {

			sprintf(clockPrint, "%02d", rtcSecLast);
			LCD_Font(630, 85, clockPrint, &DejaVu_Sans_112, 1, BLACK);
			sprintf(clockPrint, "%02d", rtcSec);
			LCD_Font(630, 85, clockPrint, &DejaVu_Sans_112, 1, ORANGE);

			LCD_Circle(300, 60, 10, 0, 1, ORANGE);
			LCD_Circle(300, 120, 10, 0, 1, ORANGE);

			if (rtcSec % 2 != 0) {
				LCD_Circle(300, 60, 9, 1, 1, ORANGE);
				LCD_Circle(300, 120, 9, 1, 1, ORANGE);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			}
			else {
				LCD_Circle(300, 60, 9, 1, 1, BLACK);
				LCD_Circle(300, 120, 9, 1, 1, BLACK);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}

			if (rtcMinLast != rtcMin) {


				sprintf(clockPrint, "%02d", rtcMinLast);
				LCD_Font(310, 170, clockPrint, &DejaVu_Sans_112, 2, BLACK);
				sprintf(clockPrint, "%02d", rtcMin);
				LCD_Font(310, 170, clockPrint, &DejaVu_Sans_112, 2, ORANGE);

				if (rtcHrsLast != rtcHrs) {

					sprintf(clockPrint, "%02d", rtcHrsLast);
					LCD_Font(0, 170, clockPrint, &DejaVu_Sans_112, 2, BLACK);
					sprintf(clockPrint, "%02d", rtcHrs);
					LCD_Font(0, 170, clockPrint, &DejaVu_Sans_112, 2, ORANGE);

					if (rtcWeekDLast != rtcWeekD) {

						static const char* days[7] = { "MO", "TU", "WE", "TH", "FR", "SA", "SU" };
						LCD_Font(710, 125, days[(7 + rtcWeekD - 2) % 7], &DejaVu_Sans_48, 1, BLACK);
						LCD_Font(710, 125, days[(7 + rtcWeekD - 1) % 7], &DejaVu_Sans_48, 1, CYAN);

						static const char* months[12] = { "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC" };

						LCD_Font(600, 125, months[(12 + rtcMonth - 2) % 12], &DejaVu_Sans_48, 1, BLACK);
						LCD_Font(600, 125, months[(12 + rtcMonth - 1) % 12], &DejaVu_Sans_48, 1, CYAN);

						sprintf(clockPrint, "%02d.%02d.%02d", rtcDateLast, rtcMonthLast, rtcYearLast);
						LCD_Font(578, 175, clockPrint, &DejaVu_Sans_48, 1, BLACK);
						sprintf(clockPrint, "%02d.%02d.%02d", rtcDate, rtcMonth, rtcYear);
						LCD_Font(578, 175, clockPrint, &DejaVu_Sans_48, 1, CYAN);

						rtcWeekDLast = rtcWeekD;
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

						LCD_Rect(1, 182, 265, 40, 1, BLUE);

						char weatherPrintT[7];

						if (temperatureLast >= 10 || (temperatureLast < 0 && temperatureLast > -10)) {
							sprintf(weatherPrintT, "%.1f 'C", temperatureLast);
							LCD_Font(61, 220, weatherPrintT, &DejaVu_Sans_48, 1, BLACK);
						}
						else if (temperatureLast < 10 && temperatureLast > 0) {
							sprintf(weatherPrintT, "%.1f 'C", temperatureLast);
							LCD_Font(87, 220, weatherPrintT, &DejaVu_Sans_48, 1, BLACK);
						}
						else if (temperatureLast <= -10) {
							sprintf(weatherPrintT, "%2d 'C", (int8_t)temperatureLast);
							LCD_Font(61, 220, weatherPrintT, &DejaVu_Sans_48, 1, BLACK);
						}

						if (temperature >= 10 || (temperature < 0 && temperature > -10)) {
							sprintf(weatherPrintT, "%.1f 'C", temperature);
							LCD_Font(61, 220, weatherPrintT, &DejaVu_Sans_48, 1, ORANGE);
						}
						else if (temperature < 10 && temperature > 0) {
							sprintf(weatherPrintT, "%.1f 'C", temperature);
							LCD_Font(87, 220, weatherPrintT, &DejaVu_Sans_48, 1, ORANGE);
						}
						else if (temperature <= -10) {
							sprintf(weatherPrintT, "%2d 'C", (int8_t)temperature);
							LCD_Font(61, 220, weatherPrintT, &DejaVu_Sans_48, 1, ORANGE);
						}

						temperatureLast = temperature;
					}

					if (humidity != humidityLast) {

						LCD_Rect(267, 182, 265, 40, 1, BLUE);

						char weatherPrintH[8];

						sprintf(weatherPrintH, "%.1f H2O", humidityLast);
						if (humidityLast >= 10)
							LCD_Font(297, 220, weatherPrintH, &DejaVu_Sans_48, 1, BLACK);
						else LCD_Font(323, 220, weatherPrintH, &DejaVu_Sans_48, 1, BLACK);

						sprintf(weatherPrintH, "%.1f H2O", humidity);
						if (humidity >= 10)
							LCD_Font(297, 220, weatherPrintH, &DejaVu_Sans_48, 1, CYAN);
						else LCD_Font(323, 220, weatherPrintH, &DejaVu_Sans_48, 1, CYAN);

						humidityLast = humidity;
					}

					if (pressureLast != pressure) {

						LCD_Rect(533, 182, 265, 40, 1, BLUE);

						char weatherPrintP[11];

						if (pressureLast >= 1000) {
							sprintf(weatherPrintP, "%02d HPa", pressureLast);
							LCD_Font(573, 220, weatherPrintP, &DejaVu_Sans_48, 1, BLACK);
						}
						else {
							sprintf(weatherPrintP, " %02d HPa", pressureLast);
							LCD_Font(573, 220, weatherPrintP, &DejaVu_Sans_48, 1, BLACK);
						}

						pressureLast = pressure;

						if (pressureLast >= 1000) {
							sprintf(weatherPrintP, "%02d HPa", pressureLast);
							LCD_Font(573, 220, weatherPrintP, &DejaVu_Sans_48, 1, GREEN);
						}
						else {
							sprintf(weatherPrintP, " %02d HPa", pressureLast);
							LCD_Font(573, 220, weatherPrintP, &DejaVu_Sans_48, 1, GREEN);
						}
					}

					if (AT24XX_Read(0) != rtcHrs) {

						AT24XX_Update(0, rtcHrs);

						for (uint16_t i = 0; i < 499; i++) hT[i] = byteS(AT24XX_Read(i * 2 + 1000), AT24XX_Read(i * 2 + 1 + 1000));
						for (uint16_t i = 498; i > 0; i--) hT[i] = hT[i - 1];
						hT[0] = (uint16_t) (temperature * 10);

						for (uint16_t i = 0; i < 499; i++) {
							AT24XX_Update(i * 2 + 1000, byteL(hT[i]));
							AT24XX_Update(i * 2 + 1 + 1000, byteH(hT[i]));
						}

						for (uint16_t i = 0; i < 499; i++) hH[i] = byteS(AT24XX_Read(i * 2 + 2000), AT24XX_Read(i * 2 + 1 + 2000));
						for (uint16_t i = 498; i > 0; i--) hH[i] = hH[i - 1];
						hH[0] = (uint16_t) (humidity * 10);

						for (uint16_t i = 0; i < 499; i++) {
							AT24XX_Update(i * 2 + 2000, byteL(hH[i]));
							AT24XX_Update(i * 2 + 1 + 2000, byteH(hH[i]));
						}

						for (uint16_t i = 0; i < 499; i++) hP[i] = byteS(AT24XX_Read(i * 2 + 3000), AT24XX_Read(i * 2 + 1 + 3000));
						for (uint16_t i = 498; i > 0; i--) hP[i] = hP[i - 1];
						hP[0] = (uint16_t) (pressure * 10);

						for (uint16_t i = 0; i < 499; i++) {
							AT24XX_Update(i * 2 + 3000, byteL(hP[i]));
							AT24XX_Update(i * 2 + 1 + 3000, byteH(hP[i]));
						}

						viewGraphs = 0;
					}

					LCD_Rect(1, 222, 265, 256, 1, BLUE);
					int16_t valMap = map(((int16_t)(temperature * 10)), MIN_TEMPERATURE_X10, MAX_TEMPERATURE_X10, 0, 255);
					if (valMap < 0) valMap = 0;
					if (valMap > 255) valMap = 255;
					LCD_Line(2 + 263, 223, 2 + 263, 477, 1, BLACK);
					if (valMap)
						LCD_Line(2 + 263, 223 + (255 - valMap), 2 + 263, 477, 1,
								RGB(255 - (255 - valMap), 0, 255 - (255 - (255 - valMap))));

					LCD_Rect(267, 222, 265, 256, 1, BLUE);
					valMap = map(((int16_t)(humidity * 10)), MIN_HUMIDITY_X10, MAX_HUMIDITY_X10, 0, 255);
					if (valMap < 0) valMap = 0;
					if (valMap > 255) valMap = 255;
					LCD_Line(268 + 263, 223, 268 + 263, 477, 1, BLACK);
					if (valMap)
						LCD_Line(268 + 263, 223 + (255 - valMap), 268 + 263, 477, 1,
								RGB(255 - (255 - valMap), 0, 255 - (255 - (255 - valMap))));

					LCD_Rect(533, 222, 265, 256, 1, BLUE);
					valMap = map(((int16_t)(pressure * 10)), MIN_PRESSURE_X10, MAX_PRESSURE_X10, 0, 255);
					if (valMap < 0) valMap = 0;
					if (valMap > 255) valMap = 255;
					LCD_Line(534 + 263, 223, 534 + 263, 477, 1, BLACK);
					if (valMap)
						LCD_Line(534 + 263, 223 + (255 - valMap), 534 + 263, 477, 1,
								RGB(255 - (255 - valMap), 0, 255 - (255 - (255 - valMap))));

					if (!viewGraphs) {

						for (uint16_t i = 0; i < 263 ; i++) {
							valMap = map(((int16_t)hT[i]), MIN_TEMPERATURE_X10, MAX_TEMPERATURE_X10, 0, 255);
							if (valMap < 0) valMap = 0;
							if (valMap > 255) valMap = 255;
							LCD_Line(2 + (262 - i), 223, 2 + (262 - i), 477, 1, BLACK);
							if (valMap)
								LCD_Line(2 + (262 - i), 223 + (255 - valMap), 2 + (262 - i), 477, 1,
										RGB(255 - (255 - valMap), 0, 255 - (255 - (255 - valMap))));
						}

						for (uint16_t i = 0; i < 263 ; i++) {
							valMap = map(((int16_t)hH[i]), MIN_HUMIDITY_X10, MAX_HUMIDITY_X10, 0, 255);
							if (valMap < 0) valMap = 0;
							if (valMap > 255) valMap = 255;
							LCD_Line(268 + (262 - i), 223, 268 + (262 - i), 477, 1, BLACK);
							if (valMap)
								LCD_Line(268 + (262 - i), 223 + (255 - valMap), 268 + (262 - i), 477, 1,
										RGB(255 - (255 - valMap), 0, 255 - (255 - (255 - valMap))));
						}

						for (uint16_t i = 0; i < 263 ; i++) {
							valMap = map(((int16_t)hP[i]), MIN_PRESSURE_X10, MAX_PRESSURE_X10, 0, 255);
							if (valMap < 0) valMap = 0;
							if (valMap > 255) valMap = 255;
							LCD_Line(534 + (262 - i), 223, 534 + (262 - i), 477, 1, BLACK);
							if (valMap)
								LCD_Line(534 + (262 - i), 223 + (255 - valMap), 534 + (262 - i), 477, 1,
										RGB(255 - (255 - valMap), 0, 255 - (255 - (255 - valMap))));
						}

						//						for (uint32_t i = 0; i <= 65536; i++) TIM1->CCR1 = i;
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
					DS3231_setWeekDay(atoi(val));

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 21-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 21-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  HAL_GPIO_WritePin(FLASH25Q_CS_GPIO_Port, FLASH25Q_CS_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : FLASH25Q_CS_Pin */
  GPIO_InitStruct.Pin = FLASH25Q_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(FLASH25Q_CS_GPIO_Port, &GPIO_InitStruct);

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

