/* vim: set ai et ts=4 sw=4: */
#ifndef __ILI9341_TOUCH_H__
#define __ILI9341_TOUCH_H__

#include <stdbool.h>
#include "gd32f30x.h"



/*** Redefine if necessary ***/

// Warning! Use SPI bus with < 1.3 Mbit speed, better ~650 Kbit to be save.
#define ILI9341_TOUCH_SPI_PORT 	SPI1
#define ILI9341_TOUCH_RCU_SPI	 	RCU_SPI1

#define ILI9341_TOUCH_CLK_Pin       	GPIO_PIN_13
#define ILI9341_TOUCH_CLK_GPIO_Port 	GPIOB
#define ILI9341_TOUCH_CLK_RCU 				RCU_GPIOB

#define ILI9341_TOUCH_MISO_Pin      	GPIO_PIN_14
#define ILI9341_TOUCH_MISO_GPIO_Port 	GPIOB
#define ILI9341_TOUCH_MISO_RCU 				RCU_GPIOB

#define ILI9341_TOUCH_MOSI_Pin       	GPIO_PIN_15
#define ILI9341_TOUCH_MOSI_GPIO_Port 	GPIOB
#define ILI9341_TOUCH_MOSI_RCU 				RCU_GPIOB

#define ILI9341_TOUCH_IRQ_Pin       	GPIO_PIN_6 
#define ILI9341_TOUCH_IRQ_GPIO_Port 	GPIOC
#define ILI9341_TOUCH_IRQ_RCU 				RCU_GPIOC

#define ILI9341_TOUCH_CS_Pin        	GPIO_PIN_5 
#define ILI9341_TOUCH_CS_GPIO_Port  	GPIOC
#define ILI9341_TOUCH_CS_RCU 					RCU_GPIOC

// change depending on screen orientation
#define ILI9341_TOUCH_SCALE_X 240
#define ILI9341_TOUCH_SCALE_Y 320

// to calibrate uncomment UART_Printf line in ili9341_touch.c
#define ILI9341_TOUCH_MIN_RAW_X 1500
#define ILI9341_TOUCH_MAX_RAW_X 31000
#define ILI9341_TOUCH_MIN_RAW_Y 3276
#define ILI9341_TOUCH_MAX_RAW_Y 30110

// call before initializing any SPI devices
void ILI9341_TouchUnselect();

bool ILI9341_TouchPressed();
bool ILI9341_TouchGetCoordinates(uint16_t* x, uint16_t* y);

void t_spi_rcu_config(void);
void t_spi_gpio_config(void);
void t_spi_config(void);
void SPI_TransmitReceive(uint32_t SPI_, uint16_t *oData, uint16_t *iData, uint16_t Size, int Timeout);
void ILI9341_Touch_init();
void SPI_Receive(uint32_t SPI_, uint16_t *iData, uint16_t Size);

void send_my_spi1(uint8_t data);

#endif // __ILI9341_TOUCH_H__
