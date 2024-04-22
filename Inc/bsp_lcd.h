/*
 * bsp_lcd.h
 *
 *  Created on: Apr 18, 2024
 *      Author: zaccko
 */

#ifndef BSP_LCD_H_
#define BSP_LCD_H_

#include "ili9341_reg.h"
#include "stm32f407xx.h"
#include "reg_util.h"


typedef struct{
 	uint16_t x1;
 	uint16_t x2;
 	uint16_t y1;
 	uint16_t y2;
 }lcd_area_t;

void bsp_lcd_init(void);
void bsp_lcd_set_display_area(uint16_t x1, uint16_t x2, uint16_t y1 , uint16_t y2);
void bsp_lcd_write(uint8_t *buffer, uint32_t nbytes);




#endif /* BSP_LCD_H_ */
