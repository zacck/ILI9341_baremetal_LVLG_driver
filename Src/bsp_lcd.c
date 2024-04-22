/*
 * bsp_lcd.c
 *
 *  Created on: Apr 18, 2024
 *      Author: zaccko
 */

#include "bsp_lcd.h"

void LCD_Pin_Init(void);
void SPI_Init(void);
void LCD_SPI_Enable(void);
void LCD_Reset(void);
void LCD_Config(void);
void lcd_write_cmd(uint8_t cmd);
void lcd_write_data(uint8_t *buffer, uint32_t len);

//Define LCD Signals
#define SPI_PORT			SPI2
#define LCD_SCL_PIN  		13U
#define	LCD_SCL_PORT		GPIOB
#define LCD_SDI_PIN			15U
#define LCD_SDI_PORT		GPIOB
#define LCD_SDO_PIN			2U
#define LCD_SDO_PORT		GPIOC
#define LCD_RESX_PIN		10U
#define LCD_RESX_PORT		GPIOD
#define LCD_CSX_PIN			11U
#define LCD_CSX_PORT		GPIOD
#define LCD_DCX_PIN			9U
#define LCD_DCX_PORT		GPIOB


//reset macros
#define LCD_RESX_HIGH()		REG_SET_BIT(LCD_RESX_PORT->ODR, LCD_RESX_PIN)
#define LCD_RESX_LOW()		REG_CLR_BIT(LCD_RESX_PORT->ODR, LCD_RESX_PIN)


//CS Macros
#define LCD_CSX_HIGH()		REG_SET_BIT(LCD_CSX_PORT->ODR, LCD_CSX_PIN)
#define LCD_CSX_LOW()			REG_CLR_BIT(LCD_CSX_PORT->ODR, LCD_CSX_PIN)

//D/C Macros
#define LCD_DCX_HIGH()		REG_SET_BIT(LCD_DCX_PORT->ODR, LCD_DCX_PIN)
#define LCD_DCX_LOW()			REG_CLR_BIT(LCD_DCX_PORT->ODR, LCD_DCX_PIN)

//LCD COMMS
#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

void lcd_set_display_area(lcd_area_t *area);

#define __disable_spi_ssm()						REG_CLR_BIT(SPI_PORT->CR1,SPI_CR1_SSM_Pos)
#define __enable_spi_ssoe()								REG_SET_BIT(SPI_PORT->CR2,SPI_CR2_SSOE_Pos)
#define __spi_set_dff_8bit()  					REG_CLR_BIT(SPI_PORT->CR1,SPI_CR1_DFF_Pos)
#define __spi_set_dff_16bit()					REG_SET_BIT(SPI_PORT->CR1,SPI_CR1_DFF_Pos)
#define __enable_spi()									REG_SET_BIT(SPI_PORT->CR1,SPI_CR1_SPE_Pos)
#define __disable_spi()									do{while(REG_READ_BIT(SPI_PORT->SR,SPI_SR_BSY_Pos)){}; \
																						REG_CLR_BIT(SPI_PORT->CR1,SPI_CR1_SPE_Pos);}while(0)



#define HIGH_16(x)     					((((uint16_t)x) >> 8U) & 0xFFU)
#define LOW_16(x)      					((((uint16_t)x) >> 0U) & 0xFFU)

static void delay_50ms(void){
	for(uint32_t i = 0 ; i<(0xFFFFU * 10U);i++);
}


void bsp_lcd_set_display_area(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
 {
	 lcd_area_t area;
	 area.x1 = x1;
	 area.x2 = x2;
	 area.y1 = y1;
	 area.y2 = y2;
	 lcd_set_display_area(&area);
 }

void lcd_set_display_area(lcd_area_t *area)
{
	uint8_t params[4];
	/*Column address set(2Ah) */
	params[0] = HIGH_16(area->x1);
	params[1] = LOW_16(area->x1);
	params[2] = HIGH_16(area->x2);
	params[3] = LOW_16(area->x2);
	lcd_write_cmd(ILI9341_CASET);
	lcd_write_data(params, 4);

	params[0] = HIGH_16(area->y1);
	params[1] = LOW_16(area->y1);
	params[2] = HIGH_16(area->y2);
	params[3] = LOW_16(area->y2);
	lcd_write_cmd(ILI9341_RASET);
	lcd_write_data(params, 4);

}


void bsp_lcd_init(void)
{
	LCD_Pin_Init();
	SPI_Init();
	LCD_SPI_Enable();
	LCD_Reset();
	LCD_Config();

}

void bsp_lcd_write(uint8_t *buffer, uint32_t nbytes)
{
	uint16_t *buff_ptr;

	__disable_spi();
	__spi_set_dff_16bit();
	__enable_spi();

	LCD_CSX_LOW();

	buff_ptr = (uint16_t*)buffer;
	while(nbytes){
		while(!REG_READ_BIT(SPI_PORT->SR,SPI_SR_TXE_Pos));
		REG_WRITE(SPI_PORT->DR,*buff_ptr);
		++buff_ptr;
		nbytes -= 2;
	}

	__disable_spi();
	LCD_CSX_HIGH();
	__spi_set_dff_8bit();
	__enable_spi();

}


void LCD_Pin_Init(void){
	RCC_TypeDef *pRcc = RCC;
	GPIO_TypeDef *pGPIOB = GPIOB;
	GPIO_TypeDef *pGPIOC = GPIOC;
	GPIO_TypeDef *pGPIOD = GPIOD;

	//enable port clocks
	REG_SET_BIT(pRcc->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Pos);
	REG_SET_BIT(pRcc->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Pos);
	REG_SET_BIT(pRcc->AHB1ENR, RCC_AHB1ENR_GPIODEN_Pos);

	//set mode & speed of control pins
	REG_SET_VAL(pGPIOD->MODER, 0x1U, 0x3, GPIO_MODER_MODE10_Pos);
	REG_CLR_BIT(pGPIOD->MODER, GPIO_OTYPER_OT10_Pos);
	REG_SET_VAL(pGPIOD->OSPEEDR, 2U, 0x3U, GPIO_OSPEEDR_OSPEED10_Pos);
	REG_SET_VAL(pGPIOD->MODER, 0x1U, 0x3, GPIO_MODER_MODE11_Pos);
	REG_CLR_BIT(pGPIOD->MODER, GPIO_OTYPER_OT11_Pos);
	REG_SET_VAL(pGPIOD->OSPEEDR, 2U, 0x3U, GPIO_OSPEEDR_OSPEED11_Pos);
	REG_SET_VAL(pGPIOB->MODER, 0x1U, 0x3, GPIO_MODER_MODE9_Pos);
	REG_CLR_BIT(pGPIOB->MODER, GPIO_OTYPER_OT9_Pos);
	REG_SET_VAL(pGPIOB->OSPEEDR, 2U, 0x3U, GPIO_OSPEEDR_OSPEED9_Pos);

	//configure SPI pins
	REG_SET_VAL(pGPIOB->MODER, 2U, 0x3, GPIO_MODER_MODE13_Pos);
	REG_CLR_BIT(pGPIOB->MODER, GPIO_OTYPER_OT13_Pos);
	REG_SET_VAL(pGPIOB->OSPEEDR, 2U, 0x3U, GPIO_OSPEEDR_OSPEED13_Pos);
	REG_SET_VAL(pGPIOB->AFR[1], 5U, 0xFU, GPIO_MODER_MODE13_Pos);
	REG_SET_VAL(pGPIOB->MODER, 2U, 0x3, GPIO_MODER_MODE15_Pos);
	REG_CLR_BIT(pGPIOB->MODER, GPIO_OTYPER_OT15_Pos);
	REG_SET_VAL(pGPIOB->OSPEEDR, 2U, 0x3U, GPIO_OSPEEDR_OSPEED15_Pos);
	REG_SET_VAL(pGPIOB->AFR[1], 5U, 0xFU, GPIO_MODER_MODE15_Pos);
	REG_SET_VAL(pGPIOC->MODER, 2U, 0x3, GPIO_MODER_MODE2_Pos);
	REG_CLR_BIT(pGPIOC->MODER, GPIO_OTYPER_OT2_Pos);
	REG_SET_VAL(pGPIOC->OSPEEDR, 2U, 0x3U, GPIO_OSPEEDR_OSPEED2_Pos);
	REG_SET_VAL(pGPIOC->AFR[0], 5U, 0xFU, GPIO_MODER_MODE2_Pos);


}


void SPI_Init(void){
	SPI_TypeDef *pSpi = SPI_PORT;
	RCC_TypeDef *pRcc = RCC;


	REG_SET_BIT(pRcc->APB1ENR, RCC_APB1ENR_SPI2EN_Pos); //enable
	REG_SET_BIT(pSpi->CR1, SPI_CR1_MSTR_Pos); // master mode
	REG_CLR_BIT(pSpi->CR1,SPI_CR1_BIDIMODE_Pos);    /* 2 lines uni directional lines*/
	REG_CLR_BIT(pSpi->CR1, SPI_CR1_DFF_Pos); /* DFF = 8bits */
	REG_SET_BIT(pSpi->CR1, SPI_CR1_SSM_Pos); /* SSM enable */
	REG_SET_BIT(pSpi->CR1, SPI_CR1_SSI_Pos); /* SSI enable */
	REG_CLR_BIT(pSpi->CR1, SPI_CR1_LSBFIRST_Pos); /* Send msb first */
	REG_SET_VAL(pSpi->CR1, 0x00U, 0x7U, SPI_CR1_BR_Pos); /* SPI clck = 42MHz/2 ==> 21 MHz */
	REG_CLR_BIT(pSpi->CR1, SPI_CR1_CPOL_Pos); /* CPOL = 0 */
	REG_CLR_BIT(pSpi->CR1, SPI_CR1_CPHA_Pos); /* CPHA = 0 */
	REG_CLR_BIT(pSpi->CR2, SPI_CR2_FRF_Pos);  /* SPI Motorola frame format*/

	// set pin defaults
	REG_SET_BIT(LCD_CSX_PORT->ODR, LCD_CSX_PIN); //CSX high
	REG_SET_BIT(LCD_RESX_PORT->ODR, LCD_RESX_PIN); //RESX HIGH
	REG_SET_BIT(LCD_DCX_PORT->ODR, LCD_RESX_PIN); //DCX HIGH

}

void LCD_SPI_Enable(void) {
	SPI_TypeDef *pSpi = SPI_PORT;
	REG_SET_BIT(pSpi->CR1, SPI_CR1_SPE_Pos);
}

void LCD_Config(void){
	uint8_t params[15];
	lcd_write_cmd(ILI9341_SWRESET);
	lcd_write_cmd(ILI9341_POWERB);
	params[0] = 0x00;
	params[1] = 0xD9;
	params[2] = 0x30;
	lcd_write_data(params, 3);
	lcd_write_cmd(ILI9341_POWER_SEQ);
		params[0]= 0x64;
		params[1]= 0x03;
		params[2]= 0X12;
		params[3]= 0X81;
		lcd_write_data(params, 4);

		lcd_write_cmd(ILI9341_DTCA);
		params[0]= 0x85;
		params[1]= 0x10;
		params[2]= 0x7A;
		lcd_write_data(params, 3);

		lcd_write_cmd(ILI9341_POWERA);
		params[0]= 0x39;
		params[1]= 0x2C;
		params[2]= 0x00;
		params[3]= 0x34;
		params[4]= 0x02;
		lcd_write_data(params, 5);

		lcd_write_cmd(ILI9341_PRC);
		params[0]= 0x20;
		lcd_write_data(params, 1);

		lcd_write_cmd(ILI9341_DTCB);
		params[0]= 0x00;
		params[1]= 0x00;
		lcd_write_data(params, 2);

		lcd_write_cmd(ILI9341_POWER1);
		params[0]= 0x1B;
		lcd_write_data(params, 1);

		lcd_write_cmd(ILI9341_POWER2);
		params[0]= 0x12;
		lcd_write_data(params, 1);

		lcd_write_cmd(ILI9341_VCOM1);
		params[0]= 0x08;
		params[1]= 0x26;
		lcd_write_data(params, 2);

		lcd_write_cmd(ILI9341_VCOM2);
		params[0]= 0XB7;
		lcd_write_data(params, 1);

		uint8_t m;
		m = MADCTL_MV | MADCTL_MY| MADCTL_BGR;

		lcd_write_cmd(ILI9341_MAC);    // Memory Access Control <Landscape setting>
		params[0]= m;
		lcd_write_data(params, 1);


		lcd_write_cmd(ILI9341_PIXEL_FORMAT);
		params[0]= 0x55; //select RGB565
		lcd_write_data(params, 1);

		lcd_write_cmd(ILI9341_FRMCTR1);
		params[0]= 0x00;
		params[1]= 0x1B;//frame rate = 70
		lcd_write_data(params, 2);

		lcd_write_cmd(ILI9341_DFC);    // Display Function Control
		params[0]= 0x0A;
		params[1]= 0xA2;
		lcd_write_data(params, 2);

		lcd_write_cmd(ILI9341_3GAMMA_EN);    // 3Gamma Function Disable
		params[0]= 0x02; //LCD_WR_DATA(0x00);
		lcd_write_data(params, 1);

		lcd_write_cmd(ILI9341_GAMMA);
		params[0]= 0x01;
		lcd_write_data(params, 1);

		lcd_write_cmd(ILI9341_PGAMMA);    //Set Gamma
		params[0]= 0x0F;
		params[1]= 0x1D;
		params[2]= 0x1A;
		params[3]= 0x0A;
		params[4]= 0x0D;
		params[5]= 0x07;
		params[6]= 0x49;
		params[7]= 0X66;
		params[8]= 0x3B;
		params[9]= 0x07;
		params[10]= 0x11;
		params[11]= 0x01;
		params[12]= 0x09;
		params[13]= 0x05;
		params[14]= 0x04;
		lcd_write_data(params, 15);

		lcd_write_cmd(ILI9341_NGAMMA);
		params[0]= 0x00;
		params[1]= 0x18;
		params[2]= 0x1D;
		params[3]= 0x02;
		params[4]= 0x0F;
		params[5]= 0x04;
		params[6]= 0x36;
		params[7]= 0x13;
		params[8]= 0x4C;
		params[9]= 0x07;
		params[10]= 0x13;
		params[11]= 0x0F;
		params[12]= 0x2E;
		params[13]= 0x2F;
		params[14]= 0x05;
		lcd_write_data(params, 15);

		lcd_write_cmd(ILI9341_RASET); //page address set
		params[0]= 0x00;
		params[1]= 0x00;
		params[2]= 0x00;
		params[3]= 0xf0; //240 rows = 0xf0
		lcd_write_data(params, 4);

		lcd_write_cmd(ILI9341_CASET);
		params[0]= 0x00;
		params[1]= 0x00;
		params[2]= 0x01;
		params[3]= 0x40; //320 columns = 0x140
		lcd_write_data(params, 4);

		lcd_write_cmd(ILI9341_SLEEP_OUT); //Exit Sleep
		delay_50ms();
		delay_50ms();
		lcd_write_cmd(ILI9341_DISPLAY_ON); //display on
}

void lcd_write_cmd(uint8_t cmd) {
	SPI_TypeDef *pSpi = SPI_PORT;
	//asert command
	LCD_CSX_LOW();
	LCD_DCX_LOW(); //for command
	// send command over SPI
	while(!REG_READ_BIT(pSpi->SR, SPI_SR_TXE_Pos));
	REG_WRITE(pSpi->DR, cmd);
	while(!REG_READ_BIT(pSpi->SR, SPI_SR_TXE_Pos));
	while(REG_READ_BIT(pSpi->SR, SPI_SR_BSY_Pos));
	LCD_CSX_HIGH();
	LCD_DCX_HIGH();

}


void lcd_write_data(uint8_t *buffer, uint32_t len) {
	SPI_TypeDef *pSpi = SPI_PORT;
	LCD_CSX_LOW();
	for(uint32_t i = 0; i < len; i++) {
		// send command over SPI
		while (!REG_READ_BIT(pSpi->SR, SPI_SR_TXE_Pos));
		REG_WRITE(pSpi->DR, buffer[i]);
	}
	while(!REG_READ_BIT(pSpi->SR, SPI_SR_TXE_Pos));
	while(REG_READ_BIT(pSpi->SR, SPI_SR_BSY_Pos));
	LCD_CSX_HIGH();

}

void LCD_Reset(void ){
	LCD_RESX_LOW();
	for(uint32_t i = 0; i < (0xFFFF * 20U); i++);
	LCD_RESX_HIGH();
	for(uint32_t i = 0; i < (0xFFFF * 20U); i++);

}



