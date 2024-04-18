/*
 * bsp_lcd.c
 *
 *  Created on: Apr 18, 2024
 *      Author: zaccko
 */

#include "stm32f407xx.h"
#include "reg_util.h"

void LCD_Pin_Init(void);
void SPI_Init(void);
void LCD_SPI_Enable(void);
void LCD_Reset(void);
void LCD_Config(void);
void LCD_Write_Cmd(uint8_t cmd);
void LCD_Write_Data(uint8_t *buffer, uint32_t len);

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
#define LCD_RESX_HIGH		REG_SET_BIT(LCD_RESX_PORT->ODR, LCD_RESX_PIN)
#define LCD_RESX_LOW		REG_CLR_BIT(LCD_RESX_PORT->ODR, LCD_RESX_PIN)


//D/C Macros
#define LCD_CSX_HIGH		REG_SET_BIT(LCD_CSX_PORT->ODR, LCD_CSX_PIN)
#define LCD_CSX_LOW			REG_CLR_BIT(LCD_CSX_PORT->ODR, LCD_CSX_PIN)


void BSP_LCD_Init(void)
{
	LCD_Pin_Init();
	SPI_Init();
	LCD_Reset();
	LCD_Config();

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
	LCD_Write_Cmd(ILI9341_SWRESET);
	LCD_Write_Cmd(ILI9341_POWERB);
	params[0] = 0x00;
	params[1] = 0xD9;
	params[2] = 0x30;
	LCD_Write_Data(params, 3);
}

void LCD_Write_Cmd(uint8_t cmd) {
	SPI_TypeDef *pSpi = SPI_PORT;
	// send command over SPI
	while(!REG_READ_BIT(pSpi->SR, SPI_SR_TXE_Pos));
	REG_WRITE(pSpi->DR, cmd);

}


void LCD_Write_Data(uint8_t *buffer, uint32_t len) {
	SPI_TypeDef *pSpi = SPI_PORT;
	while (len--) {
		// send command over SPI
		while (!REG_READ_BIT(pSpi->SR, SPI_SR_TXE_Pos));
		REG_WRITE(pSpi->DR, &buffer);
		&buffer++;
	}

}

