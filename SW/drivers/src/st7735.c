/*!****************************************************************************
 * @file		st7735.c
 * @author		d_el, 4eef
 * @version		V1.1
 * @date		19.11.2018
 * @copyright	The MIT License (MIT). Copyright (c) 2017 Storozhenko Roman
 * @brief		Driver display on controller ST7735B, ST7735R
 */

/*!****************************************************************************
 * Include
 */
#include "stm32f0xx.h"
#include "stdbool.h"
#include "gpio.h"
#include "delay.h"
#include "clock.h"
#include "st7735.h"

/*!****************************************************************************
 * Define
 */
// for 1.44 and mini
#define ST7735_TFTWIDTH_128	 128
// for mini
#define ST7735_TFTWIDTH_80	 80
// for 1.44" display
#define ST7735_TFTHEIGHT_128 128
// for 1.8" and mini display
#define ST7735_TFTHEIGHT_160  160

//static uint8_t cmd, rxBff[3], num;

/*!****************************************************************************
 * Enumeration
 */
enum lcdCommands{
	ST7735_NOP	   = 0x00,
	ST7735_SWRESET = 0x01,
	ST7735_RDDID   = 0x04,
	ST7735_RDDST   = 0x09,

	ST7735_SLPIN   = 0x10,
	ST7735_SLPOUT  = 0x11,
	ST7735_PTLON   = 0x12,
	ST7735_NORON   = 0x13,

	ST7735_INVOFF  = 0x20,
	ST7735_INVON   = 0x21,
	ST7735_DISPOFF = 0x28,
	ST7735_DISPON  = 0x29,
	ST7735_CASET   = 0x2A,
	ST7735_RASET   = 0x2B,
	ST7735_RAMWR   = 0x2C,
	ST7735_RAMRD   = 0x2E,

	ST7735_PTLAR   = 0x30,
	ST7735_COLMOD  = 0x3A,
	ST7735_MADCTL  = 0x36,

	ST7735_FRMCTR1 = 0xB1,
	ST7735_FRMCTR2 = 0xB2,
	ST7735_FRMCTR3 = 0xB3,
	ST7735_INVCTR  = 0xB4,
	ST7735_DISSET5 = 0xB6,

	ST7735_PWCTR1  = 0xC0,
	ST7735_PWCTR2  = 0xC1,
	ST7735_PWCTR3  = 0xC2,
	ST7735_PWCTR4  = 0xC3,
	ST7735_PWCTR5  = 0xC4,
	ST7735_VMCTR1  = 0xC5,

	ST7735_RDID1   = 0xDA,
	ST7735_RDID2   = 0xDB,
	ST7735_RDID3   = 0xDC,
	ST7735_RDID4   = 0xDD,

	ST7735_PWCTR6  = 0xFC,

	ST7735_GMCTRP1 = 0xE0,
	ST7735_GMCTRN1 = 0xE1
};

enum lcdDescribed{
	MADCTL_MY	= 0x80,
	MADCTL_MX	= 0x40,
	MADCTL_MV	= 0x20,
	MADCTL_ML	= 0x10,
	MADCTL_RGB	= 0x00,
	MADCTL_BGR	= 0x08,
	MADCTL_MH	= 0x04
};

enum initRflags{
	INITR_GREENTAB		= 0x0,
	INITR_REDTAB		= 0x1,
	INITR_BLACKTAB		= 0x2,
	INITR_18GREENTAB	= INITR_GREENTAB,
	INITR_18REDTAB		= INITR_REDTAB,
	INITR_18BLACKTAB	= INITR_BLACKTAB,
	INITR_144GREENTAB	= 0x1,
	INITR_MINI160x80	= 0x4
};

// Initialization commands for 7735B screens
const uint8_t Bcmd[] = {
	18,		// 18 commands in list

	ST7735_SWRESET,	0, 50,	// 1: Software reset

	ST7735_SLPOUT, 0, 255,	// 2: Out of sleep mode

	ST7735_COLMOD, 1, 10,	// 3: Set color mode
	0x05,					//		16-bit color

	ST7735_FRMCTR1, 3, 10,	// 4: Frame rate control
	0x00,					//		fastest refresh
	0x06,					//		6 lines front porch
	0x03,					//		3 lines back porch

	ST7735_MADCTL, 1, 0,	// 5: Memory access ctrl (directions)
	0x08,					//		Row addr/col addr, bottom to top refresh

	ST7735_DISSET5, 2, 0,	// 6: Display settings #5
	0x15,					//		1 clk cycle nonoverlap, 2 cycle gate
							//		rise, 3 cycle osc equalize
	0x02,					//		Fix on VTL

	ST7735_INVCTR, 1, 0,	// 7: Display inversion control
	0x0,					//		Line inversion

	ST7735_PWCTR1, 2, 10,	// 8: Power control
	0x02,					//		GVDD = 4.7V
	0x70,					//		1.0uA

	ST7735_PWCTR2, 1, 0,	//	9: Power control
	0x05,					//		VGH = 14.7V, VGL = -7.35V

	ST7735_PWCTR3, 2, 0,	// 10: Power control
	0x01,					//		Opamp current small
	0x02,					//		Boost frequency

	ST7735_VMCTR1, 2, 10,	// 11: Power control
	0x3C,					//		VCOMH = 4V
	0x38,					//		VCOML = -1.1V

	ST7735_PWCTR6, 2, 0,	// 12: Power control,
	0x11,
	0x15,

	ST7735_GMCTRP1, 16, 0,	// 13: Magical unicorn dust
	0x09, 0x16, 0x09, 0x20,		// (seriously though, not sure what
	0x21, 0x1B, 0x13, 0x19,		// these config values represent)
	0x17, 0x15, 0x1E, 0x2B, 0x04, 0x05, 0x02, 0x0E,

	ST7735_GMCTRN1, 16, 10, // 14: Sparkles and rainbows
	0x0B, 0x14, 0x08, 0x1E, // (ditto)
	0x22, 0x1D, 0x18, 0x1E, 0x1B, 0x1A, 0x24, 0x2B, 0x06, 0x06, 0x02, 0x0F,

	ST7735_CASET, 4, 0,		// 15: Column addr set
	0x00, 0x02,				//		XSTART = 2
	0x00, 0x81,				//		XEND = 129

	ST7735_RASET, 4, 0,		// 16: Row addr set, 4 args, no delay:
	0x00, 0x02,				//	   XSTART = 1
	0x00, 0x81,				//	   XEND = 160

	ST7735_NORON, 0, 10,	// 17: Normal display on

	ST7735_DISPON, 0, 255	// 18: Main screen turn on
};

// Initialization for 7735R, part 1 (red or green tab)
const uint8_t Rcmd1[] = {
	15,	// 15 commands in list

	ST7735_SWRESET, 0, 150,	 // 1: Software reset

	ST7735_SLPOUT, 0, 255,	// 2: Out of sleep mode

	ST7735_FRMCTR1, 3, 0,	// 3: Frame rate ctrl - normal mode
	0x01, 0x2C, 0x2D,		//	   Rate = fosc/(1x2+40) * (LINE+2C+2D)

	ST7735_FRMCTR2, 3, 0,	// 4: Frame rate control - idle mode
	0x01, 0x2C, 0x2D,		//	   Rate = fosc/(1x2+40) * (LINE+2C+2D)

	ST7735_FRMCTR3, 6, 0,	// 5: Frame rate ctrl - partial mode
	0x01, 0x2C, 0x2D,		//	   Dot inversion mode
	0x01, 0x2C, 0x2D,		//	   Line inversion mode

	ST7735_INVCTR, 1, 0,	// 6: Display inversion ctrl
	0x07,					//	   No inversion

	ST7735_PWCTR1, 3, 0,	// 7: Power control
	0xA2, 0x02,					  //	 -4.6V
	0x84,					//	   AUTO mode

	ST7735_PWCTR2, 1, 0,	// 8: Power control
	0xC5,					//	   VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD

	ST7735_PWCTR3, 2, 0,	// 9: Power control
	0x0A,					//	   Opamp current small
	0x00,					//	   Boost frequency

	ST7735_PWCTR4, 2, 0,	// 10: Power control
	0x8A,					//	   BCLK/2, Opamp current small & Medium low
	0x2A,

	ST7735_PWCTR5, 2, 0,	// 11: Power control
	0x8A, 0xEE,

	ST7735_VMCTR1, 1, 0,	// 12: Power control
	0x0E,

	ST7735_INVOFF, 0, 0,	// 13: Don't invert display

	ST7735_MADCTL, 1, 0,	// 14: Memory access control (directions)
	0xC8,					//	   row addr/col addr, bottom to top refresh

	ST7735_COLMOD, 1, 0,	// 15: set color mode
	0x05					//	   16-bit color
};

// Initialization for 7735R, part 2 (green tab only)
const uint8_t Rcmd2green[] = {
	2,	//	2 commands in list

	ST7735_CASET, 4, 0,		// 1: Column addr set
	0x00, 0x02,				//	   XSTART = 0
	0x00, 0x7F + 0x02,		//	   XEND = 127

	ST7735_RASET, 4, 0,		// 2: Row addr set
	0x00, 0x01,				//	   XSTART = 0
	0x00, 0x9F + 0x01		//	   XEND = 159
};

// Initialization for 7735R, part 2 (red tab only)
const uint8_t Rcmd2red[] = {
	2,	//	2 commands in list

	ST7735_CASET, 4, 0,		// 1: Column addr set
	0x00, 0x00,				//	   XSTART = 0
	0x00, 0x7F,				//	   XEND = 127

	ST7735_RASET, 4, 0,		// 2: Row addr set
	0x00, 0x00,				//	   XSTART = 0
	0x00, 0x9F				//	   XEND = 159
};

// Initialization for 7735R, part 2 (green 1.44 tab)
const uint8_t Rcmd2green144[] = {
	2,	//	2 commands in list

	ST7735_CASET, 4, 0,		// 1: Column addr set
	0x00, 0x00,				//	   XSTART = 0
	0x00, 0x7F,				//	   XEND = 127

	ST7735_RASET, 4, 0,		// 2: Row addr set
	0x00, 0x00,				//	   XSTART = 0
	0x00, 0x7F				//	   XEND = 127
};

// Initialization for 7735R, part 2 (mini 160x80)
const uint8_t Rcmd2green160x80[] = {
	2,	//	2 commands in list

	ST7735_CASET, 4, 0,		// 1: Column addr set
	0x00, 0x00,				//	   XSTART = 0
	0x00, 0x7F,				//	   XEND = 79

	ST7735_RASET, 4, 0,		// 2: Row addr set
	0x00, 0x00,				//	   XSTART = 0
	0x00, 0x9F				//	   XEND = 159
};

// Initialization for 7735R, part 3 (red or green tab)
const uint8_t Rcmd3[] = {
	4,	//	4 commands in list

	ST7735_GMCTRP1, 16, 0,	// 1: Magical unicorn dust,
	0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,

	ST7735_GMCTRN1, 16, 0,	// 2: Sparkles and rainbows
	0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,

	ST7735_NORON, 0, 10,	// 3: Normal display on

	ST7735_DISPON, 0, 100,	// 4: Main screen turn on
};

/*!****************************************************************************
 * MEMORY
 */
st7735_type             st7735;

/*!****************************************************************************
 * Function declaration
 */
void st7735_lcdCmd(uint8_t cmd);
void st7735_lcdDat(uint8_t data);
void st7735_lcdDat16(uint16_t data);
void st7735_setWndAddr(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void st7735_exeCmdList(const uint8_t *addr);
void st7735_commonInit(void);
void st7735_setRotation(uint8_t m);
void st7735_invertDisplay(bool set);
void st7735_initB(void);
void st7735_initR(uint8_t options);

void spiInit(void);
void initSpiDMA(void);
void deInitSpiDMA(void);
void spiSend(uint8_t data);
void spiRead(uint8_t addr, uint8_t *pRxBff, uint8_t num);
void timPwmInit(void);
void timPwmDeInit(void);
void timPwmSet(uint32_t val);


void spiInit(void){
	//Max speed - fPCLK/2
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;		//Clock enable
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;	//Reset module
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

	LCD_SPI->CR1 |= SPI_CR1_MSTR;
	LCD_SPI->CR1 &= ~SPI_CR1_LSBFIRST;
    LCD_SPI->CR1 |= SPI_CR1_BIDIMODE;
    LCD_SPI->CR1 |= SPI_CR1_BIDIOE;
    LCD_SPI->CR2 |= SPI_CR2_SSOE;
    LCD_SPI->CR2 &= ~SPI_CR2_DS_3;              //8-bit

	LCD_SPI->CR1 |= SPI_CR1_SPE;				//SPI enable

//	gppin_init(GPIOA, 5, alternateFunctionPushPull, 0, 0);	//SPI1_SCK
//	gppin_init(GPIOA, 7, alternateFunctionPushPull, 0, 0);	//SPI1_MOSI
}

void initSpiDMA(void){
    RCC->AHBENR     |= RCC_AHBENR_DMA1EN;
    gppin_set(GP_LCD_DC);
    LCD_SPI->CR2    |= SPI_CR2_TXDMAEN;
    LCD_SPI->CR2    |= (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_DS_3);//16-bit
    LCD_DMA->CPAR   = (uint32_t)(&(SPI1->DR));
    LCD_DMA->CMAR   = (uint32_t)&st7735.videoBff[0];
    LCD_DMA->CNDTR  = ST7735_W * ST7735_H;//sizeof(videoBff) / 2;
    LCD_DMA->CCR    |= DMA_CCR_MSIZE_0;//16-bit
    LCD_DMA->CCR    |= DMA_CCR_PSIZE_0;
    LCD_DMA->CCR    |= DMA_CCR_MINC;
    LCD_DMA->CCR    &= ~DMA_CCR_PINC;
    LCD_DMA->CCR    |= DMA_CCR_CIRC;//Circular
    LCD_DMA->CCR    |= DMA_CCR_DIR;//Mem-to-Periph
    LCD_DMA->CCR    &= ~DMA_CCR_TEIE;
    LCD_DMA->CCR    &= ~DMA_CCR_TCIE;
    LCD_DMA->CCR    |= DMA_CCR_EN;
}

void deInitSpiDMA(void){
    LCD_DMA->CCR    &= ~DMA_CCR_CIRC;
    DMA1->IFCR      |= (DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3);
    while((DMA1->ISR & DMA_ISR_TCIF3) == 0);
    LCD_DMA->CCR    &= ~DMA_CCR_EN;
    LCD_SPI->CR2    &= ~SPI_CR2_TXDMAEN;
    LCD_SPI->CR2    &= ~SPI_CR2_DS_3;//8-bit
}

void spiSend(uint8_t data){
	*((uint8_t*)&(LCD_SPI->DR)) = data;
	while((LCD_SPI->SR & SPI_SR_BSY) != 0);
}

void spiRead(uint8_t addr, uint8_t *pRxBff, uint8_t num){
    uint8_t i;
    *((uint8_t*)&(LCD_SPI->DR)) = addr;
	while((LCD_SPI->SR & SPI_SR_BSY) != 0);
    LCD_SPI->CR1 &= ~SPI_CR1_BIDIOE;
    for(i = num; i > 0; i++){
        while((LCD_SPI->SR & SPI_SR_RXNE) == 0);
        *pRxBff = LCD_SPI->DR;
        pRxBff++;
    }
    LCD_SPI->CR1 |= SPI_CR1_BIDIOE;
    __NOP();
}

void timPwmInit(void){
    RCC->APB1ENR    |= RCC_APB1ENR_TIM2EN;
    LCD_TIM->PSC    = F_APB1 / ((LCD_BRGHT_MAX * LCD_BRGHT_MAX * LCD_BRGHT_MAX * LCD_BRGHT_MAX) * LCD_BRGHT_FREQ) + 1;
    LCD_TIM->ARR    = LCD_BRGHT_MAX * LCD_BRGHT_MAX * LCD_BRGHT_MAX * LCD_BRGHT_MAX;
    LCD_TIM->CCMR1  &= ~(TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC2S_1);//CC2 is output
    LCD_TIM->CCMR1  |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);//OC2 is in PWM mode 1
    LCD_TIM->CCMR1  |= TIM_CCMR1_OC2PE;
    LCD_TIM->CCER   |= TIM_CCER_CC2E;
    LCD_TIM->CCER   &= ~TIM_CCER_CC2P;//Active high
    LCD_TIM->CR1    |= TIM_CR1_ARPE;
    LCD_TIM->CR1    &= ~TIM_CR1_DIR;
    LCD_TIM->CNT    = 0;
    LCD_TIM->CR1    |= TIM_CR1_CEN;
    LCD_TIM->EGR    |= TIM_EGR_UG;
    gppin_init(GPIOA, 1, alternateFunctionPushPull, 0, 2);//LCD_BACK
}

void timPwmDeInit(void){
    gppin_init(GPIOA, 1, outPushPull, 0, 0);//LCD_BACK
    LCD_TIM->CR1    &= ~TIM_CR1_CEN;
    LCD_TIM->CCER   &= ~TIM_CCER_CC2E;
    RCC->APB1RSTR   |= RCC_APB1RSTR_TIM2RST;
    RCC->APB1RSTR   &= ~RCC_APB1RSTR_TIM2RST;
    RCC->APB1ENR    &= ~RCC_APB1ENR_TIM2EN;
}

void timPwmSet(uint32_t val){
    LCD_TIM->CCR2 = val;
}

/*!****************************************************************************
 * @brief Send command
 */
void st7735_lcdCmd(uint8_t cmd){
	gppin_reset(GP_LCD_DC);
	spiSend(cmd);
}

/*!****************************************************************************
 * @brief Send data
 */
void st7735_lcdDat(uint8_t data){
	gppin_set(GP_LCD_DC);
	spiSend(data);
}

/*!****************************************************************************
 * @brief Send data 16 bit
 */
void st7735_lcdDat16(uint16_t data){
	gppin_set(GP_LCD_DC);
	spiSend(data >> 0x08);
	spiSend(data & 0xFF);
}

/*!****************************************************************************
 * @brief Set address window on Graphic RAM
 */
void st7735_setWndAddr(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1){
	st7735_lcdCmd(ST7735_CASET);	// column addr set
	st7735_lcdDat(0x00);
	st7735_lcdDat(x0 + st7735.xstart);		// XSTART
	st7735_lcdDat(0x00);
	st7735_lcdDat(x1 + st7735.xstart);		// XEND

	st7735_lcdCmd(ST7735_RASET);	// row addr set
	st7735_lcdDat(0x00);
	st7735_lcdDat(y0 + st7735.ystart);		// YSTART
	st7735_lcdDat(0x00);
	st7735_lcdDat(y1 + st7735.ystart);		// YEND
}

/*!****************************************************************************
 * @brief Companion code to the above tables.  Reads and issues
 * a series of LCD commands stored in PROGMEM byte array.
 */
void st7735_exeCmdList(const uint8_t *addr){
	uint8_t	 numCommands, numArgs;
	uint16_t ms;

	numCommands = *addr++;					// Number of commands to follow
	while(numCommands--){					// For each command...
		st7735_lcdCmd(*addr++);				// Read, issue command
		numArgs		= *addr++;				// Number of args to follow
		ms			= *addr++;				// Read post-command delay time (ms)
		while(numArgs--){					// For each argument...
			st7735_lcdDat(*addr++);			// Read, issue argument
		}

		if(ms){
			if(ms == 255){
				ms = 500;					// If 255, delay for 500 ms
			}
			delay_ms(ms);
		}
	}
}

/*!****************************************************************************
 * @brief Initialization code common to both 'B' and 'R' type displays
 */
void st7735_commonInit(void){
    gppin_set(GP_LCD_RST);
	delay_ms(10);
	gppin_reset(GP_LCD_RST);
	delay_ms(10);
	gppin_set(GP_LCD_RST);
	delay_ms(10);
}

/*!****************************************************************************
 * @brief Set rotation display
 */
void st7735_setRotation(uint8_t m){
	uint8_t rotation;

	st7735_lcdCmd(ST7735_MADCTL);

	rotation = m % 4; // can't be higher than 3
	switch (rotation){
		case 0:
			if ((st7735.tabcolor == INITR_BLACKTAB) || (st7735.tabcolor == INITR_MINI160x80)) {
				st7735_lcdDat(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
			}else{
				st7735_lcdDat(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
			}

			if (st7735.tabcolor == INITR_144GREENTAB) {
				st7735.height = ST7735_TFTHEIGHT_128;
				st7735.width  = ST7735_TFTWIDTH_128;
			} else if (st7735.tabcolor == INITR_MINI160x80)  {
				st7735.height = ST7735_TFTHEIGHT_160;
				st7735.width = ST7735_TFTWIDTH_80;
			} else {
				st7735.height = ST7735_TFTHEIGHT_160;
				st7735.width  = ST7735_TFTWIDTH_128;
			}
			st7735.xstart = st7735.colstart;
			st7735.ystart = st7735.rowstart;
		break;

		case 1:
			if ((st7735.tabcolor == INITR_BLACKTAB) || (st7735.tabcolor == INITR_MINI160x80)) {
				st7735_lcdDat(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
			} else {
				st7735_lcdDat(MADCTL_MY | MADCTL_MV | MADCTL_BGR);
			}

			if (st7735.tabcolor == INITR_144GREENTAB)	{
				st7735.width = ST7735_TFTHEIGHT_128;
				st7735.height = ST7735_TFTWIDTH_128;
			} else if (st7735.tabcolor == INITR_MINI160x80)  {
				st7735.width = ST7735_TFTHEIGHT_160;
				st7735.height = ST7735_TFTWIDTH_80;
			} else {
				st7735.width = ST7735_TFTHEIGHT_160;
				st7735.height = ST7735_TFTWIDTH_128;
			}
			st7735.ystart = st7735.colstart;
			st7735.xstart = st7735.rowstart;
		break;

		case 2:
			if ((st7735.tabcolor == INITR_BLACKTAB) || (st7735.tabcolor == INITR_MINI160x80)) {
				st7735_lcdDat(MADCTL_RGB);
			} else {
				st7735_lcdDat(MADCTL_BGR);
			}

			if (st7735.tabcolor == INITR_144GREENTAB) {
				st7735.height = ST7735_TFTHEIGHT_128;
				st7735.width  = ST7735_TFTWIDTH_128;
			} else if (st7735.tabcolor == INITR_MINI160x80)  {
				st7735.height = ST7735_TFTHEIGHT_160;
				st7735.width = ST7735_TFTWIDTH_80;
			} else {
				st7735.height = ST7735_TFTHEIGHT_160;
				st7735.width  = ST7735_TFTWIDTH_128;
			}
			st7735.xstart = st7735.colstart;
			st7735.ystart = st7735.rowstart;
		break;

		case 3:
			if ((st7735.tabcolor == INITR_BLACKTAB) || (st7735.tabcolor == INITR_MINI160x80)) {
				st7735_lcdDat(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
			} else {
				st7735_lcdDat(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
			}

			if (st7735.tabcolor == INITR_144GREENTAB)	{
				st7735.width = ST7735_TFTHEIGHT_128;
				st7735.height = ST7735_TFTWIDTH_128;
			} else if (st7735.tabcolor == INITR_MINI160x80)  {
				st7735.width = ST7735_TFTHEIGHT_160;
				st7735.height = ST7735_TFTWIDTH_80;
			} else {
				st7735.width = ST7735_TFTHEIGHT_160;
				st7735.height = ST7735_TFTWIDTH_128;
			}
			st7735.ystart = st7735.colstart;
			st7735.xstart = st7735.rowstart;
		break;
	}
}

/*!****************************************************************************
 * @brief Set invert display
 */
void st7735_invertDisplay(bool set){
	if(set == true){
		st7735_lcdCmd(ST7735_INVON);
	}else{
		st7735_lcdCmd(ST7735_INVOFF);
	}
}

/*!****************************************************************************
 * @brief Initialization for ST7735B screens
 */
void st7735_initB(void){
	st7735_commonInit();
    st7735_exeCmdList(Rcmd1);
	st7735_setRotation(0);
}

/*!****************************************************************************
 * @brief Initialization for ST7735R screens (green or red tabs)
 */
void st7735_initR(uint8_t options){
	st7735_commonInit();
    st7735_exeCmdList(Rcmd1);

	if(options == INITR_GREENTAB){
		st7735_exeCmdList(Rcmd2green);
		st7735.colstart = 2;
		st7735.rowstart = 1;
	}
	else if(options == INITR_144GREENTAB){
		st7735.height = ST7735_TFTHEIGHT_128;
		st7735.width = ST7735_TFTWIDTH_128;
		st7735_exeCmdList(Rcmd2green144);
		st7735.colstart = 2;
		st7735.rowstart = 3;
	}
	else if(options == INITR_MINI160x80){
		st7735.height = ST7735_TFTHEIGHT_160;
		st7735.width = ST7735_TFTWIDTH_80;
		st7735_exeCmdList(Rcmd2green160x80);
		st7735.colstart = 26;
		st7735.rowstart = 1;
	}
	else{
		// colstart, rowstart left at default '0' values
		st7735_exeCmdList(Rcmd2red);
	}

	st7735_exeCmdList(Rcmd3);

	// if black, change MADCTL color filter
	if((options == INITR_BLACKTAB) || (options == INITR_MINI160x80)){
		st7735_lcdCmd(ST7735_MADCTL);
		st7735_lcdDat(0xC0);
	}

	st7735.tabcolor = options;

	st7735_setRotation(3);
    
	st7735_setWndAddr(0, 0, st7735.width - 1, st7735.height - 1);
}

/*!****************************************************************************
 * @brief Initialize controller with specified parameters
 */
void st7735_init(void){
    gppin_set(GP_LCD_PSPLY);
    //Initialize serial interface
    spiInit();
    gppin_reset(GP_LCD_CS);
    //Set initial parameters
    st7735_initR(INITR_MINI160x80);
    //Chinese 160x80 display errors
    st7735_invertDisplay(true);
    st7735_lcdCmd(ST7735_MADCTL);
    st7735_lcdDat(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
    //SPI read test    
//    cmd = 0x0B;
//    num = 2;
//    gppin_reset(GP_LCD_DC);
//    spiRead(cmd, &rxBff[0], num);
    //Temporarely fill buffer
    for(uint16_t i = 0; i < (ST7735_W * ST7735_H); i++){
        st7735.videoBff[i] = i;
    }
    //Circuar write to RAM with DMA
    st7735_lcdCmd(ST7735_RAMWR);
    initSpiDMA();
    //Enable backlight
    timPwmInit();
    st7735_setBrightness(st7735.brghtPrev);
}

/*!****************************************************************************
 * @brief Deinitialize display and switch it off
 */
void st7735_deInit(void){
    st7735_setBrightness(LCD_BRGHT_OFF);
    timPwmDeInit();
    deInitSpiDMA();
    gppin_reset(GP_LCD_PSPLY);
}

/*!****************************************************************************
 * @brief Set sleep ON
 */
void st7735_sleepOn(void){
    st7735_setBrightness(LCD_BRGHT_OFF);
    deInitSpiDMA();
    st7735_lcdCmd(ST7735_SLPIN);
}

/*!****************************************************************************
 * @brief Set sleep OFF
 */
void st7735_sleepOff(void){
    st7735_lcdCmd(ST7735_SLPOUT);
    delay_ms(500);
    st7735_lcdCmd(ST7735_RAMWR);
    initSpiDMA();
    st7735_setBrightness(st7735.brghtPrev);
}

/*!****************************************************************************
 * @brief Set display backlight brightness
 */
void st7735_setBrightness(uint8_t level){
    if(level > LCD_BRGHT_MAX) level = LCD_BRGHT_MAX;
    if((st7735.brghtPrev == 0) && (st7735.brght == 0)) st7735.brght = level = LCD_BRGHT_MAX / 2;
    st7735.brghtPrev = st7735.brght;
    st7735.brght = level;
    timPwmSet(level * level * level * level);
}

/******************************** END OF FILE ********************************/
