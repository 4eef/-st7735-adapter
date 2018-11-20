/*!****************************************************************************
 * @file		st7735.h
 * @author		d_el, 4eef
 * @version		V1.1
 * @date		19.11.2018
 * @copyright	The MIT License (MIT). Copyright (c) 2017 Storozhenko Roman
 * @brief		Driver display on controller ST7735
 */

#ifndef st7735_H
#define st7735_H

/*!****************************************************************************
 * Include
 */
#include "stdint.h"

/*!****************************************************************************
 * Define
 */
#define ST7735_W			10
#define ST7735_H			160
#define LCD_SPI				SPI1
#define LCD_DMA             DMA1_Channel3
#define LCD_TIM             TIM2
#define LCD_BRGHT_MAX       10
#define LCD_BRGHT_MIN       1
#define LCD_BRGHT_OFF       0
#define LCD_BRGHT_FREQ      1000

/*!****************************************************************************
 * Enumeration
 */
typedef enum{
	sky = 0x54fb,
	black = 0x0000,
	white = 0xFFFF,
	green = 0x07E0,
	blue = 0x001f,
	red = 0xF800,
	yellow = 0xFFE0,
	orange = 0xAB00,
	pink = 0xF97F,
	brown = 0x8200,
	gray = 0x8410,
	lilac = 0x91D4,
	darkGreen = 0x3DA5,
	halfLightGray = 0x39E6,
	halfLightYellow = 0xFFF8,
	halfLightRed = 0xFF18,
	halfLightGreen = 0xC7F8,
	halfLightBlue = 0x861F,
}color_type;

/*!****************************************************************************
 * Typedef
 */
typedef struct{
    uint8_t         colstart;
    uint8_t         rowstart;
    uint8_t         xstart;
    uint8_t         ystart;
    uint8_t         height;
    uint8_t         width;
    uint8_t         tabcolor;
    uint8_t         brght;
    uint8_t         brghtPrev;
    uint16_t        videoBff[ST7735_W * ST7735_H];
}st7735_type;

/*!****************************************************************************
 * Exported variables
 */
extern st7735_type      st7735;

/*!****************************************************************************
 * Macro functions
 */
inline void st7735_setPixel(uint16_t x, uint16_t y, uint16_t color){
	st7735.videoBff[y * ST7735_W + x] = color;
}

/*!****************************************************************************
 * Function declaration
 */
void st7735_init(void);
void st7735_deInit(void);
void st7735_sleepOn(void);
void st7735_sleepOff(void);
void st7735_setBrightness(uint8_t level);

#endif //st7735_H
/******************************** END OF FILE ********************************/
