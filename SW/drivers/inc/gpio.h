/*!****************************************************************************
* @file     gpio.c
* @author   Storozhenko Roman - D_EL
* @version  V1.0
* @date     20.07.2016
* @brief    gpio driver for stm32L4
*/
#ifndef GPIO_H
#define GPIO_H

/*!****************************************************************************
* Include
*/
#include "stm32f0xx.h"
#include "stdint.h"

/*!****************************************************************************
* User define
*/
#define JTAG_OFF                            (1)
//MODERy
#define GPIO_INPUT                          0x00
#define GPIO_GP_OUT                         0x01
#define GPIO_AF_MODE                        0x02
#define GPIO_ANALOG_MODE                    0x03
//OTy
#define GPIO_PUSH_PULL                      0x00
#define GPIO_OPEN_DRAIN                     0x01
//PUPDRy
#define GPIO_NO_PULL_UP_DOWN                0x00
#define GPIO_PULL_UP                        0x01
#define GPIO_PULL_DOWN                      0x02
#define GPIO_RESERVED                       0x03
//EXT_I
#define EXTI_MODE_DISABLE                   0x00
#define EXTI_MODE_RISE                      0x01
#define EXTI_MODE_FALL                      0x02
#define EXTI_MODE_BOTH                      0x03

/*!****************************************************************************
* User typedef
*/
typedef enum{
    pinm0   = 1 << 0,
    pinm1   = 1 << 1,
    pinm2   = 1 << 2,
    pinm3   = 1 << 3,
    pinm4   = 1 << 4,
    pinm5   = 1 << 5,
    pinm6   = 1 << 6,
    pinm7   = 1 << 7,
    pinm8   = 1 << 8,
    pinm9   = 1 << 9,
    pinm10  = 1 << 10,
    pinm11  = 1 << 11,
    pinm12  = 1 << 12,
    pinm13  = 1 << 13,
    pinm14  = 1 << 14,
    pinm15  = 1 << 15,
}gpioPin_type;

typedef enum{
    analogMode,                         ///< 0
    floatingInput,                      ///< 1
    inputWithPullUp,                    ///< 2
    inputWithPullDown,                  ///< 3
    outPushPull,                        ///< 4
    outOpenDrain,                       ///< 5
    alternateFunctionPushPull,          ///< 6
    alternateFunctionOpenDrain          ///< 7
}gpioMode_type;

typedef enum{
    speed2MHz,
    speed10MHz,
    speed50MHz,
}gpioSpeed_type;

typedef struct{
    GPIO_TypeDef    *p;                 ///< PORT
    uint16_t        mask;               ///< MASK PIN
    gpioMode_type   mode;               ///< MODE
    uint8_t         npin;               ///< NUMBER PIN
    uint8_t         iniState    :1;     ///< Initial state
    uint8_t         nAF         :4;     ///< Number alternate function
}pinMode_type;

typedef enum{
/*0 */  GP_RES0,
/*1 */  GP_RES1,
/*2 */  GP_RES2,
/*3 */  GP_RES3,
/*4 */  GP_LCD_Back,
/*5 */  GP_LCD_CS,
/*6 */  GP_LCD_DC,
/*7 */  GP_LCD_RST,
/*8 */  GP_LCD_SCK,
/*9 */  GP_RES4,
/*10*/  GP_LCD_SDA,
/*11*/  GP_RES5,
/*12*/  GP_RES6,
/*13*/  GP_RES7,
///*14*/  GP_SWDIO,
///*15*/  GP_SWCLK,
GP_NOT_USED
}GPnum_type;

/*!****************************************************************************
* Macro functions
*/
#define makepin(port, pin, mode, iniState, nAF)    { port, (1<<pin), mode, pin, iniState, nAF }

#define _gppin_set(port, pinmask)       (port->BSRR = (pinmask))
#define _gppin_reset(port, pinmask)     (port->BRR  = (pinmask))
#define _gppin_togle(port, pinmask)     (port->ODR  ^=(pinmask))
#define _gppin_get(port, pinmask)       (port->IDR &  (pinmask))
#define gppin_set(n)                    _gppin_set(pinsMode[n].p, pinsMode[n].mask)
#define gppin_reset(n)                  _gppin_reset(pinsMode[n].p, pinsMode[n].mask)
#define gppin_togle(n)                  _gppin_togle(pinsMode[n].p, pinsMode[n].mask)
#define gppin_get(n)                    _gppin_get(pinsMode[n].p, pinsMode[n].mask)


#define GPIO_TO_INT(GPIO)               (((uint32_t)(&(GPIO->CRL)) - GPIOA_BASE)>>10)
#define __PIN_TO_EXTI_IRQn(PIN)                         (PIN == 0)? EXTI0_IRQn :\
                                                        (PIN == 1)? EXTI1_IRQn :\
                                                        (PIN == 2)? EXTI2_IRQn :\
                                                        (PIN == 3)? EXTI3_IRQn :\
                                                        (PIN == 4)? EXTI4_IRQn :\
                                                        (PIN <= 9)? EXTI9_5_IRQn : EXTI15_10_IRQn
#define _PIN_TO_EXTI_IRQn(PIN)                          __PIN_TO_EXTI_IRQn(PIN)
#define PIN_TO_EXTI_IRQn(PIN)                           _PIN_TO_EXTI_IRQn(PIN)
#define __PIN_TO_EXTI_HANDLER(PIN)                      EXTI##PIN##_IRQHandler
#define _PIN_TO_EXTI_HANDLER(PIN)                       __PIN_TO_EXTI_HANDLER(PIN)
#define PIN_TO_EXTI_HANDLER(PIN)                        _PIN_TO_EXTI_HANDLER(PIN)
#define _EXTI_INIT(GPIO, PIN, EXTI_MODE, NVIC_PRIORITY) do{\
                        AFIO->EXTICR[PIN/4] = (AFIO->EXTICR[PIN/4] & ~((uint16_t)0x0F<<((PIN % 4)<<2)))|(GPIO_TO_INT(GPIO)<<((PIN % 4)<<2));\
                        BIT_BAND_PER(EXTI->FTSR,1UL<<PIN)=!!(EXTI_MODE & 0x02);\
                        BIT_BAND_PER(EXTI->RTSR,1UL<<PIN)=!!(EXTI_MODE & 0x01);\
                        NVIC_SetPriority(PIN_TO_EXTI_IRQn(PIN),NVIC_PRIORITY);\
                        (EXTI_MODE>0)? NVIC_EnableIRQ(PIN_TO_EXTI_IRQn(PIN)): NVIC_DisableIRQ(PIN_TO_EXTI_IRQn(PIN));\
                        EXTI->PR = 1UL<<PIN;\
                        BIT_BAND_PER(EXTI->IMR,1UL<<PIN)=!!(EXTI_MODE);\
                        }while(0)    
#define EXTI_INIT(GPIO, PIN, EXTI_MODE, NVIC_PRIORITY)  _EXTI_INIT(GPIO, PIN, EXTI_MODE, NVIC_PRIORITY)
//example: EXTI_INIT(GPIOA, 9, EXTI_MODE_BOTH, 15);

/*!****************************************************************************
* Extern viriables
*/
extern pinMode_type   const pinsMode[];

/*!****************************************************************************
* Prototypes for the functions in gpio.c
*/
void gppin_init(GPIO_TypeDef *port, uint8_t npin, gpioMode_type mode, uint8_t iniState, uint8_t nAF);
void initGpios(void);
void Init_EXTI_CC1101_0(void);
void Init_EXTI_CC1101_1(void);

#endif //GPIO_H
/*************** (C) COPYRIGHT ************** END OF FILE ********* D_EL *****/
