/*!****************************************************************************
* @file     gpio.c
* @author   Storozhenko Roman - D_EL, 14.08.2016 - 4eef
* @version  V1.0
* @date     20.07.2016
* @date     02.08.2016  fix set nAF
* @brief    gpio driver for stm32F0
*/

/*!****************************************************************************
* Include
*/
#include "gpio.h"

/*!****************************************************************************
* MEMORY
*/
pinMode_type   const pinsMode[] = {
/*0 */  makepin(GPIOB,  8,  floatingInput,              0,  0),     //RES0
/*1 */  makepin(GPIOF,  0,  floatingInput,              0,  0),     //RES1
/*2 */  makepin(GPIOF,  1,  floatingInput,              0,  0),     //RES2
/*3 */  makepin(GPIOA,  0,  floatingInput,              0,  0),     //RES3
/*4 */  makepin(GPIOA,  1,  alternateFunctionPushPull,  0,  2),     //LCD_Back
/*5 */  makepin(GPIOA,  2,  outPushPull,                0,  0),     //LCD_CS
/*6 */  makepin(GPIOA,  3,  outPushPull,                0,  0),     //LCD_DC
/*7 */  makepin(GPIOA,  4,  outPushPull,                0,  0),     //LCD_RES
/*8 */  makepin(GPIOA,  5,  alternateFunctionPushPull,  0,  0),     //LCD_SCK
/*9 */  makepin(GPIOA,  6,  outPushPull,                0,  0),     //LCD_PSPLY
/*10*/  makepin(GPIOA,  7,  alternateFunctionPushPull,  0,  0),     //LCD_SDA
/*11*/  makepin(GPIOB,  1,  floatingInput,              0,  0),     //RES5
/*11*/  makepin(GPIOA,  11, floatingInput,              0,  0),     //RES6
/*11*/  makepin(GPIOA,  12, floatingInput,              0,  0),     //RES7
///*14*/  makepin(GPIOA,  13, floatingInput,              0,  0),     //SWDIO
///*15*/  makepin(GPIOA,  14, floatingInput,              0,  0),     //SWCLK
};
uint32_t pinNum = sizeof(pinsMode) / sizeof(pinMode_type);

/*!****************************************************************************
* InitAllGpio
*/
void initGpios(void){
//    RCC->APB2ENR    |= RCC_APB2ENR_AFIOEN;                      //AFIO
//#if (JTAG_OFF > 0)
//    AFIO->MAPR      |= AFIO_MAPR_SWJ_CFG_1; //JTAG
//#endif
    pinMode_type *pgpios;
    pinMode_type *pgpiosEnd;
    
    pgpios = (pinMode_type*)pinsMode;
    pgpiosEnd = pgpios + pinNum;
    
    while(pgpios < pgpiosEnd){
        gppin_init(pgpios->p, pgpios->npin, pgpios->mode, pgpios->iniState, pgpios->nAF);
        pgpios++;
    }
}

/*!****************************************************************************
*
*/
void gppin_init(GPIO_TypeDef *port, uint8_t npin, gpioMode_type mode, uint8_t iniState, uint8_t nAF){
    //Clock enable
    if(port == GPIOA)   RCC->AHBENR    |= RCC_AHBENR_GPIOAEN;
    if(port == GPIOB)   RCC->AHBENR    |= RCC_AHBENR_GPIOBEN;
    if(port == GPIOC)   RCC->AHBENR    |= RCC_AHBENR_GPIOCEN;
//    if(port == GPIOD)   RCC->AHBENR    |= RCC_AHBENR_GPIODEN;
//    if(port == GPIOE)   RCC->AHBENR    |= RCC_AHBENR_GPIOEEN;
    if(port == GPIOF)   RCC->AHBENR    |= RCC_AHBENR_GPIOFEN;
//    if(port == GPIOG)   RCC->AHBENR    |= RCC_AHBENR_GPIOGEN;
//    if(port == GPIOH)   RCC->AHBENR    |= RCC_AHBENR_GPIOHEN;
//    if(port == GPIOO)   RCC->AHBENR    |= RCC_AHBENR_GPIOOEN;
    
    if(iniState != 0){
        port->BSRR = (1<<npin);
    }
    else{
        port->BRR = (1<<npin);
    }
    
    /*
    * Clear bit field
    */
    port->MODER         &= ~(0x03 << (2 * npin));
    port->OTYPER        &= ~(1<<npin);
    port->PUPDR         &= ~(GPIO_RESERVED << (2*npin));
    port->AFR[npin / 8] &= ~(GPIO_AFRL_AFRL0_Msk << (4*(npin % 8)));
    
    switch(mode){
        case analogMode:
            port->MODER |= GPIO_ANALOG_MODE << (2*npin);
            break;
  
        case floatingInput:
            break;
            
        case inputWithPullUp:
            port->PUPDR |= GPIO_PULL_UP << (2*npin);
            break;
            
        case inputWithPullDown:
            port->PUPDR |= GPIO_PULL_DOWN << (2*npin);
            break;
            
        case outPushPull:
            port->MODER |= GPIO_GP_OUT << (2*npin);
            port->OTYPER |= GPIO_PUSH_PULL << npin;
            break;
            
        case outOpenDrain:
            port->MODER |= GPIO_GP_OUT << (2*npin);
            port->OTYPER |= GPIO_OPEN_DRAIN << npin;
            break;
            
       case alternateFunctionPushPull:
            port->MODER |= GPIO_AF_MODE << (2*npin);
            port->OTYPER |= GPIO_PUSH_PULL << npin;
            break;
            
        case alternateFunctionOpenDrain:
            port->MODER |= GPIO_AF_MODE << (2*npin);
            port->OTYPER |= GPIO_OPEN_DRAIN << npin;
            break;   
    }
    
    //Set number alternate function
    port->AFR[npin / 8] |= nAF << (4*(npin % 8));
}

/*************** (C) COPYRIGHT ************** END OF FILE ********* D_EL *****/
