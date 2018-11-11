/*!****************************************************************************
* @file    clock.c
* @author  4eef
* @version V1.0
* @date    07.08.2016
* @brief   --
*/

/*!****************************************************************************
* Include
*/
#include "clock.h"

/*!****************************************************************************
* MEMORY
*/

/*!****************************************************************************
* @brief    System clock settings initializer
* @param    
* @retval   
*/
void initClocks(void){
    RCC->CR2        |= RCC_CR2_HSI14ON;
    while((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) __NOP();
    RCC->CR2        |= RCC_CR2_HSI48ON;
    while((RCC->CR2 & RCC_CR2_HSI48RDY) == 0) __NOP();
    RCC->CFGR       |= (3 << RCC_CFGR_SW_Pos);
    while((RCC->CFGR & RCC_CFGR_SWS >> RCC_CFGR_SWS_Pos) != 3) __NOP();
    FLASH->ACR      |= FLASH_ACR_PRFTBE;
    while((FLASH->ACR & FLASH_ACR_PRFTBS) == 0) __NOP();
    FLASH->ACR      |= (1 << FLASH_ACR_LATENCY_Pos);
    RCC->CFGR3      &= ~RCC_CFGR3_USBSW;
    RCC->CFGR3      |= RCC_CFGR3_I2C1SW;
}

/***************** (C) COPYRIGHT ************** END OF FILE ******** 4eef ****/
