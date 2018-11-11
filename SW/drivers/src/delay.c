/*!****************************************************************************
* @file    Delay.c
* @author  4eef
* @version V1.0
* @date    14.08.2016
* @brief   --
*/

/*!****************************************************************************
* Include
*/
#include "Delay.h"

/*!****************************************************************************
* MEMORY
*/

/*!****************************************************************************
* @brief    Delay in microseconds
* @param    Time in microseconds
* @retval   
*/
void delay_us(uint16_t time){
    RCC->APB1ENR    |= RCC_APB1ENR_TIM3EN;					//Enable clock for delay timer
	TIM3->PSC       = F_APB1/1000000+1;
	TIM3->ARR       = time;
	TIM3->EGR       |= TIM_EGR_UG;
	TIM3->CR1       |= TIM_CR1_OPM;
	TIM3->CR1       |= TIM_CR1_CEN;
	while ((TIM3->CR1 & TIM_CR1_CEN) != 0) __NOP();
}

/***************** (C) COPYRIGHT ************** END OF FILE ******** 4eef ****/
