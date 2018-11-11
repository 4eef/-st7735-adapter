/*!****************************************************************************
* @file    clock.h
* @author  4eef
* @version V1.0
* @date    07.08.2016
* @brief   --
*/
#ifndef clock_H
#define clock_H

/*!****************************************************************************
* Include
*/
#include "stm32f0xx.h"

/*!****************************************************************************
* User define
*/
#define F_APB1                  48000000

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* User typedef
*/

/*!****************************************************************************
* Extern viriables
*/

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
void initClocks(void);
void SystemClock_Cofig(void);
void Error_Handler(void);

#endif //clock_H
/***************** (C) COPYRIGHT ************** END OF FILE ******** 4eef ****/
