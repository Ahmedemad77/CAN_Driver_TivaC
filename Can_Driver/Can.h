/******************************************************************************
*
* Module: Can module
*
* File Name: can.h
*
* Description: Header file for Can driver 
*
* Author: Ahmed Emad
******************************************************************************/
#ifndef CAN_H
#define CAN_H

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* Can Pre-Compile Configuration Header file */
#include "Can_Cfg.h"

/* Can General Types Definitions */
#include "Can_GeneralTypes.h"

/* Non AUTOSAR files */
#include "Common_Macros.h"





/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/


/************************************************************************************
* Service Name: Can_Init
* Sync/Async: Synchronous 
* Reentrancy: Non reentrant 
* Parameters (in): Can_ConfigType
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Init Bit Timing also all configured message objects
************************************************************************************/
void Can_Init(Can_ConfigType* Config);

Std_ReturnType Can_Write();
void  Can_Main_Function_Write(void);
void  Can_Main_Function_Read(void);



/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/



#endif