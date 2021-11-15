/******************************************************************************
*
* Module: Can module
*
* File Name: Can_cfg.h
*
* Description: hold the static configration for the can module
*
* Author: Ahmed Emad
******************************************************************************/

#ifndef CAN_CFG_H
#define CAN_CFG_H


/* Specifies if Controller 0,1 active or not */
#define CAN_CONTROLLER0_STATE    STD_ON
#define CAN_CONTROLLER1_STATE    STD_ON

/* Macro for defining Can clock in MHZ */
#define CAN_CLOCK       (16)

/* number of configured message objects */
#define NUMBER_OF_MESSAGE_OBJECTS  10


#endif 