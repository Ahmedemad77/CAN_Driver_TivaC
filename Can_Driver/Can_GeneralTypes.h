/******************************************************************************
*
* Module: Can driver 
*
* File Name: Can_GeneralTypes.h 
*
* Description: Include general types definitions 
*
* Author: Ahmed Emad
******************************************************************************/



#ifndef INCLUDES_CAN_GENERALTYPES_H_
#define INCLUDES_CAN_GENERALTYPES_H_ 

#include "Std_Types.h"
#include "Can_Cfg.h"

/* Specifies values for enable or disable controllers */
#define CONTROLLER_ENABLED (1U)
#define CONTROLLER_DISABLED (0U)

/* Defines Ids for differnt Can controllers */
#define CAN_CONTROLLER_0 (0U)
#define CAN_CONTROLLER_1 (1U)


#define CAN_ENABLE_RMOTE_TRASNMIT  (1U)
#define CAN_DISABLE_RMOTE_TRASNMIT  (0U)

/*
 * Macros for CAN Status
 */
#define CAN_INITIALIZED                (1U)
#define CAN_NOT_INITIALIZED            (0U)

#define CAN_BUSY (uint8)0x02

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Enum to define the values for the ID Types */
typedef enum 
{
    STANDARD,EXTENDED
}CanIdType;

/* Enum to define the values for the modes of message object*/
typedef enum 
{
    RECEIVE,TRASMIT
}CanObjectType;

/* Eum to specify Message Object Processing Type  */
typedef enum{
    INTERRUPT,
    POLLING
}Can_MesProceccingType;

/* type definition for message object ID */
typedef uint32 Can_MessObjectId;


/* type definition for message object Filter mask */
typedef uint8 Can_MessObjectLength;


/* structure to hold the data of message object */
typedef struct Can_HardwareObject
{
    /* variable to hold the data */
    uint64 MessObjData;

    /* Specifies Message ID either 11 bit OR 29 bit */
    Can_MessObjectId MessObjectId;

    /* Specifies the message object Filter */
    uint32 MessObjFilter;
    
    /* Specifies the can controller message belongs to  */
    uint8 ControllerId;

    /* Specifies the message Id standard or extended */
    uint8 IdType;

    /* Specifies the message Receive  or Trasmit */
    uint8 ObjectType;

    /* Specefies the processing of the Message either interrupt or pooling  */
    uint8 MesProcessType;

    /* Specifies Enable or Disable start message transmission 
     *  if a matching remote frame has been received */
    uint8 RmTransmitEnable;
    
    /* Specifies enable or disable filter mask */
    uint8 MessFilterEnable;

    /* Specifies the length of date of the related message */
    Can_MessObjectLength MessLength;

    /* Specifies ID of this  message relative to message objects
       incase of Tiva C ranges from 0 to 31 */
    uint8 MessageId;

}Can_HardwareObject;

/* Structure to hold the configration of bit Timing */
typedef struct Can_BaudrateConfig
{
    /* Specifies the Baud rate in Kbps Tiva C supports speeds in range 1 Kbps up to 1000 Kbps */
    uint16 BaudRate;

    /* Specifies Propgation Delay  */
    uint8 PropDelay;

    /* Specifies Segment 1  Delay  */
    uint8 Segment1; 
    
    /* Specifies Segment 2 Delay  */
    uint8 Segment2; 

    /* Specifies the number of time quanta that bit time consits of 
        in Tiva C ranges from 4 to 25 i.e. Tbit_time=n*tq */ 
    uint8 numTimeQuanta;

    /* Specifies Synchronization Jump Width  */
    uint8 SyncJumpWidth; 

}Can_BaudrateConfig;

/* Structure for all configration Parmaters passed to Init Function */     
typedef struct 
{
    Can_BaudrateConfig Can_BrateConfig;
    Can_HardwareObject Can_Messages[NUMBER_OF_MESSAGE_OBJECTS];
}Can_ConfigType;


#endif