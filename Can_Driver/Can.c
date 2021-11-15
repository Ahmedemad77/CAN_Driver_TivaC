/******************************************************************************
*
* Module: Can Driver
*
* File Name: Can.c
*
* Description: Source file for Can driver   
*
* Author: Ahmed Emad
******************************************************************************/

#include "Can.h"
#include "tm4c123gh6pm_registers.h"

/* pointer to point to the first element in the array of structrue of configration*/
STATIC const Can_ConfigType* Can_config =  NULL_PTR;


STATIC uint8 Can_Status = CAN_NOT_INITIALIZED;
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
void Can_Init(Can_ConfigType* CanConfig){

Can_config =CanConfig;

if (Can_Status==CAN_NOT_INITIALIZED)
{
    /* Enable or disable the Can Module 0   */
    if (CAN_CONTROLLER0_STATE==STD_ON)
    {
        /* activate clock to Can 0 Module */
        SYSCTL_RCGCCAN_REG |=(1<<0);
    }
    else
    {
        /* disable clock to Can 0 Module */
        SYSCTL_RCGCCAN_REG &=~(1<<0);
    }
    
    /* Enable or disable the Can Module 1  */
    if (CAN_CONTROLLER1_STATE==STD_ON)
    {
        /* activate clock to Can 1 Module */
        SYSCTL_RCGCCAN_REG |=(1<<1);
    }else
    {
        /* disable clock to Can 1 Module */
        SYSCTL_RCGCCAN_REG &=~(1<<1);
    }
    
    /* Setting the init pin in Can control 0  Register  if controller is activated*/
    if (CAN_CONTROLLER0_STATE==STD_ON)
    {
        CAN0_CTL_REG |=(1<<0);
    }
    else
    {
        /* disable clock to Can 0 Module */
        CAN0_CTL_REG &=~(1<<0);
    }
    
    /* Setting the init pin in Can control 1  Register  if controller is activated*/
    if (CAN_CONTROLLER1_STATE==STD_ON)
    {
        CAN1_CTL_REG |=(1<<0);
    }else
    {
        /* disable clock to Can 1 Module */
        CAN1_CTL_REG &=~(1<<0);
    }
    /*******************************************************************************
    *                              Baud Rate init                                  *
    *******************************************************************************/
    /* Enable access to CanBit register */
    

    /* Initialiazes the bit timing  For CAN 0*/
    if (CAN_CONTROLLER0_STATE==STD_ON)
    {
        /* Enable access to CanBit register */
        CAN0_CTL_REG |=(1<<6);
        /* first clear the time bit register */
        CAN0_BIT_REG =0;
        /* Set segment 2 timing */
        CAN0_BIT_REG |=((CanConfig->Can_BrateConfig.Segment2&0x07)<<12);
        /* Set segment 1 timing */
        CAN0_BIT_REG |=((CanConfig->Can_BrateConfig.Segment1&0x0F)<<8);
        /* Set Synchronization Jump Width Bits */
        CAN0_BIT_REG |=((CanConfig->Can_BrateConfig.SyncJumpWidth)<<6);
        /* calculate  Bit Time in microseconds */
        uint16 bitTime =1000/CanConfig->Can_BrateConfig.BaudRate;
        /* Calculate quantam time in nanoseconds */
        uint16 quantamTime = bitTime*1000/(Can_config->Can_BrateConfig.numTimeQuanta);
        /* calculate Baud Rate Prescaler  */
        uint8 baudRatePrescaler = (quantamTime* CAN_CLOCK)/1000;
        /* Set the baud rate prescaler */
        CAN0_BIT_REG |=(baudRatePrescaler&0x1F);
        /* Disable access to CanBit register */
        CAN0_CTL_REG &=~(1<<6);
    }
    else
    {
    }

    /* Initialiazes the bit timing  For CAN 1*/
    if (CAN_CONTROLLER1_STATE==STD_ON)
    {
        /* Enable access to CanBit register */
        CAN1_CTL_REG |=(1<<6);
        /* first clear the time bit register */
        CAN1_BIT_REG =0;
        /* Set segment 2 timing */
        CAN1_BIT_REG |=((CanConfig->Can_BrateConfig.Segment2&0x07)<<12);
        /* Set segment 1 timing */
        CAN1_BIT_REG |=((CanConfig->Can_BrateConfig.Segment1&0x0F)<<8);
        /* Set Synchronization Jump Width Bits */
        CAN1_BIT_REG |=((CanConfig->Can_BrateConfig.SyncJumpWidth)<<6);
        /* calculate  Bit Time in microseconds */
        uint16 bitTime =1000/CanConfig->Can_BrateConfig.BaudRate;
        /* Calculate quantam time in nanoseconds */
        uint16 quantamTime = bitTime*1000/(Can_config->Can_BrateConfig.numTimeQuanta);
        /* calculate Baud Rate Prescaler  */
        uint8 baudRatePrescaler = (quantamTime* CAN_CLOCK)/1000;
        /* Set the baud rate prescaler */
        CAN1_BIT_REG |=(baudRatePrescaler&0x1F);
        /* Disable access to CanBit register */
        CAN1_CTL_REG &=~(1<<6);
    }else
    {
    }

    /*******************************************************************************
    *                              Messages Object init                            *
    *******************************************************************************/
    
    /* Loop over all messages object and init them  */
    for (uint8 count = 0; count < NUMBER_OF_MESSAGE_OBJECTS ; count++)
    {   
        switch (Can_config->Can_Messages[count].ControllerId)
        {
        case CAN_CONTROLLER_0:
            if (Can_config->Can_Messages[count].ObjectType==TRASMIT)
            {
                /* reset MNUM for the next message to not overwrite previos one*/
                CAN0_IF1CRQ_REG &=(0XFFFFFFC0);
                
                /* Set the WRNRD bit to specify a write to the CANIFnCMASK
                 * Set MASK bit to transfer Transfer IDMASK + DIR + MXTD 
                 * of the message object
                 * Set ARB bit to Transfer ID + DIR + XTD + MSGVAL 
                 * Set the control bit to allow transfer of it 
                 */
                CAN0_IF1CMSK_REG |=(0x000000F0);

                /* Check of processing type if interrupt or polling */
                if (Can_config->Can_Messages[count].MesProcessType==POLLING)
                {
                    /* Set CLRINTPND bit in CANIFnCMASK register  */
                    CAN0_IF1CMSK_REG |=(1<<3);  
                }else
                {
                    /* Clear CLRINTPND bit in CANIFnCMASK register  */
                    CAN0_IF1CMSK_REG &=~(1<<3);  
                }
                
                /* set bits of data transfer control */
                if (Can_config->Can_Messages[count].MessLength > 3)
                {
                    /* Set both DATAA & DATAB bits */
                    CAN0_IF1CMSK_REG |=(0x00000003);
                }else
                {
                    /* Set  DATAA bit only */
                    CAN0_IF1CMSK_REG |=(0x00000002);
                }
                
                /* Check if masks used in acceptance filter and set in this case */
                if (Can_config->Can_Messages[count].MessFilterEnable==STD_ON)
                {
                    /* enable use of message filter by setting the
                     *  UMASK bit in the CANIFnMCTL register */
                    CAN0_IF1MCTL_REG |=(1<<12);
                    /* Insert The message filter  */
                    if (Can_config->Can_Messages[count].IdType==STANDARD)
                    {
                        /* Case Standard filter is used */
                        CAN0_IF1MSK2_REG &=(0xFFFFE003);
                        CAN0_IF1MSK2_REG |=(((Can_config->Can_Messages[count].MessObjFilter)&(0x000007FF))<<2);
                    }else
                    {
                        /* Extenden id is used  */

                        CAN0_IF1MSK1_REG &=(0xFFFF0000);
                        /* Set the bits 15:0 */
                        CAN0_IF1MSK1_REG |=((Can_config->Can_Messages[count].MessObjFilter)&(0x0000FFFF));
                        /* Set the bits 28:16 */
                        CAN0_IF1MSK2_REG &= (0xFFFFE000);
                        CAN0_IF1MSK2_REG |=((Can_config->Can_Messages[count].MessObjFilter&(0x1FFF0000))>>16);
                        /* Enable XTD bit and DIR bit in acceptance filter */
                        CAN0_IF1MSK2_REG |=(0X0000A000);

                    }
                    
                    
                    
                }else
                {
                }
                
                /* Set the ID field in the Can ARB Rgisters */
                if (Can_config->Can_Messages[count].IdType==EXTENDED)
                {
                    /* Extended Id */
                    CAN0_IF1ARB1_REG &= (0xFFFF0000);
                    /* Set the bits 15:0 */
                    CAN0_IF1ARB1_REG |=((Can_config->Can_Messages[count].MessageId)&(0x0000FFFF));
                    /* Set the bits 28:16  of the ID  */
                    CAN0_IF1ARB2_REG &= (0xFFFFE000);
                    CAN0_IF1ARB2_REG |=((Can_config->Can_Messages[count].MessageId&(0x1FFF0000))>>16);
                    /* Set the XTD bit to specify Extended bit 
                     * Set DIR to specifiy transmit object
                     * Set Message Field */ 
                    CAN0_IF1ARB2_REG |= (0x0000E000);

                }else
                {
                    /* Standard Id is used  */
                    CAN0_IF1ARB2_REG &=(0xFFFFE003);
                    CAN0_IF1ARB2_REG |=(((Can_config->Can_Messages[count].MessageId)&(0x000007FF))<<2);
                    /* Set DIR to specifiy transmit object
                     * Set Message Field */ 
                    CAN0_IF1ARB2_REG |= (0x0000A000);

                }
                
                /* Configure if message generates interrupt upon successful transmission */
                if (Can_config->Can_Messages[count].MesProcessType==INTERRUPT)
                {
                    /* Set the TXIE */
                    CAN0_IF1MCTL_REG |=(1<<11);
                }else
                {
                }
                
                /* Set EOB as FIFO  not supported  */
                CAN0_IF1MCTL_REG |=(1<<7);

                /* Configure the  RMTEN bit to enable the TXRQST bit to be set on the reception of a matching
                 * remote frame allowing automatic transmission */
                if (Can_config->Can_Messages[count].RmTransmitEnable==STD_ON )
                {
                    CAN0_IF1MCTL_REG |=(1<<9);
                }else
                {
                }
                

                /* Configure the DLC[3:0] field to specify the size of the data frame  */
                CAN0_IF1MCTL_REG &=(0xFFFFFFF0);
                CAN0_IF1MCTL_REG |= (Can_config->Can_Messages[count].MessLength&(0x0F));
                
                /* Load the data to be transmitted into the CAN IFn Data */
                /* Byte 1,0 */
                CAN0_IF1DA1_REG &=(0XFFFF0000);
                CAN0_IF1DA1_REG |= (uint32)(Can_config->Can_Messages[count].MessObjData&(0x000000000000FFFF));
                /* Byte 3,2 */
                CAN0_IF1DA2_REG &=(0XFFFF0000);
                CAN0_IF1DA2_REG |= (uint32)((Can_config->Can_Messages[count].MessObjData&(0x00000000FFFF0000))>>16);
                /* Byte 5,4 */
                CAN0_IF1DB1_REG &=(0XFFFF0000);
                CAN0_IF1DB1_REG |= (uint32)((Can_config->Can_Messages[count].MessObjData&(0x0000FFFF00000000))>>32);
                /* Byte 7,6 */
                CAN0_IF1DB2_REG &=(0XFFFF0000);
                CAN0_IF1DB2_REG |= (uint32)((Can_config->Can_Messages[count].MessObjData&(0xFFFF000000000000))>>48);
                
                /* Program the number of the message object to be transmitted in the MNUM field */
                CAN0_IF1CRQ_REG &=(0XFFFFFFC0);
                CAN0_IF1CRQ_REG |=(Can_config->Can_Messages[count].MessageId&(0x3F));
                
                /* Set the  TXRQST bit  */
                CAN0_IF1MCTL_REG |=(1<<8);
                
            }else
            {
                /* Configure Recive Messages objects */

                /* reset MNUM for the next message to not overwrite previos one*/
                CAN0_IF2CRQ_REG &=(0XFFFFFFC0);
                
                /* Set the WRNRD bit to specify a write to the CANIFnCMASK
                 * Set MASK bit to transfer Transfer IDMASK + DIR + MXTD 
                 * of the message object
                 * Set ARB bit to Transfer ID + DIR + XTD + MSGVAL 
                 * Set the control bit to allow transfer of it 
                 */
                CAN0_IF2CMSK_REG |=(0x000000F0);

                /* Check of processing type if interrupt or polling */
                if (Can_config->Can_Messages[count].MesProcessType==POLLING)
                {
                    /* Set CLRINTPND bit in CANIFnCMASK register  */
                    CAN0_IF2CMSK_REG |=(1<<3);  
                }else
                {
                    /* Clear CLRINTPND bit in CANIFnCMASK register  */
                    CAN0_IF2CMSK_REG &=~(1<<3);  
                }
                
                
                /* Check if masks used in acceptance filter and set in this case */
                if (Can_config->Can_Messages[count].MessFilterEnable==STD_ON)
                {
                    /* enable use of message filter by setting the
                     *  UMASK bit in the CANIFnMCTL register */
                    CAN0_IF2MCTL_REG |=(1<<12);
                    /* Insert The message filter  */
                    if (Can_config->Can_Messages[count].IdType==STANDARD)
                    {
                        /* Case Standard filter is used */
                        CAN0_IF2MSK2_REG &=(0xFFFFE003);
                        CAN0_IF2MSK2_REG |=(((Can_config->Can_Messages[count].MessObjFilter)&(0x000007FF))<<2);
                    }else
                    {
                        /* Extenden id is used  */

                        CAN0_IF2MSK1_REG &=(0xFFFF0000);
                        /* Set the bits 15:0 */
                        CAN0_IF2MSK1_REG |=((Can_config->Can_Messages[count].MessObjFilter)&(0x0000FFFF));
                        /* Set the bits 28:16 */
                        CAN0_IF2MSK2_REG &= (0xFFFFE000);
                        CAN0_IF2MSK2_REG |=((Can_config->Can_Messages[count].MessObjFilter&(0x1FFF0000))>>16);
                        /* Enable XTD bit and DIR bit in acceptance filter */
                        CAN0_IF2MSK2_REG |=(0X0000A000);

                    }
                    
                    
                    
                }else
                {
                }
                
                /* Set the ID field in the Can ARB Rgisters */
                if (Can_config->Can_Messages[count].IdType==EXTENDED)
                {
                    /* Extended Id */
                    CAN0_IF2ARB1_REG &= (0xFFFF0000);
                    /* Set the bits 15:0 */
                    CAN0_IF2ARB1_REG |=((Can_config->Can_Messages[count].MessageId)&(0x0000FFFF));
                    /* Set the bits 28:16  of the ID  */
                    CAN0_IF2ARB2_REG &= (0xFFFFE000);
                    CAN0_IF2ARB2_REG |=((Can_config->Can_Messages[count].MessageId&(0x1FFF0000))>>16);
                    /* Set the XTD bit to specify Extended bit 
                     * Clear DIR to specifiy Receive object
                     * Set Message Field */ 
                    CAN0_IF2ARB2_REG &=~(0x0000E000);
                    CAN0_IF2ARB2_REG |= (0x0000C000);

                }else
                {
                    /* Standard Id is used  */
                    CAN0_IF2ARB2_REG &=(0xFFFFE003);
                    CAN0_IF2ARB2_REG |=(((Can_config->Can_Messages[count].MessageId)&(0x000007FF))<<2);
                    /* Clear DIR to specifiy transmit object
                     * Set Message Field */ 
                    CAN0_IF2ARB2_REG &=~(0x0000E000);
                    CAN0_IF2ARB2_REG |= (0x00008000);

                }
                
                /* Configure if message generates interrupt upon successful transmission */
                if (Can_config->Can_Messages[count].MesProcessType==INTERRUPT)
                {
                    /* Set the TXIE */
                    CAN0_IF2MCTL_REG |=(1<<10);
                }else
                {
                }
                
                /* Set EOB as FIFO  not supported  */
                CAN0_IF2MCTL_REG |=(1<<7);

                /* Configure the DLC[3:0] field to specify the size of the data frame  */
                CAN0_IF2MCTL_REG &=(0xFFFFFFF0);
                CAN0_IF2MCTL_REG |= (Can_config->Can_Messages[count].MessLength&(0x0F));
                
                /* Program the number of the message object to be transmitted in the MNUM field */
                CAN0_IF2CRQ_REG &=(0XFFFFFFC0);
                CAN0_IF2CRQ_REG |=(Can_config->Can_Messages[count].MessageId&(0x3F));
                
                   
            }
            break;
            
        case CAN_CONTROLLER_1:
            if (Can_config->Can_Messages[count].ObjectType==TRASMIT)
            {
                /* reset MNUM for the next message to not overwrite previos one*/
                CAN1_IF1CRQ_REG &=(0XFFFFFFC0);
                
                /* Set the WRNRD bit to specify a write to the CANIFnCMASK
                 * Set MASK bit to transfer Transfer IDMASK + DIR + MXTD 
                 * of the message object
                 * Set ARB bit to Transfer ID + DIR + XTD + MSGVAL 
                 * Set the control bit to allow transfer of it 
                 */
                CAN1_IF1CMSK_REG |=(0x000000F0);

                /* Check of processing type if interrupt or polling */
                if (Can_config->Can_Messages[count].MesProcessType==POLLING)
                {
                    /* Set CLRINTPND bit in CANIFnCMASK register  */
                    CAN1_IF1CMSK_REG |=(1<<3);  
                }else
                {
                    /* Clear CLRINTPND bit in CANIFnCMASK register  */
                    CAN1_IF1CMSK_REG &=~(1<<3);  
                }
                
                /* set bits of data transfer control */
                if (Can_config->Can_Messages[count].MessLength > 3)
                {
                    /* Set both DATAA & DATAB bits */
                    CAN1_IF1CMSK_REG |=(0x00000003);
                }else
                {
                    /* Set  DATAA bit only */
                    CAN1_IF1CMSK_REG |=(0x00000002);
                }
                
                /* Check if masks used in acceptance filter and set in this case */
                if (Can_config->Can_Messages[count].MessFilterEnable==STD_ON)
                {
                    /* enable use of message filter by setting the
                     *  UMASK bit in the CANIFnMCTL register */
                    CAN1_IF1MCTL_REG |=(1<<12);
                    /* Insert The message filter  */
                    if (Can_config->Can_Messages[count].IdType==STANDARD)
                    {
                        /* Case Standard filter is used */
                        CAN1_IF1MSK2_REG &=(0xFFFFE003);
                        CAN1_IF1MSK2_REG |=(((Can_config->Can_Messages[count].MessObjFilter)&(0x000007FF))<<2);
                    }else
                    {
                        /* Extenden id is used  */

                        CAN1_IF1MSK1_REG &=(0xFFFF0000);
                        /* Set the bits 15:0 */
                        CAN1_IF1MSK1_REG |=((Can_config->Can_Messages[count].MessObjFilter)&(0x0000FFFF));
                        /* Set the bits 28:16 */
                        CAN1_IF1MSK2_REG &= (0xFFFFE000);
                        CAN1_IF1MSK2_REG |=((Can_config->Can_Messages[count].MessObjFilter&(0x1FFF0000))>>16);
                        /* Enable XTD bit and DIR bit in acceptance filter */
                        CAN1_IF1MSK2_REG |=(0X0000A000);

                    }
                    
                    
                    
                }else
                {
                }
                
                /* Set the ID field in the Can ARB Rgisters */
                if (Can_config->Can_Messages[count].IdType==EXTENDED)
                {
                    /* Extended Id */
                    CAN1_IF1ARB1_REG &= (0xFFFF0000);
                    /* Set the bits 15:0 */
                    CAN1_IF1ARB1_REG |=((Can_config->Can_Messages[count].MessageId)&(0x0000FFFF));
                    /* Set the bits 28:16  of the ID  */
                    CAN1_IF1ARB2_REG &= (0xFFFFE000);
                    CAN1_IF1ARB2_REG |=((Can_config->Can_Messages[count].MessageId&(0x1FFF0000))>>16);
                    /* Set the XTD bit to specify Extended bit 
                     * Set DIR to specifiy transmit object
                     * Set Message Field */ 
                    CAN1_IF1ARB2_REG |= (0x0000E000);

                }else
                {
                    /* Standard Id is used  */
                    CAN1_IF1ARB2_REG &=(0xFFFFE003);
                    CAN1_IF1ARB2_REG |=(((Can_config->Can_Messages[count].MessageId)&(0x000007FF))<<2);
                    /* Set DIR to specifiy transmit object
                     * Set Message Field */ 
                    CAN1_IF1ARB2_REG |= (0x0000A000);

                }
                
                /* Configure if message generates interrupt upon successful transmission */
                if (Can_config->Can_Messages[count].MesProcessType==INTERRUPT)
                {
                    /* Set the TXIE */
                    CAN1_IF1MCTL_REG |=(1<<11);
                }else
                {
                }
                
                /* Set EOB as FIFO  not supported  */
                CAN1_IF1MCTL_REG |=(1<<7);

                /* Configure the  RMTEN bit to enable the TXRQST bit to be set on the reception of a matching
                 * remote frame allowing automatic transmission */
                if (Can_config->Can_Messages[count].RmTransmitEnable==STD_ON )
                {
                    CAN1_IF1MCTL_REG |=(1<<9);
                }else
                {
                }
                

                /* Configure the DLC[3:0] field to specify the size of the data frame  */
                CAN1_IF1MCTL_REG &=(0xFFFFFFF0);
                CAN1_IF1MCTL_REG |= (Can_config->Can_Messages[count].MessLength&(0x0F));
                
                /* Load the data to be transmitted into the CAN IFn Data */
                /* Byte 1,0 */
                CAN1_IF1DA1_REG &=(0XFFFF0000);
                CAN1_IF1DA1_REG |= (uint32)(Can_config->Can_Messages[count].MessObjData&(0x000000000000FFFF));
                /* Byte 3,2 */
                CAN1_IF1DA2_REG &=(0XFFFF0000);
                CAN1_IF1DA2_REG |= (uint32)((Can_config->Can_Messages[count].MessObjData&(0x00000000FFFF0000))>>16);
                /* Byte 5,4 */
                CAN1_IF1DB1_REG &=(0XFFFF0000);
                CAN1_IF1DB1_REG |= (uint32)((Can_config->Can_Messages[count].MessObjData&(0x0000FFFF00000000))>>32);
                /* Byte 7,6 */
                CAN1_IF1DB2_REG &=(0XFFFF0000);
                CAN1_IF1DB2_REG |= (uint32)((Can_config->Can_Messages[count].MessObjData&(0xFFFF000000000000))>>48);
                
                /* Program the number of the message object to be transmitted in the MNUM field */
                CAN1_IF1CRQ_REG &=(0XFFFFFFC0);
                CAN1_IF1CRQ_REG |=(Can_config->Can_Messages[count].MessageId&(0x3F));
                
                /* Set the  TXRQST bit  */
                CAN1_IF1MCTL_REG |=(1<<8);
                
            }else
            {
                /* Configure Recive Messages objects */

                /* reset MNUM for the next message to not overwrite previos one*/
                CAN1_IF2CRQ_REG &=(0XFFFFFFC0);
                
                /* Set the WRNRD bit to specify a write to the CANIFnCMASK
                 * Set MASK bit to transfer Transfer IDMASK + DIR + MXTD 
                 * of the message object
                 * Set ARB bit to Transfer ID + DIR + XTD + MSGVAL 
                 * Set the control bit to allow transfer of it 
                 */
                CAN1_IF2CMSK_REG |=(0x000000F0);

                /* Check of processing type if interrupt or polling */
                if (Can_config->Can_Messages[count].MesProcessType==POLLING)
                {
                    /* Set CLRINTPND bit in CANIFnCMASK register  */
                    CAN1_IF2CMSK_REG |=(1<<3);  
                }else
                {
                    /* Clear CLRINTPND bit in CANIFnCMASK register  */
                    CAN1_IF2CMSK_REG &=~(1<<3);  
                }
                
                
                /* Check if masks used in acceptance filter and set in this case */
                if (Can_config->Can_Messages[count].MessFilterEnable==STD_ON)
                {
                    /* enable use of message filter by setting the
                     *  UMASK bit in the CANIFnMCTL register */
                    CAN1_IF2MCTL_REG |=(1<<12);
                    /* Insert The message filter  */
                    if (Can_config->Can_Messages[count].IdType==STANDARD)
                    {
                        /* Case Standard filter is used */
                        CAN1_IF2MSK2_REG &=(0xFFFFE003);
                        CAN1_IF2MSK2_REG |=(((Can_config->Can_Messages[count].MessObjFilter)&(0x000007FF))<<2);
                    }else
                    {
                        /* Extenden id is used  */

                        CAN1_IF2MSK1_REG &=(0xFFFF0000);
                        /* Set the bits 15:0 */
                        CAN1_IF2MSK1_REG |=((Can_config->Can_Messages[count].MessObjFilter)&(0x0000FFFF));
                        /* Set the bits 28:16 */
                        CAN1_IF2MSK2_REG &= (0xFFFFE000);
                        CAN1_IF2MSK2_REG |=((Can_config->Can_Messages[count].MessObjFilter&(0x1FFF0000))>>16);
                        /* Enable XTD bit and DIR bit in acceptance filter */
                        CAN1_IF2MSK2_REG |=(0X0000A000);

                    }
                    
                    
                    
                }else
                {
                }
                
                /* Set the ID field in the Can ARB Rgisters */
                if (Can_config->Can_Messages[count].IdType==EXTENDED)
                {
                    /* Extended Id */
                    CAN1_IF2ARB1_REG &= (0xFFFF0000);
                    /* Set the bits 15:0 */
                    CAN1_IF2ARB1_REG |=((Can_config->Can_Messages[count].MessageId)&(0x0000FFFF));
                    /* Set the bits 28:16  of the ID  */
                    CAN1_IF2ARB2_REG &= (0xFFFFE000);
                    CAN1_IF2ARB2_REG |=((Can_config->Can_Messages[count].MessageId&(0x1FFF0000))>>16);
                    /* Set the XTD bit to specify Extended bit 
                     * Clear DIR to specifiy Receive object
                     * Set Message Field */ 
                    CAN1_IF2ARB2_REG &=~(0x0000E000);
                    CAN1_IF2ARB2_REG |= (0x0000C000);

                }else
                {
                    /* Standard Id is used  */
                    CAN1_IF2ARB2_REG &=(0xFFFFE003);
                    CAN1_IF2ARB2_REG |=(((Can_config->Can_Messages[count].MessageId)&(0x000007FF))<<2);
                    /* Clear DIR to specifiy transmit object
                     * Set Message Field */ 
                    CAN1_IF2ARB2_REG &=~(0x0000E000);
                    CAN1_IF2ARB2_REG |= (0x00008000);

                }
                
                /* Configure if message generates interrupt upon successful transmission */
                if (Can_config->Can_Messages[count].MesProcessType==INTERRUPT)
                {
                    /* Set the TXIE */
                    CAN1_IF2MCTL_REG |=(1<<11);
                }else
                {
                }
                
                /* Set EOB as FIFO  not supported  */
                CAN1_IF2MCTL_REG |=(1<<7);

                /* Configure the DLC[3:0] field to specify the size of the data frame  */
                CAN1_IF2MCTL_REG &=(0xFFFFFFF0);
                CAN1_IF2MCTL_REG |= (Can_config->Can_Messages[count].MessLength&(0x0F));
                
                /* Program the number of the message object to be transmitted in the MNUM field */
                CAN1_IF2CRQ_REG &=(0XFFFFFFC0);
                CAN1_IF2CRQ_REG |=(Can_config->Can_Messages[count].MessageId&(0x3F));
                
                   
            }
            
            break;
        }
    }
    
    /* clear the init pin in Can control 0,1  Register to leave init state */
    CAN0_CTL_REG &=~(1<<0);
    CAN1_CTL_REG &=~(1<<0);
    
    Can_Status = CAN_INITIALIZED;

}else
{
    
}


}

/************************************************************************************
* Service Name: Can_Transmit
* Sync/Async: ASynchronous 
* Reentrancy: reentrant 
* Parameters (in): MessageId 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Set the TXRQST bit to transmit message object
************************************************************************************/
void Can_Transmit(uint8 messageId){
    /* Program the number of the message object to be transmitted in the MNUM field */
    CAN0_IF1CRQ_REG &=(0XFFFFFFC0);
    CAN0_IF1CRQ_REG |=(messageId&(0x3F));
    /* Set the  TXRQST bit  */
    CAN0_IF1MCTL_REG |=(1<<8);
                
}

/************************************************************************************
* Service Name: Can_Receive
* Sync/Async: ASynchronous 
* Reentrancy: reentrant 
* Parameters (in): MessageId , controllerId ,data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Check if the data with the corresponding message id has been received
*              if not sent remote frame with this id
************************************************************************************/
Std_ReturnType Can_Receive (uint8 messageId ,uint8 controllerId,uint64 *data){

/* flag to indicate whether new data has been received */
uint16 newDataFlag = 0;

switch (controllerId)
{
case CAN_CONTROLLER_0:
    /* Check new data bit flag  */
    if (messageId<16)
    {
        /* Check the new data  FLAG */
        newDataFlag = (uint16)(CAN0_NWDA1_REG &(1<<messageId));
    }else
    {
        newDataFlag = (uint16)(CAN0_NWDA2_REG &(1<<(messageId-16)));
    }
        
    
    /* Check if data has been received */    
    if (newDataFlag!=0)
    {
        CAN0_IF2CMSK_REG = (0x0000007F);   
        /* Program the number of the message object to be transmitted in the MNUM field */
        CAN0_IF1CRQ_REG &=(0XFFFFFFC0);
        CAN0_IF1CRQ_REG |=(messageId&(0x3F));
        /* Read the data from the interface registers */
        *data = 0;
        *data = (uint64)((uint16)(*data) | (uint16)CAN0_IF2DA1_REG);
        *data = (uint64) ((uint32)(*data)) | (CAN0_IF2DA2_REG <<16);
        *data = (*data) | (((uint64)(CAN0_IF2DB1_REG))<<32 );
        *data = (*data) | (((uint64)(CAN0_IF2DB2_REG))<<48 );

    }else /* no data has been received so sent remote frame */
    {

        /* Sent remote frame for this message object */
        /* Program the number of the message object to be transmitted in the MNUM field */
        CAN0_IF2CRQ_REG &=(0XFFFFFFC0);
        CAN0_IF2CRQ_REG |=(messageId&(0x3F));
        /* Set the  TXRQST bit  */
        CAN0_IF2MCTL_REG |=(1<<8);
    
    }
    
    break;

case CAN_CONTROLLER_1:
    /* Check if data has been received */    
    if (newDataFlag!=0)
    {
        CAN1_IF2CMSK_REG = (0x0000007F);   
        /* Program the number of the message object to be transmitted in the MNUM field */
        CAN1_IF1CRQ_REG &=(0XFFFFFFC0);
        CAN1_IF1CRQ_REG |=(messageId&(0x3F));
        /* Read the data from the interface registers */
        *data = 0;
        *data = (uint64)((uint16)(*data) | (uint16)CAN1_IF2DA1_REG);
        *data = (uint64) ((uint32)(*data)) | (CAN1_IF2DA2_REG <<16);
        *data = (*data) | (((uint64)(CAN1_IF2DB1_REG))<<32 );
        *data = (*data) | (((uint64)(CAN1_IF2DB2_REG))<<48 );

    }else /* no data has been received so sent remote frame */
    {

        /* Sent remote frame for this message object */
        /* Program the number of the message object to be transmitted in the MNUM field */
        CAN1_IF2CRQ_REG &=(0XFFFFFFC0);
        CAN1_IF2CRQ_REG |=(messageId&(0x3F));
        /* Set the  TXRQST bit  */
        CAN1_IF2MCTL_REG |=(1<<8);
    
    }
    
    break;
}

if (newDataFlag !=0)
{
    /* data has been received */
    return E_OK;
}else
{   
    /* no data has been received yet */
    return E_NOT_OK;
}

}



