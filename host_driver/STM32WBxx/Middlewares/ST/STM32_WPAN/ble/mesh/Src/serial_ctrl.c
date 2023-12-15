/**
******************************************************************************
* @file    serial_ctrl.c
* @author  BLE Mesh Team
* @brief   Serial Control file 
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "hal_common.h"
#include "serial_if.h"
#include "serial_ctrl.h"
#include "light.h"
#include "light_lc.h"
#include "vendor.h"

/** @addtogroup BlueNRG_Mesh
*  @{
*/

/** @addtogroup Middlewares_Serial_Interface
*  @{
*/

/* Private define ------------------------------------------------------------*/
#define SERIAL_MODEL_DATA_OFFSET      15

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const MODEL_OpcodeTableParam_t *Light_OpcodeTable;
const MODEL_OpcodeTableParam_t *Generic_OpcodeTable;
const MODEL_OpcodeTableParam_t *LightLC_OpcodeTable;
const MODEL_OpcodeTableParam_t *Sensor_OpcodeTable;
MOBLEUINT16 Light_OpcodeTableLength;
MOBLEUINT16 Generic_OpcodeTableLength;
MOBLEUINT16 LightLC_OpcodeTableLength;
MOBLEUINT16 Sensor_OpcodeTableLength;
extern MOBLEUINT16 Vendor_Opcodes_Table[] ;

/* Private function prototypes -----------------------------------------------*/
MOBLEUINT8 SerialCtrl_GetMinParamLength(MOBLEUINT32 opcode, const MODEL_OpcodeTableParam_t list[], MOBLEUINT16 length);
MOBLEUINT8 SerialCtrl_GetData(char *rcvdStringBuff, uint16_t rcvdStringSize, MOBLEUINT8 dataOffset, MOBLEUINT8  *data);
/* Private functions ---------------------------------------------------------*/ 
/**
* @brief  This funcrion is used to parse the string given by the user
* @param  rcvdStringBuff: buffer to store input string 
* @param  rcvdStringSize: length of the input string 
* @retval void
*/ 
void SerialCtrlVendorRead_Process(char *rcvdStringBuff, uint16_t rcvdStringSize)
{
   MOBLE_ADDRESS peer = 0;                           /*node adderess of the destination node*/
  MOBLEUINT16 command = 0;                          /*Opcode command to be executed by the destination node*/
  MOBLEUINT8 datalength = 0;
  MOBLEUINT8 elementIndex = 0;                  /*default element index*/  
  MOBLEUINT8  data [10] = {0};                  /*buffer to output property variables */
  MOBLE_RESULT result = MOBLE_RESULT_FAIL;
  
  sscanf(rcvdStringBuff+5, "%4hx %hx ", &peer,&command); 
  
    for(int i = 0; i < 6 ; i++)
    {
      if(command == Vendor_Opcodes_Table[i])
      {                 
        result = MOBLE_RESULT_SUCCESS;
        break;
      }
    
    }
  
  datalength = SerialCtrl_GetData(rcvdStringBuff, rcvdStringSize, SERIAL_MODEL_DATA_OFFSET, data);
  
  
  if(result)
  {
    TRACE_I(TF_SERIAL_CTRL,"Invalid Command\r\n");
    return;
  }
  
 else
  {
      
      result = BLEMesh_ReadRemoteData(peer,elementIndex,command, 
                                         data, datalength);   
      if(result == MOBLE_RESULT_SUCCESS)
      {
        TRACE_I(TF_SERIAL_CTRL,"Command Executed Successfully\r\n");
      }
      else
      {
        TRACE_I(TF_SERIAL_CTRL,"Invalid Opcode Parameter\r\n");
      }
  }
   
}
void SerialCtrlVendorWrite_Process(char *rcvdStringBuff, uint16_t rcvdStringSize)
{
  MOBLE_ADDRESS peer = 0;                           /*node adderess of the destination node*/
  MOBLEUINT16 command = 0;                          /*Opcode command to be executed by the destination node*/
  MOBLEUINT8 elementIndex = 0;                  /*default element index*/  
  MOBLE_RESULT result = MOBLE_RESULT_FAIL;
  MOBLEBOOL response = MOBLE_FALSE;
  MOBLEUINT8 data_buff[VENDOR_DATA_BYTE];
  MOBLEUINT16 idx=0;
  MOBLEUINT8 length;
  MOBLEUINT8 j = 1;
  
  sscanf(rcvdStringBuff+5, "%4hx %hx %hx", &peer,&command,&idx); 
  
  if(command == 0x000E)
  {
    data_buff[0] = 0x01;     /*  data write sub command; */
    length = sizeof(data_buff)-idx;
  
    for(MOBLEUINT8 i=idx;i <sizeof(data_buff);i++)
    {
      data_buff[j] = i;
      j++;
    }
  }
  else
  {
    length = SerialCtrl_GetData(rcvdStringBuff, rcvdStringSize, SERIAL_MODEL_DATA_OFFSET, data_buff);
  }
  
  for(int i = 0; i < 6 ; i++)
    {
      if(command == Vendor_Opcodes_Table[i])
      {                 
        result = MOBLE_RESULT_SUCCESS;
        break;
      }
    }
  if(result)
  {
    TRACE_I(TF_SERIAL_CTRL,"Invalid Command\r\n");
    return;
  }
  
 else
  {
      
      result = BLEMesh_SetRemoteData(peer,elementIndex,command, 
                                         data_buff, length,
                                         response, MOBLE_TRUE);   
      if(result == MOBLE_RESULT_SUCCESS)
      {
        TRACE_I(TF_SERIAL_CTRL,"Command Executed Successfully\r\n");
      }
      else
      {
        TRACE_I(TF_SERIAL_CTRL,"Invalid Opcode Parameter\r\n");
      }
  }
}
  
void SerialCtrl_Process(char *rcvdStringBuff, uint16_t rcvdStringSize)
{
  MOBLE_ADDRESS peer = 0;                           /*node adderess of the destination node*/
  MOBLEUINT16 command = 0;                          /*Opcode command to be executed by the destination node*/
  MOBLEUINT8 minParamLength = 0;                /*minimum number of properties required by a specific command*/
  MOBLEUINT8 elementIndex = 0;          /*default element index*/
  MOBLEUINT8  data [10] = {0};        /*buffer to output property variables */
  MOBLE_RESULT result;
  MOBLEBOOL response = MOBLE_TRUE;
  
  sscanf(rcvdStringBuff+5, "%4hx %hx ", &peer,&command); 
  
  /* Callback to store a pointer to Opcode table starting sddress and length of the table*/
  GenericModelServer_GetOpcodeTableCb(&Generic_OpcodeTable,&Generic_OpcodeTableLength);
  LightModelServer_GetOpcodeTableCb(&Light_OpcodeTable,&Light_OpcodeTableLength);     
  Light_LC_ModelServer_GetOpcodeTableCb(&LightLC_OpcodeTable,&LightLC_OpcodeTableLength);
  SensorModelServer_GetOpcodeTableCb(&Sensor_OpcodeTable,&Sensor_OpcodeTableLength);
  
  /* Minimum parameter length required for a valid opcade in Generic opcode table */
  minParamLength = SerialCtrl_GetMinParamLength(command,
                                                Generic_OpcodeTable,
                                                Generic_OpcodeTableLength);
  
  /* Opcode not found in Generic opcode table 
      Start finding for opcode in Light Table*/
  if (minParamLength == 0xff)
  {
    minParamLength = SerialCtrl_GetMinParamLength(command,
                                                  Light_OpcodeTable,
                                                  Light_OpcodeTableLength);
  }
  /* Opcode not found in Light opcode table
      Start finding for opcode in Light LC Table*/
  if (minParamLength == 0xff)
  {
     minParamLength = SerialCtrl_GetMinParamLength(command,
                                                   LightLC_OpcodeTable,
                                                   LightLC_OpcodeTableLength);
     
  }
  /* Opcode not found in Light LC opcode table 
      Start finding for opcode in Sensor Table*/
  if (minParamLength == 0xff)
  {
    minParamLength = SerialCtrl_GetMinParamLength(command,
                                                  Sensor_OpcodeTable,
                                                  Sensor_OpcodeTableLength);
    
    TRACE_I(TF_SERIAL_CTRL, "Min Parameter Length after sensor model check %d\r\n", 
                             minParamLength);
  }
  
  if (minParamLength != 0xff) /* Opcode found in one of the models */ 
  {
      minParamLength = SerialCtrl_GetData(rcvdStringBuff, 
                                          rcvdStringSize, 
                                          SERIAL_MODEL_DATA_OFFSET, 
                                          data);

      result = BLEMesh_SetRemoteData(peer,
                                     elementIndex,
                                     command, 
                                     data, 
                                     minParamLength,
                                     response, 
                                     MOBLE_FALSE);   
    if(result == MOBLE_RESULT_SUCCESS)
    {
      TRACE_I(TF_SERIAL_CTRL,"Command Executed Successfully\r\n");
    }
    else
    {
      TRACE_I(TF_SERIAL_CTRL,"Invalid Opcode Parameter\r\n");
    }
  }
  else
  {
    TRACE_I(TF_SERIAL_CTRL,"Unknown Opcode\r\n");
  }
}



/**
* @brief  Returns the minimum number of parameters required by a particular Opcode
* @param  opcode: Opcode of the model whose minimum number of parameters are required
* @param  list: 
* @param  length:  
* @retval MOBLEUINT16
*/ 
MOBLEUINT8 SerialCtrl_GetMinParamLength(MOBLEUINT32 opcode, const MODEL_OpcodeTableParam_t list[], MOBLEUINT16 length)
{
  for (int i = 0; i < length; i++)
  {
    if (list[i].opcode == opcode)
    {
      return list[i].min_payload_size;
    }
  }
  return 0xff;
}

/**
* @brief  This function extract the function parameter from the string
* @param  rcvdStringBuff: array of the string parsed from the serial terminal
* @param  rcvdStringSize: sizeOf rcvdStringBuff
* @param  dataOffset:
* @param  data: Output array comprising of Data
* @param  dataIndex:  
* @retval MOBLEUINT8
*/
MOBLEUINT8 SerialCtrl_GetData(char *rcvdStringBuff, uint16_t rcvdStringSize, MOBLEUINT8 dataOffset, MOBLEUINT8  *data)
{
  MOBLEUINT8 byteBuff[10] = {0};
  MOBLEUINT8 dataIndex = 0;
  int msb, lsb, byteCounter=0;           

    for(int i=dataOffset ; i<=(rcvdStringSize) ; i++)
    {
      /* check if space or NULL found */
      if(rcvdStringBuff[i] == ' '||rcvdStringBuff[i] == '\0' )
      {
        /*if number of bytes is one*/
        while(byteCounter > 0)
        {
          data[dataIndex++] = byteBuff[--byteCounter];
            
        }
      }
      else
      {
        /* take two consecutive ascii characters from the rcvdStringBuff and convert to hex values */  
        msb = Serial_CharToHexConvert(rcvdStringBuff[i]);        
        lsb = Serial_CharToHexConvert(rcvdStringBuff[i + 1 ]);
        /*join two hex values to make one hex value*/
        byteBuff[byteCounter]  = msb << 4;
        byteBuff[byteCounter] |= lsb;
        
        i++;      /*increament for loop counter as two values are used */
        byteCounter++;   /*increament byteCounter counter*/
      }
    }
        
  return dataIndex;

}



/**
* @}
*/

/**
* @}
*/
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
