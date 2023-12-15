/**
******************************************************************************
* @file    light.c
* @author  BLE Mesh Team
* @brief   Light Model middleware implementation
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
#include "hal_common.h"
#include "mesh_cfg.h"
#include "light.h"
#include "generic.h"
#include "common.h"
#include "light_lc.h"
#include "math.h"
#include "compiler.h"
#include <string.h>
/** @addtogroup MODEL_Light
*  @{
*/

/** @addtogroup Light_Model_Callbacks
*  @{
*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static Light_TimeParam_t Light_TimeParam;
static Light_ModelFlag_t Light_ModelFlag;
static Light_TemporaryStatus_t Light_TemporaryStatus;

static Light_LightnessParam_t Light_LightnessParam;
static Light_LightnessStatus_t Light_LightnessStatus;
static Light_LightnessDefaultParam_t Light_LightnessDefaultParam;
/* Initialize the light lighness range with maximum and minimuwm value as pr the spec */
static Light_LightnessRangeParam_t Light_LightnessRangeParam = {0x00,MIN_VALID_RANGE,MAX_VALID_RANGE};

static Light_CtlParam_t Light_CtlParam;
/* Initialize the parameter with minimum value of temperature and delta */
static Light_CtlStatus_t Light_CtlStatus =  {0x00,MIN_CTL_TEMP_RANGE,MIN_CTL_DELTA_VALUE,0x00,0x00,0x00,0x00};
static Light_CtlDefaultParam_t Light_CtlDefaultParam;
/* Initialize the Ctl tepmerature range with minimum and maximum value as per the spec */
static Light_CtlTemperatureRangeParam_t CtlTemperatureRangeParam = {0x00,MIN_CTL_TEMP_RANGE,MAX_CTL_TEMP_RANGE};
static Light_HslParam_t Light_HslParam;
static Light_HslStatus_t Light_HslStatus;
/* Initialize the HSL parameter range with minimum and maximum value as per the spec */
static Light_HslRangeParam_t Light_HslRangeParam = {0x00,MIN_HUE_RANGE,MAX_HUE_RANGE,
                                                     MIN_SATURATION_RANGE,MAX_SATURATION_RANGE};

Light_PublishOpcodeList_t Light_PublishOpcodeList;

extern Generic_DefaultTransitionParam_t Generic_DefaultTransitionParam;
MOBLEUINT8 LightUpdateFlag = 0;
extern MOBLEUINT16 Model_Rx_Opcode;
extern MOBLEUINT16 Model_ID;
extern MOBLE_ADDRESS Dst_Peer;
const MODEL_OpcodeTableParam_t Light_Opcodes_Table[] = 
{    
/* model_id                                     opcode                                 reliable     min_payload_size max_payload_size response_opcode                      min_response_size max_response_size */
                                                                                                                                                                           
#ifdef ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS                                                                                                                                 
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_GET,                   MOBLE_TRUE,  0,               0,                LIGHT_LIGHTNESS_STATUS,             2,                5},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_SET,                   MOBLE_TRUE,  3,               5,                LIGHT_LIGHTNESS_STATUS,             2,                5},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_SET_UNACK,             MOBLE_FALSE, 3,               5,                LIGHT_LIGHTNESS_STATUS,             2,                5},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_STATUS,                MOBLE_FALSE, 2,               5,                0,                                  2,                5},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LINEAR_GET,            MOBLE_TRUE,  0,               0,                LIGHT_LIGHTNESS_LINEAR_STATUS,      2,                5},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LINEAR_SET,            MOBLE_TRUE,  3,               5,                LIGHT_LIGHTNESS_LINEAR_STATUS,      2,                5},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LINEAR_SET_UNACK,      MOBLE_FALSE, 3,               5,                LIGHT_LIGHTNESS_LINEAR_STATUS,      2,                5},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LINEAR_STATUS,         MOBLE_FALSE, 2,               5,                0,                                  2,                5},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LAST_GET,              MOBLE_TRUE,  0,               0,                LIGHT_LIGHTNESS_LAST_STATUS,        2,                2},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LAST_STATUS,           MOBLE_FALSE, 2,               2,                0,                                  2,                2},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_DEFAULT_GET,           MOBLE_TRUE,  0,               0,                LIGHT_LIGHTNESS_DEFAULT_STATUS,     2,                2},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_DEFAULT_STATUS,        MOBLE_FALSE, 2,               2,                0,                                  2,                2},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_RANGE_GET,             MOBLE_TRUE,  0,               0,                LIGHT_LIGHTNESS_RANGE_STATUS,       5,                5},
  {LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_RANGE_STATUS,          MOBLE_FALSE, 5,               5,                0,                                  5,                5},
#endif                                                                                                                                                                     
                                                                                                                                                                           
#ifdef ENABLE_LIGHT_MODEL_CLIENT_LIGHTNESS                                                                                                                                 
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_GET,                   MOBLE_TRUE,  0,               0,                LIGHT_LIGHTNESS_STATUS,             2,                5},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_SET,                   MOBLE_TRUE,  3,               5,                LIGHT_LIGHTNESS_STATUS,             2,                5},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_SET_UNACK,             MOBLE_FALSE, 3,               5,                LIGHT_LIGHTNESS_STATUS,             2,                5},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_STATUS,                MOBLE_FALSE, 2,               5,                0,                                  2,                5},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LINEAR_GET,            MOBLE_TRUE,  0,               0,                LIGHT_LIGHTNESS_LINEAR_STATUS,      2,                5},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LINEAR_SET,            MOBLE_TRUE,  3,               5,                LIGHT_LIGHTNESS_LINEAR_STATUS,      2,                5},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LINEAR_SET_UNACK,      MOBLE_FALSE, 3,               5,                LIGHT_LIGHTNESS_LINEAR_STATUS,      2,                5},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LINEAR_STATUS,         MOBLE_FALSE, 2,               5,                0,                                  2,                5},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LAST_GET,              MOBLE_TRUE,  0,               0,                LIGHT_LIGHTNESS_LAST_STATUS,        2,                2},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_LAST_STATUS,           MOBLE_FALSE, 2,               2,                0,                                  2,                2},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_DEFAULT_GET,           MOBLE_TRUE,  0,               0,                LIGHT_LIGHTNESS_DEFAULT_STATUS,     2,                2},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_DEFAULT_STATUS,        MOBLE_FALSE, 2,               2,                0,                                  2,                2},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_RANGE_GET,             MOBLE_TRUE,  0,               0,                LIGHT_LIGHTNESS_RANGE_STATUS,       5,                5},
  {LIGHT_MODEL_CLIENT_LIGHTNESS_MODEL_ID,       LIGHT_LIGHTNESS_RANGE_STATUS,          MOBLE_FALSE, 5,               5,                0,                                  5,                5},
#endif                                                                                                                                                                     
                                                                                                                                                                           
#ifdef ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS_SETUP                                                                                                                           
  {LIGHT_MODEL_SERVER_LIGHTNESS_SETUP_MODEL_ID, LIGHT_LIGHTNESS_DEFAULT_SET,           MOBLE_TRUE,  2,               2,                LIGHT_LIGHTNESS_DEFAULT_STATUS,     2,                2},                 
  {LIGHT_MODEL_SERVER_LIGHTNESS_SETUP_MODEL_ID, LIGHT_LIGHTNESS_DEFAULT_SET_UNACK,     MOBLE_FALSE, 2,               2,                0,                                  2,                2}, 
  {LIGHT_MODEL_SERVER_LIGHTNESS_SETUP_MODEL_ID, LIGHT_LIGHTNESS_RANGE_SET,             MOBLE_TRUE,  4,               4,                LIGHT_LIGHTNESS_RANGE_STATUS,       5,                5},    
  {LIGHT_MODEL_SERVER_LIGHTNESS_SETUP_MODEL_ID, LIGHT_LIGHTNESS_RANGE_SET_UNACK,       MOBLE_FALSE, 4,               4,                0,                                  5,                5}, 
#endif                                                                                                                                                                     
                                                                                                                                                                           
#ifdef ENABLE_LIGHT_MODEL_SERVER_CTL                                                                                                                                       
  {LIGHT_MODEL_SERVER_CTL_MODEL_ID,             LIGHT_CTL_GET,                         MOBLE_TRUE,  0,               0,                LIGHT_CTL_STATUS,                   4,                9},     
  {LIGHT_MODEL_SERVER_CTL_MODEL_ID,             LIGHT_CTL_SET,                         MOBLE_TRUE,  7,               9,                LIGHT_CTL_STATUS,                   4,                9},     
  {LIGHT_MODEL_SERVER_CTL_MODEL_ID,             LIGHT_CTL_SET_UNACK,                   MOBLE_FALSE, 7,               9,                LIGHT_CTL_STATUS,                   4,                9},
  {LIGHT_MODEL_SERVER_CTL_MODEL_ID,             LIGHT_CTL_STATUS,                      MOBLE_FALSE, 4,               9,                0,                                  4,                9},
  {LIGHT_MODEL_SERVER_CTL_MODEL_ID,             LIGHT_CTL_TEMPERATURE_RANGE_GET,       MOBLE_TRUE,  0,               0,                LIGHT_CTL_TEMPERATURE_RANGE_STATUS, 5,                5},
  {LIGHT_MODEL_SERVER_CTL_MODEL_ID,             LIGHT_CTL_TEMPERATURE_RANGE_STATUS,    MOBLE_FALSE, 5,               5,                0,                                  5,                5},
  {LIGHT_MODEL_SERVER_CTL_MODEL_ID,             LIGHT_CTL_DEFAULT_GET,                 MOBLE_TRUE,  0,               0,                LIGHT_CTL_DEFAULT_STATUS,           6,                6},  
  {LIGHT_MODEL_SERVER_CTL_MODEL_ID,             LIGHT_CTL_DEFAULT_STATUS,              MOBLE_TRUE,  6,               6,                0,                                  6,                6},   
#endif                                                                                                                                                                     
                                                                                                                                                                           
#ifdef ENABLE_LIGHT_MODEL_CLIENT_CTL                                                                                                                                       
  {LIGHT_MODEL_CLIENT_CTL_MODEL_ID,             LIGHT_CTL_GET,                         MOBLE_TRUE,  0,               0,                LIGHT_CTL_STATUS,                   4,                9},     
  {LIGHT_MODEL_CLIENT_CTL_MODEL_ID,             LIGHT_CTL_SET,                         MOBLE_TRUE,  7,               9,                LIGHT_CTL_STATUS,                   4,                9},     
  {LIGHT_MODEL_CLIENT_CTL_MODEL_ID,             LIGHT_CTL_SET_UNACK,                   MOBLE_FALSE, 7,               9,                LIGHT_CTL_STATUS,                   4,                9},
  {LIGHT_MODEL_CLIENT_CTL_MODEL_ID,             LIGHT_CTL_STATUS,                      MOBLE_FALSE, 4,               9,                0,                                  4,                9},
  {LIGHT_MODEL_CLIENT_CTL_MODEL_ID,             LIGHT_CTL_TEMPERATURE_RANGE_GET,       MOBLE_TRUE,  0,               0,                LIGHT_CTL_TEMPERATURE_RANGE_STATUS, 5,                5},
  {LIGHT_MODEL_CLIENT_CTL_MODEL_ID,             LIGHT_CTL_TEMPERATURE_RANGE_STATUS,    MOBLE_FALSE, 5,               5,                0,                                  5,                5},
  {LIGHT_MODEL_CLIENT_CTL_MODEL_ID,             LIGHT_CTL_DEFAULT_GET,                 MOBLE_TRUE,  0,               0,                LIGHT_CTL_DEFAULT_STATUS,           6,                6},  
  {LIGHT_MODEL_CLIENT_CTL_MODEL_ID,             LIGHT_CTL_DEFAULT_STATUS,              MOBLE_TRUE,  6,               6,                0,                                  6,                6},   
#endif                                                                                                                                                                     
                                                                                                                                                                           
#ifdef ENABLE_LIGHT_MODEL_SERVER_CTL_SETUP                                                                                                                                 
  {LIGHT_MODEL_SERVER_CTL_SETUP_MODEL_ID,       LIGHT_CTL_DEFAULT_SET,                 MOBLE_TRUE,  6,               6,                LIGHT_CTL_DEFAULT_STATUS,           6,                6},   
  {LIGHT_MODEL_SERVER_CTL_SETUP_MODEL_ID,       LIGHT_CTL_DEFAULT_SET_UNACK,           MOBLE_FALSE, 6,               6,                LIGHT_CTL_DEFAULT_STATUS,           6,                6}, 
  {LIGHT_MODEL_SERVER_CTL_SETUP_MODEL_ID,       LIGHT_CTL_TEMPERATURE_RANGE_SET,       MOBLE_TRUE,  4,               4,                LIGHT_CTL_TEMPERATURE_RANGE_STATUS, 5,                5},   
  {LIGHT_MODEL_SERVER_CTL_SETUP_MODEL_ID,       LIGHT_CTL_TEMPERATURE_RANGE_SET_UNACK, MOBLE_FALSE, 4,               4,                0,                                  5,                5},
#endif  
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_CTL_TEMPERATURE  
  {LIGHT_MODEL_SERVER_CTL_TEMPERATURE_MODEL_ID, LIGHT_CTL_TEMPERATURE_GET,             MOBLE_TRUE,  0,               0,                LIGHT_CTL_TEMPERATURE_STATUS,       4,                9},
  {LIGHT_MODEL_SERVER_CTL_TEMPERATURE_MODEL_ID, LIGHT_CTL_TEMPERATURE_SET,             MOBLE_TRUE,  5,               7,                LIGHT_CTL_TEMPERATURE_STATUS,       4,                9},
  {LIGHT_MODEL_SERVER_CTL_TEMPERATURE_MODEL_ID, LIGHT_CTL_TEMPERATURE_SET_UNACK,       MOBLE_FALSE, 5,               7,                LIGHT_CTL_TEMPERATURE_STATUS,       4,                9},
  {LIGHT_MODEL_SERVER_CTL_TEMPERATURE_MODEL_ID, LIGHT_CTL_TEMPERATURE_STATUS,          MOBLE_FALSE, 4,               9,                0,                                  4,                9},
#endif       
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL
  {LIGHT_MODEL_SERVER_HSL_MODEL_ID,             LIGHT_HSL_GET,                         MOBLE_TRUE,  0,               0,                LIGHT_HSL_STATUS,                   6,                7}, 
  {LIGHT_MODEL_SERVER_HSL_MODEL_ID,             LIGHT_HSL_SET,                         MOBLE_TRUE,  7,               9,                LIGHT_HSL_STATUS,                   6,                7},   
  {LIGHT_MODEL_SERVER_HSL_MODEL_ID,             LIGHT_HSL_SET_UNACK,                   MOBLE_FALSE, 7,               9,                LIGHT_HSL_STATUS,                   6,                7},    
  {LIGHT_MODEL_SERVER_HSL_MODEL_ID,             LIGHT_HSL_STATUS,                      MOBLE_FALSE, 6,               7,                0,                                  6,                7},  
  {LIGHT_MODEL_SERVER_HSL_MODEL_ID,             LIGHT_HSL_TARGET_GET,                  MOBLE_TRUE,  0,               0,                LIGHT_HSL_TARGET_STATUS,            6,                7}, 
  {LIGHT_MODEL_SERVER_HSL_MODEL_ID,             LIGHT_HSL_TARGET_STATUS,               MOBLE_FALSE, 6,               7,                0,                                  6,                7},  
  {LIGHT_MODEL_SERVER_HSL_MODEL_ID,             LIGHT_HSL_DEFAULT_GET,                 MOBLE_TRUE,  0,               0,                LIGHT_HSL_DEFAULT_STATUS,           6,                6},  
  {LIGHT_MODEL_SERVER_HSL_MODEL_ID,             LIGHT_HSL_DEFAULT_STATUS,              MOBLE_FALSE, 6,               6,                0,                                  6,                6},  
  {LIGHT_MODEL_SERVER_HSL_MODEL_ID,             LIGHT_HSL_RANGE_GET,                   MOBLE_TRUE,  0,               0,                LIGHT_HSL_RANGE_STATUS,             9,                9},
  {LIGHT_MODEL_SERVER_HSL_MODEL_ID,             LIGHT_HSL_RANGE_STATUS,                MOBLE_TRUE,  9,               9,                0,                                  9,                9},
#endif
  
#ifdef ENABLE_LIGHT_MODEL_CLIENT_HSL
  {LIGHT_MODEL_CLIENT_HSL_MODEL_ID,             LIGHT_HSL_GET,                         MOBLE_TRUE,  0,               0,                LIGHT_HSL_STATUS,                   6,                7}, 
  {LIGHT_MODEL_CLIENT_HSL_MODEL_ID,             LIGHT_HSL_SET,                         MOBLE_TRUE,  7,               9,                LIGHT_HSL_STATUS,                   6,                7},   
  {LIGHT_MODEL_CLIENT_HSL_MODEL_ID,             LIGHT_HSL_SET_UNACK,                   MOBLE_FALSE, 7,               9,                LIGHT_HSL_STATUS,                   6,                7},    
  {LIGHT_MODEL_CLIENT_HSL_MODEL_ID,             LIGHT_HSL_STATUS,                      MOBLE_FALSE, 6,               7,                0,                                  6,                7},  
  {LIGHT_MODEL_CLIENT_HSL_MODEL_ID,             LIGHT_HSL_TARGET_GET,                  MOBLE_TRUE,  0,               0,                LIGHT_HSL_TARGET_STATUS,            6,                7}, 
  {LIGHT_MODEL_CLIENT_HSL_MODEL_ID,             LIGHT_HSL_TARGET_STATUS,               MOBLE_FALSE, 6,               7,                0,                                  6,                7},  
  {LIGHT_MODEL_CLIENT_HSL_MODEL_ID,             LIGHT_HSL_DEFAULT_GET,                 MOBLE_TRUE,  0,               0,                LIGHT_HSL_DEFAULT_STATUS,           6,                6},  
  {LIGHT_MODEL_CLIENT_HSL_MODEL_ID,             LIGHT_HSL_DEFAULT_STATUS,              MOBLE_FALSE, 6,               6,                0,                                  6,                6},  
  {LIGHT_MODEL_CLIENT_HSL_MODEL_ID,             LIGHT_HSL_RANGE_GET,                   MOBLE_TRUE,  0,               0,                LIGHT_HSL_RANGE_STATUS,             9,                9},
  {LIGHT_MODEL_CLIENT_HSL_MODEL_ID,             LIGHT_HSL_RANGE_STATUS,                MOBLE_TRUE,  9,               9,                0,                                  9,                9},
#endif
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL_SETUP   
  {LIGHT_MODEL_SERVER_HSL_SETUP_MODEL_ID,       LIGHT_HSL_RANGE_SET,                   MOBLE_TRUE,  8,               8,                LIGHT_HSL_RANGE_STATUS,              9,               9},   
  {LIGHT_MODEL_SERVER_HSL_SETUP_MODEL_ID,       LIGHT_HSL_RANGE_SET_UNACK,             MOBLE_FALSE, 8,               8,                0,                                   9,               9},  
  {LIGHT_MODEL_SERVER_HSL_SETUP_MODEL_ID,       LIGHT_HSL_DEFAULT_SET,                 MOBLE_TRUE,  6,               6,                LIGHT_HSL_DEFAULT_STATUS,            6,               6},   
  {LIGHT_MODEL_SERVER_HSL_SETUP_MODEL_ID,       LIGHT_HSL_DEFAULT_SET_UNACK,           MOBLE_FALSE, 6,               6,                LIGHT_HSL_DEFAULT_STATUS,            6,               6},
#endif
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL_HUE    
  {LIGHT_MODEL_SERVER_HSL_HUE_MODEL_ID,         LIGHT_HSL_HUE_GET,                     MOBLE_TRUE,  0,               0,                LIGHT_HSL_HUE_STATUS,                2,               5},   
  {LIGHT_MODEL_SERVER_HSL_HUE_MODEL_ID,         LIGHT_HSL_HUE_SET,                     MOBLE_TRUE,  3,               5,                LIGHT_HSL_HUE_STATUS,                2,               5},   
  {LIGHT_MODEL_SERVER_HSL_HUE_MODEL_ID,         LIGHT_HSL_HUE_SET_UNACK,               MOBLE_FALSE, 3,               5,                LIGHT_HSL_HUE_STATUS,                2,               5}, 
  {LIGHT_MODEL_SERVER_HSL_HUE_MODEL_ID,         LIGHT_HSL_HUE_STATUS,                  MOBLE_FALSE, 2,               5,                0,                                   2,               5},   
#endif
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL_SATURATION    
  {LIGHT_MODEL_SERVER_HSL_SATURATION_MODEL_ID,  LIGHT_HSL_SATURATION_GET,             MOBLE_TRUE,   0,               0,                LIGHT_HSL_SATURATION_STATUS,         2,               5},   
  {LIGHT_MODEL_SERVER_HSL_SATURATION_MODEL_ID,  LIGHT_HSL_SATURATION_SET,             MOBLE_TRUE,   3,               5,                LIGHT_HSL_SATURATION_STATUS,         2,               5},    
  {LIGHT_MODEL_SERVER_HSL_SATURATION_MODEL_ID,  LIGHT_HSL_SATURATION_SET_UNACK,       MOBLE_FALSE,  3,               5,                LIGHT_HSL_SATURATION_STATUS,         2,               5},    
  {LIGHT_MODEL_SERVER_HSL_SATURATION_MODEL_ID,  LIGHT_HSL_SATURATION_STATUS,          MOBLE_FALSE,  2,               5,                0,                                   2,               5},    
#endif   
  {0}		
};

/* Private function prototypes -----------------------------------------------*/
MOBLE_RESULT Light_Ctl_Status(MOBLEUINT8* pLightCtl_status, MOBLEUINT32 *pLength);
MOBLE_RESULT Light_CtlTemperature_Status(MOBLEUINT8* pLightCtlTemp_status, MOBLEUINT32 *pLength);
WEAK_FUNCTION (MOBLE_RESULT Appli_Light_Lightness_Set(Light_LightnessStatus_t* pLight_LightnessParam,
                                                      MOBLEUINT8 OptionalValid));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Linear_Set(Light_LightnessStatus_t* pLight_LightnessLinearParam,
                                                            MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Default_Set(Light_LightnessDefaultParam_t* pLight_LightnessDefaultParam,
                                                             MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Range_Set(Light_LightnessRangeParam_t* pLight_LightnessRangeParam,
                                                           MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Ctl_Set(Light_CtlStatus_t* pLight_CtlParam,
                                               MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlTemperature_Set(Light_CtlStatus_t* pLight_CtltempParam,
                                                          MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlTemperature_Range_Set(Light_CtlTemperatureRangeParam_t* pLight_CtlTempRangeParam,
                                                                MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlDefault_Set(Light_CtlDefaultParam_t* pLight_CtlDefaultParam,
                                                      MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Hsl_Set(Light_HslStatus_t* pLight_HslParam,
                                               MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslHue_Set(Light_HslStatus_t* pLight_HslHueParam,
                                                  MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslSaturation_Set(Light_HslStatus_t* pLight_HslSaturationParam,
                                                         MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslDefault_Set(Light_HslStatus_t* pLight_HslDefaultParam,
                                                      MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslRange_Set(Light_HslRangeParam_t* pLight_HslRangeParam,
                                                    MOBLEUINT8 OptionalValid));   
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetLightnessStatus(MOBLEUINT8* lLightnessState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetLightnessLinearStatus(MOBLEUINT8* lLightnessState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetLightnessDefaultStatus(MOBLEUINT8* lDefaultState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetLightnessLastStatus(MOBLEUINT8* lLastState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetLightnessRangeStatus(MOBLEUINT8* lRangeState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetCtlLightStatus(MOBLEUINT8* lCtlLightState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetCtlTemperatureStatus(MOBLEUINT8* lCtlTempState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetCtlTemperatureRange(MOBLEUINT8* lCtlTempRange));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetCtlDefaultStatus(MOBLEUINT8* lCtlDefaultState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslStatus(MOBLEUINT8* lHslState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslHueStatus(MOBLEUINT8* lHslHueState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslSaturationStatus(MOBLEUINT8* lHslSaturationState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslHueRange(MOBLEUINT8* lHslHueRange));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslSatRange(MOBLEUINT8* lHslSatRange));
WEAK_FUNCTION(void HSL_RGB_Conversion(void));
WEAK_FUNCTION(void RgbF_Create(MOBLEUINT16 value1, MOBLEUINT16 value2, MOBLEUINT16 value3));
WEAK_FUNCTION(void Light_UpdatePWMValue(MOBLEUINT8 state));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslDefaultStatus(MOBLEUINT8* lHslDefaultState));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Status(MOBLEUINT8 const *pLightness_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Linear_Status(MOBLEUINT8 const *pLightnessLinear_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Default_Status(MOBLEUINT8 const *pLightnessDefault_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Range_Status(MOBLEUINT8 const *pLightnessRange_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Ctl_Status(MOBLEUINT8 const *pLightCtl_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlTemperature_Status(MOBLEUINT8 const *pLightCtlTemp_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlTemperature_Range_Status(MOBLEUINT8 const *pCtlTempRange_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlDefault_Status(MOBLEUINT8 const *pCtlDefault_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Hsl_Status(MOBLEUINT8 const *pHsl_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslHue_Status(MOBLEUINT8 const *pHslHue_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslSaturation_Status(MOBLEUINT8 const *pHslSaturation_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslDefault_Status(MOBLEUINT8 const *pHslDefault_status, MOBLEUINT32 pLength));
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslRange_Status(MOBLEUINT8 const *pHslRange_status, MOBLEUINT32 pLength));

/* Private functions ---------------------------------------------------------*/

/**
* @brief Light_Publish_Reset:Function for publication list management 
* return void
*/
void Light_Publish_Reset(void)
{
  MOBLEUINT8 index = 0;
  
  while(index < Light_PublishOpcodeList.BindedStateCount)
  {
    Light_PublishOpcodeList.Model_ID[index] = 0;
    Light_PublishOpcodeList.PublishStateOpcode[index] = 0;
    index++;
  }
  
  Light_PublishOpcodeList.BindedStateCount = 0;
}

/**
* @brief Light_Publish_Add:Function for publication list management 
* @param model_id: Model ID
* @param opcode: opcode
* return void
*/
void Light_Publish_Add(MOBLEUINT16 model_id, MOBLEUINT16 opcode)
{
  MOBLEUINT8 index = 0;
  
  while((Light_PublishOpcodeList.PublishStateOpcode[index] != opcode) &&
        (Light_PublishOpcodeList.PublishStateOpcode[index] != 0) &&
        (index < Light_PublishOpcodeList.BindedStateCount))
  {
    index++;
  }
  
  if((index > (Light_PublishOpcodeList.BindedStateCount - 1)) &&
     (Light_PublishOpcodeList.PublishStateOpcode[index] == 0))
  {
    Light_PublishOpcodeList.BindedStateCount++;
    Light_PublishOpcodeList.Model_ID[index] = model_id;
    Light_PublishOpcodeList.PublishStateOpcode[index] = opcode;
  }
}

/**
* @brief  Light_Lightness_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library
* @param  plightness_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Lightness_Set(const MOBLEUINT8* plightness_param, MOBLEUINT32 length)
{
  /* 6.3.1.2 Light Lightness Set
  
  Lightness             2B The target value of the Light Lightness Actual state. 
  TID                   1B Transaction Identifier
  Transition Time       1B Format as defined in Section 3.1.3. (Optional)
  Delay                 1B Message execution delay in 5 millisecond steps (C.1)
  */
  TRACE_M(TF_LIGHT_M, "Light_Lightness_Set callback received \r\n");
  
  Light_LightnessParam.TargetLightnessStatus = (plightness_param[0] | plightness_param[1] << 8);
  Light_LightnessParam.TID = plightness_param[2];
#ifdef ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS_SETUP  
  /* Binding of actual lightness value with lightness range */
  Light_Actual_RangeBinding(&Light_LightnessParam);
#endif  
  /* Check for Optional Parameters */ 
  if ((length > 3) && (plightness_param[3] !=0))
  {
    /* Transition_Time & Delay_Time Present */
    Light_LightnessParam.TransitionTime = plightness_param[3];
    Light_LightnessParam.Delay_Time = plightness_param[4]; 
    
    /* Copy the received data in status message which needs
    to be set in application messages */  
    Light_LightnessStatus.TargetValue16 = Light_LightnessParam.TargetLightnessStatus;
    Light_LightnessStatus.RemainingTime = Light_LightnessParam.TransitionTime;
    
    /* copy status parameters in Temporary parameters for transition 
    process.
    */
    Light_TemporaryStatus.TargetParam_1 = Light_LightnessStatus.TargetValue16;    
    Light_TemporaryStatus.RemainingTime = Light_LightnessStatus.RemainingTime;
    
    /* Function to calculate time parameters, step resolution
    step size for transition state machine.
    */
    Light_GetStepValue(plightness_param[3]);
    /*option parameter flag, enable to sent all required parameter in status.*/
    Light_ModelFlag.LightTransitionFlag = LIGHT_LIGHTNESS_TRANSITION_START;  
    Light_ModelFlag.LightOptionalParam = 1;
  }
  else
  {
#ifdef ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME   
    Light_LightnessDefaultTransitionValue(&Light_LightnessParam);
#else
    /* When no optional parameter received, target value will
    be set as present value in application.
    */
    Light_LightnessStatus.LightnessPresentValue16 = Light_LightnessParam.TargetLightnessStatus; 
#endif   
     
  }
  if(Light_LightnessStatus.LightnessPresentValue16 > 0x00)
  {
    Light_LightnessStatus.LightnessLastStatus = Light_LightnessStatus.LightnessPresentValue16;
  }
  /* Application Callback */
  (LightAppli_cb.Lightness_Set_cb)(&Light_LightnessStatus, 0);
  
#ifdef ENABLE_MODEL_BINDING    
  /* Binding of actual light lightness with other models */
  Light_Lightness_Binding(BINDING_LIGHT_LIGHTNESS_ACTUAL_SET , length);  
#endif  
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  Light Lightness Status
* @param  pLightness_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Lightness_Status(MOBLEUINT8* pLightness_status, MOBLEUINT32* pLength)
{
  /* 
  Following is the status message:
  Present Lightness     1B The present value of the Generic OnOff state. 
  Target Lightness      1B The target value of the Generic OnOff state (optional).
  Remaining Time        1B Time to complete the transition to the target Light Lightness
  
  */ 
  
  TRACE_M(TF_LIGHT_M, "Light_Lightness_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#824B! \n\r");
  
  /* This condition will be processed when message received will have
  transition time and delay parameter.
  or the transition will be in process
  */
  if((Light_ModelFlag.LightOptionalParam == 1) || (Light_TimeParam.StepValue != 0))
  {
    *(pLightness_status) = Light_LightnessStatus.LightnessPresentValue16;
    *(pLightness_status+1) = Light_LightnessStatus.LightnessPresentValue16 >> 8;
    *(pLightness_status+2) = Light_LightnessStatus.TargetValue16;
    *(pLightness_status+3) = Light_LightnessStatus.TargetValue16 >>8;
    *(pLightness_status+4) = Light_LightnessStatus.RemainingTime;
    *pLength = 5;     
    Light_ModelFlag.LightOptionalParam = 0;    
  }
  else
  {    
    /* When no optional parameter received then, target value will
    be sent in status message.
    */    
    *(pLightness_status) = Light_LightnessStatus.LightnessPresentValue16 ;
    *(pLightness_status+1) = Light_LightnessStatus.LightnessPresentValue16 >> 8;
    *pLength = 2;          
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  Light_Lightness_Linear_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library
* @param  plightnessLinear_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Lightness_Linear_Set(const MOBLEUINT8* plightnessLinear_param, MOBLEUINT32 length)
{
  /* 6.3.1.6 Light Lightness Linear Set
  
  Lightness             2B The target value of the Light Lightness Actual state. 
  TID                   1B Transaction Identifier
  Transition Time       1B Format as defined in Section 3.1.3. (Optional)
  Delay                 1B Message execution delay in 5 millisecond steps (C.1)
  */
  MOBLEUINT8 OptionalValid = 0;
  
  TRACE_M(TF_LIGHT_M, "Light_Lightness_Linear_Set callback received \r\n");
  
  Light_LightnessParam.TargetLightnessLinearStatus = ((plightnessLinear_param[0])
                                                      | (plightnessLinear_param[1] << 8));
  Light_LightnessParam.TID = plightnessLinear_param[2];
  
  /* Check for Optional Parameters */ 
  if ((length > 3) && (plightnessLinear_param[3] !=0))
  {
    /* Transition_Time & Delay_Time Present */
    Light_LightnessParam.TransitionTime = plightnessLinear_param[3];
    Light_LightnessParam.Delay_Time = plightnessLinear_param[4]; 
    
    /* Copy the received data in status message which needs
    to be set in application messages */
    Light_LightnessStatus.TargetValue16 = Light_LightnessParam.TargetLightnessLinearStatus;
    Light_LightnessStatus.RemainingTime = Light_LightnessParam.TransitionTime;
    /* copy status parameters in Temporary parameters for transition 
    process.
    */
    Light_TemporaryStatus.TargetParam_1 = Light_LightnessStatus.TargetValue16;    
    Light_TemporaryStatus.RemainingTime = Light_LightnessStatus.RemainingTime;
    /* Function to calculate time parameters, step resolution
    step size for transition state machine.
    */
    
    Light_GetStepValue(plightnessLinear_param[3]);
    /*transition process enable flag. */
    Light_ModelFlag.LightTransitionFlag = LIGHT_LINEAR_TRANSITION_START; 
    /*option parameter flag, enable to sent all required parameter in status.*/
    Light_ModelFlag.LightOptionalParam = 1;
  }
  else
  {
    Light_LightnessStatus.LinearPresentValue16 = Light_LightnessParam .TargetLightnessLinearStatus;   
  }
  
  /* Application Callback */
  (LightAppli_cb.Lightness_Linear_Set_cb)(&Light_LightnessStatus, OptionalValid);
  
#ifdef ENABLE_MODEL_BINDING   
  /* Binding of data of light lightness with light linear */
  Light_Lightness_Linear_Binding(BINDING_LIGHT_LIGHTNESS_LINEAR_SET ,length);
#endif   

#ifdef ENABLE_LIGHT_MODEL_SERVER_LC  
  Light_Linear_ActualBinding(length);
#endif    
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light Lightness Linear Status
* @param  pLightnessLinear_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Lightness_Linear_Status(MOBLEUINT8* pLightnessLinear_status, MOBLEUINT32 *pLength)
{
  /* 
  Following is the status message:
  Present Lightness     2B The present value of the Light Lightness Linear state. 
  Target Lightness      2B The target value of the Light Lightness Linear state (Optional).
  Remaining Time        1B Time to complete the transition to the target Light Lightness Li
  */
  
  TRACE_M(TF_LIGHT_M, "Light_Lightness_Linear_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#824F! \n\r");
  
  /* This condition will be processed when message received will have
  transition time and delay parameter.
  or the transition will be in process
  */
  if((Light_ModelFlag.LightOptionalParam == 1) || (Light_TimeParam.StepValue != 0))
  {
    *(pLightnessLinear_status) = Light_LightnessStatus.LinearPresentValue16;
    *(pLightnessLinear_status+1) = Light_LightnessStatus.LinearPresentValue16 >> 8;
    *(pLightnessLinear_status+2) = Light_LightnessStatus.TargetValue16;
    *(pLightnessLinear_status+3) = Light_LightnessStatus.TargetValue16 >>8;
    *(pLightnessLinear_status+4) = Light_LightnessStatus.RemainingTime;
    *pLength = 5;
    
    Light_ModelFlag.LightOptionalParam = 0U;          
  }
  else
  { 
    /* When no optional parameter received then, target value will
    be sent in status message.
    */
    *(pLightnessLinear_status) = Light_LightnessStatus.LinearPresentValue16;
    *(pLightnessLinear_status+1) = Light_LightnessStatus.LinearPresentValue16 >> 8;
    *pLength = 2;
    
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  Light_Lightness_Last_Set: This function is called for both Acknowledged and 
*         unacknowledged message. The Acknowledgement is taken care by the Library
* @param  plightnessLast_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Lightness_Last_Set(const MOBLEUINT8* plightnessLast_param, MOBLEUINT32 length)
{
  Light_LightnessStatus.LightnessLastStatus = plightnessLast_param[1] << 8;
  Light_LightnessStatus.LightnessLastStatus |= plightnessLast_param[0];
  
  return MOBLE_RESULT_SUCCESS;
}  

/**
* @brief  Light Lightness Last Status
* @param  pLightnessLast_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Lightness_Last_Status(MOBLEUINT8* pLightnessLast_status, MOBLEUINT32 *pLength)
{
  /* 6.3.1.10 Light Lightness Last State
  Following is the status message:
  Lightness     2B The value of the Light Lightness Last. 
  */
  
  TRACE_M(TF_LIGHT_M, "Light_Lightness_Last_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#8253! \n\r");
  
  *pLightnessLast_status = Light_LightnessStatus.LightnessLastStatus ;
  *(pLightnessLast_status+1) = Light_LightnessStatus.LightnessLastStatus >> 8;
  *pLength = 2;
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_Lightness_Default_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library
* @param  plightnessDefault_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Lightness_Default_Set(const MOBLEUINT8* plightnessDefault_param, MOBLEUINT32 length)
{
  /* 6.3.1.12 Light Default Lightness Set
  Lightness   2B The target value of the Light Lightness Actual state. 
  */
  MOBLEUINT8 OptionalValid = 0;
  
  TRACE_M(TF_LIGHT_M, "Light_Lightness_Default_Set callback received \r\n");  
  
  Light_LightnessDefaultParam.LightnessDefaultStatus = plightnessDefault_param[1] << 8;
  Light_LightnessDefaultParam.LightnessDefaultStatus |= plightnessDefault_param[0];
  
  /* Application Callback */
  (LightAppli_cb.Lightness_Default_Set_cb)(&Light_LightnessDefaultParam, OptionalValid);
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light Lightness Default Status
* @param  pLightnessDefault_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Lightness_Default_Status(MOBLEUINT8* pLightnessDefault_status, MOBLEUINT32 *pLength)
{
  /* 
  Following is the status message:
  Lightness     2B The value of the Light Lightness Default state. 
  */
  
  TRACE_M(TF_LIGHT_M, "Light_Lightness_Default_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#8255! \n\r");
  
  /* Function call back to get the values from application*/
  *pLightnessDefault_status = Light_LightnessDefaultParam.LightnessDefaultStatus ;
  *(pLightnessDefault_status+1) = Light_LightnessDefaultParam.LightnessDefaultStatus >>8;
  *pLength = 2;
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_Lightness_Range_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library
* @param  plightnessRange_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Lightness_Range_Set(const MOBLEUINT8* plightnessRange_param, MOBLEUINT32 length)
{
  /* 6.3.1.16 Light Lightness Range Set 
  Min Range    2B The Minimum range value of the Light Lightness state. 
  Max Range    2B The Maximum range value of the Light Lightness state         
  */
  MOBLEUINT8 OptionalValid = 0;
  
  TRACE_M(TF_LIGHT_M, "Light_Lightness_Range_Set callback received \r\n");
  
  Light_LightnessRangeParam.MinRangeStatus = plightnessRange_param[1] << 8;
  Light_LightnessRangeParam.MinRangeStatus |= plightnessRange_param[0];
  
  Light_LightnessRangeParam.MaxRangeStatus = plightnessRange_param[3] << 8;
  Light_LightnessRangeParam.MaxRangeStatus |= plightnessRange_param[2];
  Light_LightnessRangeParam.StatusCode = SUCCESS_STATUS_CODE;
  
  /* condition to check the minimum range value according to the specification*/
  /* set the status code for status message according to value of range*/  
  
  /* Application Callback */
  (LightAppli_cb.Lightness_Range_Set_cb)(&Light_LightnessRangeParam, OptionalValid);
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_Lightness_Range_Status
* @param  pLightnessRange_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Lightness_Range_Status(MOBLEUINT8* pLightnessRange_status, MOBLEUINT32 *pLength)
{
  /* 6.3.1.18 Light Lightness Range Status  
  Min Range      2B The Minimum range value of the Light Lightness state. 
  Max Range      2B The Maximum range value of the Light Lightness state         
  */
  
  TRACE_M(TF_LIGHT_M, "Light_Lightness_Range_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#8257! \n\r");
  
  *(pLightnessRange_status) = Light_LightnessRangeParam.StatusCode;
  *(pLightnessRange_status+1) = Light_LightnessRangeParam.MinRangeStatus;
  *(pLightnessRange_status+2) = Light_LightnessRangeParam.MinRangeStatus >> 8;
  *(pLightnessRange_status+3) = Light_LightnessRangeParam.MaxRangeStatus;
  *(pLightnessRange_status+4) = Light_LightnessRangeParam.MaxRangeStatus >> 8;
  *pLength = 5;
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_CTL_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library
* @param  pLightCtl_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Ctl_Set(const MOBLEUINT8* pLightCtl_param, MOBLEUINT32 length)
{
  /* 6.3.2.2 Light Lightness Ctl Set
  
  Ctl Lightness         2B The target value of the Light Lightness Ctl state.
  Ctl Temperature       2B The target value of the Light Lightness Ctl state.
  Ctl Delta Uv          2B The target value of the Light Lightness Ctl state.
  TID                   1B Transaction Identifier
  Transition Time       1B Format as defined in Section 3.1.3. (Optional)
  Delay                 1B Message execution delay in 5 millisecond steps (C.1)
  */
  
  TRACE_M(TF_LIGHT_M, "Light_Ctl_Set callback received \r\n");
  
  /* Received parameters from client */
  Light_CtlParam.CTL_Lightness    = pLightCtl_param[1] << 8;
  Light_CtlParam.CTL_Lightness   |= pLightCtl_param[0];
  Light_CtlParam.CTL_Temperature  = pLightCtl_param[3] << 8;
  Light_CtlParam.CTL_Temperature |= pLightCtl_param[2];
  Light_CtlParam.CTL_DeltaUv      = pLightCtl_param[5] << 8;
  Light_CtlParam.CTL_DeltaUv     |= pLightCtl_param[4];
  Light_CtlParam.CTL_TID          = pLightCtl_param[6];
  
  if((Light_CtlParam.Last_Lightness == Light_CtlParam.CTL_Lightness)
     && (Light_CtlParam.Last_Temperature == Light_CtlParam.CTL_Temperature))
  {
    TRACE_M(TF_LIGHT_M, "Received Value is same to last values received \r\n");
  }
  
  Light_CtlParam.Last_Lightness = Light_CtlParam.CTL_Lightness;
  Light_CtlParam.Last_Temperature = Light_CtlParam.CTL_Temperature;
  Light_CtlStatus.PresentCtlDelta16 = Light_CtlParam.CTL_DeltaUv;
  
  if((length > 7) && (pLightCtl_param[7] !=0)) 
  {
    Light_CtlParam.CTL_TransitionTime = pLightCtl_param[7];    
    Light_CtlParam.CTL_Delay          = pLightCtl_param[8];
    
    Light_CtlStatus.TargetCtlLightness16 = Light_CtlParam.CTL_Lightness;
    Light_CtlStatus.TargetCtlTemperature16 = Light_CtlParam.CTL_Temperature;
    Light_CtlStatus.RemainingTime = Light_CtlParam.CTL_TransitionTime;
    
    Light_TemporaryStatus.TargetParam_1 = Light_CtlStatus.TargetCtlLightness16;
    Light_TemporaryStatus.TargetParam_2 = Light_CtlStatus.TargetCtlTemperature16;
    Light_GetStepValue(pLightCtl_param[7]);  
    Light_ModelFlag.LightOptionalParam = 1;
    Light_ModelFlag.LightTransitionFlag = LIGHT_CTL_TRANSITION_START;
  }
  else
  {
#ifdef ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME   
     Light_CTLDefaultTransitionValue(&Light_CtlParam);
#else
    /* When no optional parameter received, target value will
    be set as present value in application.
    */
    Light_CtlStatus.PresentCtlLightness16 = Light_CtlParam.CTL_Lightness;
    Light_CtlStatus.PresentCtlTemperature16 = Light_CtlParam.CTL_Temperature;
#endif    
  }
  /* Application Callback */
  (LightAppli_cb.Light_Ctl_Set_cb)(&Light_CtlStatus, 0);  
  
#ifdef ENABLE_MODEL_BINDING   
  /* binding of light ctl with actual lightness */
  Light_Ctl_LightActual_Binding(BINDING_LIGHT_CTL_SET);
#endif 
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light CTL Status
* @param  pLightCtl_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Ctl_Status(MOBLEUINT8* pLightCtl_status, MOBLEUINT32 *pLength)
{
  /* 6.3.2.4 Light Lightness Ctl Status 
  Present Ctl Lightness           2B The Present ctl Lightness value of the model state. 
  Present Ctl Temperature         2B The Present ctl Temperature value of the model state. 
  Target Ctl Lightness            2B The Target ctl Lightness value of the model state.
  Target Ctl Temeperature         2B The Target ctl Temperature value of the model state.
  Remaining Time                  1B The Remaining Time value of the model state        
  */
  
  TRACE_M(TF_LIGHT_M,"Light_Ctl_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#825D! \n\r");
  
  if((Light_ModelFlag.LightOptionalParam == 1) || (Light_TimeParam.StepValue != 0))
  {
    *(pLightCtl_status) =  Light_CtlStatus.PresentCtlLightness16; 
    *(pLightCtl_status+1) = Light_CtlStatus.PresentCtlLightness16  >> 8;
    *(pLightCtl_status+2) = Light_CtlStatus.PresentCtlTemperature16;
    *(pLightCtl_status+3) = Light_CtlStatus.PresentCtlTemperature16 >>8;
    *(pLightCtl_status+4) = Light_CtlStatus.TargetCtlLightness16;
    *(pLightCtl_status+5) = Light_CtlStatus.TargetCtlLightness16 >>8;
    *(pLightCtl_status+6) = Light_CtlStatus.TargetCtlTemperature16;
    *(pLightCtl_status+7) = Light_CtlStatus.TargetCtlTemperature16 >>8;
    *(pLightCtl_status+8) = Light_CtlStatus.RemainingTime;
    *pLength = 9;
    Light_ModelFlag.LightOptionalParam = 0;     
  }
  else
  {     
    *(pLightCtl_status) =  Light_CtlStatus.PresentCtlLightness16 ; 
    *(pLightCtl_status+1) = Light_CtlStatus.PresentCtlLightness16 >> 8;
    *(pLightCtl_status+2) = Light_CtlStatus.PresentCtlTemperature16;
    *(pLightCtl_status+3) = Light_CtlStatus.PresentCtlTemperature16 >> 8;
    *pLength = 4;         
  }
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_CTLTemperature_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library
* @param  pLightCtlTemp_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_CtlTemperature_Set(const MOBLEUINT8* pLightCtlTemp_param, MOBLEUINT32 length)
{
  /* 6.3.2.6 Light Ctl Temperature Set 
  Ctl Temperature       2B The target value of the Light Temperature Ctl state.
  Ctl Delta Uv          2B The target value of the Light Temperature Ctl state.
  TID                   1B Transaction Identifier
  Transition Time       1B Format as defined in Section 3.1.3. (Optional)
  Delay                 1B Message execution delay in 5 millisecond steps (C.1)
  */
  MOBLEUINT8 OptionalValid = 0;   
  
  TRACE_M(TF_LIGHT_M,"Light_CtlTemperature_Set callback received \r\n");
  
  /* Received parameters from client */
  Light_CtlParam.CTL_Temperature   = pLightCtlTemp_param[1] << 8;
  Light_CtlParam.CTL_Temperature  |= pLightCtlTemp_param[0];
  Light_CtlParam.CTL_DeltaUv  = pLightCtlTemp_param[3] << 8;
  Light_CtlParam.CTL_DeltaUv |= pLightCtlTemp_param[2];
  Light_CtlParam.CTL_TID     = pLightCtlTemp_param[4] << 8;
  
  Light_CtlTemperature_TempRangeBinding(&Light_CtlParam);
  /* Binding with ctl temperature range value */
  
  if((length > 5) && (pLightCtlTemp_param[5] !=0))
  {
    Light_CtlParam.CTL_TransitionTime = pLightCtlTemp_param[5];
    Light_CtlParam.CTL_Delay          = pLightCtlTemp_param[6];
    
    Light_CtlStatus.TargetCtlTemperature16 = Light_CtlParam.CTL_Temperature;
    Light_CtlStatus.TargetCtlDeltaUv16 = Light_CtlParam.CTL_DeltaUv;
    Light_CtlStatus.RemainingTime = Light_CtlParam.CTL_TransitionTime;
    
    Light_TemporaryStatus.TargetParam_1 = Light_CtlStatus.TargetCtlTemperature16;
    Light_TemporaryStatus.TargetParam_2 = Light_CtlStatus.TargetCtlDeltaUv16;
    Light_GetStepValue(pLightCtlTemp_param[5]); 
    Light_ModelFlag.LightOptionalParam = 1;
    Light_ModelFlag.LightTransitionFlag = LIGHT_TEMPERATURE_TRANSITION_START;
  }
  else
  {
#ifdef ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME       
    Light_CTLTemperatureDefaultTransitionValue(&Light_CtlParam);
#else
    /* When no optional parameter received, target value will
    be set as present value in application.
    */
    Light_CtlStatus.PresentCtlTemperature16 = Light_CtlParam.CTL_Temperature;
    Light_CtlStatus.PresentCtlDelta16 = Light_CtlParam.CTL_DeltaUv;
#endif    
  }
  
  /* Application Callback */
  (LightAppli_cb.Light_CtlTemperature_Set_cb)(&Light_CtlStatus, OptionalValid);
#ifdef ENABLE_MODEL_BINDING  
  /* Binding of data Ctl Temperature  with Generic Level */
  Light_CtlTemperature_Binding();
#endif   
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_CtlTemperature_Status
* @param  pLightCtlTemp_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_CtlTemperature_Status(MOBLEUINT8* pLightCtlTemp_status, MOBLEUINT32 *pLength)
{
  /* 6.3.2.4 Light Lightness Ctl Status
  
  Present Ctl Temperature         2B The Present ctl Temperature value of the model state. 
  Present Ctl Delta Uv            2B The Present ctl Delta Uv value of the model state. 
  Target Ctl Temperature          2B The Target ctl Tepmerature value of the model state.
  Target Ctl Delta Uv             2B The Target ctl Delta Uv value of the model state.
  Remaining Time                  1B The Remaining Time value of the model state
  */
  
  TRACE_M(TF_LIGHT_M,"Light_CtlTemperature_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#8261! \n\r");
  
  if((Light_ModelFlag.LightOptionalParam == 1) || (Light_TimeParam.StepValue != 0))
  {
    *(pLightCtlTemp_status) = Light_CtlStatus.PresentCtlTemperature16; 
    *(pLightCtlTemp_status+1) = Light_CtlStatus.PresentCtlTemperature16  >> 8;
    *(pLightCtlTemp_status+2) = Light_CtlStatus.PresentCtlDelta16;
    *(pLightCtlTemp_status+3) = Light_CtlStatus.PresentCtlDelta16 >>8;
    *(pLightCtlTemp_status+4) = Light_CtlStatus.TargetCtlTemperature16;
    *(pLightCtlTemp_status+5) = Light_CtlStatus.TargetCtlTemperature16 >>8;
    *(pLightCtlTemp_status+6) = Light_CtlStatus.TargetCtlDeltaUv16;
    *(pLightCtlTemp_status+7) = Light_CtlStatus.TargetCtlDeltaUv16 >>8;
    *(pLightCtlTemp_status+8) = Light_CtlStatus.RemainingTime;
    *pLength = 9;
    Light_ModelFlag.LightOptionalParam = 0;    
  }
  else
  {   
    /* When no optional parameter received then, target value will
    be sent in status message.
    */
    *(pLightCtlTemp_status) =  Light_CtlStatus.PresentCtlTemperature16; 
    *(pLightCtlTemp_status+1) = Light_CtlStatus.PresentCtlTemperature16  >> 8;
    *(pLightCtlTemp_status+2) = Light_CtlStatus.PresentCtlDelta16;
    *(pLightCtlTemp_status+3) = Light_CtlStatus.PresentCtlDelta16 >>8;
    *pLength = 4;             
  }
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_CtlTemperature_Range_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library.
* @param  plightCtlTempRange_param : Pointer to the parameters received for message.
* @param  length : Length of the parameters received for message.
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_CtlTemperature_Range_Set(const MOBLEUINT8* plightCtlTempRange_param, MOBLEUINT32 length)
{
  /* 6.3.2.10 Light Ctl Temperature Set
  
  Min Range          2B The minimum value for Ctl temperature state.
  Max Range          2B The maximum value for Ctl temperature state.
  */  
  MOBLEUINT8 OptionalValid = 0;
  TRACE_M(TF_LIGHT_M,"Light_CtlTemperature_Range_Set callback received \r\n"); 
  
  CtlTemperatureRangeParam.MinRangeStatus = plightCtlTempRange_param[1] << 8;
  CtlTemperatureRangeParam.MinRangeStatus |= plightCtlTempRange_param[0];
  
  CtlTemperatureRangeParam.MaxRangeStatus = plightCtlTempRange_param[3] << 8;
  CtlTemperatureRangeParam.MaxRangeStatus |= plightCtlTempRange_param[2];
  CtlTemperatureRangeParam.StatusCode = SUCCESS_STATUS_CODE;
  
  /* condition to check the minimum range value according to the specification*/
  /* set the status code for status message according to value of range*/  
  if((CtlTemperatureRangeParam.MinRangeStatus < MIN_CTL_TEMP_RANGE) ||
     (CtlTemperatureRangeParam.MinRangeStatus >MAX_CTL_TEMP_RANGE))
  {
    CtlTemperatureRangeParam.StatusCode = MIN_RANGE_INVALID_STATUS_CODE;
  }
  if((CtlTemperatureRangeParam.MaxRangeStatus < MIN_CTL_TEMP_RANGE) ||
     (CtlTemperatureRangeParam.MaxRangeStatus >MAX_CTL_TEMP_RANGE))
  {
    CtlTemperatureRangeParam.StatusCode = MIN_RANGE_INVALID_STATUS_CODE;
  }
  
  if((CtlTemperatureRangeParam.MaxRangeStatus == UNKNOWN_CTL_TEMP_VALUE) ||
     (CtlTemperatureRangeParam.MaxRangeStatus < CtlTemperatureRangeParam.MinRangeStatus))
  {
    CtlTemperatureRangeParam.StatusCode = MAX_RANGE_INVALID_STATUS_CODE;
  } 
  /* Application Callback */
  (LightAppli_cb.Light_CtlTemperature_Range_Set_cb)(&CtlTemperatureRangeParam, OptionalValid);
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_CtlTemperature_Range_Status
* @param  pCtlTempRange_status: Pointer to the status message, which needs to be updated.
* @param  pLength: Pointer to the Length of the Status message.
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_CtlTemperature_Range_Status(MOBLEUINT8* pCtlTempRange_status, MOBLEUINT32 *pLength)
{
  /* 6.3.2.12 Light Ctl Temperature status 
  Status Code        1B The Status code for the temperature range status.
  Min Range          2B The minimum value for Ctl temperature state.
  Max Range          2B The maximum value for Ctl temperature state.
  */
  
  TRACE_M(TF_LIGHT_M,"Light_CtlTemperature_Range_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#8262! \n\r");
  
  *pCtlTempRange_status = CtlTemperatureRangeParam.StatusCode;
  *(pCtlTempRange_status+1) = CtlTemperatureRangeParam.MinRangeStatus ;
  *(pCtlTempRange_status+2) = CtlTemperatureRangeParam.MinRangeStatus >> 8;
  *(pCtlTempRange_status+3) = CtlTemperatureRangeParam.MaxRangeStatus ;
  *(pCtlTempRange_status+4) = CtlTemperatureRangeParam.MaxRangeStatus >> 8;
  
  *pLength = 5;
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_CtlDefault_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library.
* @param  pCtlDefault_param : Pointer to the parameters received for message.
* @param  length : Length of the parameters received for message.
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_CtlDefault_Set(const MOBLEUINT8* pCtlDefault_param, MOBLEUINT32 length)
{
  /* 6.3.2.6 Light Ctl Default Set  
  Ctl Lightness         2B The Default value of the Ctl Lightness.
  Ctl Temperature       2B The Default value of the Ctl Temperature..
  Ctl Delta Uv          2B The Default value of the Ctl Delta Uv..  
  */
  MOBLEUINT8 OptionalValid = 0;
  
  TRACE_M(TF_LIGHT_M,"Light_CtlDefault_Set callback received \r\n");
  
  Light_CtlDefaultParam.CtlDefaultLightness16 = pCtlDefault_param[1] << 8;
  Light_CtlDefaultParam.CtlDefaultLightness16 |= pCtlDefault_param[0];
  
  Light_CtlDefaultParam.CtlDefaultTemperature16 = pCtlDefault_param[3] << 8;
  Light_CtlDefaultParam.CtlDefaultTemperature16 |= pCtlDefault_param[2];
  
  Light_CtlDefaultParam.CtlDefaultDeltaUv = pCtlDefault_param[5] << 8;
  Light_CtlDefaultParam.CtlDefaultDeltaUv |= pCtlDefault_param[4];
  
  /* Application Callback */
  (LightAppli_cb.Light_CtlDefault_Set_cb)(&Light_CtlDefaultParam , OptionalValid);
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_CtlDefault_Status
* @param  pCtlDefault_status: Pointer to the status message, which needs to be updated.
* @param  pLength: Pointer to the Length of the Status message.
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_CtlDefault_Status(MOBLEUINT8* pCtlDefault_status, MOBLEUINT32 *pLength)
{
  /* 6.3.2.16 Light Ctl Default Status 
  Ctl Lightness         2B The Default value of the Ctl Lightness.
  Ctl Temperature       2B The Default value of the Ctl Temperature..
  Ctl Delta Uv          2B The Default value of the Ctl Delta Uv.. 
  */
  
  TRACE_M(TF_LIGHT_M,"Light_Ctl_DefaultStatus callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#8267! \n\r");
  
  *pCtlDefault_status = Light_CtlDefaultParam.CtlDefaultLightness16 ;
  *(pCtlDefault_status +1) = Light_CtlDefaultParam.CtlDefaultLightness16 >> 8;
  *(pCtlDefault_status +2) = Light_CtlDefaultParam.CtlDefaultTemperature16;
  *(pCtlDefault_status +3) = Light_CtlDefaultParam.CtlDefaultTemperature16 >> 8;
  *(pCtlDefault_status +4) = Light_CtlDefaultParam.CtlDefaultDeltaUv;
  *(pCtlDefault_status +5) = Light_CtlDefaultParam.CtlDefaultDeltaUv >> 8;
  
  *pLength = 6;
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_Hsl_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library
* @param  pHsl_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Hsl_Set(const MOBLEUINT8* pHsl_param, MOBLEUINT32 length)
{
  /* 6.1.4.1 Light Hsl Set  
  HSL Lightness           2B The target value of the Light HSL Lightness state.
  HSL Hue                 2B The target value of the Light HSL Hue state.
  HSL Saturation          2B The target value of the Light HSL Saturation state.
  TID                     1B Transaction Identifier
  Transition Time         1B Format as defined in Section 3.1.3. (Optional)
  Delay                   1B Message execution delay in 5 millisecond steps (C.1)
  */
  MOBLEUINT8 OptionalValid = 0;
  
  TRACE_M(TF_LIGHT_M,"Light_Hsl_Set callback received \r\n");
  
  Light_HslParam.HslLightness16 = (MOBLEUINT16)pHsl_param[1] << 8;
  Light_HslParam.HslLightness16 |= (MOBLEUINT16)pHsl_param[0];
  
  Light_HslParam.HslHueLightness16 = (MOBLEUINT16)pHsl_param[3] << 8;
  Light_HslParam.HslHueLightness16 |= (MOBLEUINT16)pHsl_param[2];
  
  Light_HslParam.HslSaturation16 = (MOBLEUINT16)pHsl_param[5] << 8;
  Light_HslParam.HslSaturation16 |= (MOBLEUINT16)pHsl_param[4];
  
  Light_HslParam.Hsl_TID = pHsl_param[6];
  
  if((length > 7U) && (pHsl_param[7] !=0))
  {
    Light_HslParam.Hsl_TransitionTime = pHsl_param[7];
    Light_HslParam.Hsl_Delay = pHsl_param[8];
    
    Light_HslStatus.TargetHslLightness16 = Light_HslParam.HslLightness16;
    Light_HslStatus.TargetHslHueLightness16 = Light_HslParam.HslHueLightness16;
    Light_HslStatus.TargetHslSaturation16 = Light_HslParam.HslSaturation16;
    Light_HslStatus.RemainingTime = Light_HslParam.Hsl_TransitionTime;
    
    Light_TemporaryStatus.TargetParam_1 = Light_HslStatus.TargetHslLightness16;
    Light_TemporaryStatus.TargetParam_2 = Light_HslStatus.TargetHslHueLightness16;
    Light_TemporaryStatus.TargetParam_3 = Light_HslStatus.TargetHslSaturation16;
    
    Light_GetStepValue(pHsl_param[7]);
    Light_ModelFlag.LightOptionalParam = 1;
    Light_ModelFlag.LightTransitionFlag = LIGHT_HSL_TRANSITION_START;    
  }
  else
  {
    Light_HslStatus.PresentHslLightness16 = Light_HslParam.HslLightness16;
    Light_HslStatus.PresentHslHueLightness16 = Light_HslParam.HslHueLightness16;
    Light_HslStatus.PresentHslSaturation16 = Light_HslParam.HslSaturation16;
  }
  
  /* Application Callback */
  (LightAppli_cb.Light_Hsl_Set_cb)(&Light_HslStatus , OptionalValid);
  
#ifdef ENABLE_MODEL_BINDING   
  /* Light Hsl Lightness binding with lightness Actual */
  Light_HslLightness_LightnessActualBinding();
#endif  
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_Hsl_Status
* @param  pHsl_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_Hsl_Status(MOBLEUINT8* pHsl_status, MOBLEUINT32 *pLength)
{
  /* 6.3.3.4 Light Hsl Status 
  HSL Lightness           2B The present value of the Light HSL Lightness state.
  HSL Hue                 2B The present value of the Light HSL Hue state.
  HSL Saturation          2B The present value of the Light HSL Saturation state.
  Remaining Time          1B The remaining time value for transition of state.
  */
  
  TRACE_M(TF_LIGHT_M,"Light_Hsl_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#826D! \n\r");
  
  if((Light_ModelFlag.LightOptionalParam == 1) || (Light_TimeParam.StepValue != 0))
  {
  *(pHsl_status) = Light_HslStatus.PresentHslLightness16 ; 
  *(pHsl_status+1) = Light_HslStatus.PresentHslLightness16 >> 8;
  *(pHsl_status+2) = Light_HslStatus.PresentHslHueLightness16;
  *(pHsl_status+3) = Light_HslStatus.PresentHslHueLightness16 >> 8;
  *(pHsl_status+4) = Light_HslStatus.PresentHslSaturation16;
  *(pHsl_status+5) = Light_HslStatus.PresentHslSaturation16 >> 8;
    *(pHsl_status+6) = Light_HslStatus.TargetHslLightness16;
    *(pHsl_status+7) = Light_HslStatus.TargetHslLightness16 >> 8;
    *(pHsl_status+8) = Light_HslStatus.PresentHslHueLightness16;
    *(pHsl_status+9) = Light_HslStatus.PresentHslHueLightness16 >> 8;
    *(pHsl_status+10) = Light_HslStatus.TargetHslSaturation16;
    *(pHsl_status+11) = Light_HslStatus.TargetHslSaturation16 >> 8;
    *(pHsl_status+12) = Light_HslStatus.RemainingTime;
  
    *pLength = 13;
    Light_ModelFlag.LightOptionalParam = 0;     
  }
  else
  {            
    *(pHsl_status) = Light_HslStatus.PresentHslLightness16 ; 
    *(pHsl_status+1) = Light_HslStatus.PresentHslLightness16 >> 8;
    *(pHsl_status+2) = Light_HslStatus.PresentHslHueLightness16;
    *(pHsl_status+3) = Light_HslStatus.PresentHslHueLightness16 >> 8;
    *(pHsl_status+4) = Light_HslStatus.PresentHslSaturation16;
    *(pHsl_status+5) = Light_HslStatus.PresentHslSaturation16 >> 8;
    
    *pLength = 6;         
  }
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_HslHue_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library
* @param  pHslHue_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_HslHue_Set(const MOBLEUINT8* pHslHue_param, MOBLEUINT32 length)
{
  /* 6.3.3.8 Light Hsl Hue Set  
  HSL Lightness           2B The target value of the Light HSL Lightness state.
  HSL Hue                 2B The target value of the Light HSL Hue state.
  HSL Saturation          2B The target value of the Light HSL Saturation state.
  TID                     1B Transaction Identifier
  Transition Time         1B Format as defined in Section 3.1.3. (Optional)
  Delay                   1B Message execution delay in 5 millisecond steps (C.1)
  */
  MOBLEUINT8 OptionalValid = 0; 
  
  TRACE_M(TF_LIGHT_M,"Light_HslHue_Set callback received \r\n");
  
  Light_HslParam.HslHueLightness16 = (MOBLEUINT16)pHslHue_param[1] << 8;
  Light_HslParam.HslHueLightness16 |= (MOBLEUINT16)pHslHue_param[0];
  
  Light_HslParam.Hsl_TID = pHslHue_param[2];
  Light_HslHue_RangeBinding(&Light_HslParam);
  if((length > 3U) && (pHslHue_param[3] !=0))
  {
    Light_HslParam.Hsl_TransitionTime = pHslHue_param[3];
    Light_HslParam.Hsl_Delay = pHslHue_param[4]; 
    
    Light_HslStatus.TargetHslHueLightness16 = Light_HslParam.HslHueLightness16;
    Light_HslStatus.RemainingTime = Light_HslParam.Hsl_TransitionTime;    
    /* copy status parameters in Temporary parameters for transition 
    process.
    */
    Light_TemporaryStatus.TargetParam_1 = Light_HslStatus.TargetHslHueLightness16;    
    
    /* Function to calculate time parameters, step resolution
    step size for transition state machine.
    */
    Light_GetStepValue(pHslHue_param[3]);
    
    Light_ModelFlag.LightOptionalParam = 1;
    Light_ModelFlag.LightTransitionFlag = LIGHT_HSL_HUE_TRANSITION_START;    
  }
  else
  {
    Light_HslStatus.PresentHslHueLightness16 = Light_HslParam.HslHueLightness16;
  }
  /* Application Callback */
  (LightAppli_cb.Light_HslHue_Set_cb)(&Light_HslStatus , OptionalValid);
  
#ifdef ENABLE_MODEL_BINDING   
   /* Light Hsl Hue  binding with Generic level */
   Light_Hsl_Hue_Binding();
#endif    
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_HslHue_Status
* @param  pHslHue_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_HslHue_Status(MOBLEUINT8* pHslHue_status, MOBLEUINT32 *pLength)
{
  /* 6.3.3.10 Light Hsl Hue Status 
  Present HSL Hue         2B The Present value of the Light HSL Hue state.
  Target HSL Hue          2B The target value of the Light HSL Hue state.
  Remaining Time          1B The remaining time value for transition of state.
  */
  
  TRACE_M(TF_LIGHT_M,"Light_HslHue_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#826E! \n\r");
  
  *(pHslHue_status) = Light_HslStatus.PresentHslHueLightness16;
  *(pHslHue_status+1) = Light_HslStatus.PresentHslHueLightness16 >> 8;
  
  if((Light_ModelFlag.LightOptionalParam == 1) || (Light_TimeParam.StepValue != 0))
  {   
    *(pHslHue_status+2) = Light_HslStatus.TargetHslHueLightness16;
    *(pHslHue_status+3) = Light_HslStatus.TargetHslHueLightness16 >> 8;
    *(pHslHue_status+4) = Light_HslStatus.RemainingTime;
    *pLength = 5;
    Light_ModelFlag.LightOptionalParam = 5;     
  }
  else
  {            
    *pLength = 2;         
  }
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_HslSaturation_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library
* @param  pHslSaturation_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_HslSaturation_Set(const MOBLEUINT8* pHslSaturation_param, MOBLEUINT32 length)
{
  /* 6.3.3.12 Light Hsl Saturation Set  
  HSL Saturation          2B The target value of the Light HSL Saturation state.
  TID                     1B Transaction Identifier
  Transition Time         1B Format as defined in Section 3.1.3. (Optional)
  Delay                   1B Message execution delay in 5 millisecond steps (C.1)
  */
  
  TRACE_M(TF_LIGHT_M,"Light_HslSaturation_Set callback received \r\n");
  
  MOBLEUINT8 OptionalValid = 0;
  
  Light_HslParam.HslSaturation16 = (MOBLEUINT16)pHslSaturation_param[1] << 8;
  Light_HslParam.HslSaturation16 |= (MOBLEUINT16)pHslSaturation_param[0];
  
  Light_HslParam.Hsl_TID = pHslSaturation_param[2];
  Light_HslSaturation_RangeBinding(&Light_HslParam);
  if((length > 3U) && (pHslSaturation_param[3] != 0))
  {
    Light_HslParam.Hsl_TransitionTime = pHslSaturation_param[3];
    Light_HslParam.Hsl_Delay = pHslSaturation_param[4];
    
    Light_HslStatus.TargetHslSaturation16 = Light_HslParam.HslSaturation16;
    Light_HslStatus.RemainingTime = Light_HslParam.Hsl_TransitionTime;
    
    /* copy status parameters in Temporary parameters for transition 
    process.
    */
    Light_TemporaryStatus.TargetParam_1 = Light_HslStatus.TargetHslSaturation16;    
    
    /* Function to calculate time parameters, step resolution
    step size for transition state machine.
    */
    Light_GetStepValue(pHslSaturation_param[3]);
    
    Light_ModelFlag.LightOptionalParam = 1;
    Light_ModelFlag.LightTransitionFlag = LIGHT_HSL_SATURATION_TRANSITION_START; 
  }
  else
  {
    Light_HslStatus.PresentHslSaturation16= Light_HslParam.HslSaturation16; 
  }
  /* Application Callback */
  (LightAppli_cb.Light_HslSaturation_Set_cb)(&Light_HslStatus , OptionalValid);
  
#ifdef ENABLE_MODEL_BINDING   
  /* Light Hsl Saturation binding with Generic level model*/
  Light_Hsl_Saturation_Binding();
#endif    
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_HslSaturation_Status
* @param  pHslSaturation_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_HslSaturation_Status(MOBLEUINT8* pHslSaturation_status, MOBLEUINT32 *pLength)
{
  /* 6.3.3.14 Light Hsl Saturation Status 
  Present HSL Saturation         2B The Present value of the Light HSL Hue state.
  Target HSL Saturation          2B The target value of the Light HSL Hue state.
  Remaining Time          1B The remaining time value for transition of state.
  */
  
  TRACE_M(TF_LIGHT_M,"Light_HslSaturation_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#8272! \n\r");
  
  *(pHslSaturation_status) = Light_HslStatus.PresentHslSaturation16;
  *(pHslSaturation_status+1) = Light_HslStatus.PresentHslSaturation16 >> 8;
  
  if((Light_ModelFlag.LightOptionalParam == 1) || (Light_TimeParam.StepValue != 0))
  {          
    *(pHslSaturation_status+2) = Light_HslStatus.TargetHslSaturation16;
    *(pHslSaturation_status+3) = Light_HslStatus.TargetHslSaturation16 >> 8;
    *(pHslSaturation_status+4) = Light_HslStatus.RemainingTime;
    *pLength = 5;
    Light_ModelFlag.LightOptionalParam = 0;         
  }
  else
  {            
    *pLength = 2;         
  }
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_HslDefault_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library
* @param  pHslDefault_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_HslDefault_Set(const MOBLEUINT8* pHslDefault_param, MOBLEUINT32 length)
{
  /* 6.3.3.16 Light Hsl Saturation Set  
  HSL Lightness           2B The target value of the Light HSL Lightness state.
  HSL Hue                 2B The target value of the Light HSL Hue state.
  HSL Saturation          2B The target value of the Light HSL Saturation state.
  */
  MOBLEUINT8 OptionalValid = 0;
  
  TRACE_M(TF_LIGHT_M,"Light_HslDefault_Set callback received \r\n"); 
  
  Light_HslStatus.PresentHslLightness16 = (MOBLEUINT16)pHslDefault_param[1] << 8;
  Light_HslStatus.PresentHslLightness16 |= (MOBLEUINT16)pHslDefault_param[0];
  
  Light_HslStatus.PresentHslHueLightness16 = (MOBLEUINT16)pHslDefault_param[3] << 8;
  Light_HslStatus.PresentHslHueLightness16 |= (MOBLEUINT16)pHslDefault_param[2];
  
  Light_HslStatus.PresentHslSaturation16 = (MOBLEUINT16)pHslDefault_param[5] << 8;
  Light_HslStatus.PresentHslSaturation16 |= (MOBLEUINT16)pHslDefault_param[4];
  
  /* Application Callback */
  (LightAppli_cb.Light_HslDefault_Set_cb)(&Light_HslStatus , OptionalValid);
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_HslDefault_Status
* @param  pHslDefault_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_HslDefault_Status(MOBLEUINT8* pHslDefault_status, MOBLEUINT32 *pLength)
{
  /* 6.3.3.18 Light Hsl Default Status 
  HSL Lightness           2B The target value of the Light HSL Lightness state.
  HSL Hue                 2B The target value of the Light HSL Hue state.
  HSL Saturation          2B The target value of the Light HSL Saturation state.
  */
  
  TRACE_M(TF_LIGHT_M,"Light_HslDefault_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#827B! \n\r");
  
  *pHslDefault_status = Light_HslStatus.PresentHslLightness16;
  *(pHslDefault_status+1) = Light_HslStatus.PresentHslLightness16 >> 8;
  *(pHslDefault_status+2) = Light_HslStatus.PresentHslHueLightness16;
  *(pHslDefault_status+3) = Light_HslStatus.PresentHslHueLightness16 >> 8;
  *(pHslDefault_status+4) = Light_HslStatus.PresentHslSaturation16;
  *(pHslDefault_status+5) = Light_HslStatus.PresentHslSaturation16 >> 8;
  
  *pLength = 6; 
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_HslRange_Set: This function is called for both Acknowledged and 
* unacknowledged message. The Acknowledgement is taken care by the Library
* @param  pHslRange_param : Pointer to the parameters received for message
* @param  length : Length of the parameters received for message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_HslRange_Set(const MOBLEUINT8* pHslRange_param, MOBLEUINT32 length)
{
  /* 6.3.3.20 Light Hsl Range Set  
  Hue Range Min          The value of the Hue Range Min field of the Light HSL Hue Range state
  Hue Range Max          The value of the Hue Range Max field of the Light HSL Hue Range state
  Saturation Range Min   The value of the Saturation Range Min field of the Light HSL Saturation Range state
  Saturation Range Max   The value of the Saturation Range Max field of the Light HSL Saturation Range state
  */
  MOBLEUINT8 OptionalValid = 0;
  TRACE_M(TF_LIGHT_M,"Light_HslRange_Set callback received \r\n");
  
  Light_HslRangeParam.HslHueMinRange16 = (MOBLEUINT16)pHslRange_param[1] << 8;
  Light_HslRangeParam.HslHueMinRange16 |= (MOBLEUINT16)pHslRange_param[0];
  
  Light_HslRangeParam.HslHueMaxRange16 = (MOBLEUINT16)pHslRange_param[3] << 8;
  Light_HslRangeParam.HslHueMaxRange16 |= (MOBLEUINT16)pHslRange_param[2];
  
  Light_HslRangeParam.HslMinSaturation16 = (MOBLEUINT16)pHslRange_param[5] << 8;
  Light_HslRangeParam.HslMinSaturation16 |= (MOBLEUINT16)pHslRange_param[4];
  
  Light_HslRangeParam.HslMaxSaturation16 = (MOBLEUINT16)pHslRange_param[7] << 8;
  Light_HslRangeParam.HslMaxSaturation16 |= (MOBLEUINT16)pHslRange_param[6];
  
  /* Application Callback */
  (LightAppli_cb.Light_HslRange_Set_cb)(&Light_HslRangeParam , OptionalValid);
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_HslRange_Status
* @param  pHslRange_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_HslRange_Status(MOBLEUINT8* pHslRange_status, MOBLEUINT32 *pLength)
{
  /* 6.3.3.22 Light Hsl Range Set 
  Hue Range Min          The value of the Hue Range Min field of the Light HSL Hue Range state
  Hue Range Max          The value of the Hue Range Max field of the Light HSL Hue Range state
  Saturation Range Min   The value of the Saturation Range Min field of the Light HSL Saturation Range state
  Saturation Range Max   The value of the Saturation Range Max field of the Light HSL Saturation Range state
  */   
  
  TRACE_M(TF_LIGHT_M,"Light_HslRange_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#827D! \n\r");
  
  /* Function call back to get the values from application*/   
  *pHslRange_status = Light_HslRangeParam.Status_Code;  
  *(pHslRange_status+1) = Light_HslRangeParam.HslHueMinRange16;
  *(pHslRange_status+2) = Light_HslRangeParam.HslHueMinRange16 >> 8;
  *(pHslRange_status+3) = Light_HslRangeParam.HslHueMaxRange16;
  *(pHslRange_status+4) = Light_HslRangeParam.HslHueMaxRange16 >> 8;
  *(pHslRange_status+5) = Light_HslRangeParam.HslMinSaturation16;
  *(pHslRange_status+6) = Light_HslRangeParam.HslMinSaturation16 >> 8;
  *(pHslRange_status+7) = Light_HslRangeParam.HslMaxSaturation16;
  *(pHslRange_status+8) = Light_HslRangeParam.HslMaxSaturation16 >> 8;
  
  *pLength = 9;
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Light_HslTarget_Status
* @param  pHslTarget_status: Pointer to the status message, which needs to be updated
* @param  pLength: Pointer to the Length of the Status message
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Light_HslTarget_Status(MOBLEUINT8* pHslTarget_status, MOBLEUINT32 *pLength)
{
  /* 6.3.3.5 Light HSL Target Get
  HSL Lightness           2B The target value of the Light HSL Lightness state.
  HSL Hue                 2B The target value of the Light HSL Hue state.
  HSL Saturation          2B The target value of the Light HSL Saturation state.
  */
  
  TRACE_M(TF_LIGHT_M,"Light_HslTarget_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#8279! \n\r");
  /* Function call back to get the values from application*/
  
  *(pHslTarget_status) = Light_HslStatus.TargetHslLightness16 ; 
  *(pHslTarget_status+1) = Light_HslStatus.TargetHslLightness16 >> 8;
  *(pHslTarget_status+2) = Light_HslStatus.TargetHslHueLightness16;
  *(pHslTarget_status+3) = Light_HslStatus.TargetHslHueLightness16 >> 8;
  *(pHslTarget_status+4) = Light_HslStatus.TargetHslSaturation16;
  *(pHslTarget_status+5) = Light_HslStatus.TargetHslSaturation16 >> 8; 
  
  if((Light_ModelFlag.LightOptionalParam == 1) || (Light_TimeParam.StepValue != 0))
  {          
    *(pHslTarget_status+6) = Light_HslStatus.RemainingTime;
    *pLength = 7;
    Light_ModelFlag.LightOptionalParam = 0;     
  }
  else
  {            
    *pLength = 6;         
  }
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  LightModelServer_GetLightOpcodeTableCb: This function is call-back 
from the library to send Model Opcode Table info to library
* @param  MODEL_OpcodeTableParam_t:  Pointer to the Light Model opcode array 
* @param  length: Pointer to the Length of Light Model opcode array
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT LightModelServer_GetOpcodeTableCb(const MODEL_OpcodeTableParam_t **data, 
                                               MOBLEUINT16 *length)
{
  *data = Light_Opcodes_Table;
  *length = sizeof(Light_Opcodes_Table)/sizeof(Light_Opcodes_Table[0]);
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  LightModelServer_GetStatusRequestCb : This function is call-back 
from the library to send response to the message from peer
* @param  peer_addr: Address of the peer
* @param  dst_peer: destination send by peer for this node. It can be a
*                                                     unicast or group address 
* @param  opcode: Received opcode of the Status message callback
* @param  pResponsedata: Pointer to the buffer to be updated with status
* @param  plength: Pointer to the Length of the data, to be updated by application
* @param  pRxData: Pointer to the data received in packet.
* @param  dataLength: length of the data in packet.
* @param  response: Value to indicate wheather message is acknowledged meassage or not.
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT LightModelServer_GetStatusRequestCb(MOBLE_ADDRESS peer_addr, 
                                    MOBLE_ADDRESS dst_peer, 
                                    MOBLEUINT16 opcode, 
                                    MOBLEUINT8 *pResponsedata, 
                                    MOBLEUINT32 *plength, 
                                    MOBLEUINT8 const *pRxData,
                                    MOBLEUINT32 dataLength,
                                    MOBLEBOOL response)
{
  switch(opcode)
  {
#ifdef ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS
    
  case LIGHT_LIGHTNESS_STATUS:
    {
      Light_Lightness_Status(pResponsedata, plength);
      break;
    }    
  case LIGHT_LIGHTNESS_LINEAR_STATUS:
    {
      Light_Lightness_Linear_Status(pResponsedata, plength);
      break;
    }     
  case LIGHT_LIGHTNESS_LAST_STATUS:
    {
      Light_Lightness_Last_Status(pResponsedata, plength);
      break;
    }   
  case LIGHT_LIGHTNESS_DEFAULT_STATUS:
    {
      Light_Lightness_Default_Status(pResponsedata, plength);
      break;
    }  
  case LIGHT_LIGHTNESS_RANGE_STATUS:
    {
      Light_Lightness_Range_Status(pResponsedata, plength);
      break;
    } 
#endif  
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_CTL 
    
  case LIGHT_CTL_STATUS:
    {      
      Light_Ctl_Status(pResponsedata , plength);    
      break;
    }    
  case LIGHT_CTL_TEMPERATURE_RANGE_STATUS:
    {    
      Light_CtlTemperature_Range_Status(pResponsedata , plength);
      break;
    }
  case LIGHT_CTL_DEFAULT_STATUS:
    {
      Light_CtlDefault_Status(pResponsedata , plength);          
      break;
    } 
#endif
    
#if defined ENABLE_LIGHT_MODEL_SERVER_CTL || defined ENABLE_LIGHT_MODEL_SERVER_CTL_TEMPERATURE 
    
  case LIGHT_CTL_TEMPERATURE_STATUS:    
    {
      Light_CtlTemperature_Status(pResponsedata , plength);
      break;
    }
#endif
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL    
  case LIGHT_HSL_STATUS:
    {
      Light_Hsl_Status(pResponsedata , plength);         
      break;
    } 
  case LIGHT_HSL_DEFAULT_STATUS:
    {
      Light_HslDefault_Status(pResponsedata , plength);         
      break;
    }    
  case LIGHT_HSL_RANGE_STATUS:
    {
      Light_HslRange_Status(pResponsedata , plength);         
      break;
    } 
  case LIGHT_HSL_TARGET_STATUS:
    {
      Light_HslTarget_Status(pResponsedata , plength);
      break;
    }
#endif
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL_HUE    
  case LIGHT_HSL_HUE_STATUS:
    {
      Light_HslHue_Status(pResponsedata , plength);          
      break;
    } 
#endif
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL_SATURATION    
  case LIGHT_HSL_SATURATION_STATUS:
    {
      Light_HslSaturation_Status(pResponsedata , plength);        
      break;
    } 
#endif
  default:
    {
      break;
    }
  }
  return MOBLE_RESULT_SUCCESS;    
}


/**
* @brief  LightModelServer_ProcessMessageCb: This is a callback function from
*         the library whenever a Light Model message is received
* @param  peer_addr: Address of the peer
* @param  dst_peer: destination send by peer for this node. It can be a
*                                                     unicast or group address 
* @param  opcode: Received opcode of the Status message callback
* @param  pData: Pointer to the buffer to be updated with status
* @param  length: Length of the parameters received 
* @param  response: if TRUE, the message is an acknowledged message 
* @param  pRxData: Pointer to the data received in packet.
* @param  dataLength: length of the data in packet.
* @param  response: Value to indicate wheather message is acknowledged meassage or not.
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT LightModelServer_ProcessMessageCb(MOBLE_ADDRESS peer_addr, 
                                               MOBLE_ADDRESS dst_peer, 
                                               MOBLEUINT16 opcode, 
                                               MOBLEUINT8 const *pRxData, 
                                               MOBLEUINT32 dataLength, 
                                               MOBLEBOOL response
                                                 )
{
  MOBLE_RESULT result = MOBLE_RESULT_SUCCESS;
  MOBLE_ADDRESS publishAddress;
  MOBLEUINT8 elementNumber;
  MOBLEUINT8 modelStateChangeFlag = MOBLE_FALSE;  
  MOBLE_ADDRESS my_Address;  
  
  Model_Rx_Opcode = opcode;
  my_Address = BLEMesh_GetAddress();
  Dst_Peer = dst_peer;
  
  TRACE_M(TF_LIGHT_M,"dst_peer = %.2X , peer_add = %.2X, opcode= %.2X,response= %.2X \r\n ",dst_peer, peer_addr, opcode,response);
  
  switch(opcode)
  {
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS  
    
  case LIGHT_LIGHTNESS_SET:
  case LIGHT_LIGHTNESS_SET_UNACK:
    {       
      result = Chk_ParamMinMaxValidity(MIN_VALID_RANGE ,pRxData , MAX_VALID_RANGE );
      
      if(result == MOBLE_RESULT_SUCCESS)
      {
        if(!MOBLE_FAILED(result = Chk_TidValidity(peer_addr,dst_peer,pRxData[2])))
        {
          Light_Lightness_Set(pRxData, dataLength);
          Model_ID = (MOBLEUINT16)LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID;
          modelStateChangeFlag = MOBLE_TRUE;          
        }
      }
      break;
    }
  case LIGHT_LIGHTNESS_LINEAR_SET:
  case LIGHT_LIGHTNESS_LINEAR_SET_UNACK: 
    {
      result = Chk_ParamMinMaxValidity(MIN_VALID_RANGE , pRxData , MAX_VALID_RANGE);
      
      if(result == MOBLE_RESULT_SUCCESS)
      {
        if(!MOBLE_FAILED(result = Chk_TidValidity(peer_addr,dst_peer,pRxData[2])))
        {
          Light_Lightness_Linear_Set(pRxData, dataLength);
          Model_ID = (MOBLEUINT16)LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID;
          modelStateChangeFlag = MOBLE_TRUE;
        }
      }
      break;
    }
  case LIGHT_LIGHTNESS_STATUS:
    {
      Light_Client_Lightness_Status(pRxData, dataLength);
      break;
    }    
  case LIGHT_LIGHTNESS_LINEAR_STATUS:
    {
      Light_Client_Lightness_Linear_Status(pRxData, dataLength);
      break;
    }     
  case LIGHT_LIGHTNESS_LAST_STATUS:
    {
      Light_Client_Lightness_Last_Status(pRxData, dataLength);
      break;
    }   
  case LIGHT_LIGHTNESS_DEFAULT_STATUS:
    {
      Light_Client_Lightness_Default_Status(pRxData, dataLength);
      break;
    }  
  case LIGHT_LIGHTNESS_RANGE_STATUS:
    {
      Light_Client_Lightness_Range_Status(pRxData, dataLength);
      break;
    }
#endif 
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS_SETUP
    
  case LIGHT_LIGHTNESS_DEFAULT_SET:
  case LIGHT_LIGHTNESS_DEFAULT_SET_UNACK:
    {      
      result = Chk_ParamMinMaxValidity(MIN_VALID_RANGE , pRxData , MAX_VALID_RANGE);
      
      if(result == MOBLE_RESULT_SUCCESS)
      {
        Light_Lightness_Default_Set(pRxData, dataLength);
      }
      break;
    }
  case LIGHT_LIGHTNESS_RANGE_SET:
  case LIGHT_LIGHTNESS_RANGE_SET_UNACK:
    {
      result = Chk_RangeValidity(MIN_VALID_RANGE , pRxData , MAX_VALID_RANGE);
      
      if(result == MOBLE_RESULT_SUCCESS)
      {
        Light_Lightness_Range_Set(pRxData, dataLength);
      }
      break;
    }
#endif
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_CTL
    
  case LIGHT_CTL_SET:
  case LIGHT_CTL_SET_UNACK:
    {       
      result = Chk_TwoParamValidity(MIN_VALID_RANGE ,MAX_VALID_RANGE ,MIN_CTL_TEMP_RANGE ,MAX_CTL_TEMP_RANGE ,pRxData);
      
      if(result == MOBLE_RESULT_SUCCESS)
      {
        if(!MOBLE_FAILED(result = Chk_TidValidity(peer_addr,dst_peer,pRxData[6])))
        {
          Light_Ctl_Set(pRxData, dataLength);
          Model_ID = (MOBLEUINT16)LIGHT_MODEL_SERVER_CTL_MODEL_ID;
          modelStateChangeFlag = MOBLE_TRUE;    
        }        
      }        
      break;
    }
  case LIGHT_CTL_STATUS:
    {      
      Light_Client_Ctl_Status(pRxData, dataLength);    
      break;
    }    
  case LIGHT_CTL_TEMPERATURE_RANGE_STATUS:
    {    
      Light_Client_CtlTemperature_Range_Status(pRxData, dataLength);
      break;
    }
  case LIGHT_CTL_DEFAULT_STATUS:
    {
      Light_Client_CtlDefault_Status(pRxData, dataLength);          
      break;
    } 
#endif
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_CTL_TEMPERATURE  
    
  case LIGHT_CTL_TEMPERATURE_SET:
  case LIGHT_CTL_TEMPERATURE_SET_UNACK:
    {
      result = Chk_ParamMinMaxValidity(MIN_CTL_TEMP_RANGE , pRxData , MAX_CTL_TEMP_RANGE);
      
      if(result == MOBLE_RESULT_SUCCESS)
      {
        if(!MOBLE_FAILED(result = Chk_TidValidity(peer_addr,dst_peer,pRxData[4])))
        {
          Light_CtlTemperature_Set(pRxData, dataLength);
          Model_ID = (MOBLEUINT16)LIGHT_MODEL_SERVER_CTL_TEMPERATURE_MODEL_ID;
          modelStateChangeFlag = MOBLE_TRUE;
        }
      }
      break;
    }
  case LIGHT_CTL_TEMPERATURE_STATUS:    
    {
      Light_Client_CtlTemperature_Status(pRxData, dataLength);
      break;
    }
#endif
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_CTL_SETUP
    
  case LIGHT_CTL_TEMPERATURE_RANGE_SET:
  case LIGHT_CTL_TEMPERATURE_RANGE_SET_UNACK:
    {
      result = Chk_RangeValidity(MIN_CTL_TEMP_RANGE , pRxData , MAX_CTL_TEMP_RANGE);
      
      if(result == MOBLE_RESULT_SUCCESS)
      {
        Light_CtlTemperature_Range_Set(pRxData, dataLength);
      }
      break;
    }
  case LIGHT_CTL_DEFAULT_SET:
  case LIGHT_CTL_DEFAULT_SET_UNACK:
    { 
      result = Chk_MultiParamValidity(MIN_VALID_RANGE ,MAX_VALID_RANGE ,
                                                 MIN_CTL_TEMP_RANGE ,MAX_CTL_TEMP_RANGE ,
                                                  MIN_CTL_DELTA_VALUE ,MAX_CTL_DELTA_VALUE ,pRxData);
      
      if(result == MOBLE_RESULT_SUCCESS) 
      {
        Light_CtlDefault_Set(pRxData, dataLength);
      }
      break;
    }
#endif
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL
    
  case LIGHT_HSL_SET:                           
  case LIGHT_HSL_SET_UNACK:
    {
      result = Chk_MultiParamValidityAllUnsigned(MIN_VALID_RANGE ,MAX_VALID_RANGE ,
                                                    MIN_VALID_RANGE ,MAX_VALID_RANGE ,
                                                      MIN_VALID_RANGE ,MAX_VALID_RANGE ,pRxData);
      
      if(result == MOBLE_RESULT_SUCCESS) 
      {
        if(!MOBLE_FAILED(result = Chk_TidValidity(peer_addr,dst_peer,pRxData[6])))
        {
          Light_Hsl_Set(pRxData, dataLength);
          Model_ID = (MOBLEUINT16)LIGHT_MODEL_SERVER_HSL_MODEL_ID;
          modelStateChangeFlag = MOBLE_TRUE;
        }
      }
      break;
    }
  case LIGHT_HSL_STATUS:
    {
      Light_Client_Hsl_Status(pRxData, dataLength);         
      break;
    }
  case LIGHT_HSL_DEFAULT_STATUS:
    {
      Light_Client_HslDefault_Status(pRxData, dataLength);         
      break;
    }    
  case LIGHT_HSL_RANGE_STATUS:
    {
      Light_Client_HslRange_Status(pRxData, dataLength);         
      break;
    } 
  case LIGHT_HSL_TARGET_STATUS:
    {
      Light_Client_HslTarget_Status(pRxData, dataLength);
      break;
    }
#endif
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL_HUE
    
  case LIGHT_HSL_HUE_SET:                       
  case LIGHT_HSL_HUE_SET_UNACK: 
    {
      result = Chk_ParamMinMaxValidity(MIN_VALID_RANGE , pRxData , MAX_VALID_RANGE);
      
      if(result == MOBLE_RESULT_SUCCESS)
      {
        if(!MOBLE_FAILED(result = Chk_TidValidity(peer_addr,dst_peer,pRxData[2])))
        {
          Light_HslHue_Set(pRxData, dataLength);
          Model_ID = (MOBLEUINT16)LIGHT_MODEL_SERVER_HSL_HUE_MODEL_ID;
          modelStateChangeFlag = MOBLE_TRUE;
          
        }
      }
      break;
    }
  case LIGHT_HSL_HUE_STATUS:
    {
      Light_Client_HslHue_Status(pRxData, dataLength);          
      break;
    } 
#endif 
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL_SETUP
    
  case LIGHT_HSL_DEFAULT_SET:                   
  case LIGHT_HSL_DEFAULT_SET_UNACK: 
    {       
      Light_HslDefault_Set(pRxData, dataLength);
      break;
    }
  case LIGHT_HSL_RANGE_SET:                     
  case LIGHT_HSL_RANGE_SET_UNACK:
    {
      result = Chk_HslRangeValidity(pRxData ,MIN_HUE_RANGE,MAX_HUE_RANGE,
                                  MIN_SATURATION_RANGE,MAX_SATURATION_RANGE);
      if(result == MOBLE_RESULT_SUCCESS)
      {
      Light_HslRange_Set(pRxData , dataLength);
      }
      break;
    }
#endif
    
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL_SATURATION
    
  case LIGHT_HSL_SATURATION_SET:                     
  case LIGHT_HSL_SATURATION_SET_UNACK:
    {
      if(!MOBLE_FAILED(result = Chk_TidValidity(peer_addr,dst_peer,pRxData[2])))
      {
        Light_HslSaturation_Set(pRxData, dataLength);
        Model_ID = (MOBLEUINT16)LIGHT_MODEL_SERVER_HSL_SATURATION_MODEL_ID;
          modelStateChangeFlag = MOBLE_TRUE;
      }
      break;
    }
  case LIGHT_HSL_SATURATION_STATUS:
    {
      Light_Client_HslSaturation_Status(pRxData, dataLength);        
      break;
    } 
#endif  
  default:
    {
      break;
    }    
  } /* Switch ends */
  
  if((result == MOBLE_RESULT_SUCCESS) && (response == MOBLE_TRUE) && (ADDRESS_IS_UNICAST(dst_peer)))
  {
    Model_SendResponse(peer_addr, my_Address,opcode,pRxData, dataLength);                                           
  }
  
  /*
     Publish the status to publish address if the publication is set by client 
     publication is independent of the response of the message.if the condition 
     for publication is full filled as per specification then the status will be 
     published.
  */
    elementNumber = BLE_GetElementNumber();
    publishAddress = BLEMesh_GetPublishAddress(elementNumber,Model_ID);
  
  if((result == MOBLE_RESULT_SUCCESS) && (publishAddress != 0x0000) && (modelStateChangeFlag == MOBLE_TRUE)
     && (ADDRESS_IS_UNICAST(dst_peer)))
  {
    Model_SendResponse(publishAddress,my_Address,opcode,pRxData,dataLength);
    
    modelStateChangeFlag = MOBLE_FALSE;  
    
    TRACE_I(TF_LIGHT_M,"Publishing state to the address  %.2X \r\n",publishAddress);
  }

  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_Lightness_Status: Function called when status of the model 
*        received on the client.
* @param pLightness_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_Lightness_Status(MOBLEUINT8 const *pLightness_status, MOBLEUINT32 plength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_CLIENT_M,"Light_Client_Lightness_Status received \r\n");
  LightAppli_cb.Lightness_Status_cb(pLightness_status, plength);

  for(i = 0; i < plength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_Lightness_Status: %d\r\n",
            pLightness_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_Lightness_Linear_Status: Function called when status of the model 
*        received on the client.
* @param pLightnessLinear_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_Lightness_Linear_Status(MOBLEUINT8 const *pLightnessLinear_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_Lightness_Linear_Status received \r\n");
  LightAppli_cb.Lightness_Linear_Status_cb(pLightnessLinear_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_Lightness_Linear_Status: %d\r\n",
            pLightnessLinear_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_Lightness_Last_Status: Function called when status of the model 
*        received on the client.
* @param pLightnessLast_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_Lightness_Last_Status(MOBLEUINT8 const *pLightnessLast_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_Lightness_Last_Status received \r\n");
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_Lightness_Last_Status: %d\r\n",
            pLightnessLast_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_Lightness_Default_Status: Function called when status of the model 
*        received on the client.
* @param pLightnessDefault_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_Lightness_Default_Status(MOBLEUINT8 const *pLightnessDefault_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_Lightness_Default_Status received \r\n");
  LightAppli_cb.Lightness_Default_Status_cb(pLightnessDefault_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_Lightness_Default_Status: %d\r\n",
            pLightnessDefault_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_Lightness_Range_Status: Function called when status of the model 
*        received on the client.
* @param pLightnessRange_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_Lightness_Range_Status(MOBLEUINT8 const *pLightnessRange_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_Lightness_Range_Status received \r\n");
  LightAppli_cb.Lightness_Range_Status_cb(pLightnessRange_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_Client_Lightness_Range_Status: %d\r\n",
            pLightnessRange_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_Ctl_Status: Function called when status of the model 
*        received on the client.
* @param pLightCtl_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_Ctl_Status(MOBLEUINT8 const *pLightCtl_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_Ctl_Status received \r\n");
  LightAppli_cb.Light_Ctl_Status_cb(pLightCtl_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_Ctl_Status: %d\r\n",
            pLightCtl_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_CtlTemperature_Range_Status: Function called when status of the model 
*        received on the client.
* @param pCtlTempRange_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_CtlTemperature_Range_Status(MOBLEUINT8 const *pCtlTempRange_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_CtlTemperature_Range_Status received \r\n");
  LightAppli_cb.Light_CtlTemperature_Range_Status_cb(pCtlTempRange_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_CtlTemperature_Range_Status: %d\r\n",
            pCtlTempRange_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_CtlDefault_Status: Function called when status of the model 
*        received on the client.
* @param pCtlDefault_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_CtlDefault_Status(MOBLEUINT8 const *pCtlDefault_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_Ctl_DefaultStatus received \r\n");
  LightAppli_cb.Light_CtlDefault_Status_cb(pCtlDefault_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_Ctl_DefaultStatus: %d\r\n",
            pCtlDefault_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_CtlTemperature_Status: Function called when status of the model 
*        received on the client.
* @param pLightCtlTemp_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_CtlTemperature_Status(MOBLEUINT8 const *pLightCtlTemp_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_CtlTemperature_Status received \r\n");
  LightAppli_cb.Light_CtlTemperature_Status_cb(pLightCtlTemp_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_CtlTemperature_Status: %d\r\n",
            pLightCtlTemp_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_Hsl_Status: Function called when status of the model 
*        received on the client.
* @param pHsl_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_Hsl_Status(MOBLEUINT8 const *pHsl_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_Hsl_Status received \r\n");
  LightAppli_cb.Light_Hsl_Status_cb(pHsl_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_Hsl_Status: %d\r\n",
            pHsl_status[i]);
  }
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_HslDefault_Status: Function called when status of the model 
*        received on the client.
* @param pHslDefault_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_HslDefault_Status(MOBLEUINT8 const *pHslDefault_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_HslDefault_Status received \r\n");
  LightAppli_cb.Light_HslDefault_Status_cb(pHslDefault_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_HslDefault_Status: %d\r\n",
            pHslDefault_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_HslRange_Status: Function called when status of the model 
*        received on the client.
* @param pHslRange_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_HslRange_Status(MOBLEUINT8 const *pHslRange_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_HslRange_Status received \r\n");
  LightAppli_cb.Light_HslRange_Status_cb(pHslRange_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_HslRange_Status: %d\r\n",
            pHslRange_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_HslTarget_Status: Function called when status of the model 
*        received on the client.
* @param pHslTarget_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_HslTarget_Status(MOBLEUINT8 const *pHslTarget_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_HslTarget_Status received \r\n");
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_HslTarget_Status: %d\r\n",
            pHslTarget_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_HslHue_Status: Function called when status of the model 
*        received on the client.
* @param pHslHue_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_HslHue_Status(MOBLEUINT8 const *pHslHue_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_HslHue_Status received \r\n");
  LightAppli_cb.Light_HslHue_Status_cb(pHslHue_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_HslHue_Status: %d\r\n",
            pHslHue_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_Client_HslSaturation_Status: Function called when status of the model 
*        received on the client.
* @param pHslSaturation_status: ointer to the parameters received for message
* @param plength: Length of the parameters received for message
* return MOBLE_RESULT_SUCCESS.
*/
MOBLE_RESULT Light_Client_HslSaturation_Status(MOBLEUINT8 const *pHslSaturation_status, MOBLEUINT32 pLength)
{
  MOBLEUINT32 i;
  
  TRACE_M(TF_LIGHT_M,"Light_HslSaturation_Status received \r\n");
  LightAppli_cb.Light_HslSaturation_Status_cb(pHslSaturation_status, pLength);
  
  for(i = 0; i < pLength; i++)
  {
    TRACE_M(TF_SERIAL_CTRL,"Light_HslSaturation_Status: %d\r\n",
            pHslSaturation_status[i]);
  }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_TransitionBehaviourSingle_Param funtion is used for the Light Lightness model
*         when transition time is  received in message.This function is used for 
*         single paramter transition.
* @param GetValue: Pointer of the array
* @retval MOBLE_RESULT
*/
MOBLE_RESULT Light_TransitionBehaviourSingle_Param(MOBLEUINT8 *GetValue)
{
  
  static MOBLEUINT8 Clockflag = 0;
  static MOBLEUINT32 Check_time;
  MOBLEUINT16 targetRange;
  MOBLEUINT16 targetSlot;
  
  if(Clockflag == 0)
  {
    Check_time = Clock_Time();
    Clockflag = 1;
  }
  /* Values from application are copied into temporary vaiables for processing */    
  Light_TemporaryStatus.PresentParam_1 = GetValue[1] << 8;
  Light_TemporaryStatus.PresentParam_1 |= GetValue[0];   
  /*if condition to wait untill the time is equal to the given resolution time */  
  if(((Clock_Time()- Check_time) >= Light_TimeParam.Res_Value))
  {
    if(Light_TimeParam.StepValue == 0)
    {
      Light_TimeParam.StepValue = 1;
    }
    
    if(Light_TemporaryStatus.TargetParam_1 > Light_TemporaryStatus.PresentParam_1)
    {
      /* calulating the target range to be acheived which is traget value
      which is traget value minus present value if target value is greater 
      than present value.   
      */
      targetRange = Light_TemporaryStatus.TargetParam_1 - Light_TemporaryStatus.PresentParam_1;  
      /*target slot = time to cover in single step */
      targetSlot = targetRange/Light_TimeParam.StepValue;
      /* target slot added to present value to achieve target value */
      Light_TemporaryStatus.PresentParam_1 += targetSlot;             
    }              
    else
    { 
      /* if present value is greater than target value, this condition executes */
      targetRange = Light_TemporaryStatus.PresentParam_1 - Light_TemporaryStatus.TargetParam_1;;
      targetSlot = targetRange/Light_TimeParam.StepValue;          
      Light_TemporaryStatus.PresentParam_1 -= targetSlot;
    } 
    
    Light_TimeParam.StepValue--;         
    /* updating the remaining time after each step covered*/
    Light_TemporaryStatus.RemainingTime =  Light_TimeParam.StepValue  | (Light_TimeParam.ResBitValue << 6) ;
    LightUpdateFlag = VALUE_UPDATE_SET;
    Check_time = 0;
    Clockflag = 0;
    
    /* when transition is completed, disable the transition by disabling 
    transition flag
    */
    if(Light_TimeParam.StepValue == 0)
    {              
      Light_ModelFlag.LightTransitionFlag = LIGHT_TRANSITION_STOP;
      Light_ModelFlag.Light_Trnsn_Cmplt = 1;
     
    }
    
 TRACE_M(TF_LIGHT_M,"Inside light lightness transition time at %ld, Current state 0x%.2x ,target state 0x%.2x , Remaining Time 0x%.2x\n\r",
            Clock_Time(),Light_TemporaryStatus.PresentParam_1,Light_TemporaryStatus.TargetParam_1,\
                                         Light_TemporaryStatus.RemainingTime);
    
    
  }
  
  return MOBLE_RESULT_SUCCESS;         
} 


/**
* @brief Light_TransitionBehaviourMulti_Param funtion is used for the Light Lightness model
*         when transition time is  received in message.This function is used for
*         the multiple parameters in transition.
* @param GetValue: Pointer of the array
* @param 
* @retval MOBLE_RESULT
*/
MOBLE_RESULT Light_TransitionBehaviourMulti_Param(MOBLEUINT8 *GetValue , MOBLEUINT8 param_Count)
{
  
  static MOBLEUINT8 Clockflag = 0;
  static MOBLEUINT32 Check_time;
  MOBLEUINT16 targetRangeLightness;
  MOBLEUINT16 targetRangeTemperature;
  MOBLEUINT16 targetSlotParam_1;
  MOBLEUINT16 targetSlotParam_2;
  MOBLEUINT16 targetSlotParam_3;
  
  if(Clockflag == 0)
  {
    Check_time = Clock_Time();
    Clockflag = 1;
  }
  /* Values from application are copied into Temporary vaiables for processing */
  Light_TemporaryStatus.PresentParam_1 = GetValue[1] << 8;
  Light_TemporaryStatus.PresentParam_1 |= GetValue[0];
  Light_TemporaryStatus.PresentParam_2 = GetValue[3] << 8;
  Light_TemporaryStatus.PresentParam_2 |= GetValue[2];
  Light_TemporaryStatus.PresentParam_3 = GetValue[5] << 8;
  Light_TemporaryStatus.PresentParam_3 |= GetValue[4];
  /*if condition to wait untill the time is equal to the given resolution time */   
  if(((Clock_Time()- Check_time) >= Light_TimeParam.Res_Value))
  {
    if(Light_TimeParam.StepValue == 0)
    {
      Light_TimeParam.StepValue = 1;
    }
    
    if(Light_TemporaryStatus.TargetParam_1 > Light_TemporaryStatus.PresentParam_1)
    {
      /* calulating the target range to be acheived which is traget value
      which is traget value minus present value if target value is greater 
      than present value.   
      */
      targetRangeLightness =  Light_TemporaryStatus.TargetParam_1 - Light_TemporaryStatus.PresentParam_1;            
      /*target slot = time to cover in single step */
      targetSlotParam_1 = targetRangeLightness/Light_TimeParam.StepValue;
      /* target slot added to present value to achieve target value */
      Light_TemporaryStatus.PresentParam_1 += targetSlotParam_1;             
    }              
    else
    { 
      /* condition execute when transition is negative */
      /* target range = total range to be covered */ 
      targetRangeLightness = Light_TemporaryStatus.PresentParam_1 - Light_TemporaryStatus.TargetParam_1; 
      /*target slot = time to cover in single step */
      targetSlotParam_1 = targetRangeLightness/Light_TimeParam.StepValue;
      /*target slot = time to cover in single step */
      Light_TemporaryStatus.PresentParam_1 -= targetSlotParam_1;
    } 
    
    if(Light_TemporaryStatus.TargetParam_2 > Light_TemporaryStatus.PresentParam_2 )
    {
      targetRangeTemperature = Light_TemporaryStatus.TargetParam_2 - Light_TemporaryStatus.PresentParam_2;
      targetSlotParam_2 = targetRangeTemperature/Light_TimeParam.StepValue; 
      Light_TemporaryStatus.PresentParam_2 += targetSlotParam_2;
    }
    else
    {
      targetRangeTemperature = Light_TemporaryStatus.PresentParam_2 - Light_TemporaryStatus.TargetParam_2;
      targetSlotParam_2 = targetRangeTemperature/Light_TimeParam.StepValue; 
      Light_TemporaryStatus.PresentParam_2 -= targetSlotParam_2;
    }
    
    if(param_Count == 3)
    {
      if(Light_TemporaryStatus.TargetParam_3 > Light_TemporaryStatus.PresentParam_3 )
      {
        targetRangeTemperature = Light_TemporaryStatus.TargetParam_3 - Light_TemporaryStatus.PresentParam_3;
        targetSlotParam_3 = targetRangeTemperature/Light_TimeParam.StepValue; 
        Light_TemporaryStatus.PresentParam_3 += targetSlotParam_3;
      }
      else
      {
        targetRangeTemperature = Light_TemporaryStatus.PresentParam_3 - Light_TemporaryStatus.TargetParam_3;
        targetSlotParam_3 = targetRangeTemperature/Light_TimeParam.StepValue; 
        Light_TemporaryStatus.PresentParam_3 -= targetSlotParam_3;
      }
    }
    
    Light_TimeParam.StepValue--;                           
    /* updating the remaining time after each step covered*/
    Light_TemporaryStatus.RemainingTime = Light_TimeParam.StepValue | (Light_TimeParam.ResBitValue << 6) ;
    LightUpdateFlag = VALUE_UPDATE_SET;
    Check_time = 0;
    Clockflag = 0;
    /* when transition is completed, disable the transition by disabling 
    transition flag
    */
    
    if(Light_TimeParam.StepValue <= 0)
    {
      Light_ModelFlag.LightTransitionFlag = LIGHT_TRANSITION_STOP;  
      Light_ModelFlag.Light_Trnsn_Cmplt = 1;      
    }    
    TRACE_M(TF_LIGHT_M,"Inside light transition %ld,PV1 0x%.2x ,TV1 0x%.2x ,PV2 0x%.2x , TV2 0x%.2x,PV3 0x%.2x ,TV3 0x%.2x, RT 0x%.2x \n\r",
            Clock_Time(),Light_TemporaryStatus.PresentParam_1,Light_TemporaryStatus.TargetParam_1, 
                Light_TemporaryStatus.PresentParam_2,Light_TemporaryStatus.TargetParam_2,
                 Light_TemporaryStatus.PresentParam_3, Light_TemporaryStatus.TargetParam_3,
                                                        Light_TemporaryStatus.RemainingTime);
    
    
  }
  return MOBLE_RESULT_SUCCESS;         
} 

/**
* @brief  Function to publish the status of the node to the publish address.
*         The Publishing of all the model states will depend on the model message received 
          and supporting binding with other models.
* @param  void
* @param  void
*/ 
void Model_BindingPublishStatus(void)
{
  MOBLE_ADDRESS publishAddress;
  MOBLEUINT8 elementNumber;
  MOBLEUINT16 opcode;
  MOBLEUINT8 const pRxData[12] = {0};
  MOBLEUINT32 dataLength = 0;
  MOBLEUINT8 count = 0;
  MOBLE_ADDRESS my_Address;

   if(Light_PublishOpcodeList.BindedStateCount > 0)
   {
      for(count = 0; count < Light_PublishOpcodeList.BindedStateCount; count++) 
      {
        my_Address = BLEMesh_GetAddress();
        elementNumber = BLE_GetElementNumber();       
        publishAddress = BLEMesh_GetPublishAddress(elementNumber, Light_PublishOpcodeList.Model_ID[count]);
        opcode = (MOBLEUINT16)Light_PublishOpcodeList.PublishStateOpcode[count];
        
#if 0
        if((publishAddress != 0x0000 ) && (ADDRESS_IS_UNICAST(Dst_Peer)))
#else
        if((publishAddress != 0x0000 ) && 
           ((ADDRESS_IS_UNICAST(Dst_Peer)) || (ADDRESS_IS_GROUP(Dst_Peer))))
#endif
        {
          Model_SendResponse(publishAddress,my_Address,opcode,pRxData,dataLength);
          TRACE_I(TF_LIGHT_M,"Binded publishing address %.2x opcode %.2x model id %.2x \r\n",
                  publishAddress,Light_PublishOpcodeList.PublishStateOpcode[count],Light_PublishOpcodeList.Model_ID[count]);
        }       
      }
//      Light_PublishOpcodeList.BindedStateCount = 0;
      Light_Publish_Reset();
   }
   
}

/**
* @brief  Light_GetStepValue:Function calculates parameters for transition time 
*         for light model.
* @param stepParam is transition time set value.
* retval void.
*/
void Light_GetStepValue(MOBLEUINT8 stepParam)
{
  /*
  Two MSB bit of transition time is dedicated to resolution.
  00 = resolution is 100 ms.
  01 = resolution is 1000 ms. 
  10 = resolution is 10000 ms.
  11 = resolution is 600000 ms. 
  Last bits from 0 to 5th index is step number.
  */
  
  Light_TimeParam.ResBitValue = stepParam >> 6 ;
  Light_TimeParam.Res_Value = Get_StepResolutionValue(Light_TimeParam.ResBitValue);
  Light_TimeParam.StepValue = stepParam & 0x3F ;
  
  if(Light_TimeParam.Res_Value >= 100)
  {
    Light_TimeParam.Res_Value = Light_TimeParam.Res_Value/TRANSITION_SCALER;
    Light_TimeParam.StepValue = (Light_TimeParam.StepValue * TRANSITION_SCALER);
  }
  
  TRACE_M(TF_LIGHT_M," step resolution 0x%.2lx, number of step 0x%.2x \r\n",Light_TimeParam.Res_Value ,
          Light_TimeParam.StepValue  );   
}


/**
* @brief  Function to execute the transition state machine for particular Light Model
* @param  void
* @retval void
*/ 
void Lighting_Process(void)
{ 
  MOBLE_ADDRESS publishAddress;
  MOBLE_ADDRESS my_Address;
  MOBLEUINT8 elementNumber;
  MOBLEUINT8 const pRxData[12] = {0};
  MOBLEUINT32 dataLength = 0;
#if defined ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS || defined ENABLE_LIGHT_MODEL_SERVER_CTL \
  || defined ENABLE_LIGHT_MODEL_SERVER_HSL
    
    MOBLEUINT8 Light_GetBuff[8];
#endif
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS
  if(Light_ModelFlag.LightTransitionFlag == LIGHT_LIGHTNESS_TRANSITION_START)
  {  
    /* Fetching the state value from the applcation layer to middle layer */
    (Appli_Light_GetStatus_cb.GetLightLightness_cb)(Light_GetBuff);     
    /* Transition function called to change the state in transition */
    Light_TransitionBehaviourSingle_Param(Light_GetBuff);
    if(LightUpdateFlag == VALUE_UPDATE_SET)
    {
      /* updating the state value after the each transition step complete */
      LightLightnessStateUpdate_Process();
      /*Application Callback */
      (LightAppli_cb.Lightness_Set_cb)(&Light_LightnessStatus,0);  
      /* Binding of actual light lightness with other models */
      Light_Lightness_Binding(BINDING_LIGHT_LIGHTNESS_ACTUAL_SET , 0);
      LightUpdateFlag = VALUE_UPDATE_RESET;
    }
  }  
  if(Light_ModelFlag.LightTransitionFlag == LIGHT_LINEAR_TRANSITION_START)
  {
    /* Fetching the state value from the applcation layer to middle layer */
    (Appli_Light_GetStatus_cb.GetLightLightnessLinear_cb)(Light_GetBuff); 
    /* Transition function called to change the state in transition */
    Light_TransitionBehaviourSingle_Param(Light_GetBuff);
    if(LightUpdateFlag == VALUE_UPDATE_SET)
    {
      /* updating the state value after the each transition step complete */
      LightLinearStateUpdate_Process();
      /*Application Callback */
      (LightAppli_cb.Lightness_Linear_Set_cb)(&Light_LightnessStatus,0);
      /* Binding of data of light lightness with light linear */
      Light_Lightness_Binding(BINDING_LIGHT_LIGHTNESS_LINEAR_SET , 0);
      LightUpdateFlag = VALUE_UPDATE_RESET;
    }
  }
#endif
#ifdef ENABLE_LIGHT_MODEL_SERVER_CTL   
  if(Light_ModelFlag.LightTransitionFlag == LIGHT_CTL_TRANSITION_START)
  {
    (Appli_Light_GetStatus_cb.GetLightCtl_cb)(Light_GetBuff);
    Light_TransitionBehaviourMulti_Param(Light_GetBuff , 0);
    if(LightUpdateFlag == VALUE_UPDATE_SET)
    {
      LightCtlStateUpdate_Process();
      (LightAppli_cb.Light_Ctl_Set_cb)(&Light_CtlStatus , 0);
      /* binding of light ctl with actual lightness */
      Light_Ctl_LightActual_Binding(BINDING_LIGHT_CTL_SET);
      LightUpdateFlag = VALUE_UPDATE_RESET;
    }
  }
  
  if(Light_ModelFlag.LightTransitionFlag == LIGHT_TEMPERATURE_TRANSITION_START)
  {
    (Appli_Light_GetStatus_cb.GetLightCtlTemp_cb)(Light_GetBuff);
    Light_TransitionBehaviourMulti_Param(Light_GetBuff , 0);
    if(LightUpdateFlag == VALUE_UPDATE_SET)
    {
      LightCtlTemperatureStateUpdate_Process();
      (LightAppli_cb.Light_CtlTemperature_Set_cb)(&Light_CtlStatus , 0);
      /* Binding of data Ctl Temperature  with Generic Level */
      Light_CtlTemperature_Binding();
      LightUpdateFlag = VALUE_UPDATE_RESET;
    }
  }
#endif
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL   
  if(Light_ModelFlag.LightTransitionFlag == LIGHT_HSL_TRANSITION_START)
  {
    (Appli_Light_GetStatus_cb.GetLightHsl_cb)(Light_GetBuff);
    Light_TransitionBehaviourMulti_Param(Light_GetBuff , 3);
    if(LightUpdateFlag == VALUE_UPDATE_SET)
    {
      LightHslStateUpdate_Process();
      (LightAppli_cb.Light_Hsl_Set_cb)(&Light_HslStatus , 0);
      /* Light Hsl Lightness binding with lightness Actual */
      Light_HslLightness_LightnessActualBinding();
      LightUpdateFlag = VALUE_UPDATE_RESET;
    }
  }
  
  if(Light_ModelFlag.LightTransitionFlag == LIGHT_HSL_HUE_TRANSITION_START)
  {
    (Appli_Light_GetStatus_cb.GetLightHslHue_cb)(Light_GetBuff);
    Light_TransitionBehaviourSingle_Param(Light_GetBuff);
    if(LightUpdateFlag == VALUE_UPDATE_SET)
    {
      LightHslHueStateUpdate_Process();
      (LightAppli_cb.Light_HslHue_Set_cb)(&Light_HslStatus , 0);
      /* Light Hsl Hue  binding with Generic level */
      Light_Hsl_Hue_Binding();
      LightUpdateFlag = VALUE_UPDATE_RESET;
    }
  }
  
  if(Light_ModelFlag.LightTransitionFlag == LIGHT_HSL_SATURATION_TRANSITION_START)
  {
    (Appli_Light_GetStatus_cb.GetLightHslSaturation_cb)(Light_GetBuff);
    Light_TransitionBehaviourSingle_Param(Light_GetBuff);
    if(LightUpdateFlag == VALUE_UPDATE_SET)
    {
      LightHslSaturationStateUpdate_Process();
      (LightAppli_cb.Light_HslSaturation_Set_cb)(&Light_HslStatus , 0);
      /* Light Hsl Saturation binding with Generic level model*/
      Light_Hsl_Saturation_Binding();
      LightUpdateFlag = VALUE_UPDATE_RESET;
    }
  }
#endif  
  
  if(Light_ModelFlag.Light_Trnsn_Cmplt == MOBLE_TRUE)
  {         
    elementNumber = BLE_GetElementNumber();
    my_Address = BLEMesh_GetAddress();
    publishAddress = BLEMesh_GetPublishAddress(elementNumber,Model_ID);
    if(publishAddress != 0x00)
    {
      Model_SendResponse(publishAddress,my_Address,Model_Rx_Opcode,pRxData,dataLength);
    }
    Light_ModelFlag.Light_Trnsn_Cmplt = MOBLE_FALSE;
  }
#ifdef  ENABLE_MODEL_BINDING 
  Model_BindingPublishStatus();
#endif  
}


/**
* @brief LightLightnessStateUpdate_Process:Function to update the parametes of light
*         lightness model in application file from Temporary parameter in model file.
* @param void
* return MOBLE_RESULT.
*/
MOBLE_RESULT LightLightnessStateUpdate_Process(void)
{
  Light_LightnessStatus.LightnessPresentValue16 = Light_TemporaryStatus.PresentParam_1;
  Light_LightnessStatus.TargetValue16 = Light_TemporaryStatus.TargetParam_1;
  Light_LightnessStatus.RemainingTime = Light_TemporaryStatus.RemainingTime ;
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief Light_LC_LightnessStateUpdate_Process:Function to update the parametes of light
*        lightness model in application file from Temporary parameter in model file.
* @param void
* return MOBLE_RESULT.
*/
MOBLE_RESULT Light_LC_LightnessStateUpdate_Process(Light_LC_TemporaryStatus_t *lc_Temp_Value)
{
  Light_LightnessStatus.LightnessPresentValue16 = lc_Temp_Value->PresentParam_1;
  Light_LightnessStatus.TargetValue16 = lc_Temp_Value->TargetParam_1;
  Light_LightnessStatus.RemainingTime = lc_Temp_Value->RemainingTime;
  
  (LightAppli_cb.Lightness_Set_cb)(&Light_LightnessStatus,0); 

  return MOBLE_RESULT_SUCCESS;
}
/**
* @brief LightLinearStateUpdate_Process:function to update the parametes of light 
*        lightness linear model in application file from Temporary parameter in model file.
* @param void
* return MOBLE_RESULT.
*/
MOBLE_RESULT LightLinearStateUpdate_Process(void)
{
  Light_LightnessStatus.LinearPresentValue16 = Light_TemporaryStatus.PresentParam_1;
  Light_LightnessStatus.TargetValue16 =  Light_TemporaryStatus.TargetParam_1;
  Light_LightnessStatus.RemainingTime = Light_TemporaryStatus.RemainingTime ;
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief LightCtlStateUpdate_Process:Function to update the parametes of light  
*        CTL model in application file from Temporary parameter in model file.
* @param void
* return MOBLE_RESULT.
*/
MOBLE_RESULT LightCtlStateUpdate_Process(void)
{
  Light_CtlStatus.PresentCtlLightness16 =Light_TemporaryStatus.PresentParam_1; 
  Light_CtlStatus.PresentCtlTemperature16 = Light_TemporaryStatus.PresentParam_2;
  Light_CtlStatus.PresentCtlDelta16 = Light_TemporaryStatus.PresentParam_3;
  Light_CtlStatus.RemainingTime = Light_TemporaryStatus.RemainingTime;
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief LightCtlTemperatureStateUpdate_Process:Function to update the parametes of 
*        light CTL Temperature model in application  file from Temporary parameter in model file.
* @param void
* return MOBLE_RESULT.
*/
MOBLE_RESULT LightCtlTemperatureStateUpdate_Process(void)
{
  Light_CtlStatus.PresentCtlTemperature16 = Light_TemporaryStatus.PresentParam_1; 
  Light_CtlStatus.PresentCtlDelta16 = Light_TemporaryStatus.PresentParam_2;
  Light_CtlStatus.RemainingTime = Light_TemporaryStatus.RemainingTime;
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief LightHslStateUpdate_Process:Function to update the parametes of 
*        light HSL model in application  file from Temporary parameter in model file.
* @param void
* return MOBLE_RESULT.
*/
MOBLE_RESULT LightHslStateUpdate_Process(void)
{
  Light_HslStatus.PresentHslLightness16= Light_TemporaryStatus.PresentParam_1; 
  Light_HslStatus.PresentHslHueLightness16 = Light_TemporaryStatus.PresentParam_2;
  Light_HslStatus.PresentHslSaturation16 = Light_TemporaryStatus.PresentParam_3; 
  Light_HslStatus.RemainingTime = Light_TemporaryStatus.RemainingTime;
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief LightHslHueStateUpdate_Process:Function to update the parametes of 
*        light HSL Hue model in application  file from Temporary parameter in model file.
* @param void
* return MOBLE_RESULT.
*/
MOBLE_RESULT LightHslHueStateUpdate_Process(void)
{
  Light_HslStatus.PresentHslHueLightness16= Light_TemporaryStatus.PresentParam_1; 
  Light_HslStatus.RemainingTime = Light_TemporaryStatus.RemainingTime;
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief LightHslSaturationStateUpdate_Process:Function to update the parametes of 
*        light HSL Saturation model in application  file from Temporary parameter in model file.
* @param void
* return MOBLE_RESULT.
*/
MOBLE_RESULT LightHslSaturationStateUpdate_Process(void)
{
  Light_HslStatus.PresentHslSaturation16 = Light_TemporaryStatus.PresentParam_1; 
  Light_HslStatus.RemainingTime = Light_TemporaryStatus.RemainingTime;
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief Light_Ctl_LightActual_Binding:Function for binding the data of actual 
*        lightness and light Ctl set.
* @param bindingFlag: value pass to function from light lightness
* return void
*/
void Light_Ctl_LightActual_Binding(MOBLEUINT8 bindingFlag)
{ 
  if(bindingFlag == BINDING_LIGHT_CTL_SET)
  {
    Light_LightnessStatus.LightnessPresentValue16 = Light_CtlParam.CTL_Lightness;
    Light_Publish_Add(LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID, 
                      LIGHT_LIGHTNESS_SET_UNACK);
//    Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_LIGHTNESS_SET_UNACK;
//    Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID;
//    Light_PublishOpcodeList.BindedStateCount++;
    
    TRACE_M(TF_LIGHT_M, "Light_Ctl_LightActual_Binding Function - Present Light lightness value 0x%.2x \r\n" 
         , Light_LightnessStatus.LightnessPresentValue16);
  }
  if(bindingFlag == BINDING_LIGHT_LIGHTNESS_ACTUAL_SET)
  {
    Light_CtlStatus.PresentCtlLightness16 = Light_LightnessParam.TargetLightnessStatus;   
    Light_Publish_Add(LIGHT_MODEL_SERVER_CTL_MODEL_ID, 
                      LIGHT_CTL_SET_UNACK);
//    Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_CTL_SET_UNACK;
//    Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_CTL_MODEL_ID;
//    Light_PublishOpcodeList.BindedStateCount++;    
    
    TRACE_M(TF_LIGHT_M, "Light Lightness set Function - Present CTL lightness value 0x%.2x \r\n" 
         , Light_CtlStatus.PresentCtlLightness16);
  }
  
  //Light_Lightness_Binding(bindingFlag , 0);/*Commented by SJ as it was creating the infinity loop*/
  
   
}

/**
* @brief Light_Lightness_Binding:Function for binding the data of actual 
lightness and Linear Lightness is implicit binding with  generic on off state.              
* @param bindingFlag: flag value passed in function 
* @param length:length of data
* return void
*/
void Light_Lightness_Linear_Binding(MOBLEUINT8 bindingFlag ,MOBLEUINT32 length)
{
  if(bindingFlag == BINDING_LIGHT_LIGHTNESS_LINEAR_SET)
  {
    Light_Linear_ActualBinding(length);
    Light_Publish_Add(LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID, 
                      LIGHT_LIGHTNESS_SET);
//    Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_LIGHTNESS_SET;
//    Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID;    
//    Light_PublishOpcodeList.BindedStateCount++;
  }
  Light_Lightness_Binding(bindingFlag , 0);
}

/**
* @brief Light_Lightness_Binding:Function for binding the data of actual 
lightness and Linear Lightness is implicit binding with  generic on off state.              
* @param bindingFlag: flag value passed in function 
* @param length:length of data
* return void
*/
void Light_Lightness_Binding(MOBLEUINT8 bindingFlag ,MOBLEUINT32 length)
{ 
  /*
  6.1.2.1.1 - actual lightness = 655354 * squareroot(linear lightness/ 655354).
  */
 
  if(bindingFlag != BINDING_LIGHT_LIGHTNESS_LINEAR_SET)
  {
    Light_Actual_LinearBinding(); 
    Light_Publish_Add(LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID, 
                      LIGHT_LIGHTNESS_SET);
//    Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_LIGHTNESS_SET;
//    Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID;    
//    Light_PublishOpcodeList.BindedStateCount++;
  }
    /* Implicit binding of linear lightness with generic on off.
    linear lightnes set -> actual lightness set -> generic on off set.
    As linear lightness changes, it changes the value of actual lightness
    which in turn changes the state of generic on off state.
    */
#ifdef ENABLE_GENERIC_MODEL_SERVER_ONOFF  
    /* Binding of data b/w light lightness actual and Generic on off */
  if(bindingFlag != BINDING_GENERIC_ON_OFF_SET)
  {
    LightActual_GenericOnOffBinding(&Light_LightnessStatus);
    Light_Publish_Add(GENERIC_MODEL_SERVER_ONOFF_MODEL_ID, 
                      GENERIC_ON_OFF_SET_UNACK);
//    Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= GENERIC_ON_OFF_SET_UNACK;
//    Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = GENERIC_MODEL_SERVER_ONOFF_MODEL_ID;
//    Light_PublishOpcodeList.BindedStateCount++;
  }
#endif 
  
    /* Implicit binding of linear lightness with generic level.
    linear lightnes set -> actual lightness set -> generic Level set.
    As linear lightness changes, it changes the value of actual lightness
    which in turn changes the state of generic level state.
    */
#ifdef ENABLE_GENERIC_MODEL_SERVER_LEVEL  
  /* Binding of data b/w light lightness actual and Generic Level */
  if(bindingFlag != BINDING_GENERIC_LEVEL_SET)
  {
    LightActual_GenericLevelBinding(&Light_LightnessStatus);
    Light_Publish_Add(GENERIC_MODEL_SERVER_LEVEL_MODEL_ID, 
                      GENERIC_LEVEL_SET_UNACK);
//    Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= GENERIC_LEVEL_SET_UNACK;
//    Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = GENERIC_MODEL_SERVER_LEVEL_MODEL_ID;
//    Light_PublishOpcodeList.BindedStateCount++;
  }
#endif   
     
#ifdef ENABLE_LIGHT_MODEL_SERVER_CTL  
  /* Binding of Light Ctl with Actual Light Lightness */
  if(bindingFlag != BINDING_LIGHT_CTL_SET)
  {
    Light_Ctl_LightActual_Binding(BINDING_LIGHT_LIGHTNESS_ACTUAL_SET);
    Light_Publish_Add(LIGHT_MODEL_SERVER_CTL_MODEL_ID, 
                      LIGHT_CTL_SET_UNACK);
//    Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_CTL_SET_UNACK;
//    Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_CTL_MODEL_ID;
//    Light_PublishOpcodeList.BindedStateCount++;
  }
#endif  
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL  
  /* Binding of data b/W Actual Lightness and Hsl Lightness.*/
  if(bindingFlag != BINDING_LIGHT_HSL_SET)
  {
    Light_ActualLightness_HslLightnessBinding(&Light_LightnessStatus);
    Light_Publish_Add(LIGHT_MODEL_SERVER_HSL_MODEL_ID, 
                      LIGHT_HSL_SET_UNACK);
//    Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_HSL_SET_UNACK;
//    Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_HSL_MODEL_ID;
//    Light_PublishOpcodeList.BindedStateCount++;
  }
#endif     
   
  TRACE_M(TF_LIGHT_M,"Light_Lightness_Binding - Actual Lightness value 0x%.2x , Linear Lightness value 0x%.2x \r\n " , 
                                                  Light_LightnessStatus.LightnessPresentValue16,
                                                  Light_LightnessStatus.LinearPresentValue16  ); 
 
}

/**
* @brief  GenericOnOff_LightActualBinding: Data binding b/w Generic On Off and 
*         light lightness Actual. this function will set the actual light lightness
*         value at the time of  generic on off set. 
* @param onOff_param: Pointer to the data which needs to be checked.
* return void.
*/
void GenericOnOff_LightActualBinding(Generic_OnOffStatus_t* onOff_param)
{
  /*
  6.1.2.2.3 - Binding of actual light lightness with Generic on off.
  As generic on off state changes, the actual lightness value will
  change.
  */
  /* condition is depends on the generic on off state */  
  if(onOff_param->Present_OnOff_State == 0x00)
  {
    Light_LightnessStatus.LightnessPresentValue16 = 0x00;    
  }
  else if((onOff_param->Present_OnOff_State == 0x01) && 
          (Light_LightnessDefaultParam.LightnessDefaultStatus == 0x00))
  {
    Light_LightnessStatus.LightnessPresentValue16 = Light_LightnessStatus.LightnessLastStatus;
  }
  else if((onOff_param->Present_OnOff_State == 0x01) && 
          (Light_LightnessDefaultParam.LightnessDefaultStatus != 0x000))
  {
    Light_LightnessStatus.LightnessPresentValue16 = Light_LightnessDefaultParam.LightnessDefaultStatus;
  }
  else
  {
    /* no condition to Execute */
  }
  Light_Publish_Add(LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID, 
                    LIGHT_LIGHTNESS_SET_UNACK);
//  Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_LIGHTNESS_SET_UNACK;
//  Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID;
//  Light_PublishOpcodeList.BindedStateCount++;
  
  /*As Generic on off changes, it will chnage the value of the light linear due to change in the light lightness 
    value due to binding of the states.
 */
  
  Light_Lightness_Binding(BINDING_GENERIC_ON_OFF_SET,0);

 
}

/*
* @Brief  GenericLevel_LightBinding: Data binding b/w Generic Level and 
*         light lightness Actual. this function will set the actual light lightness
*         value at the time of  generic Level set. 
* @param gLevel_param: Pointer to the data which needs to be checked.
* return void.
*/
void GenericLevel_LightBinding(Generic_LevelStatus_t* gLevel_param , MOBLEUINT8 flag)
{ 
  /*
  6.1.2.2.2 - Binding of actual light lightness with generic level
  As generic Level changes, the actual lightness value will
  change.
  */
#ifdef ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS    
  Light_PublishOpcodeList.BindedStateCount = 0;
  Light_LightnessStatus.LightnessPresentValue16 = gLevel_param->Present_Level16 + 32768;   
  
  Light_Publish_Add(LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID, 
                    LIGHT_LIGHTNESS_SET_UNACK);
//  Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_LIGHTNESS_SET_UNACK;
//  Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID;
//  Light_PublishOpcodeList.BindedStateCount++;
  
  /*As Generic Level changes, it will change the value of the light linear due to change in the light lightness 
    value due to binding of the states.
    */
  Light_Actual_LinearBinding();
  
  Light_Publish_Add(LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID, 
                    LIGHT_LIGHTNESS_LINEAR_SET_UNACK);
//  Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_LIGHTNESS_LINEAR_SET_UNACK;
//  Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID;
//  Light_PublishOpcodeList.BindedStateCount++;
  
#endif  
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_CTL_TEMPERATURE
  if(flag != BINDING_LIGHT_CTL_TEMP_SET)
  {
    GenericLevel_CtlTempBinding(gLevel_param);
    Light_Publish_Add(LIGHT_MODEL_SERVER_CTL_TEMPERATURE_MODEL_ID, 
                      LIGHT_CTL_TEMPERATURE_SET_UNACK);
//    Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_CTL_TEMPERATURE_SET_UNACK;
//    Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_CTL_TEMPERATURE_MODEL_ID;
//    Light_PublishOpcodeList.BindedStateCount++;
  }
#endif  
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL_HUE  
  if(flag != BINDIG_LIGHT_HSL_HUE_SET)
  {
    GenericLevel_HslHueBinding(gLevel_param);
    Light_Publish_Add(LIGHT_MODEL_SERVER_HSL_HUE_MODEL_ID, 
                      LIGHT_HSL_HUE_SET_UNACK);
//    Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_HSL_HUE_SET_UNACK;
//    Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_HSL_HUE_MODEL_ID;
//    Light_PublishOpcodeList.BindedStateCount++;
  }
#endif  
  
#ifdef ENABLE_LIGHT_MODEL_SERVER_HSL_SATURATION  
  if(flag != BINDIG_LIGHT_HSL_SATURATION_SET)
  {
    GenericLevel_HslSaturationBinding(gLevel_param);
    Light_Publish_Add(LIGHT_MODEL_SERVER_HSL_SATURATION_MODEL_ID, 
                      LIGHT_HSL_SATURATION_SET_UNACK);
//    Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_HSL_SATURATION_SET_UNACK;
//    Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_HSL_SATURATION_MODEL_ID;
//    Light_PublishOpcodeList.BindedStateCount++;
  }
#endif    
  
}

/**
* @brief Light_Actual_LinearBinding:Light_Actual_LinearBinding:Function used for 
*        binding the data of actual lightness and lineaer lightness.this function
*        changes the value of linear lightness as actual lightness value is set.
* @param void 
* return MOBLEUINT16
*/
MOBLEUINT16 Light_Actual_LinearBinding(void)
{
  
  float productValue;
  
  productValue = pow((((float)Light_LightnessStatus.LightnessPresentValue16)/65535) , 2);
  Light_LightnessStatus.LinearPresentValue16 = (MOBLEUINT16)(65535 * productValue);
 
  return Light_LightnessStatus.LinearPresentValue16; 
}


/**
* @brief Light_Linear_ActualBinding:Function used for binding the data of actual 
*        lightness and linear lightness. this function changes the value of Actual 
*        lightness as Linear lightness value is set.
* @param length: length of the data 
* @param binding_Flag: flag for the binding
* return MOBLEUINT16
*/
MOBLEUINT16 Light_Linear_ActualBinding(MOBLEUINT32 length)
{
  /*
  6.1.2.1.1 - actual lightness = 655354 * squareroot(linear lightness/ 655354).
  */
  float productValue;
  if(length <= 3)
  {
    productValue = sqrt((((float)Light_LightnessStatus.LinearPresentValue16) / 65535));
    Light_LightnessStatus.LightnessPresentValue16 = (MOBLEUINT16)(65535 * productValue);      
    return Light_LightnessStatus.LightnessPresentValue16;  
    
}
  else
  {
    productValue = sqrt((((float)Light_LightnessStatus.TargetValue16) / 65535));
    Light_LightnessStatus.TargetValue16 = (MOBLEUINT16)(65535 * productValue);
    Light_TemporaryStatus.TargetParam_1 = Light_LightnessStatus.TargetValue16; 
    return Light_LightnessStatus.TargetValue16;  
  }
}


/**
* @brief Light_Actual_RangeBinding:Function used for binding the data of actual 
*        lightness and lightness range this function set the value of Actual 
*        lightness according to the min range  and max range value.
* @param lightActual: Pointer which needs to be set. 
* return void
*/
void Light_Actual_RangeBinding(Light_LightnessParam_t* lightActual)
{
  /*
  6.1.2.2.5 - actual lightness = range minimum if actual lighteness is less than range min
  value.
  actual lightness = range maximum if actual lighteness is greater than range max
  value.
  */
  if((lightActual->TargetLightnessStatus < Light_LightnessRangeParam.MinRangeStatus)
     && (lightActual->TargetLightnessStatus != 0))
  {
    lightActual->TargetLightnessStatus = Light_LightnessRangeParam.MinRangeStatus;
  }
  else if((lightActual->TargetLightnessStatus > Light_LightnessRangeParam.MaxRangeStatus)
          && (lightActual->TargetLightnessStatus != 0))
  {
    lightActual->TargetLightnessStatus = Light_LightnessRangeParam.MaxRangeStatus;
  }
  else
  {
    /* no condition to execute */
  }
}

/**
* @brief Light_CtlTemperature_Binding: Function used for binding the data of 
*        Ctl temperature with other models.
* @param void 
* return void
*/
void Light_CtlTemperature_Binding(void)
{
#ifdef ENABLE_GENERIC_MODEL_SERVER_LEVEL  
  Light_CtlTemp_GenericLevelBinding(&Light_CtlStatus);
  
  Light_Publish_Add(GENERIC_MODEL_SERVER_LEVEL_MODEL_ID, 
                    GENERIC_LEVEL_SET_UNACK);
//  Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= GENERIC_LEVEL_SET_UNACK;
//  Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = GENERIC_MODEL_SERVER_LEVEL_MODEL_ID;
//  Light_PublishOpcodeList.BindedStateCount++;
#endif  
  
}

/**
* @brief Light_CtlTemperature_TempRangeBinding: Function used for binding the data of 
*        Ctl temperature and Ctl temperature range this function changes the value of 
*        Ctl temperature according to the min range  and max range value.
* @param ctlTempemerature: Pointer which needs to be set.  
* return void
*/
void Light_CtlTemperature_TempRangeBinding(Light_CtlParam_t* ctlTemperature)
{
  /*
  6.1.2.2.5 - actual lightness = range minimum if actual lighteness is less than range min
  value.
  actual lightness = range maximum if actual lighteness is greater than range max
  value.
  */
  
  if(ctlTemperature->CTL_Temperature < CtlTemperatureRangeParam.MinRangeStatus)        
  {
    ctlTemperature->CTL_Temperature = CtlTemperatureRangeParam.MinRangeStatus;
  }
  else if(ctlTemperature->CTL_Temperature > CtlTemperatureRangeParam.MaxRangeStatus)           
  {
    ctlTemperature->CTL_Temperature = CtlTemperatureRangeParam.MaxRangeStatus;
  }
  else
  {
    /* no condition to execute */
  }
}

/**
* @brief GenericLevel_CtlTempBinding: Data binding b/w Generic level and Ctl
*        Temperature set.
* @param bLevelParam: pointer to the structure, which should be set.
* return void.
*/
void GenericLevel_CtlTempBinding(Generic_LevelStatus_t * bLevelParam)
{
  /* 6.1.3.1.1 Binding with the Generic Level state.
  Light CTL Temperature = T_MIN + (Generic Level + 32768) * (T_MAX - T_MIN) / 65535
  T_MIN = minimum range of ctl temperature
  T_MAX = maximum range of ctl temperature
  */
  MOBLEUINT32 productValue;
  
  productValue = (bLevelParam->Present_Level16 + 32768) * (MAX_CTL_TEMP_RANGE - MIN_CTL_TEMP_RANGE) ;
  Light_CtlStatus.PresentCtlTemperature16 = MIN_CTL_TEMP_RANGE + ((productValue) /65535);
  
}

/**
* @brief  Light_HslLightness_LightnessActualBinding: Data binding b/w Hsl lightness and 
*         Lightness Atual . this function will set the Lightness Actual
*         value at the time of  Hsl Lightness value set. 
* @param void
* return void.
*/
void Light_HslLightness_LightnessActualBinding(void)
{
  /* 6.1.4.7.1 Binding with the Light Lightness Actual state
  Light Lightness Actual = Light HSL Lightness
  */
  Light_LightnessStatus.LightnessPresentValue16= Light_HslStatus.PresentHslLightness16;
  Light_Publish_Add(LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID, 
                    LIGHT_LIGHTNESS_SET_UNACK);
//  Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= LIGHT_LIGHTNESS_SET_UNACK;
//  Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = LIGHT_MODEL_SERVER_LIGHTNESS_MODEL_ID;
//  Light_PublishOpcodeList.BindedStateCount++;
  
  Light_Lightness_Binding(BINDING_LIGHT_HSL_SET , 0); 
}

/**
* @brief Light_HSL_HUE_Binding: Function used for binding the data of 
*        hsl hue parameter with other models.
* @param void.  
* return void
*/
void Light_Hsl_Hue_Binding(void)     
{

#ifdef ENABLE_GENERIC_MODEL_SERVER_LEVEL  
  /* Hsl Hue binding with Generic Level */
  Light_HslHue_GenericLevelBinding(&Light_HslStatus);
  Light_Publish_Add(GENERIC_MODEL_SERVER_LEVEL_MODEL_ID, 
                    GENERIC_LEVEL_SET_UNACK);
//  Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= GENERIC_LEVEL_SET_UNACK;
//  Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = GENERIC_MODEL_SERVER_LEVEL_MODEL_ID;
//  Light_PublishOpcodeList.BindedStateCount++;
#endif  
}

/**
* @brief Light_HSL_Saturation_Binding: Function used for binding the data of 
*        hsl saturation parameter with other models.
* @param void.  
* return void
*/
void Light_Hsl_Saturation_Binding(void)     
{

#ifdef ENABLE_GENERIC_MODEL_SERVER_LEVEL  
  /* Hsl Hue binding with Generic Level */
  Light_HslSaturation_GenericLevelBinding(&Light_HslStatus);
  Light_Publish_Add(GENERIC_MODEL_SERVER_LEVEL_MODEL_ID, 
                    GENERIC_LEVEL_SET_UNACK);
//  Light_PublishOpcodeList.PublishStateOpcode[Light_PublishOpcodeList.BindedStateCount]= GENERIC_LEVEL_SET_UNACK;
//  Light_PublishOpcodeList.Model_ID[Light_PublishOpcodeList.BindedStateCount] = GENERIC_MODEL_SERVER_LEVEL_MODEL_ID;
//  Light_PublishOpcodeList.BindedStateCount++;
#endif  
}

/**
* @brief GenericLevel_HslSaturationBinding: Data binding b/w Generic level and Hsl
*        Hue set.
* @param bLevelParam: pointer to the structure, which should be set.
* return void.
*/
void GenericLevel_HslSaturationBinding(Generic_LevelStatus_t * bLevelParam)
{
  /* 6.1.4.1.1 Binding with the Generic Level state
  Light HSL Hue = Generic Level + 32768
  */
  
  Light_HslStatus.PresentHslSaturation16 = bLevelParam->Present_Level16 + 32768;   
}
/*
* @brief GenericLevel_HslHueBinding: Data binding b/w Generic level and Hsl
*        Hue set.
* @param bLevelParam: pointer to the structure, which should be set.
* return void.
*/
void GenericLevel_HslHueBinding(Generic_LevelStatus_t * bLevelParam)
{ 
  /*  6.1.4.1.1 Binding with the Generic Level state
  Light HSL Hue = Generic Level + 32768
  */  
  Light_HslStatus.PresentHslHueLightness16 = bLevelParam->Present_Level16 + 32768;   
}

/**
* @brief Light_HslHue_RangeBinding: Function used for binding the data of 
*        Hsl Hue value and Hsl Hue range. this function changes the value of 
*        Hsl Hue according to the min range  and max range value.
* @param bHslHueParam: Pointer which needs to be set.  
* return void
*/
void Light_HslHue_RangeBinding(Light_HslParam_t* bHslHueParam)
{
  /*
  6.1.4.1.3 Binding with the HSL Hue Range state
  Light HSL Hue = Light HSL Hue Range Min->for (Light HSL Hue < Light HSL Hue Range Min)
  Light HSL Hue = Light HSL Hue Range Max->for (Light HSL Hue > Light HSL Hue Range Min)
  */
  
  if(bHslHueParam->HslHueLightness16 < Light_HslRangeParam.HslHueMinRange16)        
  {
    bHslHueParam->HslHueLightness16 = Light_HslRangeParam.HslHueMinRange16;
  }
  else if(bHslHueParam->HslHueLightness16 > Light_HslRangeParam.HslHueMaxRange16)           
  {
    bHslHueParam->HslHueLightness16 = Light_HslRangeParam.HslHueMaxRange16;
  }
  else
  {
    /* no condition to execute */
  }
}

/**
* @brief Light_HslSaturation_RangeBinding: Function used for binding the data of 
*        Hsl saturation value and Hsl saturation range. this function changes the value of 
*        Hsl saturation according to the min range  and max range value.
* @param bHslSatParam: Pointer which needs to be set.  
* return void
*/
void Light_HslSaturation_RangeBinding(Light_HslParam_t* bHslSatParam)
{
  /*
  6.1.4.4.3 Binding with the HSL Saturation Range state
  Light HSL Saturation = Light HSL Saturation Range Min Min->for (Light HSL Saturation < Light HSL Saturation Min)
  Light HSL Saturation = Light HSL Saturation Range Max->for (Light HSL Saturation > Light HSL Saturation Max)
  */
  
  if(bHslSatParam->HslSaturation16 < Light_HslRangeParam.HslMinSaturation16)        
  {
    bHslSatParam->HslSaturation16 = Light_HslRangeParam.HslMinSaturation16;
  }
  else if(bHslSatParam->HslSaturation16 > Light_HslRangeParam.HslMaxSaturation16)           
  {
    bHslSatParam->HslSaturation16 = Light_HslRangeParam.HslMaxSaturation16;
  }
  else
  {
    /* no condition to execute */
  }
}

/**
* @brief Light_Linear_Ligth_LC_binding: Function used for binding the data of 
*        Light_LC_lightness output data with the Light lightness linear data.
* @param lc_OutValue: value of Light LC lightness..  
* return void
*/
void Light_Linear_Ligth_LC_binding(MOBLEUINT16 lc_OutValue)
{
  Light_LightnessStatus.LinearPresentValue16 = lc_OutValue;
  Light_Lightness_Binding(BINDING_LIGHT_LIGHTNESS_LINEAR_SET , 3);
}

/**
* @brief  Light_ActualLightness_HslLightnessBinding: Data binding b/w Actual Lightness and 
*         Hsl lightness . this function will set the Hsl Lightness value
*         at the time of  Lightness Actual value set. 
* @param bActualLightParam: Pointer to the data which needs to be checked.
* return void.
*/
void Light_ActualLightness_HslLightnessBinding(Light_LightnessStatus_t* bActualLightParam)
{
  /* 6.1.4.7.1 Binding with the Light Lightness Actual state
  Light HSL Lightness = Light Lightness Actual
  */
  Light_HslStatus.PresentHslLightness16= bActualLightParam->LightnessPresentValue16;
  
  TRACE_M(TF_LIGHT_M, "Light_ActualLightness_HslLightnessBinding Function - Present HSL lightness value 0x%.2x \r\n" 
         , Light_HslStatus.PresentHslLightness16); 
}

/**
* @brief function called in Light lightness when the default transition time is enabled.
* @param pLightnessValue: pointer to the function
* return void.
*/
void Light_LightnessDefaultTransitionValue(Light_LightnessParam_t* pLightnessValue)
{
  Light_TemporaryStatus.TargetParam_1 = pLightnessValue->TargetLightnessStatus;   
  Light_TemporaryStatus.RemainingTime = Generic_DefaultTransitionParam.DefaultTransitionTime; 
  Light_GetStepValue((MOBLEUINT8)Generic_DefaultTransitionParam.DefaultTransitionTime);
  Light_ModelFlag.LightTransitionFlag = LIGHT_LIGHTNESS_TRANSITION_START;
  Light_ModelFlag.LightOptionalParam = 1;
}

/**
* @brief function called in Light CTL set when the default transition time is enabled.
* @param pCTLValue: pointer to the function
* return void.
*/
void Light_CTLDefaultTransitionValue(Light_CtlParam_t* pCTLValue)
{
  Light_TemporaryStatus.TargetParam_1 = pCTLValue->CTL_Lightness;
  Light_TemporaryStatus.TargetParam_2 = pCTLValue->CTL_Temperature;
  Light_GetStepValue((MOBLEUINT8)Generic_DefaultTransitionParam.DefaultTransitionTime);
  Light_ModelFlag.LightTransitionFlag = LIGHT_CTL_TRANSITION_START;
  Light_ModelFlag.LightOptionalParam = 1;
}

/**
* @brief function called in Light CTL temperature set when the default transition time is enabled.
* @param pCTLValue:pointer to the function
* return void.
*/
void Light_CTLTemperatureDefaultTransitionValue(Light_CtlParam_t* pCTLValue)
{
  Light_TemporaryStatus.TargetParam_1 = pCTLValue->CTL_Temperature;
  Light_TemporaryStatus.TargetParam_2 = pCTLValue->CTL_DeltaUv;
  Light_GetStepValue((MOBLEUINT8)Generic_DefaultTransitionParam.DefaultTransitionTime);
  Light_ModelFlag.LightTransitionFlag = LIGHT_TEMPERATURE_TRANSITION_START;
  Light_ModelFlag.LightOptionalParam = 1;
}

/**
Weak function are defined to support the original function if they are not
included in firmware.
There is no use of this function for application development purpose.
*/
WEAK_FUNCTION (MOBLE_RESULT Appli_Light_Lightness_Set(Light_LightnessStatus_t* pLight_LightnessParam,
                                                      MOBLEUINT8 OptionalValid))
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Linear_Set(Light_LightnessStatus_t* pLight_LightnessLinearParam,
                                                            MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Default_Set(Light_LightnessDefaultParam_t* pLight_LightnessDefaultParam,
                                                             MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Range_Set(Light_LightnessRangeParam_t* pLight_LightnessRangeParam,
                                                           MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Ctl_Set(Light_CtlStatus_t* pLight_CtlParam,
                                               MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlTemperature_Set(Light_CtlStatus_t* pLight_CtltempParam,
                                                          MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlTemperature_Range_Set(Light_CtlTemperatureRangeParam_t* pLight_CtlTempRangeParam,
                                                                MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlDefault_Set(Light_CtlDefaultParam_t* pLight_CtlDefaultParam,
                                                      MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Hsl_Set(Light_HslStatus_t* pLight_HslParam,
                                               MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslHue_Set(Light_HslStatus_t* pLight_HslHueParam,
                                                  MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslSaturation_Set(Light_HslStatus_t* pLight_HslSaturationParam,
                                                         MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslDefault_Set(Light_HslStatus_t* pLight_HslDefaultParam,
                                                      MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslRange_Set(Light_HslRangeParam_t* pLight_HslRangeParam,
                                                    MOBLEUINT8 OptionalValid))   
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Status(MOBLEUINT8 const *pLightness_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Linear_Status(MOBLEUINT8 const *pLightnessLinear_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Default_Status(MOBLEUINT8 const *pLightnessDefault_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Lightness_Range_Status(MOBLEUINT8 const *pLightnessRange_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Ctl_Status(MOBLEUINT8 const *pLightCtl_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlTemperature_Status(MOBLEUINT8 const *pLightCtlTemp_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlTemperature_Range_Status(MOBLEUINT8 const *pCtlTempRange_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_CtlDefault_Status(MOBLEUINT8 const *pCtlDefault_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_Hsl_Status(MOBLEUINT8 const *pHsl_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslHue_Status(MOBLEUINT8 const *pHslHue_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslSaturation_Status(MOBLEUINT8 const *pHslSaturation_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslDefault_Status(MOBLEUINT8 const *pHslDefault_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}

WEAK_FUNCTION(MOBLE_RESULT Appli_Light_HslRange_Status(MOBLEUINT8 const *pHslRange_status, MOBLEUINT32 pLength))  
{ return MOBLE_RESULT_SUCCESS;}



WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetLightnessStatus(MOBLEUINT8* lLightnessState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetLightnessLinearStatus(MOBLEUINT8* lLightnessState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetLightnessDefaultStatus(MOBLEUINT8* lDefaultState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetLightnessLastStatus(MOBLEUINT8* lLastState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetLightnessRangeStatus(MOBLEUINT8* lRangeState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetCtlLightStatus(MOBLEUINT8* lCtlLightState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetCtlTemperatureStatus(MOBLEUINT8* lCtlTempState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetCtlTemperatureRange(MOBLEUINT8* lCtlTempRange))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetCtlDefaultStatus(MOBLEUINT8* lCtlDefaultState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslStatus(MOBLEUINT8* lHslState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslHueStatus(MOBLEUINT8* lHslHueState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslSaturationStatus(MOBLEUINT8* lHslSaturationState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslHueRange(MOBLEUINT8* lHslHueRange))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslSatRange(MOBLEUINT8* lHslSatRange))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(MOBLE_RESULT Appli_Light_GetHslDefaultStatus(MOBLEUINT8* lHslDefaultState))
{  return MOBLE_RESULT_SUCCESS; }
WEAK_FUNCTION(void HSL2RGB_Conversion(void));
WEAK_FUNCTION(void RgbF_Create(MOBLEUINT16 value1, MOBLEUINT16 value2, MOBLEUINT16 value3));
WEAK_FUNCTION(void Light_UpdatePWMValue(MOBLEUINT8 state));
/**
* @}
*/

/**
* @}
*/


/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/


