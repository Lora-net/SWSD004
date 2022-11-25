/*!
 * @file      tracker_utility.h
 *
 * @brief     Tracker application utility definition
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TRACKER_UTILITY_H
#define TRACKER_UTILITY_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>
#include <stdbool.h>
#include "smtc_hal.h"
#include "smtc_modem_api.h"
#include "lr11xx_system_types.h"
#include "lorawan_key_config.h"
#include "wifi_middleware.h"
#include "gnss_middleware.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/* Internal Log Tag */
#define TAG_GNSS_PCB 0x01
#define TAG_GNSS_PATCH 0x02
#define TAG_WIFI 0x03
#define TAG_NEXT_SCAN 0x04

/* Internal Log Len */
#define WIFI_SINGLE_BEACON_LEN 0x08
#define WIFI_TIMESTAMP_LEN 0x04
#define GNSS_TIMING_LEN 0x08
#define GNSS_TOKEN_LEN 0x01
#define GNSS_NB_SAT_LEN 0x01
#define GNSS_PROFILE_LEN 0x01
#define GNSS_TIMESTAMP_LEN 0x04
#define GNSS_LAST_SCAN_IN_GROUP_LEN 0x01

/* Tracker application commands */

/* Application & Board */
#define GET_FW_VERSION_CMD 0x01
#define GET_FW_VERSION_LEN 0x00
#define GET_FW_VERSION_ANSWER_LEN 0x03
#define GET_HW_VERSION_CMD 0x32
#define GET_HW_VERSION_LEN 0x00
#define GET_HW_VERSION_ANSWER_LEN 0x04
#define GET_STACK_VERSION_CMD 0x33
#define GET_STACK_VERSION_LEN 0x00
#define GET_STACK_VERSION_ANSWER_LEN 0x02
#define GET_MODEM_FIRMWARE_VERSION_CMD 0x34
#define GET_MODEM_FIRMWARE_VERSION_LEN 0x00
#define GET_MODEM_FIRMWARE_VERSION_ANSWER_LEN 0x03
#define SET_LR11XX_UPDATE_CMD 0x31
#define SET_LR11XX_UPDATE_LEN 0x92
#define SET_LR11XX_UPDATE_ANSWER_LEN 0x02
#define GET_MODEM_STATUS_CMD 0x45
#define GET_MODEM_STATUS_LEN 0x00
#define GET_MODEM_STATUS_ANSWER_LEN 0x02
#define GET_MODEM_DATE_CMD 0x46
#define GET_MODEM_DATE_LEN 0x00
#define GET_MODEM_DATE_ANSWER_LEN 0x04
#define GET_TRACKER_TYPE_CMD 0x54
#define GET_TRACKER_TYPE_LEN 0x00
#define GET_TRACKER_TYPE_ANSWER_LEN 0x01
#define GET_LR11XX_FIRMWARE_VERSION_CMD 0x55
#define GET_LR11XX_FIRMWARE_VERSION_LEN 0x00
#define GET_LR11XX_FIRMWARE_VERSION_ANSWER_LEN 0x01

/* Board */
#define GET_BOARD_VOLTAGE_CMD 0x40
#define GET_BOARD_VOLTAGE_LEN 0x00
#define GET_BOARD_VOLTAGE_ANSWER_LEN 0x02

/* LoRaWAN */
#define SET_LORAWAN_DEVEUI_CMD 0x02
#define SET_LORAWAN_DEVEUI_LEN 0x08
#define GET_LORAWAN_DEVEUI_CMD 0x03
#define GET_LORAWAN_DEVEUI_LEN 0x00
#define GET_LORAWAN_DEVEUI_ANSWER_LEN 0x08
#define SET_LORAWAN_JOINEUI_CMD 0x04
#define SET_LORAWAN_JOINEUI_LEN 0x08
#define GET_LORAWAN_JOINEUI_CMD 0x05
#define GET_LORAWAN_JOINEUI_LEN 0x00
#define GET_LORAWAN_JOINEUI_ANSWER_LEN 0x08
#define SET_LORAWAN_APPKEY_CMD 0x06
#define SET_LORAWAN_APPKEY_LEN 0x10
#define GET_LORAWAN_APPKEY_CMD 0x07
#define GET_LORAWAN_APPKEY_LEN 0x00
#define GET_LORAWAN_APPKEY_ANSWER_LEN 0x10
#define SET_LORAWAN_REGION_CMD 0x35
#define SET_LORAWAN_REGION_LEN 0x01
#define GET_LORAWAN_REGION_CMD 0x36
#define GET_LORAWAN_REGION_LEN 0x00
#define GET_LORAWAN_REGION_ANSWER_LEN 0x01
#define GET_LORAWAN_PIN_CMD 0x39
#define GET_LORAWAN_PIN_LEN 0x00
#define GET_LORAWAN_PIN_ANSWER_LEN 0x04
#define SET_LORAWAN_JOIN_SERVER_CMD 0x3A
#define SET_LORAWAN_JOIN_SERVER_LEN 0x01
#define GET_LORAWAN_JOIN_SERVER_CMD 0x3B
#define GET_LORAWAN_JOIN_SERVER_LEN 0x00
#define GET_LORAWAN_JOIN_SERVER_ANSWER_LEN 0x01
#define GET_LORAWAN_CHIP_EUI_CMD 0x44
#define GET_LORAWAN_CHIP_EUI_LEN 0x00
#define GET_LORAWAN_CHIP_EUI_ANSWER_LEN 0x08
#define GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_CMD 0x50
#define GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_LEN 0x00
#define GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_ANSWER_LEN 0x02
#define SET_LORAWAN_SUB_REGION_CMD 0x56
#define SET_LORAWAN_SUB_REGION_LEN 0x01
#define GET_LORAWAN_SUB_REGION_CMD 0x57
#define GET_LORAWAN_SUB_REGION_LEN 0x00
#define GET_LORAWAN_SUB_REGION_ANSWER_LEN 0x01

/* GNSS */
#define SET_GNSS_CONSTELLATION_CMD 0x0A
#define SET_GNSS_CONSTELLATION_LEN 0x01
#define GET_GNSS_CONSTELLATION_CMD 0x0B
#define GET_GNSS_CONSTELLATION_LEN 0x00
#define GET_GNSS_CONSTELLATION_ANSWER_LEN 0x01
#define SET_GNSS_ASSISTANCE_POSITION_CMD 0x0C
#define SET_GNSS_ASSISTANCE_POSITION_LEN 0x08
#define GET_GNSS_ASSISTANCE_POSITION_CMD 0x0D
#define GET_GNSS_ASSISTANCE_POSITION_LEN 0x00
#define GET_GNSS_ASSISTANCE_POSITION_ANSWER_LEN 0x08
#define SET_GNSS_ANTENNA_SEL_CMD 0x0E
#define SET_GNSS_ANTENNA_SEL_LEN 0x01
#define GET_GNSS_ANTENNA_SEL_CMD 0x0F
#define GET_GNSS_ANTENNA_SEL_LEN 0x00
#define GET_GNSS_ANTENNA_SEL_ANSWER_LEN 0x01
#define GNSS_ALMANAC_UPDATE_CMD 0x2C
#define GNSS_ALMANAC_UPDATE_LEN 0x3E
#define GET_GNSS_LAST_ALMANAC_UPDATE_CMD 0x2D
#define GET_GNSS_LAST_ALMANAC_UPDATE_LEN 0x00
#define GET_GNSS_LAST_ALMANAC_UPDATE_ANSWER_LEN 0x04
#define GET_GNSS_LAST_NB_SV_CMD 0x51
#define GET_GNSS_LAST_NB_SV_LEN 0x00
#define GET_GNSS_LAST_NB_SV_ANSWER_LEN 0x01

/* Tracker Application */
#define SET_USE_ACCELEROMETER_CMD 0x24
#define SET_USE_ACCELEROMETER_LEN 0x01
#define GET_USE_ACCELEROMETER_CMD 0x25
#define GET_USE_ACCELEROMETER_LEN 0x00
#define GET_USE_ACCELEROMETER_ANSWER_LEN 0x01
#define SET_APP_MOBILE_SCAN_INTERVAL_CMD 0x26
#define SET_APP_MOBILE_SCAN_INTERVAL_LEN 0x02
#define GET_APP_MOBILE_SCAN_INTERVAL_CMD 0x27
#define GET_APP_MOBILE_SCAN_INTERVAL_LEN 0x00
#define GET_APP_MOBILE_SCAN_INTERVAL_ANSWER_LEN 0x02
#define SET_APP_STATIC_SCAN_INTERVAL_CMD 0x28
#define SET_APP_STATIC_SCAN_INTERVAL_LEN 0x02
#define GET_APP_STATIC_SCAN_INTERVAL_CMD 0x29
#define GET_APP_STATIC_SCAN_INTERVAL_LEN 0x00
#define GET_APP_STATIC_SCAN_INTERVAL_ANSWER_LEN 0x02
#define SET_APP_RESET_CMD 0x2B
#define SET_APP_RESET_LEN 0x00
#define SET_AIRPLANE_MODE_CMD 0x37
#define SET_AIRPLANE_MODE_LEN 0x01
#define GET_AIRPLANE_MODE_CMD 0x38
#define GET_AIRPLANE_MODE_LEN 0x00
#define GET_AIRPLANE_MODE_ANSWER_LEN 0x01
#define SET_SCAN_PRIORITY_CMD 0x3C
#define SET_SCAN_PRIORITY_LEN 0x01
#define GET_SCAN_PRIORITY_CMD 0x3D
#define GET_SCAN_PRIORITY_LEN 0x00
#define GET_SCAN_PRIORITY_ANSWER_LEN 0x01
#define SET_APP_FLUSH_INTERNAL_LOG_CMD 0x2A
#define SET_APP_FLUSH_INTERNAL_LOG_LEN 0x00
#define SET_APP_INTERNAL_LOG_CMD 0x41
#define SET_APP_INTERNAL_LOG_LEN 0x01
#define GET_APP_INTERNAL_LOG_CMD 0x42
#define GET_APP_INTERNAL_LOG_LEN 0x00
#define GET_APP_INTERNAL_LOG_ANSWER_LEN 0x01
#define READ_APP_INTERNAL_LOG_CMD 0x43
#define READ_APP_INTERNAL_LOG_LEN 0x00
#define GET_APP_INTERNAL_LOG_REMANING_SPACE_CMD 0x49
#define GET_APP_INTERNAL_LOG_REMANING_SPACE_LEN 0x00
#define GET_APP_INTERNAL_LOG_REMANING_SPACE_ANSWER_LEN 0x01
#define GET_APP_ACCUMULATED_CHARGE_CMD 0x4A
#define GET_APP_ACCUMULATED_CHARGE_LEN 0x00
#define GET_APP_ACCUMULATED_CHARGE_ANSWER_LEN 0x04
#define RESET_APP_ACCUMULATED_CHARGE_CMD 0x4B
#define RESET_APP_ACCUMULATED_CHARGE_LEN 0x00
#define GET_APP_TRACKER_SETTINGS_CMD 0x4C
#define GET_APP_TRACKER_SETTINGS_LEN 0x00
#define GET_APP_TRACKER_SETTINGS_ANSWER_LEN 0x37
#define GET_APP_TRACKER_SETTINGS_VERSION 0x01
#define GET_APP_SYSTEM_SANITY_CHECK_CMD 0x53
#define GET_APP_SYSTEM_SANITY_CHECK_LEN 0x00
#define GET_APP_SYSTEM_SANITY_CHECK_ANSWER_LEN 0x01

#define NOT_SUPPORTED_COMMAND_CMD 0xFF
#define NOT_SUPPORTED_COMMAND_LEN 0x01

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief End to end utility return status
 */
typedef enum
{
    TRACKER_ERROR   = 0x00,
    TRACKER_SUCCESS = 0x01,
} tracker_return_status_t;

/*!
 * @brief Tracker system sanity check mask
 */
typedef enum
{
    TRACKER_GNSS_SCAN_SUCCESSFUL_ONCE = 0x01,  //!< Means that at least once GNSS scan has been successful
    TRACKER_WIFI_SCAN_SUCCESSFUL_ONCE = 0x02,  //!< Means that at least once Wi-Fi scan has been successful
    TRACKER_DOWNLINK_SUCCESSFUL_ONCE  = 0x04,  //!< Means that at least once Applicative downlink has been received
} tracker_system_sanity_check_mask_t;

/*!
 * @brief Tracker scan priority
 */
typedef enum
{
    TRACKER_GNSS_PRIORITY = 0x00,  //!< Means that GNSS scan has the priority
    TRACKER_WIFI_PRIORITY = 0x01,  //!< Means that Wi-Fi scan has the priority
    TRACKER_NO_PRIORITY   = 0x02,  //!< Means no priority, GNSS and Wi-Fi are performed
} tracker_scan_priority_t;

typedef enum smtc_modem_sub_region_e
{
    SMTC_MODEM_NO_SUB_REGION    = 0x00,
    SMTC_MODEM_SUB_REGION_JAPAN = 0x01,  //!< JAPAN is part of the AS-923 group 1 region
} smtc_modem_sub_region_t;

/*!
 * @brief Demo app context structure
 */
typedef struct
{
    /* Time variables */
    uint32_t start_sequence_timestamp;

    /* Context */
    uint8_t tracker_context_empty;

    /* Board */
    uint16_t voltage;
    bool     reset_board_asked;

    /* LoRaWAN Parameters */
    uint8_t                 dev_eui[8];
    uint8_t                 join_eui[8];
    uint8_t                 app_key[16];
    uint8_t                 chip_eui[8];
    uint8_t                 lorawan_pin[4];
    bool                    lorawan_parameters_have_changed;
    smtc_modem_region_t     lorawan_region;
    smtc_modem_sub_region_t lorawan_sub_region;
    bool                    use_semtech_join_server;

    /* LR11XX & Modem version information */
    lr11xx_system_version_t      lr11xx_fw_version;
    smtc_modem_version_t         firmware_version;
    smtc_modem_lorawan_version_t lorawan_version;
    bool                         has_lr11xx_trx_firmware;

    /* Application Parameters */
    bool                               accelerometer_used;
    uint32_t                           mobile_scan_interval;
    uint32_t                           static_scan_interval;
    uint8_t                            accelerometer_move_history;
    bool                               airplane_mode;
    tracker_scan_priority_t            scan_priority;
    bool                               internal_log_enable;
    uint8_t                            tracker_settings_payload_len;
    uint8_t                            tracker_settings_payload[242];
    tracker_system_sanity_check_mask_t system_sanity_check;

    /* BLE parameters */
    bool ble_connected;
    bool ble_disconnected;
    bool ble_advertisement_on;
    bool new_value_to_set;
    bool ble_cmd_received;

    /* GNSS Parameters */
    uint8_t  gnss_antenna_sel;
    uint32_t gnss_last_almanac_update;
    float    gnss_assistance_position_latitude;
    float    gnss_assistance_position_longitude;

    /* Results values */
    gnss_mw_event_data_terminated_t gnss_nb_scan_sent;
    uint8_t                         last_nb_detected_satellites;
    wifi_mw_event_data_terminated_t wifi_nb_scan_sent;
    uint8_t                         last_nb_detected_mac_address;
    int16_t                         temperature;
    uint32_t                        lbm_scan_charge_mAh;
    uint32_t                        gnss_scan_charge_uAh;
    uint32_t                        wifi_scan_charge_uAh;
    uint32_t                        accumulated_charge_mAh;

    /* parameters for flash read/write operations */
    bool     internal_log_flush_request;
    uint8_t  internal_log_empty;
    uint16_t nb_scan;
    uint32_t flash_addr_start;
    uint32_t flash_addr_end;
    uint32_t flash_addr_current;
    uint32_t flash_remaining_space;
} tracker_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Init and store the demo app internal log context in the flash memory in the dedicated memory zone
 *
 * @param [out] demo app return status \ref tracker_return_status_t
 */
tracker_return_status_t tracker_init_internal_log_ctx( void );

/*!
 * @brief Restore the internal log context from the flash memory and set in /ref FieldTest_t structure
 *
 * @note if SMTC_FAIL is returned call init_field_test_ctx
 *
 * @returns SMTC_SUCESS/SMTC_FAIL
 */
tracker_return_status_t tracker_restore_internal_log_ctx( void );

/*!
 * @brief Store the Wi-Fi results in internal log flash memory
 *
 * @param [in] wifi_scan_results Wi-Fi scan results \ref wifi_mw_event_data_scan_done_t
 */
void tracker_store_wifi_in_internal_log( wifi_mw_event_data_scan_done_t* wifi_scan_results );

/*!
 * @brief Store the GNSS results in internal log flash memory
 *
 * @param [in] gnss_scan_results GNSS scan results \ref gnss_mw_event_data_scan_done_t
 */
void tracker_store_gnss_in_internal_log( const gnss_mw_event_data_scan_done_t* gnss_scan_results );

/*!
 * @brief Store the demo app context in the flash memory in the dedicated memory zone
 */
void tracker_store_app_ctx( void );

/*!
 * @brief Init the Demo app context
 *
 * @param [in] dev_eui LoRaWAN Device Eui
 * @param [in] join_eui LoRaWAN Join Eui
 * @param [in] app_key LoRaWAN Application Key
 * @param [in] store_in_flash store the context in flash
 */
void tracker_init_app_ctx( const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* app_key,
                           const bool store_in_flash );

/*!
 * @brief Restore the demo app context from the flash memory and set in /ref tracker_ctx_t structure
 *
 * @returns [out] SMTC_SUCESS/SMTC_FAIL
 */
tracker_return_status_t tracker_restore_app_ctx( void );

/*!
 * @brief Restore the logs results from the flash memory, parse and display it
 */
void tracker_restore_internal_log( void );

/*!
 * @brief Erase the internal log et create a new internal log context.
 */
void tracker_reset_internal_log( void );

/*!
 * @brief Parse the commands coming from outside.
 *
 * @param [in] stack_id The stack identifier
 * @param [in] payload payload to parse
 * @param [out] buffer_out answer output buffer
 * @param [in] buffer_out_len len of the output buffer
 * @param [in] all_command_enable activate or no all the commands
 *
 * @returns size of buffer_out
 */
uint8_t tracker_parse_cmd( uint8_t stack_id, uint8_t* payload, uint8_t* buffer_out, const uint8_t buffer_out_len,
                           bool all_commands_enable );

/*!
 * @brief Return the battery level based on the Modem charge.
 *
 * @returns battery level in percentage
 */
uint8_t tracker_get_battery_level( void );

/*!
 * @brief Return the computed CRC
 *
 * @param [in] initial_value initial value of the CRC
 * @param [in] buffer Buffer containing data used to compute the CRC
 * @param [in] length Length of buffer
 *
 * @returns CRC value
 */
inline static uint8_t tracker_ctx_compute_crc( const uint8_t initial_value, const uint8_t* buffer, uint16_t length )
{
    uint8_t crc = initial_value;

    for( uint16_t i = 0; i < length; i++ )
    {
        uint8_t extract = buffer[i];
        uint8_t sum;

        for( uint8_t j = 8; j > 0; j-- )
        {
            sum = ( crc ^ extract ) & 0x01;
            crc >>= 1;

            if( sum != 0 )
            {
                crc ^= 0x65;
            }

            extract >>= 1;
        }
    }

    return crc;
}

#ifdef __cplusplus
}
#endif

#endif  // TRACKER_UTILITY_H

/* --- EOF ------------------------------------------------------------------ */
