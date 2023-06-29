/*!
 * @ingroup   tracker
 * @file      tracker_utility.c
 *
 * @brief     Demo application end to end utility implementation.
 *
 * @copyright
 * @parblock
 * Revised BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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
 * @endparblock
 */

/*!
 * @addtogroup tracker
 * Demo application end to end utility implementation.
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <time.h>
#include <string.h>
#include "smtc_hal.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_api.h"
#include "smtc_board_ralf.h"
#include "smtc_basic_modem_lr11xx_api_extension.h"
#include "lr1110_trk1xks_board.h"
#include "tracker_utility.h"
#include "main_tracker_application.h"
#include "apps_modem_common.h"
#include "apps_utilities.h"
#include "lorawan_key_config.h"

#include "ralf.h"
#include "lr11xx_wifi_types.h"
#include "lr11xx_bootloader.h"
#include "lr11xx_system.h"
#include "lr11xx_gnss.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define NB_CHUNK_LR11XX 1703
#define NB_CHUNK_ALMANAC 42
#define CHUNK_INTERNAL_LOG 145
#define INTERNAL_LOG_BUFFER_LEN 3000

/*!
 * @brief Offset in second between GPS EPOCH and UNIX EPOCH time
 */
#define OFFSET_BETWEEN_GPS_EPOCH_AND_UNIX_EPOCH 315964800

/*!
 * @brief define the context areas
 */
#define FLASH_USER_INTERNAL_LOG_CTX_START_PAGE ( 195 )

#define FLASH_USER_INTERNAL_LOG_CTX_START_ADDR ADDR_FLASH_PAGE( FLASH_USER_INTERNAL_LOG_CTX_START_PAGE )
#define FLASH_USER_INTERNAL_LOG_CTX_END_ADDR \
    ( FLASH_END_ADDR_OF_PAGE(                \
        FLASH_USER_INTERNAL_LOG_CTX_START_PAGE ) ) /* End @ of user tracker internal log ctx Flash area */

#define FLASH_USER_TRACKER_CTX_START_PAGE ( 196 )

#define FLASH_USER_TRACKER_CTX_START_ADDR ADDR_FLASH_PAGE( FLASH_USER_TRACKER_CTX_START_PAGE )
#define FLASH_USER_TRACKER_CTX_END_ADDR \
    ( FLASH_END_ADDR_OF_PAGE( FLASH_USER_TRACKER_CTX_START_PAGE ) ) /* End @ of user tracker ctx Flash area */

#define FLASH_USER_MODEME_TRACKER_CTX_START_PAGE ( 202 )
#define FLASH_USER_MODEME_TRACKER_CTX_START_ADDR ADDR_FLASH_PAGE( FLASH_USER_MODEME_TRACKER_CTX_START_PAGE )
#define FLASH_USER_MODEME_TRACKER_CTX_END_ADDR \
    ( FLASH_END_ADDR_OF_PAGE( FLASH_USER_MODEME_TRACKER_CTX_START_PAGE ) ) /* End @ of user tracker ctx Flash area */

#define FLASH_USER_MODEME_TRACKER_REGION_OFFSET ( 69 )

#define TRACKER_CONTEXT_SIZE 128
#define INTERNAL_LOG_CONTEXT_SIZE 32
#define INTERNAL_LOG_SCAN_BUFFER 256

/*!
 * @brief Tracker parameter limits
 */
#define MOBILE_SCAN_INTERVAL_MIN 120
#define MOBILE_SCAN_INTERVAL_MAX 1800
#define STATIC_SCAN_INTERVAL_MIN 600
#define STATIC_SCAN_INTERVAL_MAX 86400

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Modem radio
 */
extern ralf_t* modem_radio;

/*!
 * @brief Demo application context structure
 */
tracker_ctx_t tracker_ctx;

/*!
 * @brief Buffer containing chunk during lr11xx modem update
 */
static uint32_t chunk_buffer[128];

/*!
 * @brief Buffer index pointing on the chunk last byte during lr11xx modem update
 */
uint8_t chunk_buffer_index;

/*!
 * @brief LR11XX flash offset used during lr11xx update
 */
uint32_t lr11xx_flash_offset;

/*!
 * @brief Buffer containing internal logs scan during the read internal log command
 */
static uint8_t internal_log_buffer[INTERNAL_LOG_BUFFER_LEN];

/*!
 * @brief Scan len of the internal_log_buffer during the read internal log command
 */
uint16_t internal_log_buffer_len;

/*!
 * @brief Scan index ongoing during the read internal log command
 */
uint32_t internal_log_scan_index = 1;

/*!
 * @brief Internal Log tracker settings send status
 */
bool internal_log_tracker_settings_sent = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Print the device settings
 */
static void tracker_print_device_settings( void );

/*!
 * @brief Get the tracker settings in order to send over BLE
 *
 * @param [out] out_buffer Buffer where is stored the scan
 * @param [in] out_buffer_len Len of the buffer where is stored the scan
 * @param [out] buffer_len Length of the scan stored into the buffer
 */
static void tracker_get_device_settings( uint8_t* out_buffer, const uint16_t out_buffer_len, uint16_t* buffer_len );

/*!
 *@brief Read 4 bytes from array buffer at index and interpret it as uint32_t LSB. Increment index so that
 * it is position on the byte just after the uint32 read (so possibly after the end of array if the
 * uint32 that is being read is at the end of the buffer).
 * @param [in] dev_eui LoRaWAN Device Eui
 * @param [in] join_eui LoRaWAN Join Eui
 *
 * @returns uint32_t value
 */
static uint32_t get_uint32_from_array_at_index_and_inc( const uint8_t* array, uint16_t* index );

/*!
 *@brief Store in the flash memory the scan
 * @param [in] buffer buffer to store in flash
 * @param [in] nb_variable_elements number of element to store
 * @param [in] len length of the buffer
 */
static void tracker_store_internal_log( uint8_t* buffer, uint8_t nb_variable_elements, uint16_t len );

/*!
 * @brief return the user memory flash remaning space.
 *
 * @returns the user memory flash remaning space in percentage
 */
static uint8_t tracker_get_remaining_memory_space( void );

/*!
 * @brief Store the internal log context in the flash memory and set in /ref FieldTest_t structure
 */
static void tracker_store_internal_log_ctx( void );

/*!
 * @brief Restore the internal log of one given scan_number from the flash memory.
 *
 * @param [in] scan_number number of the scan to get
 * @param [out] out_buffer buffer where is stored the scan
 * @param [in] out_buffer_len buffer len where is stored the scan
 * @param [out] buffer_len length of the scan stored into the buffer
 */
static void tracker_get_one_scan_from_internal_log( uint16_t scan_number, uint8_t* out_buffer,
                                                    const uint16_t out_buffer_len, uint16_t* buffer_len );

/*!
 * @brief Erase the scan results and the internal log context from the flash memory
 */
static void tracker_erase_internal_log( void );

/*!
 * @brief Erase full user memory flash between end of the app and the contexts area
 */
static void tracker_erase_full_user_flash_memory( void );

/*!
 * @brief Check if a LoRaWAN region is already existing on the Modem-E tracker memory flash
 *
 * @param [out] existing_region existing LoRaWAN region
 *
 * @returns true is a region is existing, false otherwise
 */
static bool tracker_check_for_existing_region( uint8_t* existing_region );

/*!
 * @brief Get the oldest and the newest almanac date from the LR11XX
 *
 * @param [in] context Radio abstraction
 * @param [out] oldest_almanac_date oldest sv date
 * @param [out] newest_almanac_date newest sv date
 *
 * @return true if almanacs dates have been recovered, false if not
 */
static bool smtc_board_get_almanac_dates( const void* context, uint32_t* oldest_almanac_date,
                                          uint32_t* newest_almanac_date );

/*!
 * @brief Check the validity of the tracker parameters, if a paramater is out of bound, it's reinitialized to his
 * default value.
 */
static void tracker_check_app_ctx( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

tracker_return_status_t tracker_init_internal_log_ctx( void )
{
    uint8_t ctx_buf[INTERNAL_LOG_CONTEXT_SIZE];
    hal_flash_read_buffer( FLASH_USER_INTERNAL_LOG_CTX_START_ADDR, ctx_buf, INTERNAL_LOG_CONTEXT_SIZE );
    tracker_ctx.internal_log_empty = ctx_buf[0];

    if( tracker_ctx.internal_log_empty == FLASH_BYTE_EMPTY_CONTENT )
    {
        HAL_DBG_TRACE_INFO( "New internal log context\n" );

        tracker_ctx.nb_scan               = 0;
        tracker_ctx.flash_addr_start      = hal_flash_get_user_start_addr( );
        tracker_ctx.flash_addr_current    = tracker_ctx.flash_addr_start;
        tracker_ctx.flash_addr_end        = FLASH_USER_END_ADDR;
        tracker_ctx.flash_remaining_space = tracker_ctx.flash_addr_end - tracker_ctx.flash_addr_current;
        tracker_store_internal_log_ctx( );
    }
    else
    {
        return TRACKER_ERROR;
    }
    return TRACKER_SUCCESS;
}

tracker_return_status_t tracker_restore_internal_log_ctx( void )
{
    uint8_t ctx_buf[INTERNAL_LOG_CONTEXT_SIZE];
    uint8_t index = 0;

    hal_flash_read_buffer( FLASH_USER_INTERNAL_LOG_CTX_START_ADDR, ctx_buf, INTERNAL_LOG_CONTEXT_SIZE );

    tracker_ctx.internal_log_flush_request = false;
    tracker_ctx.internal_log_empty         = ctx_buf[0];
    index                                  = 1;

    if( tracker_ctx.internal_log_empty == FLASH_BYTE_EMPTY_CONTENT )
    {
        return TRACKER_ERROR;
    }
    else
    {
        tracker_ctx.nb_scan = ctx_buf[index++];
        tracker_ctx.nb_scan += ( uint32_t ) ctx_buf[index++] << 8;

        tracker_ctx.flash_addr_start = ctx_buf[index++];
        tracker_ctx.flash_addr_start += ( uint32_t ) ctx_buf[index++] << 8;
        tracker_ctx.flash_addr_start += ( uint32_t ) ctx_buf[index++] << 16;
        tracker_ctx.flash_addr_start += ( uint32_t ) ctx_buf[index++] << 24;

        if( hal_flash_get_user_start_addr( ) != tracker_ctx.flash_addr_start )
        {
            HAL_DBG_TRACE_ERROR( "flash_addr_start wrong, erase user internal log\n" );
            tracker_erase_full_user_flash_memory( );

            return TRACKER_ERROR;
        }

        hal_flash_set_user_start_addr( tracker_ctx.flash_addr_start );

        tracker_ctx.flash_addr_end = ctx_buf[index++];
        tracker_ctx.flash_addr_end += ( uint32_t ) ctx_buf[index++] << 8;
        tracker_ctx.flash_addr_end += ( uint32_t ) ctx_buf[index++] << 16;
        tracker_ctx.flash_addr_end += ( uint32_t ) ctx_buf[index++] << 24;

        tracker_ctx.flash_addr_current = ctx_buf[index++];
        tracker_ctx.flash_addr_current += ( uint32_t ) ctx_buf[index++] << 8;
        tracker_ctx.flash_addr_current += ( uint32_t ) ctx_buf[index++] << 16;
        tracker_ctx.flash_addr_current += ( uint32_t ) ctx_buf[index++] << 24;

        tracker_ctx.flash_remaining_space = ctx_buf[index++];
        tracker_ctx.flash_remaining_space += ( uint32_t ) ctx_buf[index++] << 8;
        tracker_ctx.flash_remaining_space += ( uint32_t ) ctx_buf[index++] << 16;
        tracker_ctx.flash_remaining_space += ( uint32_t ) ctx_buf[index++] << 24;

        /* Compute CRC of the context */
        if( ctx_buf[index] != tracker_ctx_compute_crc( 0xFF, ctx_buf, index ) )
        {
            HAL_DBG_TRACE_ERROR( "Internal log context CRC is wrong, erase user internal log\n" );
            tracker_erase_full_user_flash_memory( );

            return TRACKER_ERROR;
        }

        /* Internal log context */
        HAL_DBG_TRACE_MSG( "#Internal log context:\n" );
        HAL_DBG_TRACE_PRINTF( "#\tnb_scan : %d\n", tracker_ctx.nb_scan );
        HAL_DBG_TRACE_PRINTF( "#\tflash_addr_start : %08X\n", tracker_ctx.flash_addr_start );
        HAL_DBG_TRACE_PRINTF( "#\tflash_addr_current : %08X\n", tracker_ctx.flash_addr_current );
        HAL_DBG_TRACE_PRINTF( "#\tflash_user_end_addr : %08X\n", FLASH_USER_END_ADDR );
        HAL_DBG_TRACE_PRINTF( "#\tflash_remaining_space : %d%%\n\n", tracker_get_remaining_memory_space( ) );
    }
    return TRACKER_SUCCESS;
}

void tracker_reset_internal_log( void )
{
    if( tracker_ctx.internal_log_empty != FLASH_BYTE_EMPTY_CONTENT )
    {
        tracker_erase_internal_log( );

        tracker_ctx.internal_log_empty = FLASH_BYTE_EMPTY_CONTENT;
    }

    /* Reinit the internal log context */
    if( tracker_init_internal_log_ctx( ) != TRACKER_SUCCESS )
    {
        HAL_DBG_TRACE_ERROR( "tracker_init_internal_log_ctx failed\n" );
    }
}

tracker_return_status_t tracker_restore_app_ctx( void )
{
    uint8_t  ctx_buf[TRACKER_CONTEXT_SIZE];
    uint32_t tracker_version = 0;

    hal_flash_read_buffer( FLASH_USER_TRACKER_CTX_START_ADDR, ctx_buf, TRACKER_CONTEXT_SIZE );

    tracker_ctx.tracker_context_empty = ctx_buf[0];

    if( tracker_ctx.tracker_context_empty == FLASH_BYTE_EMPTY_CONTENT )
    {
        return TRACKER_ERROR;
    }
    else
    {
        uint8_t ctx_buf_idx = 1;
        int32_t latitude    = 0;
        int32_t longitude   = 0;

        memcpy( tracker_ctx.dev_eui, ctx_buf + ctx_buf_idx, SET_LORAWAN_DEVEUI_LEN );
        ctx_buf_idx += SET_LORAWAN_DEVEUI_LEN;
        memcpy( tracker_ctx.join_eui, ctx_buf + ctx_buf_idx, SET_LORAWAN_JOINEUI_LEN );
        ctx_buf_idx += SET_LORAWAN_JOINEUI_LEN;
        memcpy( tracker_ctx.app_key, ctx_buf + ctx_buf_idx, SET_LORAWAN_APPKEY_LEN );
        ctx_buf_idx += SET_LORAWAN_APPKEY_LEN;

        if( memcmp( tracker_ctx.dev_eui, tracker_ctx.join_eui, SET_LORAWAN_DEVEUI_LEN ) == 0 )
        {
            /* DevEUI == JoinEUI something goes wrong, reinit tracker context */
            return TRACKER_ERROR;
        }

        /* GNSS Parameters */
        tracker_ctx.gnss_antenna_sel = ctx_buf[ctx_buf_idx++];

        latitude = ctx_buf[ctx_buf_idx++];
        latitude += ctx_buf[ctx_buf_idx++] << 8;
        latitude += ctx_buf[ctx_buf_idx++] << 16;
        latitude += ctx_buf[ctx_buf_idx++] << 24;
        tracker_ctx.gnss_assistance_position_latitude = ( float ) latitude / 10000000;

        longitude = ctx_buf[ctx_buf_idx++];
        longitude += ctx_buf[ctx_buf_idx++] << 8;
        longitude += ctx_buf[ctx_buf_idx++] << 16;
        longitude += ctx_buf[ctx_buf_idx++] << 24;
        tracker_ctx.gnss_assistance_position_longitude = ( float ) longitude / 10000000;

        tracker_ctx.gnss_last_almanac_update = ctx_buf[ctx_buf_idx++];
        tracker_ctx.gnss_last_almanac_update += ctx_buf[ctx_buf_idx++] << 8;
        tracker_ctx.gnss_last_almanac_update += ctx_buf[ctx_buf_idx++] << 16;
        tracker_ctx.gnss_last_almanac_update += ctx_buf[ctx_buf_idx++] << 24;

        /* Application Parameters */
        tracker_ctx.accelerometer_used   = ctx_buf[ctx_buf_idx++];
        tracker_ctx.mobile_scan_interval = ctx_buf[ctx_buf_idx++];
        tracker_ctx.mobile_scan_interval += ctx_buf[ctx_buf_idx++] << 8;
        tracker_ctx.mobile_scan_interval += ctx_buf[ctx_buf_idx++] << 16;
        tracker_ctx.mobile_scan_interval += ctx_buf[ctx_buf_idx++] << 24;

        tracker_ctx.static_scan_interval = ctx_buf[ctx_buf_idx++];
        tracker_ctx.static_scan_interval += ctx_buf[ctx_buf_idx++] << 8;
        tracker_ctx.static_scan_interval += ctx_buf[ctx_buf_idx++] << 16;
        tracker_ctx.static_scan_interval += ctx_buf[ctx_buf_idx++] << 24;

        tracker_ctx.lorawan_region          = ( smtc_modem_region_t ) ctx_buf[ctx_buf_idx++];
        tracker_ctx.use_semtech_join_server = ctx_buf[ctx_buf_idx++];
        tracker_ctx.airplane_mode           = ctx_buf[ctx_buf_idx++];
        tracker_ctx.scan_priority           = ( tracker_scan_priority_t ) ctx_buf[ctx_buf_idx++];
        tracker_ctx.internal_log_enable     = ctx_buf[ctx_buf_idx++];

        tracker_ctx.accumulated_charge_mAh = ctx_buf[ctx_buf_idx++];
        tracker_ctx.accumulated_charge_mAh += ctx_buf[ctx_buf_idx++] << 8;
        tracker_ctx.accumulated_charge_mAh += ctx_buf[ctx_buf_idx++] << 16;
        tracker_ctx.accumulated_charge_mAh += ctx_buf[ctx_buf_idx++] << 24;

        tracker_ctx.lorawan_sub_region = ctx_buf[ctx_buf_idx++];
        if( tracker_ctx.lorawan_sub_region == 0xFF )
        {
            tracker_ctx.lorawan_sub_region = SMTC_MODEM_NO_SUB_REGION;
        }

        tracker_version = ctx_buf[ctx_buf_idx++];
        tracker_version += ctx_buf[ctx_buf_idx++] << 8;
        tracker_version += ctx_buf[ctx_buf_idx++] << 16;

        tracker_check_app_ctx( );

        tracker_print_device_settings( );

        /* Compute CRC of the context, check the CRC only if a firmware version is existing in the flash context and
         * equal to the installed firmware, the CRC can't be guaranteed on a previous firmware version after a FOTA (if
         * the CRC was not existing for instance) */
        if( ( ctx_buf[ctx_buf_idx] != tracker_ctx_compute_crc( 0xFF, ctx_buf, ctx_buf_idx ) ) &&
            ( tracker_version == ( TRACKER_SUB_MINOR_APP_VERSION + ( TRACKER_MINOR_APP_VERSION << 8 ) +
                                   ( TRACKER_MAJOR_APP_VERSION << 16 ) ) ) )
        {
            HAL_DBG_TRACE_ERROR( "Tracker context crc is wrong\n" );

            return TRACKER_ERROR;
        }
    }
    return TRACKER_SUCCESS;
}

void tracker_store_app_ctx( void )
{
    uint8_t ctx_buf[TRACKER_CONTEXT_SIZE];
    uint8_t ctx_buf_idx     = 0;
    int32_t latitude        = 0;
    int32_t longitude       = 0;
    uint8_t tracker_ctx_crc = 0;

    if( tracker_ctx.tracker_context_empty != FLASH_BYTE_EMPTY_CONTENT )
    {
        hal_flash_erase_page( FLASH_USER_TRACKER_CTX_START_ADDR, 1 );
    }

    /* Context exists */
    ctx_buf[ctx_buf_idx++] = tracker_ctx.tracker_context_empty;

    /* LoRaWAN Parameter */
    memcpy( ctx_buf + ctx_buf_idx, tracker_ctx.dev_eui, SET_LORAWAN_DEVEUI_LEN );
    ctx_buf_idx += SET_LORAWAN_DEVEUI_LEN;
    memcpy( ctx_buf + ctx_buf_idx, tracker_ctx.join_eui, SET_LORAWAN_JOINEUI_LEN );
    ctx_buf_idx += SET_LORAWAN_JOINEUI_LEN;
    memcpy( ctx_buf + ctx_buf_idx, tracker_ctx.app_key, SET_LORAWAN_APPKEY_LEN );
    ctx_buf_idx += SET_LORAWAN_APPKEY_LEN;

    /* GNSS Parameters */
    ctx_buf[ctx_buf_idx++] = tracker_ctx.gnss_antenna_sel;

    latitude               = tracker_ctx.gnss_assistance_position_latitude * 10000000;
    ctx_buf[ctx_buf_idx++] = latitude;
    ctx_buf[ctx_buf_idx++] = latitude >> 8;
    ctx_buf[ctx_buf_idx++] = latitude >> 16;
    ctx_buf[ctx_buf_idx++] = latitude >> 24;

    longitude              = tracker_ctx.gnss_assistance_position_longitude * 10000000;
    ctx_buf[ctx_buf_idx++] = longitude;
    ctx_buf[ctx_buf_idx++] = longitude >> 8;
    ctx_buf[ctx_buf_idx++] = longitude >> 16;
    ctx_buf[ctx_buf_idx++] = longitude >> 24;

    ctx_buf[ctx_buf_idx++] = tracker_ctx.gnss_last_almanac_update;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.gnss_last_almanac_update >> 8;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.gnss_last_almanac_update >> 16;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.gnss_last_almanac_update >> 24;

    /* Application Parameters */
    ctx_buf[ctx_buf_idx++] = tracker_ctx.accelerometer_used;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.mobile_scan_interval;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.mobile_scan_interval >> 8;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.mobile_scan_interval >> 16;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.mobile_scan_interval >> 24;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.static_scan_interval;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.static_scan_interval >> 8;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.static_scan_interval >> 16;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.static_scan_interval >> 24;

    ctx_buf[ctx_buf_idx++] = tracker_ctx.lorawan_region;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.use_semtech_join_server;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.airplane_mode;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.scan_priority;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.internal_log_enable;

    ctx_buf[ctx_buf_idx++] = tracker_ctx.accumulated_charge_mAh;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.accumulated_charge_mAh >> 8;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.accumulated_charge_mAh >> 16;
    ctx_buf[ctx_buf_idx++] = tracker_ctx.accumulated_charge_mAh >> 24;

    ctx_buf[ctx_buf_idx++] = tracker_ctx.lorawan_sub_region;

    ctx_buf[ctx_buf_idx++] = TRACKER_SUB_MINOR_APP_VERSION;
    ctx_buf[ctx_buf_idx++] = TRACKER_MINOR_APP_VERSION;
    ctx_buf[ctx_buf_idx++] = TRACKER_MAJOR_APP_VERSION;

    /* Compute CRC of the context */
    tracker_ctx_crc        = tracker_ctx_compute_crc( 0xFF, ctx_buf, ctx_buf_idx );
    ctx_buf[ctx_buf_idx++] = tracker_ctx_crc;

    hal_flash_write_buffer( FLASH_USER_TRACKER_CTX_START_ADDR, ctx_buf, ctx_buf_idx );
}

void tracker_init_app_ctx( const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* app_key,
                           const bool store_in_flash )
{
    uint8_t existing_region = LORAWAN_REGION;

    HAL_DBG_TRACE_INFO( "New Tracker context\n" );

    /* Context exists */
    tracker_ctx.tracker_context_empty = 1;

    /* LoRaWAN Parameter */
    memcpy( tracker_ctx.dev_eui, dev_eui, 8 );
    memcpy( tracker_ctx.join_eui, join_eui, 8 );
    memcpy( tracker_ctx.app_key, app_key, 16 );
    tracker_ctx.use_semtech_join_server = true;

    /*Check if a LoRaWAN region is already existing in the case of Modem-E tracker migration*/
    if( tracker_check_for_existing_region( &existing_region ) == false )
    {
        tracker_ctx.lorawan_region = LORAWAN_REGION;
    }
    else
    {
        tracker_ctx.lorawan_region = existing_region;
    }

    if( ( tracker_ctx.lorawan_region == SMTC_MODEM_REGION_AS_923_GRP1 ) && ACTIVATE_SUB_REGION_JAPAN )
    {
        tracker_ctx.lorawan_sub_region = SMTC_MODEM_SUB_REGION_JAPAN;
    }
    else
    {
        tracker_ctx.lorawan_sub_region = SMTC_MODEM_NO_SUB_REGION;
    }

    /* GNSS Parameters */
    tracker_ctx.gnss_antenna_sel = GNSS_PCB_ANTENNA;
    /* Set default position to Semtech France */
    tracker_ctx.gnss_assistance_position_latitude  = 45.208;
    tracker_ctx.gnss_assistance_position_longitude = 5.781;
    tracker_ctx.gnss_last_almanac_update           = 0;
    tracker_ctx.scan_priority                      = TRACKER_GNSS_PRIORITY;

    /* Application Parameters */
    tracker_ctx.accelerometer_used     = true;
    tracker_ctx.mobile_scan_interval   = MOBILE_SCAN_INTERVAL_DEFAULT;
    tracker_ctx.static_scan_interval   = STATIC_SCAN_INTERVAL_DEFAULT;
    tracker_ctx.airplane_mode          = true;
    tracker_ctx.internal_log_enable    = true;
    tracker_ctx.accumulated_charge_mAh = 0;

    if( store_in_flash == true )
    {
        tracker_store_app_ctx( );
    }
}

void tracker_store_wifi_in_internal_log( wifi_mw_event_data_scan_done_t* wifi_scan_results )
{
    uint8_t  scan_buf[INTERNAL_LOG_SCAN_BUFFER] = { 0 };
    uint16_t index = 3;  // index 0 1 and 2 are reserved for the scan length and the number of elements

    if( tracker_ctx.flash_remaining_space > INTERNAL_LOG_SCAN_BUFFER )
    {
        /* Scan number */
        tracker_ctx.nb_scan++;  // Increase the nb_scan
        scan_buf[index++] = tracker_ctx.nb_scan;
        scan_buf[index++] = tracker_ctx.nb_scan >> 8;

        scan_buf[index++] = TAG_WIFI;
        scan_buf[index++] = WIFI_SINGLE_BEACON_LEN * wifi_scan_results->nbr_results + WIFI_TIMESTAMP_LEN;

        /* Scan Timestamp */
        scan_buf[index++] = wifi_scan_results->timestamp;
        scan_buf[index++] = wifi_scan_results->timestamp >> 8;
        scan_buf[index++] = wifi_scan_results->timestamp >> 16;
        scan_buf[index++] = wifi_scan_results->timestamp >> 24;

        for( uint8_t i = 0; i < wifi_scan_results->nbr_results; i++ )
        {
            scan_buf[index]     = wifi_scan_results->results[i].rssi;
            scan_buf[index + 1] = ( ( wifi_scan_results->results[i].channel & 0x0F ) |
                                    ( ( wifi_scan_results->results[i].type & 0x03 ) << 4 ) );
            memcpy( &scan_buf[index + 2], wifi_scan_results->results[i].mac_address, 6 );
            index += WIFI_SINGLE_BEACON_LEN;
        }

        tracker_store_internal_log( scan_buf, 1, index );
    }
}

void tracker_store_gnss_in_internal_log( const gnss_mw_event_data_scan_done_t* gnss_scan_results )
{
    uint8_t  scan_buf[INTERNAL_LOG_SCAN_BUFFER] = { 0 };
    uint8_t  nb_variable_elements               = 0;
    uint16_t index = 3;  // index 0 1 and 2 are reserved for the scan length and the number of elements

    if( tracker_ctx.flash_remaining_space > INTERNAL_LOG_SCAN_BUFFER )
    {
        /* Scan number */
        tracker_ctx.nb_scan++;  // Increase the nb_scan
        scan_buf[index++] = tracker_ctx.nb_scan;
        scan_buf[index++] = tracker_ctx.nb_scan >> 8;

        if( gnss_scan_results->nb_scans_valid > 0 )
        {
            /* GNSS scan */
            for( uint8_t nav_index = 0; nav_index < gnss_scan_results->nb_scans_valid; nav_index++ )
            {
                if( gnss_scan_results->scans[nav_index].nav_size > 0 )
                {
                    if( tracker_ctx.gnss_antenna_sel == GNSS_PCB_ANTENNA )
                    {
                        scan_buf[index++] = TAG_GNSS_PCB;
                    }
                    else
                    {
                        scan_buf[index++] = TAG_GNSS_PATCH;
                    }

                    scan_buf[index++] = GNSS_TOKEN_LEN + GNSS_NB_SAT_LEN + GNSS_TIMESTAMP_LEN +
                                        GNSS_LAST_SCAN_IN_GROUP_LEN + GNSS_PROFILE_LEN +
                                        gnss_scan_results->scans[nav_index].nav_size;
                    /* Scan Timestamp */
                    scan_buf[index++] = gnss_scan_results->scans[nav_index].timestamp;
                    scan_buf[index++] = gnss_scan_results->scans[nav_index].timestamp >> 8;
                    scan_buf[index++] = gnss_scan_results->scans[nav_index].timestamp >> 16;
                    scan_buf[index++] = gnss_scan_results->scans[nav_index].timestamp >> 24;
                    scan_buf[index++] = gnss_scan_results->token;
                    scan_buf[index++] = gnss_scan_results->scans[nav_index].nb_svs;
                    /* Last scan of the group ? */
                    if( nav_index == ( gnss_scan_results->nb_scans_valid - 1 ) )
                    {
                        scan_buf[index++] = 1;
                    }
                    else
                    {
                        scan_buf[index++] = 0;
                    }
                    scan_buf[index++] = gnss_scan_results->context.mode;

                    memcpy( &scan_buf[index], gnss_scan_results->scans[nav_index].nav,
                            gnss_scan_results->scans[nav_index].nav_size );
                    index += gnss_scan_results->scans[nav_index].nav_size;
                    nb_variable_elements++;
                }
            }
        }
        else
        {
            uint32_t timestamp = apps_modem_common_get_utc_time( );

            if( gnss_scan_results->aiding_position_check_size != 0 )
            {
                scan_buf[index++] = TAG_APC_MSG;

                scan_buf[index++] = GNSS_TIMESTAMP_LEN + gnss_scan_results->aiding_position_check_size;

                /* Scan Timestamp */
                scan_buf[index++] = timestamp;
                scan_buf[index++] = timestamp >> 8;
                scan_buf[index++] = timestamp >> 16;
                scan_buf[index++] = timestamp >> 24;

                memcpy( &scan_buf[index], gnss_scan_results->aiding_position_check_msg,
                        gnss_scan_results->aiding_position_check_size );
                index += gnss_scan_results->aiding_position_check_size;
            }
            else
            {
                if( tracker_ctx.gnss_antenna_sel == GNSS_PCB_ANTENNA )
                {
                    scan_buf[index++] = TAG_GNSS_PCB;
                }
                else
                {
                    scan_buf[index++] = TAG_GNSS_PATCH;
                }

                scan_buf[index++] = GNSS_TOKEN_LEN + GNSS_NB_SAT_LEN + GNSS_TIMESTAMP_LEN +
                                    GNSS_LAST_SCAN_IN_GROUP_LEN + GNSS_PROFILE_LEN;
                /* Scan Timestamp */
                scan_buf[index++] = timestamp;
                scan_buf[index++] = timestamp >> 8;
                scan_buf[index++] = timestamp >> 16;
                scan_buf[index++] = timestamp >> 24;
                scan_buf[index++] = 0;  // Set the token to 0 when no sv detected
                scan_buf[index++] = 0;  // 0 SV detected
                scan_buf[index++] = 1;  // set a default GNSS mode
                scan_buf[index++] = gnss_scan_results->context.mode;
            }

            nb_variable_elements++;
        }

        tracker_store_internal_log( scan_buf, nb_variable_elements, index );
    }
}

void tracker_restore_internal_log( void )
{
    uint8_t   scan_buf[INTERNAL_LOG_SCAN_BUFFER];
    uint16_t  scan_buf_index    = 0;
    uint8_t   nb_elements       = 0;
    uint8_t   nb_elements_index = 0;
    uint8_t   tag_element       = 0;
    uint16_t  scan_len          = 0;
    uint16_t  nb_scan_index     = 1;
    uint16_t  scan_number;
    uint32_t  next_scan_addr = tracker_ctx.flash_addr_start;
    time_t    scan_timestamp = 0;
    struct tm epoch_time;
    uint32_t  job_counter = 0;

    tracker_print_device_settings( );

    while( nb_scan_index <= tracker_ctx.nb_scan )
    {
        /* read the scan lentgh */
        hal_flash_read_buffer( next_scan_addr, scan_buf, 2 );
        scan_len = scan_buf[0];
        scan_len += ( uint32_t ) scan_buf[1] << 8;

        /* read the rest of the scan */
        hal_flash_read_buffer( next_scan_addr + 2, scan_buf, scan_len - 2 );

        nb_elements = scan_buf[scan_buf_index++];

        /* Scan number */
        scan_number = scan_buf[scan_buf_index++];
        scan_number += ( uint16_t ) scan_buf[scan_buf_index++] << 8;

        while( nb_elements_index < nb_elements )
        {
            uint8_t len = 0;
            tag_element = scan_buf[scan_buf_index++];  // get the element
            len         = scan_buf[scan_buf_index++];  // get the size element

            switch( tag_element )
            {
            case TAG_GNSS_PATCH:
            case TAG_GNSS_PCB:
            {
                /* Scan Timestamp */
                scan_timestamp = get_uint32_from_array_at_index_and_inc( scan_buf, &scan_buf_index );
                memcpy( &epoch_time, localtime( &scan_timestamp ), sizeof( struct tm ) );

                uint16_t nav_len = len - GNSS_TOKEN_LEN - GNSS_NB_SAT_LEN - GNSS_TIMESTAMP_LEN -
                                   GNSS_LAST_SCAN_IN_GROUP_LEN - GNSS_PROFILE_LEN;

                uint8_t token     = scan_buf[scan_buf_index++];
                uint8_t nb_sat    = scan_buf[scan_buf_index++];
                uint8_t last_scan = scan_buf[scan_buf_index++];
                uint8_t profile   = scan_buf[scan_buf_index++];

                /* Display Raw NAV Message*/
                HAL_DBG_TRACE_PRINTF( "[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900, epoch_time.tm_mon + 1,
                                      epoch_time.tm_mday, epoch_time.tm_hour, epoch_time.tm_min, epoch_time.tm_sec );
                HAL_DBG_TRACE_PRINTF( "[%d - %d] ", job_counter, tag_element );

                if( nav_len > 0 )
                {
                    HAL_DBG_TRACE_MSG( "01" );
                    for( uint8_t i = 0; i < nav_len; i++ )
                    {
                        HAL_DBG_TRACE_PRINTF( "%02X", scan_buf[scan_buf_index++] );
                    }
                }
                else
                {
                    HAL_DBG_TRACE_MSG( "0007" );
                }
                HAL_DBG_TRACE_PRINTF( ",%d,%d,%d,%d\n", token, last_scan, nb_sat, profile );

                if( last_scan == 1 )
                {
                    job_counter++;
                }
                break;
            }
            case TAG_APC_MSG:
            {
                /* Scan Timestamp */
                scan_timestamp = get_uint32_from_array_at_index_and_inc( scan_buf, &scan_buf_index );
                memcpy( &epoch_time, localtime( &scan_timestamp ), sizeof( struct tm ) );

                uint16_t apc_len = len - GNSS_TIMESTAMP_LEN;

                /* Display Raw NAV Message*/
                HAL_DBG_TRACE_PRINTF( "[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900, epoch_time.tm_mon + 1,
                                      epoch_time.tm_mday, epoch_time.tm_hour, epoch_time.tm_min, epoch_time.tm_sec );
                HAL_DBG_TRACE_PRINTF( "[%d - %d] ", job_counter, tag_element );

                for( uint8_t i = 0; i < apc_len; i++ )
                {
                    HAL_DBG_TRACE_PRINTF( "%02X", scan_buf[scan_buf_index++] );
                }

                HAL_DBG_TRACE_PRINTF( "\n" );

                break;
            }
            case TAG_WIFI:
            {
                int8_t                           wifi_rssi;
                uint8_t                          wifi_data;
                lr11xx_wifi_channel_t            wifi_channel;
                lr11xx_wifi_signal_type_result_t wifi_type;
                char                             wifi_type_char = 'B';

                /* Scan Timestamp */
                scan_timestamp = get_uint32_from_array_at_index_and_inc( scan_buf, &scan_buf_index );
                memcpy( &epoch_time, localtime( &scan_timestamp ), sizeof( struct tm ) );

                for( uint8_t i = 0; i < ( ( len - WIFI_TIMESTAMP_LEN ) / WIFI_SINGLE_BEACON_LEN ); i++ )
                {
                    HAL_DBG_TRACE_PRINTF( "[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900, epoch_time.tm_mon + 1,
                                          epoch_time.tm_mday, epoch_time.tm_hour, epoch_time.tm_min,
                                          epoch_time.tm_sec );
                    HAL_DBG_TRACE_PRINTF( "[%d - %d] ", job_counter, tag_element );

                    wifi_rssi = scan_buf[scan_buf_index++];

                    wifi_data    = scan_buf[scan_buf_index++];
                    wifi_channel = ( lr11xx_wifi_channel_t )( wifi_data & 0x0F );
                    wifi_type    = ( lr11xx_wifi_signal_type_result_t )( ( wifi_data & 0x30 ) >> 4 );

                    switch( wifi_type )
                    {
                    case LR11XX_WIFI_TYPE_RESULT_B:
                        wifi_type_char = 'B';
                        break;
                    case LR11XX_WIFI_TYPE_RESULT_G:
                        wifi_type_char = 'G';
                        break;
                    case LR11XX_WIFI_TYPE_RESULT_N:
                        wifi_type_char = 'N';
                        break;
                    default:
                        break;
                    }

                    /* Display MAC address */
                    for( uint8_t i = 0; i < 5; i++ )
                    {
                        HAL_DBG_TRACE_PRINTF( "%02X:", scan_buf[scan_buf_index++] );
                    }
                    HAL_DBG_TRACE_PRINTF( "%02X,", scan_buf[scan_buf_index++] );

                    /* Display RSSI */
                    HAL_DBG_TRACE_PRINTF( "CHANNEL_%d,TYPE_%c,%d,0,0,0,0\n", wifi_channel, wifi_type_char, wifi_rssi );
                }
                job_counter++;
                break;
            }
            case TAG_NEXT_SCAN:
            {
                next_scan_addr = scan_buf[scan_buf_index++];
                next_scan_addr += ( uint16_t ) scan_buf[scan_buf_index++] << 8;
                next_scan_addr += ( uint32_t ) scan_buf[scan_buf_index++] << 16;
                next_scan_addr += ( uint32_t ) scan_buf[scan_buf_index++] << 24;
                break;
            }
            default:
            {
                scan_buf_index += len;
            }
            break;
            }
            nb_elements_index++;
        }
        nb_scan_index++;
        nb_elements_index = 0;
        scan_buf_index    = 0;  // reset the index;
    }
}

uint8_t tracker_parse_cmd( uint8_t stack_id, uint8_t* payload, uint8_t* buffer_out, const uint8_t buffer_out_len,
                           bool all_commands_enable )
{
    uint8_t nb_elements         = 0;
    uint8_t nb_elements_index   = 0;
    uint8_t payload_index       = 0;
    uint8_t output_buffer_index = 1;
    uint8_t tag                 = 0;
    uint8_t len                 = 0;
    uint8_t res_size            = 0;

    nb_elements = payload[payload_index++];

    buffer_out[0] = 0;  // ensure that byte 0 is set to 0 at the beggining.

    tracker_ctx.ble_cmd_received =
        true;  // Notify the application that ble cmd has been received to reset the connection timeout

    if( nb_elements > 0 )
    {
        while( nb_elements_index < nb_elements )
        {
            tag = payload[payload_index++];
            len = payload[payload_index++];

            if( ( output_buffer_index + 2 + len ) >
                buffer_out_len )  // Avoid buffer overflow, check if the output_buffer_index + TAG + LEN + VALUE is not
                                  // superior to buffer_out_len
            {
                HAL_DBG_TRACE_ERROR(
                    "if command if treated output_buffer_index will be > buffer_out_len, leave the function and send "
                    "buffer_out\n" );
                nb_elements_index = nb_elements;
            }

            switch( tag )
            {
            case GET_TRACKER_TYPE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_TRACKER_TYPE_CMD;
                buffer_out[output_buffer_index++] = GET_TRACKER_TYPE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = 1;  // 1 means Tracker with LBM

                payload_index += GET_TRACKER_TYPE_LEN;
                break;
            }

            case GET_FW_VERSION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_FW_VERSION_CMD;
                buffer_out[output_buffer_index++] = GET_FW_VERSION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = TRACKER_MAJOR_APP_VERSION;
                buffer_out[output_buffer_index++] = TRACKER_MINOR_APP_VERSION;
                buffer_out[output_buffer_index++] = TRACKER_SUB_MINOR_APP_VERSION;

                payload_index += GET_FW_VERSION_LEN;
                break;
            }

            case GET_HW_VERSION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_HW_VERSION_CMD;
                buffer_out[output_buffer_index++] = GET_HW_VERSION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = ( uint8_t )( TRACKER_PCB_HW_NUMBER >> 8 );
                buffer_out[output_buffer_index++] = ( uint8_t ) TRACKER_PCB_HW_NUMBER;
                buffer_out[output_buffer_index++] = TRACKER_MAJOR_PCB_HW_VERSION;
                buffer_out[output_buffer_index++] = TRACKER_MINOR_PCB_HW_VERSION;

                payload_index += GET_HW_VERSION_LEN;
                break;
            }

            case GET_STACK_VERSION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_STACK_VERSION_CMD;
                buffer_out[output_buffer_index++] = GET_STACK_VERSION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_version.major;
                buffer_out[output_buffer_index++] =
                    tracker_ctx.lorawan_version.minor << 4 | tracker_ctx.lorawan_version.patch;

                payload_index += GET_STACK_VERSION_LEN;
                break;
            }

            case GET_MODEM_FIRMWARE_VERSION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_MODEM_FIRMWARE_VERSION_CMD;
                buffer_out[output_buffer_index++] = GET_MODEM_FIRMWARE_VERSION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.firmware_version.major;
                buffer_out[output_buffer_index++] = tracker_ctx.firmware_version.minor;
                buffer_out[output_buffer_index++] = tracker_ctx.firmware_version.patch;

                payload_index += GET_MODEM_FIRMWARE_VERSION_LEN;
                break;
            }

            case GET_LR11XX_FIRMWARE_VERSION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LR11XX_FIRMWARE_VERSION_CMD;
                buffer_out[output_buffer_index++] = GET_LR11XX_FIRMWARE_VERSION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.lr11xx_fw_version.fw >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.lr11xx_fw_version.fw;

                payload_index += GET_LR11XX_FIRMWARE_VERSION_LEN;
                break;
            }

            case GET_MODEM_STATUS_CMD:
            {
                smtc_modem_status_mask_t modem_status;
                smtc_modem_get_status( stack_id, &modem_status );

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_MODEM_STATUS_CMD;
                buffer_out[output_buffer_index++] = GET_MODEM_STATUS_ANSWER_LEN;
                /* Add if the tracker is clock synchronized */
                if( apps_modem_common_get_utc_time( ) != 0 )
                {
                    buffer_out[output_buffer_index++] = 1;
                }
                else
                {
                    buffer_out[output_buffer_index++] = 0;
                }
                buffer_out[output_buffer_index++] = modem_status;

                payload_index += GET_MODEM_STATUS_LEN;
                break;
            }

            case GET_MODEM_DATE_CMD:
            {
                uint32_t date = apps_modem_common_get_utc_time( );

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_MODEM_DATE_CMD;
                buffer_out[output_buffer_index++] = GET_MODEM_DATE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = date >> 24;
                buffer_out[output_buffer_index++] = date >> 16;
                buffer_out[output_buffer_index++] = date >> 8;
                buffer_out[output_buffer_index++] = date;

                payload_index += GET_MODEM_DATE_LEN;
                break;
            }

            case SET_LR11XX_UPDATE_CMD:
            {
                if( all_commands_enable )
                {
                    uint16_t lr11xx_fragment_id;
                    uint8_t  i = 0;

                    lr11xx_fragment_id = ( uint16_t ) payload[payload_index] << 8;
                    lr11xx_fragment_id += payload[payload_index + 1];

                    /* set the date */
                    if( lr11xx_fragment_id == 0 )
                    {
                        tracker_ctx.lorawan_parameters_have_changed = true;
                        chunk_buffer_index                          = 0;
                        lr11xx_flash_offset                         = 0;
                        memset( chunk_buffer, 0, sizeof( chunk_buffer ) );

                        /* Switch in bootloader */
                        smtc_board_set_radio_in_dfu( modem_radio->ral.context );

                        /* Erase Flash */
                        lr11xx_bootloader_erase_flash( modem_radio->ral.context );
                    }

                    for( i = 0; i < len - 2; i += 4 )
                    {
                        /* fill the chunk buffer, note : the + 2 represents the lr11xx_fragment_id len in bytes */
                        chunk_buffer[chunk_buffer_index] = ( uint32_t ) payload[payload_index + 2 + i] << 24;
                        chunk_buffer[chunk_buffer_index] += ( uint32_t ) payload[payload_index + 2 + i + 1] << 16;
                        chunk_buffer[chunk_buffer_index] += ( uint32_t ) payload[payload_index + 2 + i + 2] << 8;
                        chunk_buffer[chunk_buffer_index] += ( uint32_t ) payload[payload_index + 2 + i + 3];
                        chunk_buffer_index++;
                    }

                    if( chunk_buffer_index >= 64 )
                    {
                        uint8_t nb_elem_to_shift = chunk_buffer_index - 64;

                        lr11xx_bootloader_write_flash_encrypted( modem_radio->ral.context, lr11xx_flash_offset,
                                                                 chunk_buffer, 64 );

                        lr11xx_flash_offset += 0x100;

                        /* shift the buffer from 64 */
                        for( i = 0; i < nb_elem_to_shift; i++ )
                        {
                            chunk_buffer[i] = chunk_buffer[i + 64];
                        }

                        chunk_buffer_index -= 64;
                    }

                    if( lr11xx_fragment_id == NB_CHUNK_LR11XX )
                    {
                        lr11xx_system_version_t version;

                        /* Push the rest */
                        lr11xx_bootloader_write_flash_encrypted( modem_radio->ral.context, lr11xx_flash_offset,
                                                                 chunk_buffer, chunk_buffer_index - 1 );

                        lr11xx_hal_reset( modem_radio->ral.context );
                        hal_mcu_delay_ms( 1500 );

                        lr11xx_system_get_version( modem_radio->ral.context, &version );
                        HAL_DBG_TRACE_PRINTF( "LR11XX : hw:%#02X / type:%#02X / fw:%#04X\n", version.hw, version.type,
                                              version.fw );

                        /* new modem version, reset the board */
                        if( tracker_ctx.has_lr11xx_trx_firmware )
                        {
                            tracker_ctx.lorawan_parameters_have_changed = true;
                        }
                    }

                    HAL_DBG_TRACE_PRINTF( "lr11xx_fragment_id %d\n", lr11xx_fragment_id );

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LR11XX_UPDATE_CMD;
                    buffer_out[output_buffer_index++] = SET_LR11XX_UPDATE_ANSWER_LEN;
                    buffer_out[output_buffer_index++] = lr11xx_fragment_id >> 8;
                    buffer_out[output_buffer_index++] = lr11xx_fragment_id;
                }

                payload_index += SET_LR11XX_UPDATE_LEN;
                break;
            }

            case GET_LORAWAN_PIN_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_PIN_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_PIN_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_pin[0];
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_pin[1];
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_pin[2];
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_pin[3];

                payload_index += GET_LORAWAN_PIN_LEN;
                break;
            }

            case SET_LORAWAN_DEVEUI_CMD:
            {
                tracker_ctx.new_value_to_set = true;
                memcpy( tracker_ctx.dev_eui, payload + payload_index, SET_LORAWAN_DEVEUI_LEN );
                tracker_ctx.lorawan_parameters_have_changed = true;

                /* Ack the CMD */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_DEVEUI_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_DEVEUI_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.dev_eui, SET_LORAWAN_DEVEUI_LEN );
                output_buffer_index += SET_LORAWAN_DEVEUI_LEN;

                payload_index += SET_LORAWAN_DEVEUI_LEN;
                break;
            }

            case GET_LORAWAN_DEVEUI_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_DEVEUI_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_DEVEUI_ANSWER_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.dev_eui, GET_LORAWAN_DEVEUI_ANSWER_LEN );
                output_buffer_index += GET_LORAWAN_DEVEUI_ANSWER_LEN;

                payload_index += GET_LORAWAN_DEVEUI_LEN;
                break;
            }

            case GET_LORAWAN_CHIP_EUI_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_CHIP_EUI_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_CHIP_EUI_ANSWER_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.chip_eui, GET_LORAWAN_CHIP_EUI_ANSWER_LEN );
                output_buffer_index += GET_LORAWAN_CHIP_EUI_ANSWER_LEN;

                payload_index += GET_LORAWAN_CHIP_EUI_LEN;
                break;
            }

            case SET_LORAWAN_JOINEUI_CMD:
            {
                tracker_ctx.new_value_to_set = true;
                memcpy( tracker_ctx.join_eui, payload + payload_index, SET_LORAWAN_JOINEUI_LEN );
                tracker_ctx.lorawan_parameters_have_changed = true;

                /* Ack the CMD */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_JOINEUI_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_JOINEUI_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.join_eui, SET_LORAWAN_JOINEUI_LEN );
                output_buffer_index += SET_LORAWAN_JOINEUI_LEN;

                payload_index += SET_LORAWAN_JOINEUI_LEN;
                break;
            }

            case GET_LORAWAN_JOINEUI_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOINEUI_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOINEUI_ANSWER_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.join_eui, GET_LORAWAN_JOINEUI_ANSWER_LEN );
                output_buffer_index += GET_LORAWAN_JOINEUI_ANSWER_LEN;

                payload_index += GET_LORAWAN_JOINEUI_LEN;
                break;
            }

            case SET_LORAWAN_APPKEY_CMD:
            {
                tracker_ctx.new_value_to_set = true;
                memcpy( tracker_ctx.app_key, payload + payload_index, SET_LORAWAN_APPKEY_LEN );
                tracker_ctx.lorawan_parameters_have_changed = true;

                /* Ack the CMD */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_APPKEY_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_APPKEY_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.app_key, SET_LORAWAN_APPKEY_LEN );
                output_buffer_index += SET_LORAWAN_APPKEY_LEN;

                payload_index += SET_LORAWAN_APPKEY_LEN;
                break;
            }

            case GET_LORAWAN_APPKEY_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_APPKEY_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_APPKEY_ANSWER_LEN;
                memcpy( buffer_out + output_buffer_index, tracker_ctx.app_key, GET_LORAWAN_APPKEY_ANSWER_LEN );
                output_buffer_index += GET_LORAWAN_APPKEY_ANSWER_LEN;

                payload_index += GET_LORAWAN_APPKEY_LEN;
                break;
            }

            case GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_CMD:
            {
                uint16_t nb_of_uplinks_before_network_controlled;
                uint16_t nb_of_uplinks_before_reset;
                smtc_modem_connection_timeout_get_current_values( stack_id, &nb_of_uplinks_before_network_controlled,
                                                                  &nb_of_uplinks_before_reset );

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_ANSWER_LEN;
                buffer_out[output_buffer_index++] = nb_of_uplinks_before_reset >> 8;
                buffer_out[output_buffer_index++] = nb_of_uplinks_before_reset;

                payload_index += GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_LEN;
                break;
            }

            case SET_GNSS_CONSTELLATION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_GNSS_CONSTELLATION_CMD;
                buffer_out[output_buffer_index++] = SET_GNSS_CONSTELLATION_LEN;
                buffer_out[output_buffer_index++] = payload[payload_index];

                payload_index += SET_GNSS_CONSTELLATION_LEN;
                break;
            }

            case GET_GNSS_CONSTELLATION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_CONSTELLATION_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_CONSTELLATION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = 3;

                payload_index += GET_GNSS_CONSTELLATION_LEN;
                break;
            }

            case SET_GNSS_ASSISTANCE_POSITION_CMD:
            {
                tracker_ctx.new_value_to_set = true;
                int32_t latitude             = 0;
                int32_t longitude            = 0;
                int32_t latitude_ack         = 0;
                int32_t longitude_ack        = 0;

                /* calcul latitude */
                latitude = ( uint32_t ) payload[payload_index] << 24;
                latitude += ( uint32_t ) payload[payload_index + 1] << 16;
                latitude += ( uint16_t ) payload[payload_index + 2] << 8;
                latitude += payload[payload_index + 3];

                /* calcul longitude */
                longitude = ( uint32_t ) payload[payload_index + 4] << 24;
                longitude += ( uint32_t ) payload[payload_index + 5] << 16;
                longitude += ( uint16_t ) payload[payload_index + 6] << 8;
                longitude += payload[payload_index + 7];

                tracker_ctx.gnss_assistance_position_latitude  = ( float ) latitude / 10000000;
                tracker_ctx.gnss_assistance_position_longitude = ( float ) longitude / 10000000;

                /* Send ACK */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_GNSS_ASSISTANCE_POSITION_CMD;
                buffer_out[output_buffer_index++] = SET_GNSS_ASSISTANCE_POSITION_LEN;

                /* calcul latitude */
                latitude_ack                      = tracker_ctx.gnss_assistance_position_latitude * 10000000;
                buffer_out[output_buffer_index++] = latitude_ack >> 24;
                buffer_out[output_buffer_index++] = latitude_ack >> 16;
                buffer_out[output_buffer_index++] = latitude_ack >> 8;
                buffer_out[output_buffer_index++] = latitude_ack;

                /* calcul longitude */
                longitude_ack                     = tracker_ctx.gnss_assistance_position_longitude * 10000000;
                buffer_out[output_buffer_index++] = longitude_ack >> 24;
                buffer_out[output_buffer_index++] = longitude_ack >> 16;
                buffer_out[output_buffer_index++] = longitude_ack >> 8;
                buffer_out[output_buffer_index++] = longitude_ack;

                payload_index += SET_GNSS_ASSISTANCE_POSITION_LEN;
                break;
            }

            case GET_GNSS_ASSISTANCE_POSITION_CMD:
            {
                int32_t latitude  = 0;
                int32_t longitude = 0;

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_ASSISTANCE_POSITION_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_ASSISTANCE_POSITION_ANSWER_LEN;

                /* calcul latitude */
                latitude                          = tracker_ctx.gnss_assistance_position_latitude * 10000000;
                buffer_out[output_buffer_index++] = latitude >> 24;
                buffer_out[output_buffer_index++] = latitude >> 16;
                buffer_out[output_buffer_index++] = latitude >> 8;
                buffer_out[output_buffer_index++] = latitude;

                /* calcul longitude */
                longitude                         = tracker_ctx.gnss_assistance_position_longitude * 10000000;
                buffer_out[output_buffer_index++] = longitude >> 24;
                buffer_out[output_buffer_index++] = longitude >> 16;
                buffer_out[output_buffer_index++] = longitude >> 8;
                buffer_out[output_buffer_index++] = longitude;

                payload_index += GET_GNSS_ASSISTANCE_POSITION_LEN;
                break;
            }

            case SET_GNSS_ANTENNA_SEL_CMD:
            {
                tracker_ctx.new_value_to_set = true;
                if( ( payload[payload_index] >= 1 ) && ( payload[payload_index] <= 2 ) )
                {
                    tracker_ctx.gnss_antenna_sel = payload[payload_index];
                }
                else
                {
                    if( payload[payload_index] < 1 )
                    {
                        tracker_ctx.gnss_antenna_sel = GNSS_PATCH_ANTENNA;
                    }
                    else
                    {
                        tracker_ctx.gnss_antenna_sel = GNSS_PCB_ANTENNA;
                    }
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_GNSS_ANTENNA_SEL_CMD;
                buffer_out[output_buffer_index++] = SET_GNSS_ANTENNA_SEL_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_antenna_sel;

                payload_index += SET_GNSS_ANTENNA_SEL_LEN;
                break;
            }

            case GET_GNSS_ANTENNA_SEL_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_ANTENNA_SEL_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_ANTENNA_SEL_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_antenna_sel;

                payload_index += GET_GNSS_ANTENNA_SEL_LEN;
                break;
            }

            case GET_GNSS_LAST_ALMANAC_UPDATE_CMD:
            {
                bool     get_almanac_dates_status = false;
                uint32_t oldest_almanac_date      = 0;
                uint32_t newest_almanac_date      = 0;

                /* get the dates form the LR11XX */
                if( tracker_ctx.has_lr11xx_trx_firmware == true )
                {
                    get_almanac_dates_status = smtc_board_get_almanac_dates(
                        modem_radio->ral.context, &oldest_almanac_date, &newest_almanac_date );
                }

                if( get_almanac_dates_status == true )
                {
                    if( oldest_almanac_date > tracker_ctx.gnss_last_almanac_update )
                    {
                        tracker_ctx.gnss_last_almanac_update = oldest_almanac_date;
                    }
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_ALMANAC_UPDATE_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_ALMANAC_UPDATE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_last_almanac_update >> 24;
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_last_almanac_update >> 16;
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_last_almanac_update >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_last_almanac_update;

                payload_index += GET_GNSS_LAST_ALMANAC_UPDATE_LEN;
                break;
            }

            case GNSS_ALMANAC_UPDATE_CMD:
            {
                if( all_commands_enable )
                {
                    uint16_t alamac_fragment_id;
                    alamac_fragment_id = ( uint16_t ) payload[payload_index] << 8;
                    alamac_fragment_id += payload[payload_index + 1];
                    uint8_t         almanac_one_sv_buffer[20];
                    uint32_t        almanac_date;
                    static uint32_t local_almanac_crc = 0;

                    if( tracker_ctx.has_lr11xx_trx_firmware )
                    {
                        for( uint8_t i = 0; i < 3; i++ )
                        {
                            /* note : the + 2 represents the alamac_fragment_id len in bytes */
                            memcpy( almanac_one_sv_buffer,
                                    payload + payload_index + 2 + ( LR11XX_GNSS_SINGLE_ALMANAC_WRITE_SIZE * i ), 20 );
                            lr11xx_gnss_almanac_update( modem_radio->ral.context, almanac_one_sv_buffer, 1 );
                        }
                    }

                    /* set the date */
                    if( alamac_fragment_id == 0 )
                    {
                        /* note : the + 2 represents the alamac_fragment_id len in bytes */
                        almanac_date = ( ( payload[payload_index + 2 + 2] << 8 ) + payload[payload_index + 2 + 1] );
                        /* almanac_date =  Initial GPS date (6 jan 1980) + 24h * 3600s * ( 1024 weeks * 2 (there have
                           been two wrappings) * 7 days + almanac_date ) */
                        almanac_date =
                            ( OFFSET_BETWEEN_GPS_EPOCH_AND_UNIX_EPOCH + 24 * 3600 * ( 2048 * 7 + almanac_date ) );
                        tracker_ctx.gnss_last_almanac_update = almanac_date;

                        local_almanac_crc =
                            ( payload[payload_index + 2 + 6] << 24 ) + ( payload[payload_index + 2 + 5] << 16 ) +
                            ( payload[payload_index + 2 + 4] << 8 ) + ( payload[payload_index + 2 + 3] );
                    }

                    if( alamac_fragment_id == NB_CHUNK_ALMANAC && ( tracker_ctx.has_lr11xx_trx_firmware == true ) )
                    {
                        lr11xx_gnss_context_status_bytestream_t context_status_bytestream;
                        lr11xx_gnss_context_status_t            context_status;

                        if( lr11xx_gnss_get_context_status( modem_radio->ral.context, context_status_bytestream ) !=
                            LR11XX_STATUS_OK )
                        {
                            HAL_DBG_TRACE_ERROR( "Failed to get gnss context status\n" );
                        }
                        else
                        {
                            // parse context bytestream to get almanac crc
                            lr11xx_gnss_parse_context_status_buffer( context_status_bytestream, &context_status );

                            // check almanac crc again to decide if the update is successful or not
                            if( context_status.global_almanac_crc != local_almanac_crc )
                            {
                                HAL_DBG_TRACE_ERROR( "Local almanac doesn't match LR11XX almanac -> update fails\n" );
                                /* reset the date in case of wrong almanac update */
                                tracker_ctx.gnss_last_almanac_update = 0;
                            }
                            else
                            {
                                HAL_DBG_TRACE_INFO( "Almanac update succeeds\n" );
                                /* store the new almanac update date just once */
                                tracker_ctx.new_value_to_set = true;
                            }
                        }
                    }

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = GNSS_ALMANAC_UPDATE_CMD;
                    buffer_out[output_buffer_index++] = GNSS_ALMANAC_UPDATE_LEN;
                    buffer_out[output_buffer_index++] = alamac_fragment_id >> 8;
                    buffer_out[output_buffer_index++] = alamac_fragment_id;

                    /* note : the + 2 represents the alamac_fragment_id len in bytes */
                    memcpy( buffer_out + output_buffer_index, payload + payload_index + 2, 60 );
                    output_buffer_index += 60;
                }

                payload_index += GNSS_ALMANAC_UPDATE_LEN;
                break;
            }

            case GET_GNSS_LAST_NB_SV_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_NB_SV_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_NB_SV_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.last_nb_detected_satellites;

                payload_index += GET_GNSS_LAST_NB_SV_LEN;
                break;
            }

            case SET_USE_ACCELEROMETER_CMD:
            {
                tracker_ctx.new_value_to_set = true;
                if( payload[payload_index] <= 1 )
                {
                    tracker_ctx.accelerometer_used = payload[payload_index];
                }
                else
                {
                    /* Clip the value */
                    tracker_ctx.accelerometer_used = 1;
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_USE_ACCELEROMETER_CMD;
                buffer_out[output_buffer_index++] = SET_USE_ACCELEROMETER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.accelerometer_used;

                payload_index += SET_USE_ACCELEROMETER_LEN;
                break;
            }

            case GET_USE_ACCELEROMETER_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_USE_ACCELEROMETER_CMD;
                buffer_out[output_buffer_index++] = GET_USE_ACCELEROMETER_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.accelerometer_used;

                payload_index += GET_USE_ACCELEROMETER_LEN;
                break;
            }

            case SET_APP_MOBILE_SCAN_INTERVAL_CMD:
            {
                uint16_t mobile_scan_interval = 0;

                tracker_ctx.new_value_to_set = true;

                mobile_scan_interval = ( uint16_t ) payload[payload_index] << 8;
                mobile_scan_interval += payload[payload_index + 1];

                if( ( mobile_scan_interval >= MOBILE_SCAN_INTERVAL_MIN ) &&
                    ( mobile_scan_interval <= MOBILE_SCAN_INTERVAL_MAX ) )
                {
                    tracker_ctx.mobile_scan_interval = mobile_scan_interval;
                }
                else
                {
                    /* Clip the value */
                    if( payload[payload_index] < MOBILE_SCAN_INTERVAL_MIN )
                    {
                        tracker_ctx.mobile_scan_interval = MOBILE_SCAN_INTERVAL_MIN;
                    }
                    else
                    {
                        tracker_ctx.mobile_scan_interval = MOBILE_SCAN_INTERVAL_MAX;
                    }
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_APP_MOBILE_SCAN_INTERVAL_CMD;
                buffer_out[output_buffer_index++] = SET_APP_MOBILE_SCAN_INTERVAL_LEN;
                buffer_out[output_buffer_index++] = ( tracker_ctx.mobile_scan_interval ) >> 8;
                buffer_out[output_buffer_index++] = ( tracker_ctx.mobile_scan_interval );

                payload_index += SET_APP_MOBILE_SCAN_INTERVAL_LEN;
                break;
            }

            case GET_APP_MOBILE_SCAN_INTERVAL_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_MOBILE_SCAN_INTERVAL_CMD;
                buffer_out[output_buffer_index++] = GET_APP_MOBILE_SCAN_INTERVAL_ANSWER_LEN;
                buffer_out[output_buffer_index++] = ( tracker_ctx.mobile_scan_interval ) >> 8;
                buffer_out[output_buffer_index++] = ( tracker_ctx.mobile_scan_interval );

                payload_index += GET_APP_MOBILE_SCAN_INTERVAL_LEN;
                break;
            }

            case SET_APP_STATIC_SCAN_INTERVAL_CMD:
            {
                uint16_t static_scan_interval = 0;

                tracker_ctx.new_value_to_set = true;

                static_scan_interval = ( uint16_t ) payload[payload_index] << 8;
                static_scan_interval += payload[payload_index + 1];
                static_scan_interval *= 60;

                if( ( static_scan_interval >= STATIC_SCAN_INTERVAL_MIN ) &&
                    ( static_scan_interval <= STATIC_SCAN_INTERVAL_MAX ) )
                {
                    tracker_ctx.static_scan_interval = static_scan_interval;
                }
                else
                {
                    /* Clip the value */
                    if( payload[payload_index] < STATIC_SCAN_INTERVAL_MIN )
                    {
                        tracker_ctx.static_scan_interval = STATIC_SCAN_INTERVAL_MIN;
                    }
                    else
                    {
                        tracker_ctx.static_scan_interval = STATIC_SCAN_INTERVAL_MAX;
                    }
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_APP_STATIC_SCAN_INTERVAL_CMD;
                buffer_out[output_buffer_index++] = SET_APP_STATIC_SCAN_INTERVAL_LEN;
                buffer_out[output_buffer_index++] = ( ( tracker_ctx.static_scan_interval / 60 ) >> 8 );
                buffer_out[output_buffer_index++] = ( tracker_ctx.static_scan_interval / 60 );

                payload_index += SET_APP_STATIC_SCAN_INTERVAL_LEN;
                break;
            }

            case GET_APP_STATIC_SCAN_INTERVAL_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_STATIC_SCAN_INTERVAL_CMD;
                buffer_out[output_buffer_index++] = GET_APP_STATIC_SCAN_INTERVAL_ANSWER_LEN;
                buffer_out[output_buffer_index++] = ( ( tracker_ctx.static_scan_interval / 60 ) >> 8 );
                buffer_out[output_buffer_index++] = ( tracker_ctx.static_scan_interval / 60 );

                payload_index += GET_APP_STATIC_SCAN_INTERVAL_LEN;
                break;
            }

            case SET_LORAWAN_REGION_CMD:
            {
                if( ( payload[payload_index] >= 1 ) && ( payload[payload_index] <= 12 ) )
                {
                    tracker_ctx.new_value_to_set                = true;
                    tracker_ctx.lorawan_region                  = ( smtc_modem_region_t ) payload[payload_index];
                    tracker_ctx.lorawan_parameters_have_changed = true;
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_REGION_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_REGION_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_region;

                payload_index += SET_LORAWAN_REGION_LEN;
                break;
            }

            case GET_LORAWAN_REGION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_REGION_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_REGION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_region;

                payload_index += GET_LORAWAN_REGION_LEN;
                break;
            }

            case SET_LORAWAN_SUB_REGION_CMD:
            {
                if( payload[payload_index] <= 1 )
                {
                    tracker_ctx.new_value_to_set                = true;
                    tracker_ctx.lorawan_sub_region              = ( smtc_modem_sub_region_t ) payload[payload_index];
                    tracker_ctx.lorawan_parameters_have_changed = true;
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_SUB_REGION_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_SUB_REGION_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_sub_region;

                payload_index += SET_LORAWAN_SUB_REGION_LEN;
                break;
            }

            case GET_LORAWAN_SUB_REGION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_SUB_REGION_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_SUB_REGION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.lorawan_sub_region;

                payload_index += GET_LORAWAN_SUB_REGION_LEN;
                break;
            }

            case SET_LORAWAN_JOIN_SERVER_CMD:
            {
                if( payload[payload_index] <= 1 )
                {
                    tracker_ctx.new_value_to_set                = true;
                    tracker_ctx.use_semtech_join_server         = payload[payload_index];
                    tracker_ctx.lorawan_parameters_have_changed = true;
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_JOIN_SERVER_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_JOIN_SERVER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.use_semtech_join_server;

                payload_index += SET_LORAWAN_JOIN_SERVER_LEN;
                break;
            }

            case GET_LORAWAN_JOIN_SERVER_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOIN_SERVER_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOIN_SERVER_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.use_semtech_join_server;

                payload_index += GET_LORAWAN_JOIN_SERVER_LEN;
                break;
            }

            case SET_AIRPLANE_MODE_CMD:
            {
                if( all_commands_enable )
                {
                    tracker_ctx.new_value_to_set = true;

                    if( payload[payload_index] <= 1 )
                    {
                        if( tracker_ctx.airplane_mode != payload[payload_index] )
                        {
                            /* If the airplane mode changes reset the tracker */
                            tracker_ctx.lorawan_parameters_have_changed = true;

                            tracker_ctx.airplane_mode = payload[payload_index];
                        }
                    }
                    else
                    {
                        /* Clip the value */
                        tracker_ctx.airplane_mode = 0;
                    }
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_AIRPLANE_MODE_CMD;
                buffer_out[output_buffer_index++] = SET_AIRPLANE_MODE_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.airplane_mode;

                payload_index += SET_AIRPLANE_MODE_LEN;
                break;
            }

            case GET_AIRPLANE_MODE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_AIRPLANE_MODE_CMD;
                buffer_out[output_buffer_index++] = GET_AIRPLANE_MODE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.airplane_mode;

                payload_index += GET_AIRPLANE_MODE_LEN;
                break;
            }

            case SET_SCAN_PRIORITY_CMD:
            {
                tracker_ctx.new_value_to_set = true;
                if( payload[payload_index] <= TRACKER_NO_PRIORITY )
                {
                    tracker_ctx.scan_priority = ( tracker_scan_priority_t ) payload[payload_index];
                }
                else
                {
                    /* Clip the value */
                    tracker_ctx.scan_priority = TRACKER_GNSS_PRIORITY;
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_SCAN_PRIORITY_CMD;
                buffer_out[output_buffer_index++] = SET_SCAN_PRIORITY_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.airplane_mode;

                payload_index += SET_SCAN_PRIORITY_LEN;
                break;
            }

            case GET_SCAN_PRIORITY_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_SCAN_PRIORITY_CMD;
                buffer_out[output_buffer_index++] = GET_SCAN_PRIORITY_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.scan_priority;

                payload_index += GET_SCAN_PRIORITY_LEN;
                break;
            }

            case GET_BOARD_VOLTAGE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_BOARD_VOLTAGE_CMD;
                buffer_out[output_buffer_index++] = GET_BOARD_VOLTAGE_ANSWER_LEN;
                tracker_ctx.voltage               = smtc_modem_hal_get_voltage( ) * 20;
                buffer_out[output_buffer_index++] = tracker_ctx.voltage >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.voltage;

                payload_index += GET_BOARD_VOLTAGE_LEN;
                break;
            }

            case GET_APP_ACCUMULATED_CHARGE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_ACCUMULATED_CHARGE_CMD;
                buffer_out[output_buffer_index++] = GET_APP_ACCUMULATED_CHARGE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.accumulated_charge_mAh >> 24;
                buffer_out[output_buffer_index++] = tracker_ctx.accumulated_charge_mAh >> 16;
                buffer_out[output_buffer_index++] = tracker_ctx.accumulated_charge_mAh >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.accumulated_charge_mAh;

                payload_index += GET_APP_ACCUMULATED_CHARGE_LEN;
                break;
            }

            case RESET_APP_ACCUMULATED_CHARGE_CMD:
            {
                tracker_ctx.new_value_to_set       = true;
                tracker_ctx.accumulated_charge_mAh = 0;

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = RESET_APP_ACCUMULATED_CHARGE_CMD;
                buffer_out[output_buffer_index++] = RESET_APP_ACCUMULATED_CHARGE_LEN;

                payload_index += RESET_APP_ACCUMULATED_CHARGE_LEN;
                break;
            }

            case GET_APP_SYSTEM_SANITY_CHECK_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_SYSTEM_SANITY_CHECK_CMD;
                buffer_out[output_buffer_index++] = GET_APP_SYSTEM_SANITY_CHECK_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.system_sanity_check;

                payload_index += GET_APP_SYSTEM_SANITY_CHECK_LEN;
                break;
            }

            case SET_APP_INTERNAL_LOG_CMD:
            {
                tracker_ctx.new_value_to_set = true;
                if( payload[payload_index] <= 1 )
                {
                    tracker_ctx.internal_log_enable = payload[payload_index];
                }
                else
                {
                    /* Clip the value */
                    tracker_ctx.internal_log_enable = 0;
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_APP_INTERNAL_LOG_CMD;
                buffer_out[output_buffer_index++] = SET_APP_INTERNAL_LOG_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.internal_log_enable;

                payload_index += SET_USE_ACCELEROMETER_LEN;
                break;
            }

            case GET_APP_INTERNAL_LOG_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_INTERNAL_LOG_CMD;
                buffer_out[output_buffer_index++] = GET_APP_INTERNAL_LOG_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_ctx.internal_log_enable;

                payload_index += GET_APP_INTERNAL_LOG_LEN;
                break;
            }

            case GET_APP_INTERNAL_LOG_REMANING_SPACE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_INTERNAL_LOG_REMANING_SPACE_CMD;
                buffer_out[output_buffer_index++] = GET_APP_INTERNAL_LOG_REMANING_SPACE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_get_remaining_memory_space( );

                payload_index += GET_APP_INTERNAL_LOG_REMANING_SPACE_LEN;
                break;
            }

            case READ_APP_INTERNAL_LOG_CMD:
            {
                if( all_commands_enable )
                {
                    uint8_t        answer_len = 0;
                    static uint8_t internal_buffer[CHUNK_INTERNAL_LOG];

                    /* Send the device settings */
                    if( internal_log_tracker_settings_sent == false )
                    {
                        if( internal_log_buffer_len == 0 )
                        {
                            tracker_get_device_settings( internal_log_buffer, INTERNAL_LOG_BUFFER_LEN,
                                                         &internal_log_buffer_len );
                        }

                        if( internal_log_buffer_len > CHUNK_INTERNAL_LOG )
                        {
                            memcpy( internal_buffer, internal_log_buffer, CHUNK_INTERNAL_LOG );

                            /* shift the buffer from internal_log_buffer_len */
                            for( uint16_t i = 0; i < ( internal_log_buffer_len - CHUNK_INTERNAL_LOG ); i++ )
                            {
                                internal_log_buffer[i] = internal_log_buffer[i + CHUNK_INTERNAL_LOG];
                            }

                            answer_len = CHUNK_INTERNAL_LOG + 1;

                            internal_log_buffer_len -= CHUNK_INTERNAL_LOG;
                        }
                        else
                        {
                            memcpy( internal_buffer, internal_log_buffer, internal_log_buffer_len );

                            answer_len = internal_log_buffer_len + 1;

                            internal_log_buffer_len            = 0;
                            internal_log_tracker_settings_sent = true;
                        }
                    }
                    else
                    {
                        /* Send the internal log */
                        if( internal_log_scan_index <= tracker_ctx.nb_scan )
                        {
                            if( internal_log_buffer_len == 0 )
                            {
                                tracker_get_one_scan_from_internal_log( internal_log_scan_index, internal_log_buffer,
                                                                        INTERNAL_LOG_BUFFER_LEN,
                                                                        &internal_log_buffer_len );
                            }

                            if( internal_log_buffer_len > CHUNK_INTERNAL_LOG )
                            {
                                memcpy( internal_buffer, internal_log_buffer, CHUNK_INTERNAL_LOG );

                                /* shift the buffer from internal_log_buffer_len */
                                for( uint16_t i = 0; i < ( internal_log_buffer_len - CHUNK_INTERNAL_LOG ); i++ )
                                {
                                    internal_log_buffer[i] = internal_log_buffer[i + CHUNK_INTERNAL_LOG];
                                }

                                answer_len = CHUNK_INTERNAL_LOG + 1;

                                internal_log_buffer_len -= CHUNK_INTERNAL_LOG;
                            }
                            else
                            {
                                memcpy( internal_buffer, internal_log_buffer, internal_log_buffer_len );

                                answer_len = internal_log_buffer_len + 1;

                                internal_log_buffer_len = 0;
                                internal_log_scan_index++;
                            }
                        }
                        else
                        {
                            internal_log_scan_index            = 1;
                            internal_log_tracker_settings_sent = false;
                        }
                    }

                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = READ_APP_INTERNAL_LOG_CMD;
                    buffer_out[output_buffer_index++] = answer_len;

                    if( answer_len > 0 )
                    {
                        buffer_out[output_buffer_index++] =
                            ( ( internal_log_scan_index - 1 ) * 100 ) / tracker_ctx.nb_scan;
                        memcpy( buffer_out + output_buffer_index, internal_buffer, answer_len - 1 );

                        output_buffer_index += answer_len - 1;
                    }
                }

                payload_index += READ_APP_INTERNAL_LOG_LEN;
                break;
            }

            case SET_APP_FLUSH_INTERNAL_LOG_CMD:
            {
                tracker_ctx.internal_log_flush_request = true;

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_APP_FLUSH_INTERNAL_LOG_CMD;
                buffer_out[output_buffer_index++] = SET_APP_FLUSH_INTERNAL_LOG_LEN;

                payload_index += SET_APP_FLUSH_INTERNAL_LOG_LEN;

                break;
            }

            case GET_APP_TRACKER_SETTINGS_CMD:
            {
                uint32_t oldest_almanac_date = 0;
                uint32_t newest_almanac_date = 0;
                uint16_t nb_of_uplinks_before_network_controlled;
                uint16_t nb_of_uplinks_before_reset;

                /* get the dates form the LR11XX */
                smtc_board_get_almanac_dates( modem_radio->ral.context, &oldest_almanac_date, &newest_almanac_date );

                smtc_modem_connection_timeout_get_current_values( stack_id, &nb_of_uplinks_before_network_controlled,
                                                                  &nb_of_uplinks_before_reset );

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_TRACKER_SETTINGS_CMD;
                buffer_out[output_buffer_index++] = GET_APP_TRACKER_SETTINGS_ANSWER_LEN;

                /* All settings pattern version */
                buffer_out[output_buffer_index++] = GET_APP_TRACKER_SETTINGS_VERSION;
                /* firmware version */
                buffer_out[output_buffer_index++] = tracker_ctx.firmware_version.major;
                buffer_out[output_buffer_index++] = tracker_ctx.firmware_version.minor;
                buffer_out[output_buffer_index++] = tracker_ctx.firmware_version.patch;
                /* LoRaWAN Join Sever usage */
                buffer_out[output_buffer_index++] = tracker_ctx.use_semtech_join_server;
                /* GNSS Antenna sel */
                buffer_out[output_buffer_index++] = tracker_ctx.gnss_antenna_sel;
                /* GNSS Assistance position */
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_assistance_position_latitude * 10000000 ) ) >> 24;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_assistance_position_latitude * 10000000 ) ) >> 16;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_assistance_position_latitude * 10000000 ) ) >> 8;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_assistance_position_latitude * 10000000 ) );
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_assistance_position_longitude * 10000000 ) ) >> 24;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_assistance_position_longitude * 10000000 ) ) >> 16;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_assistance_position_longitude * 10000000 ) ) >> 8;
                buffer_out[output_buffer_index++] =
                    ( ( int32_t )( tracker_ctx.gnss_assistance_position_longitude * 10000000 ) );
                /* GNSS oldest almanac */
                buffer_out[output_buffer_index++] = oldest_almanac_date >> 24;
                buffer_out[output_buffer_index++] = oldest_almanac_date >> 16;
                buffer_out[output_buffer_index++] = oldest_almanac_date >> 8;
                buffer_out[output_buffer_index++] = oldest_almanac_date;
                /* GNSS newest almanac */
                buffer_out[output_buffer_index++] = newest_almanac_date >> 24;
                buffer_out[output_buffer_index++] = newest_almanac_date >> 16;
                buffer_out[output_buffer_index++] = newest_almanac_date >> 8;
                buffer_out[output_buffer_index++] = newest_almanac_date;
                /* Scan priority */
                buffer_out[output_buffer_index++] = tracker_ctx.scan_priority;
                /* App accelerometer used */
                buffer_out[output_buffer_index++] = tracker_ctx.accelerometer_used;
                /* App scan interval */
                buffer_out[output_buffer_index++] = tracker_ctx.mobile_scan_interval >> 8;
                buffer_out[output_buffer_index++] = tracker_ctx.mobile_scan_interval;
                /* App keep alive interval */
                buffer_out[output_buffer_index++] = ( ( tracker_ctx.static_scan_interval / 60 ) >> 8 );
                buffer_out[output_buffer_index++] = ( tracker_ctx.static_scan_interval / 60 );
                /* App internal log enable */
                buffer_out[output_buffer_index++] = tracker_ctx.internal_log_enable;
                /* App internal log remaining memory space */
                buffer_out[output_buffer_index++] = tracker_get_remaining_memory_space( );
                /* Nb of uplinks before reset */
                buffer_out[output_buffer_index++] = nb_of_uplinks_before_reset >> 8;
                buffer_out[output_buffer_index++] = nb_of_uplinks_before_reset;
                /* Last sv number detected */
                buffer_out[output_buffer_index++] = tracker_ctx.last_nb_detected_satellites;
                /* Last mac address number detected */
                buffer_out[output_buffer_index++] = tracker_ctx.last_nb_detected_mac_address;
                /* Sanity check bit mask */
                buffer_out[output_buffer_index++] = tracker_ctx.system_sanity_check;

                payload_index += GET_APP_TRACKER_SETTINGS_LEN;

                break;
            }

            case SET_APP_RESET_CMD:
            {
                tracker_ctx.reset_board_asked = true;

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_APP_RESET_CMD;
                buffer_out[output_buffer_index++] = SET_APP_RESET_LEN;

                payload_index += SET_APP_RESET_LEN;

                break;
            }

            default:
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = NOT_SUPPORTED_COMMAND_CMD;
                buffer_out[output_buffer_index++] = NOT_SUPPORTED_COMMAND_LEN;
                buffer_out[output_buffer_index++] = tag;

                payload_index += len;

                break;
            }

            nb_elements_index++;
        }
    }

    if( output_buffer_index > 1 )  // if > 1 it means there is something to send
    {
        res_size = output_buffer_index;
    }

    return res_size;
}

uint8_t tracker_get_battery_level( void )
{
    if( tracker_ctx.accumulated_charge_mAh > ( TRACKER_BOARD_BATTERY_CAPACITY * 0.8 ) )
    {
        return 0;
    }
    else
    {
        return ( 100 - ( ( tracker_ctx.accumulated_charge_mAh / ( TRACKER_BOARD_BATTERY_CAPACITY * 0.8 ) ) * 100 ) );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool tracker_check_for_existing_region( uint8_t* existing_region )
{
    uint8_t tracker_modem_ctx_exist = FLASH_BYTE_EMPTY_CONTENT;
    bool    has_existing_region     = false;

    hal_flash_read_buffer( FLASH_USER_MODEME_TRACKER_CTX_START_ADDR, &tracker_modem_ctx_exist, 1 );

    if( tracker_modem_ctx_exist != FLASH_BYTE_EMPTY_CONTENT )
    {
        hal_flash_read_buffer( FLASH_USER_MODEME_TRACKER_CTX_START_ADDR + FLASH_USER_MODEME_TRACKER_REGION_OFFSET,
                               existing_region, 1 );

        has_existing_region = true;
    }
    return has_existing_region;
}

static void tracker_print_device_settings( void )
{
    HAL_DBG_TRACE_MSG( "\n#Tracker Settings :\n" );

    HAL_DBG_TRACE_MSG( "#LoRaWAN Settings :\n" );
    HAL_DBG_TRACE_PRINTF( "#\tLORA BASIC MODEM VERSION : LORAWAN : %#02X%#02X%#02X | FIRMWARE : %#02X%#02X%#02X \n",
                          tracker_ctx.lorawan_version.major, tracker_ctx.lorawan_version.minor,
                          tracker_ctx.lorawan_version.patch, tracker_ctx.firmware_version.major,
                          tracker_ctx.firmware_version.minor, tracker_ctx.firmware_version.patch );

    /* Device EUI */
    HAL_DBG_TRACE_PRINTF( "#\tDevice Eui  : %02X", tracker_ctx.dev_eui[0] );
    for( int i = 1; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "-%02X", tracker_ctx.dev_eui[i] );
    }
    HAL_DBG_TRACE_MSG( "\n" );

    /* Join EUI */
    HAL_DBG_TRACE_PRINTF( "#\tAppEui      : %02X", tracker_ctx.join_eui[0] );
    for( int i = 1; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "-%02X", tracker_ctx.join_eui[i] );
    }
    HAL_DBG_TRACE_PRINTF( "\n" );

    /* AppKey / Semtech JS */
    if( tracker_ctx.use_semtech_join_server )
    {
        HAL_DBG_TRACE_MSG( "#\tAppKey      : Semtech join server used\n" );
    }
    else
    {
        HAL_DBG_TRACE_PRINTF( "#\tAppKey      : %02X", tracker_ctx.app_key[0] );
        for( int i = 1; i < 16; i++ )
        {
            HAL_DBG_TRACE_PRINTF( "-%02X", tracker_ctx.app_key[i] );
        }
        HAL_DBG_TRACE_PRINTF( "\n" );
    }

    /* LoRaWAN settings */
    HAL_DBG_TRACE_PRINTF( "#\tlorawan_region : %d\n", tracker_ctx.lorawan_region );
    HAL_DBG_TRACE_PRINTF( "#\tlorawan_sub_region : %d\n", tracker_ctx.lorawan_sub_region );

    /* GNSS settings */
    HAL_DBG_TRACE_MSG( "#GNSS Settings:\n" );
    HAL_DBG_TRACE_PRINTF( "#\tAntenna sel : %d\n", tracker_ctx.gnss_antenna_sel );
    HAL_DBG_TRACE_PRINTF( "#\tassistance position latitude : %f\n", tracker_ctx.gnss_assistance_position_latitude );
    HAL_DBG_TRACE_PRINTF( "#\tassistance position longitude : %f\n", tracker_ctx.gnss_assistance_position_longitude );
    HAL_DBG_TRACE_PRINTF( "#\tlast almanac update : %d\n", tracker_ctx.gnss_last_almanac_update );

    /* Application settings */
    HAL_DBG_TRACE_MSG( "#Application settings:\n" );
    HAL_DBG_TRACE_PRINTF( "#\taccelerometer_used : %d\n", tracker_ctx.accelerometer_used );
    HAL_DBG_TRACE_PRINTF( "#\ttracker_ctx.accumulated_charge : %d mAh\n", tracker_ctx.accumulated_charge_mAh );
    HAL_DBG_TRACE_PRINTF( "#\tmobile_scan_interval : %d s\n", tracker_ctx.mobile_scan_interval );
    HAL_DBG_TRACE_PRINTF( "#\tstatic_scan_interval : %d min\n", tracker_ctx.static_scan_interval / 60 );
    HAL_DBG_TRACE_PRINTF( "#\tscan_priority : %d\n", tracker_ctx.scan_priority );
    HAL_DBG_TRACE_PRINTF( "#\tairplane_mode : %d\n", tracker_ctx.airplane_mode );
    HAL_DBG_TRACE_PRINTF( "#\tinternal_log_enable : %d\n", tracker_ctx.internal_log_enable );
    HAL_DBG_TRACE_PRINTF( "#\ttracker_fw_version : %d.%d.%d\n", TRACKER_MAJOR_APP_VERSION, TRACKER_MINOR_APP_VERSION,
                          TRACKER_SUB_MINOR_APP_VERSION );
}

static void tracker_get_device_settings( uint8_t* out_buffer, const uint16_t out_buffer_len, uint16_t* buffer_len )
{
    *buffer_len = 0;

    /* LoRaWAN settings */
    *buffer_len +=
        snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "#LoRaWAN Settings :\n" );

    *buffer_len += snprintf(
        ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
        "#\tLORA BASIC MODEM VERSION : LORAWAN : %#02X%#02X%#02X  | FIRMWARE : %#02X%#02X%#02X  \n",
        tracker_ctx.lorawan_version.major, tracker_ctx.lorawan_version.minor, tracker_ctx.lorawan_version.patch,
        tracker_ctx.firmware_version.major, tracker_ctx.firmware_version.minor, tracker_ctx.firmware_version.patch );

    /* Device EUI */
    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "#\tDevEui : %02X",
                             tracker_ctx.dev_eui[0] );

    for( int i = 1; i < 8; i++ )
    {
        *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "-%02X",
                                 tracker_ctx.dev_eui[i] );
    }

    /* Join EUI */
    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "\r\n#\tAppEui : %02X", tracker_ctx.join_eui[0] );

    for( int i = 1; i < 8; i++ )
    {
        *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "-%02X",
                                 tracker_ctx.join_eui[i] );
    }

    /* AppKey / Semtech JS */
    if( tracker_ctx.use_semtech_join_server )
    {
        *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                 "\r\n#\tAppKey : Semtech join server used" );
    }
    else
    {
        *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                 "\r\n#\tAppKey : %02X", tracker_ctx.app_key[0] );

        for( int i = 1; i < 16; i++ )
        {
            *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "-%02X",
                                     tracker_ctx.app_key[i] );
        }
    }

    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "\r\n#\tlorawan_region : %d\r\n", tracker_ctx.lorawan_region );

    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "\r\n#\tlorawan_sub_region : %d\r\n", tracker_ctx.lorawan_sub_region );

    /* GNSS settings */
    *buffer_len +=
        snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "#GNSS Settings:\r\n" );

    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "#\tAntenna sel : %d\r\n", tracker_ctx.gnss_antenna_sel );

    *buffer_len +=
        snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                  "#\tassistance position latitude : %f\r\n", tracker_ctx.gnss_assistance_position_latitude );

    *buffer_len +=
        snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                  "#\tassistance position longitude : %f\r\n", tracker_ctx.gnss_assistance_position_longitude );

    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "#\tlast almanac update : %ld\r\n", tracker_ctx.gnss_last_almanac_update );

    /* Application settings */
    *buffer_len +=
        snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "#Application settings:\r\n" );

    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "#\taccelerometer_used : %d\r\n", tracker_ctx.accelerometer_used );

    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "#\taccumulated_charge : %ld mAh\r\n", tracker_ctx.accumulated_charge_mAh );

    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "#\tmobile_scan_interval : %ld s\r\n", tracker_ctx.mobile_scan_interval );

    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "#\tstatic_scan_interval : %ld min\r\n", tracker_ctx.static_scan_interval / 60 );

    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "#\tairplane_mode : %d\r\n", tracker_ctx.airplane_mode );

    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "#\tscan_priority : %d\r\n", tracker_ctx.scan_priority );

    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                             "#\ttracker_fw_version : %d.%d.%d\r\n", TRACKER_MAJOR_APP_VERSION,
                             TRACKER_MINOR_APP_VERSION, TRACKER_SUB_MINOR_APP_VERSION );
}

static uint32_t get_uint32_from_array_at_index_and_inc( const uint8_t* array, uint16_t* index )
{
    const uint16_t local_index = *index;
    const uint32_t value = array[local_index] + ( array[local_index + 1] << 8 ) + ( array[local_index + 2] << 16 ) +
                           ( array[local_index + 3] << 24 );
    *index = local_index + 4;
    return value;
}

static void tracker_store_internal_log( uint8_t* buffer, uint8_t nb_variable_elements, uint16_t len )
{
    uint16_t index_next_addr = 0;
    uint32_t next_scan_addr  = 0;
    uint8_t  read_buffer[256];
    bool     memory_flash_is_empty = true;

    /* Next scan addr */
    buffer[len++] = TAG_NEXT_SCAN;
    buffer[len++] = 4;
    /* Complete index for FLASH_TYPEPROGRAM_DOUBLEWORD operation and define the next addr */
    index_next_addr = len;
    len += 4;
    if( ( len % 8 ) != 0 )  // 4: anticipate the buffer increment
    {
        len = len + ( 8 - ( len % 8 ) );
    }
    next_scan_addr              = tracker_ctx.flash_addr_current + len;
    buffer[index_next_addr]     = next_scan_addr;
    buffer[index_next_addr + 1] = next_scan_addr >> 8;
    buffer[index_next_addr + 2] = next_scan_addr >> 16;
    buffer[index_next_addr + 3] = next_scan_addr >> 24;

    /* Scan Len */
    buffer[0] = len;
    buffer[1] = len >> 8;

    /* nb elements */
    buffer[2] = nb_variable_elements + 1;  // +1 because of the next address scan

    hal_flash_read_buffer( tracker_ctx.flash_addr_current, read_buffer, len );
    for( uint8_t i = 0; i < len; i++ )
    {
        if( read_buffer[i] != FLASH_BYTE_EMPTY_CONTENT )
        {
            memory_flash_is_empty = false;
        }
    }

    if( memory_flash_is_empty == true )
    {
        hal_flash_write_buffer( tracker_ctx.flash_addr_current, buffer, len );
        tracker_ctx.flash_addr_current = next_scan_addr;
        tracker_store_internal_log_ctx( );

        HAL_DBG_TRACE_PRINTF( "Internal Log memory space remaining: %d %%\n\n", tracker_get_remaining_memory_space( ) );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "Try to write Internal Log in memory which is not empty, skip the log\n\n" );
    }
}

static uint8_t tracker_get_remaining_memory_space( void )
{
    return ( tracker_ctx.flash_remaining_space * 100 ) / ( tracker_ctx.flash_addr_end - tracker_ctx.flash_addr_start );
}

static void tracker_store_internal_log_ctx( void )
{
    uint8_t ctx_buf[INTERNAL_LOG_CONTEXT_SIZE];
    uint8_t index           = 0;
    uint8_t tracker_ctx_crc = 0;

    if( tracker_ctx.internal_log_empty != FLASH_BYTE_EMPTY_CONTENT )
    {
        hal_flash_erase_page( FLASH_USER_INTERNAL_LOG_CTX_START_ADDR, 1 );
    }
    else
    {
        tracker_ctx.internal_log_empty = 1;
    }

    ctx_buf[index++] = tracker_ctx.internal_log_empty;

    ctx_buf[index++] = tracker_ctx.nb_scan;
    ctx_buf[index++] = tracker_ctx.nb_scan >> 8;

    ctx_buf[index++] = tracker_ctx.flash_addr_start;
    ctx_buf[index++] = tracker_ctx.flash_addr_start >> 8;
    ctx_buf[index++] = tracker_ctx.flash_addr_start >> 16;
    ctx_buf[index++] = tracker_ctx.flash_addr_start >> 24;

    ctx_buf[index++] = tracker_ctx.flash_addr_end;
    ctx_buf[index++] = tracker_ctx.flash_addr_end >> 8;
    ctx_buf[index++] = tracker_ctx.flash_addr_end >> 16;
    ctx_buf[index++] = tracker_ctx.flash_addr_end >> 24;

    ctx_buf[index++] = tracker_ctx.flash_addr_current;
    ctx_buf[index++] = tracker_ctx.flash_addr_current >> 8;
    ctx_buf[index++] = tracker_ctx.flash_addr_current >> 16;
    ctx_buf[index++] = tracker_ctx.flash_addr_current >> 24;

    tracker_ctx.flash_remaining_space = tracker_ctx.flash_addr_end - tracker_ctx.flash_addr_current;

    ctx_buf[index++] = tracker_ctx.flash_remaining_space;
    ctx_buf[index++] = tracker_ctx.flash_remaining_space >> 8;
    ctx_buf[index++] = tracker_ctx.flash_remaining_space >> 16;
    ctx_buf[index++] = tracker_ctx.flash_remaining_space >> 24;

    /* Compute CRC of the context */
    tracker_ctx_crc  = tracker_ctx_compute_crc( 0xFF, ctx_buf, index );
    ctx_buf[index++] = tracker_ctx_crc;

    hal_flash_write_buffer( FLASH_USER_INTERNAL_LOG_CTX_START_ADDR, ctx_buf, index );
}

static void tracker_get_one_scan_from_internal_log( uint16_t scan_number, uint8_t* out_buffer,
                                                    const uint16_t out_buffer_len, uint16_t* buffer_len )
{
    uint8_t   scan_buf[255];
    uint16_t  scan_buf_index    = 0;
    uint8_t   nb_elements       = 0;
    uint8_t   nb_elements_index = 0;
    uint8_t   tag_element       = 0;
    uint16_t  scan_len          = 0;
    uint16_t  nb_scan_index     = 1;
    uint32_t  next_scan_addr    = tracker_ctx.flash_addr_start;
    time_t    scan_timestamp    = 0;
    struct tm epoch_time;
    uint32_t  job_counter = 0;

    *buffer_len = 0;

    /* Retrieve the scan_number flash address */
    while( nb_scan_index <= scan_number )
    {
        /* read the scan lentgh */
        hal_flash_read_buffer( next_scan_addr, scan_buf, 2 );
        scan_len = scan_buf[0];
        scan_len += ( uint32_t ) scan_buf[1] << 8;

        hal_flash_read_buffer( next_scan_addr + 2, scan_buf, scan_len - 2 );

        nb_elements = scan_buf[scan_buf_index++];

        scan_buf_index += 2;  // element which are always stored

        while( nb_elements_index < nb_elements )
        {
            uint8_t len = 0;
            tag_element = scan_buf[scan_buf_index++];  // get the element
            len         = scan_buf[scan_buf_index++];  // get the size element

            switch( tag_element )
            {
            case TAG_GNSS_PATCH:
            case TAG_GNSS_PCB:
            {
                uint8_t last_scan = scan_buf[scan_buf_index + 6];
                if( last_scan == 1 )
                {
                    job_counter++;
                }
                scan_buf_index += len;
                break;
            }
            case TAG_WIFI:
            {
                job_counter++;
                scan_buf_index += len;
                break;
            }
            case TAG_NEXT_SCAN:
            {
                next_scan_addr = scan_buf[scan_buf_index++];
                next_scan_addr += ( uint16_t ) scan_buf[scan_buf_index++] << 8;
                next_scan_addr += ( uint32_t ) scan_buf[scan_buf_index++] << 16;
                next_scan_addr += ( uint32_t ) scan_buf[scan_buf_index++] << 24;
                break;
            }
            default:
                scan_buf_index += len;
                break;
            }
            nb_elements_index++;
        }
        nb_scan_index++;
        nb_elements_index = 0;
        scan_buf_index    = 0;  // reset the index;
    }

    /* Get the scan asked */

    /* number elements to get */
    nb_elements = scan_buf[scan_buf_index++];

    /* Scan number */
    scan_number = scan_buf[scan_buf_index++];
    scan_number += ( uint16_t ) scan_buf[scan_buf_index++] << 8;

    while( nb_elements_index < nb_elements )
    {
        uint8_t len = 0;
        tag_element = scan_buf[scan_buf_index++];  // get the element
        len         = scan_buf[scan_buf_index++];  // get the size element

        HAL_DBG_TRACE_PRINTF( "tag_element %d len %d\n", tag_element, len );

        switch( tag_element )
        {
        case TAG_GNSS_PATCH:
        case TAG_GNSS_PCB:
        {
            /* Scan Timestamp */
            scan_timestamp = get_uint32_from_array_at_index_and_inc( scan_buf, &scan_buf_index );
            memcpy( &epoch_time, localtime( &scan_timestamp ), sizeof( struct tm ) );

            uint16_t nav_len = len - GNSS_TOKEN_LEN - GNSS_NB_SAT_LEN - GNSS_TIMESTAMP_LEN -
                               GNSS_LAST_SCAN_IN_GROUP_LEN - GNSS_PROFILE_LEN;
            uint8_t token     = scan_buf[scan_buf_index++];
            uint8_t nb_sat    = scan_buf[scan_buf_index++];
            uint8_t last_scan = scan_buf[scan_buf_index++];
            uint8_t profile   = scan_buf[scan_buf_index++];

            /* Display Raw NAV Message*/
            *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                     "[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900, epoch_time.tm_mon + 1,
                                     epoch_time.tm_mday, epoch_time.tm_hour, epoch_time.tm_min, epoch_time.tm_sec );

            *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                     "[%ld - %d] ", job_counter, tag_element );

            if( nav_len > 0 )
            {
                *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "01" );

                for( uint16_t i = 0; i < nav_len; i++ )
                {
                    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                             "%02X", scan_buf[scan_buf_index++] );
                }
            }
            else
            {
                *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "0007" );
            }

            *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                     ",%d,%d,%d,%d\r\n", token, last_scan, nb_sat, profile );

            break;
        }
        case TAG_APC_MSG:
        {
            HAL_DBG_TRACE_PRINTF( "Restore APC\n" );
            /* Scan Timestamp */
            scan_timestamp = get_uint32_from_array_at_index_and_inc( scan_buf, &scan_buf_index );
            memcpy( &epoch_time, localtime( &scan_timestamp ), sizeof( struct tm ) );

            uint16_t apc_len = len - GNSS_TIMESTAMP_LEN;

            /* Display Raw NAV Message*/
            *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                     "[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900, epoch_time.tm_mon + 1,
                                     epoch_time.tm_mday, epoch_time.tm_hour, epoch_time.tm_min, epoch_time.tm_sec );

            *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                     "[%ld - %d] ", job_counter, tag_element );

            for( uint16_t i = 0; i < apc_len; i++ )
            {
                *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "%02X",
                                         scan_buf[scan_buf_index++] );
            }

            *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "\r\n" );

            break;
        }
        case TAG_WIFI:
        {
            int8_t                           wifi_rssi;
            uint8_t                          wifi_data;
            lr11xx_wifi_channel_t            wifi_channel;
            lr11xx_wifi_signal_type_result_t wifi_type;
            char                             wifi_type_char = 'B';

            /* Scan Timestamp */
            scan_timestamp = get_uint32_from_array_at_index_and_inc( scan_buf, &scan_buf_index );
            memcpy( &epoch_time, localtime( &scan_timestamp ), sizeof( struct tm ) );

            for( uint8_t i = 0; i < ( ( len - WIFI_TIMESTAMP_LEN ) / WIFI_SINGLE_BEACON_LEN ); i++ )
            {
                *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                         "[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900, epoch_time.tm_mon + 1,
                                         epoch_time.tm_mday, epoch_time.tm_hour, epoch_time.tm_min, epoch_time.tm_sec );

                *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                         "[%ld - %d] ", job_counter, tag_element );

                wifi_rssi = scan_buf[scan_buf_index++];

                wifi_data    = scan_buf[scan_buf_index++];
                wifi_channel = ( lr11xx_wifi_channel_t )( wifi_data & 0x0F );
                wifi_type    = ( lr11xx_wifi_signal_type_result_t )( ( wifi_data & 0x30 ) >> 4 );

                switch( wifi_type )
                {
                case LR11XX_WIFI_TYPE_RESULT_B:
                    wifi_type_char = 'B';
                    break;
                case LR11XX_WIFI_TYPE_RESULT_G:
                    wifi_type_char = 'G';
                    break;
                case LR11XX_WIFI_TYPE_RESULT_N:
                    wifi_type_char = 'N';
                    break;
                default:
                    break;
                }

                /* Display MAC address */
                for( uint8_t i = 0; i < 5; i++ )
                {
                    *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                             "%02X:", scan_buf[scan_buf_index++] );
                }

                *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len, "%02X,",
                                         scan_buf[scan_buf_index++] );

                /* Display Scan Information */
                *buffer_len += snprintf( ( char* ) ( out_buffer + *buffer_len ), out_buffer_len - *buffer_len,
                                         "CHANNEL_%d,TYPE_%c,%d,0,0,0,0\r\n", wifi_channel, wifi_type_char, wifi_rssi );
            }
            break;
        }
        case TAG_NEXT_SCAN:
        {
            next_scan_addr = scan_buf[scan_buf_index++];
            next_scan_addr += ( uint16_t ) scan_buf[scan_buf_index++] << 8;
            next_scan_addr += ( uint32_t ) scan_buf[scan_buf_index++] << 16;
            next_scan_addr += ( uint32_t ) scan_buf[scan_buf_index++] << 24;
            break;
        }
        default:
        {
            scan_buf_index += len;
        }
        break;
        }
        nb_elements_index++;
    }
}

static void tracker_erase_internal_log( void )
{
    uint8_t nb_page_to_erase = 0;

    if( tracker_ctx.nb_scan > 0 )
    {
        nb_page_to_erase =
            ( ( tracker_ctx.flash_addr_current - tracker_ctx.flash_addr_start ) / ADDR_FLASH_PAGE_SIZE ) + 1;
        /* Erase scan results */
        hal_flash_erase_page( tracker_ctx.flash_addr_start, nb_page_to_erase );
    }
    /* Erase ctx */
    hal_flash_erase_page( FLASH_USER_INTERNAL_LOG_CTX_START_ADDR, 1 );
}

static void tracker_erase_full_user_flash_memory( void )
{
    uint8_t nb_page_to_erase = 0;

    nb_page_to_erase = ( ( FLASH_USER_END_ADDR - hal_flash_get_user_start_addr( ) ) / ADDR_FLASH_PAGE_SIZE ) + 1;
    /* Erase scan results */
    hal_flash_erase_page( hal_flash_get_user_start_addr( ), nb_page_to_erase );

    /* Erase ctx */
    hal_flash_erase_page( FLASH_USER_INTERNAL_LOG_CTX_START_ADDR, 1 );
}

static bool smtc_board_get_almanac_dates( const void* context, uint32_t* oldest_almanac_date,
                                          uint32_t* newest_almanac_date )
{
    lr11xx_status_t status       = LR11XX_STATUS_ERROR;
    uint8_t         i            = 0;
    uint16_t        almanac_date = 0;

    *oldest_almanac_date = 0;
    *newest_almanac_date = 0;

    for( i = 0; i < 127; i++ )
    {
        /* Suspend the modem radio access */
        smtc_modem_suspend_before_user_radio_access( );

        status = lr11xx_gnss_get_almanac_age_for_satellite( context, i, &almanac_date );

        /* Resume the modem radio access */
        smtc_modem_resume_after_user_radio_access( );

        if( status != LR11XX_STATUS_OK )
        {
            return false;
        }
        if( almanac_date > 0 )
        {
            if( ( *oldest_almanac_date == 0 ) && ( *newest_almanac_date == 0 ) )
            {
                *oldest_almanac_date = almanac_date;
                *newest_almanac_date = almanac_date;
            }
            else
            {
                if( almanac_date < *oldest_almanac_date )
                {
                    *oldest_almanac_date = almanac_date;
                }
                if( almanac_date > *newest_almanac_date )
                {
                    *newest_almanac_date = almanac_date;
                }
            }
        }
    }
    return true;
}

static void tracker_check_app_ctx( void )
{
    bool new_paramater_to_store = false;
    /* GNSS Parameters */
    if( ( tracker_ctx.gnss_antenna_sel != GNSS_PATCH_ANTENNA ) && ( tracker_ctx.gnss_antenna_sel != GNSS_PCB_ANTENNA ) )
    {
        new_paramater_to_store       = true;
        tracker_ctx.gnss_antenna_sel = GNSS_PCB_ANTENNA;
        HAL_DBG_TRACE_WARNING( "gnss_antenna_sel out of bounds, set to default value\n" );
    }

    /* Application Parameters */
    if( ( tracker_ctx.mobile_scan_interval < MOBILE_SCAN_INTERVAL_MIN ) ||
        ( tracker_ctx.mobile_scan_interval > MOBILE_SCAN_INTERVAL_MAX ) )
    {
        new_paramater_to_store           = true;
        tracker_ctx.mobile_scan_interval = MOBILE_SCAN_INTERVAL_DEFAULT;
        HAL_DBG_TRACE_WARNING( "mobile_scan_interval out of bounds, set to default value\n" );
    }

    if( ( tracker_ctx.static_scan_interval < STATIC_SCAN_INTERVAL_MIN ) ||
        ( tracker_ctx.static_scan_interval > STATIC_SCAN_INTERVAL_MAX ) )
    {
        new_paramater_to_store           = true;
        tracker_ctx.static_scan_interval = STATIC_SCAN_INTERVAL_DEFAULT;
        HAL_DBG_TRACE_WARNING( "static_scan_interval out of bounds, set to default value\n" );
    }

    if( ( tracker_ctx.scan_priority != TRACKER_GNSS_PRIORITY ) && ( tracker_ctx.scan_priority != TRACKER_NO_PRIORITY ) )
    {
        new_paramater_to_store    = true;
        tracker_ctx.scan_priority = TRACKER_GNSS_PRIORITY;
        HAL_DBG_TRACE_WARNING( "scan_priority out of bounds, set to default value\n" );
    }

    if( new_paramater_to_store )
    {
        tracker_store_app_ctx( );
    }
}

/* --- EOF ------------------------------------------------------------------ */
