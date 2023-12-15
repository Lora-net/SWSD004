/*!
 * @ingroup   tracker_application_autonomous
 * @file      main_tracker_application_autonomous.c
 *
 * @brief     LoRa Basics Modem LR11XX tracker autonomous application implementation.
 *
 * @copyright
 * @parblock
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * @endparblock
 */

/*!
 * @addtogroup tracker_application autonomous
 * LoRa Basics Modem LR11XX tracker application
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "main_tracker_application.h"
#include "tracker_utility.h"
#include "lorawan_key_config.h"
#include "smtc_board.h"
#include "smtc_hal.h"
#include "apps_utilities.h"
#include "apps_modem_common.h"
#include "apps_modem_event.h"

#include "lr11xx_gnss.h"
#include "lr11xx_system.h"
#include "lr11xx_bootloader.h"

#include "smtc_modem_api.h"
#include "smtc_modem_geolocation_api.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_test_api.h"
#include "smtc_modem_utilities.h"
#include "smtc_board_ralf.h"

#include "lr1110_trk1xks_board.h"
#include "smtc_lr11xx_board.h"
#include "ble_thread.h"

#include <stdio.h>
#include <string.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * @brief Force a new tracker context in flash memory
 */
#define FORCE_NEW_TRACKER_CONTEXT 0

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Tracker context structure
 */
extern tracker_ctx_t tracker_ctx;

/*!
 * @brief Stack identifier
 */
static uint8_t stack_id = 0;

/*!
 * @brief Radio context
 */
void* radio_context;

uint8_t dev_eui[LORAWAN_DEVICE_EUI_LEN] = LORAWAN_DEVICE_EUI;
uint8_t join_eui[LORAWAN_JOIN_EUI_LEN]  = LORAWAN_JOIN_EUI;
uint8_t app_key[LORAWAN_APP_KEY_LEN]    = LORAWAN_APP_KEY;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Init the applicativ tracker context
 *
 * @param [in] dev_eui LoRaWAN Device Eui
 * @param [in] join_eui LoRaWAN Join Eui
 * @param [in] app_key LoRaWAN Application Key
 */
static void tracker_app_init_context( uint8_t* dev_eui, uint8_t* join_eui, uint8_t* app_key );

/*!
 * @brief Lorawan default init
 */
static void tracker_app_modem_configure_lorawan_params( void );

/*!
 * @brief   Check if the tracker is in static mode
 *
 * @returns  [true : tracker is static, false : tracker in movement]
 */
static bool tracker_app_is_tracker_in_static_mode( void );

/*!
 * @brief Parse the received downlink
 *
 * @param [in] port LoRaWAN port
 * @param [in] payload Payload Buffer
 * @param [in] size Payload size
 */
static void tracker_app_parse_downlink_frame( uint8_t port, const uint8_t* payload, uint8_t size );

/*!
 * @brief Update in flash context the accumulated charge if has changed
 *
 * @param [in] charge_mAh the actual charge
 */
static void tracker_app_store_new_accumulated_charge( uint32_t charge_mAh );

/*!
 * @brief read and build sensor payload then send it
 */
static void tracker_app_read_and_send_sensors( void );

/*!
 * @brief build tracker settings payload in TLV format
 *
 * @param [in] buffer Buffer containing the tracker settings
 * @param [in] len Len of the buffer
 */
static void tracker_app_build_and_send_tracker_settings( const uint8_t* buffer, uint8_t len );

/*!
 * @brief Get, Check and display the lr11xx firmware version
 */
static bool tracker_app_lr11xx_check_firmware_version( void* context );

/*!
 * @brief Get, Check and display the LoRa Basics Modem firmware version
 */
static void tracker_app_display_lbm_version( void );

/*!
 * @brief Set the ADR according to the used region
 */
static void tracker_app_set_adr( void );

/**
 * @brief Set the maximum TX output power following the region used
 *
 * @param [in] region the region used by the modem, see \ref smtc_modem_region_t
 * @param [in] sub_region the sub region used by the modem, see \ref smtc_modem_sub_region_t
 */
static void tracker_app_set_max_tx_power_supported( smtc_modem_region_t region, smtc_modem_sub_region_t sub_region );

/*!
 * @addtogroup basics_modem_evt_callback
 * LoRa Basics Modem event callbacks
 * @{
 */

/*!
 * @brief Reset event callback
 */
static void on_modem_reset( void );

/*!
 * @brief Network Joined event callback
 */
static void on_modem_network_joined( void );

/*!
 * @brief Downlink data event callback.
 *
 * @param [in] payload Received buffer pointer
 * @param [in] length    Received buffer size
 * @param [in] metadata          Structure holding downlink metadata
 * @param [in] remaining_data_nb Number of downlink data remaining
 */
static void on_modem_down_data( const uint8_t* payload, uint8_t size, smtc_modem_dl_metadata_t metadata,
                                uint8_t remaining_data_nb );

/*!
 * @brief GNSS scan done event callback
 *
 * @param [in] data Scan done data, see \ref smtc_modem_gnss_event_data_scan_done_t
 */
static void on_gnss_scan_done( smtc_modem_gnss_event_data_scan_done_t data );

/*!
 * @brief GNSS scan terminated event callback
 *
 * @param [in] data Scan terminated data, see \ref smtc_modem_gnss_event_data_terminated_t
 */
static void on_gnss_scan_terminated( smtc_modem_gnss_event_data_terminated_t data );

/*!
 * @brief GNSS almanac demodulation update
 *
 * @param [in] data Demodulation update data, see \ref smtc_modem_almanac_demodulation_event_data_almanac_update_t
 */
static void on_gnss_almanac_demod_update( smtc_modem_almanac_demodulation_event_data_almanac_update_t data );

/*!
 * @brief WIFI scan done event callback
 *
 * @param [in] data Scan done data, see \ref smtc_modem_wifi_event_data_scan_done_t
 */
static void on_wifi_scan_done( smtc_modem_wifi_event_data_scan_done_t data );

/*!
 * @brief WIFI scan terminated event callback
 *
 * @param [in] data Scan terminated data, see \ref smtc_modem_wifi_event_data_terminated_t
 */
static void on_wifi_scan_terminated( smtc_modem_wifi_event_data_terminated_t data );

/*!
 * @brief Alarm
 */
static void on_alarm_event( void );

/*!
 * @brief LoRaWAN MAC time answer
 */
static void on_lorawan_mac_time( smtc_modem_event_mac_request_status_t status, uint32_t gps_time_s,
                                 uint32_t gps_fractional_s );

/*!
 * @brief LoRaWAN Link status
 */
static void on_link_status( smtc_modem_event_mac_request_status_t status, uint8_t margin, uint8_t gw_cnt );

/*!
 * @brief Tracker application startup
 */
static void tracker_app_start( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    uint32_t sleep_time_ms = 0;

    static apps_modem_event_callback_t smtc_event_callback = {
        .down_data                 = on_modem_down_data,
        .joined                    = on_modem_network_joined,
        .reset                     = on_modem_reset,
        .gnss_scan_done            = on_gnss_scan_done,
        .gnss_terminated           = on_gnss_scan_terminated,
        .wifi_scan_done            = on_wifi_scan_done,
        .wifi_terminated           = on_wifi_scan_terminated,
        .alarm                     = on_alarm_event,
        .lorawan_mac_time          = on_lorawan_mac_time,
        .link_status               = on_link_status,
        .gnss_almanac_demod_update = on_gnss_almanac_demod_update,
    };

    /* Initialise the ralf_t object corresponding to the board */
    radio_context = smtc_board_get_radio_context( );
    smtc_modem_set_radio_context( radio_context );

    /* Init board and peripherals */
    hal_mcu_init( );
    smtc_board_init_periph( );

    /* Check LR11XX Firmware version */
    if( tracker_app_lr11xx_check_firmware_version( radio_context ) != true )
    {
        HAL_DBG_TRACE_INFO( "Something goes wrong with the LR11XX firmware, stay in BLE mode and update it\n" );
        tracker_app_init_context( dev_eui, join_eui, app_key );
        while( 1 )
        {
            // Stay in BLE while the LR11XX firmware is not installed
            start_ble_thread( 0, stack_id );
        }
    }

    /* Init the Lora Basics Modem event callbacks */
    apps_modem_event_init( &smtc_event_callback );

    /* Init the modem and use apps_modem_event_process as event callback, please note that the callback will be called
     * immediately after the first call to modem_run_engine because of the reset detection */
    smtc_modem_init( &apps_modem_event_process );

    tracker_app_start( );

    while( 1 )
    {
        /* Execute modem runtime, this function must be called again in sleep_time_ms milliseconds or sooner. */
        sleep_time_ms = smtc_modem_run_engine( );

        if( get_hall_effect_irq_state( ) == true )
        {
            /* Effect hall irq, reset the board*/
            smtc_modem_hal_reset_mcu( );
        }
        else
        {
            /* go in low power */
            hal_mcu_disable_irq( );
            if( smtc_modem_is_irq_flag_pending( ) == false )
            {
                hal_mcu_set_sleep_for_ms( sleep_time_ms );
            }
            hal_mcu_enable_irq( );

            if( tracker_ctx.airplane_mode == false )
            {
                /* Wake up from static mode thanks the accelerometer ? */
                if( ( get_accelerometer_irq1_state( ) == true ) &&
                    ( tracker_app_is_tracker_in_static_mode( ) == true ) )
                {
                    /* Start Hall Effect sensors while the tracker moves */
                    smtc_board_hall_effect_enable( true );

                    /* abort and relauch middlewares */
                    HAL_DBG_TRACE_INFO( "WAKE UP ABORT CURRENT RP TASKS\n" );
                    if( smtc_modem_gnss_scan_cancel( stack_id ) == SMTC_MODEM_RC_OK )
                    {
                        smtc_modem_gnss_scan_aggregate( stack_id, false );
                        smtc_modem_gnss_scan( stack_id, SMTC_MODEM_GNSS_MODE_MOBILE, 0 );
                    }
                    smtc_modem_wifi_scan_cancel( stack_id );
                }
            }
            else
            {
                /* Wake up thanks the accelerometer and in airplane mode ? */
                if( is_accelerometer_detected_moved( ) == true )
                {
                    smtc_board_hall_effect_enable_for_duration( TRACKER_AIRPLANE_HALL_EFFECT_TIMEOUT_MS );
                    HAL_DBG_TRACE_PRINTF( "Start hall effect sensor for %ds\n",
                                          TRACKER_AIRPLANE_HALL_EFFECT_TIMEOUT_MS / 1000 );
                }
            }
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void tracker_app_start( void )
{
    /* Notify user that the board is initialized */
    smtc_board_leds_blink( smtc_board_get_led_all_mask( ), 100, 2 );

    HAL_DBG_TRACE_MSG( "\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem Tracker application ==== ######\n\n" );
    HAL_DBG_TRACE_PRINTF( "APP VERSION : %d.%d.%d\n\n", TRACKER_MAJOR_APP_VERSION, TRACKER_MINOR_APP_VERSION,
                          TRACKER_SUB_MINOR_APP_VERSION );

    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem Version ==== ######\n" );
    tracker_app_display_lbm_version( );

    /* Init Tracker context */
    HAL_DBG_TRACE_INFO( "###### ===== Init context ==== ######\n" );
    tracker_app_init_context( dev_eui, join_eui, app_key );

    /* Initialize LoRaWAN parameters */
    tracker_app_modem_configure_lorawan_params( );

    /* Start the BLE thread*/
    start_ble_thread( TRACKER_ADV_TIMEOUT_MS, stack_id );

    if( tracker_ctx.airplane_mode == false )
    {
        /* Start the Join process */
        ASSERT_SMTC_MODEM_RC( smtc_modem_join_network( stack_id ) );

        HAL_DBG_TRACE_INFO( "###### ===== JOINING ==== ######\n\n" );

        /* Init store and forward service */
        smtc_modem_store_and_forward_set_state( stack_id, true );

        /* Init geolocation services */
        ASSERT_SMTC_MODEM_RC( smtc_modem_gnss_send_mode( stack_id, tracker_ctx.geolocation_send_mode ) );
        ASSERT_SMTC_MODEM_RC( smtc_modem_wifi_send_mode( stack_id, tracker_ctx.geolocation_send_mode ) );
        smtc_modem_gnss_scan_aggregate( stack_id, false );

        /* Program GNSS scan */
        ASSERT_SMTC_MODEM_RC( smtc_modem_gnss_scan( stack_id, SMTC_MODEM_GNSS_MODE_MOBILE, 20 ) );

        /* Start almanac demodulation service */
        smtc_modem_almanac_demodulation_set_constellations( stack_id, SMTC_MODEM_GNSS_CONSTELLATION_GPS_BEIDOU );
        smtc_modem_almanac_demodulation_start( stack_id );
    }
    else
    {
        HAL_DBG_TRACE_MSG( "TRACKER IN AIRPLANE MODE\n\n" );

        /* Stop Hall Effect sensors while the tracker is static in airplane mode */
        smtc_board_hall_effect_enable( false );

        /* Reset accelerometer IRQ if any */
        is_accelerometer_detected_moved( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER GNSS FUNCTION TYPES ---------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER LORAWAN FUNCTION TYPES ------------------------------------------
 */

static void tracker_app_init_context( uint8_t* dev_eui, uint8_t* join_eui, uint8_t* app_key )
{
    /* Store or restore the Tracker context */
    if( ( tracker_restore_app_ctx( ) != TRACKER_SUCCESS ) || ( FORCE_NEW_TRACKER_CONTEXT == 1 ) )
    {
        /* When the production keys are used, DevEUI = ChipEUI and JoinEUI is the one defined in
         * lorawan_key_config.h
         */
        if( tracker_ctx.has_lr11xx_trx_firmware )
        {
            smtc_modem_get_chip_eui( stack_id, dev_eui );
        }

        /* Init the tracker global context and store it only if there is a lr11xx firmware */
        tracker_init_app_ctx( dev_eui, join_eui, app_key, tracker_ctx.has_lr11xx_trx_firmware );

        /* Init the tracker internal log context */
        if( tracker_init_internal_log_ctx( ) != TRACKER_SUCCESS )
        {
            HAL_DBG_TRACE_ERROR( "tracker_init_internal_log_ctx failed\n" );
        }
    }
    else
    {
        /* Restore the tracker internal log context */
        if( tracker_restore_internal_log_ctx( ) != TRACKER_SUCCESS )
        {
            /* Init the tracker internal log context */
            if( tracker_init_internal_log_ctx( ) != TRACKER_SUCCESS )
            {
                HAL_DBG_TRACE_ERROR( "tracker_init_internal_log_ctx failed\n" );
            }
        }

        /* Set the restored LoRaWAN Keys */
        memcpy( dev_eui, tracker_ctx.dev_eui, LORAWAN_DEVICE_EUI_LEN );
        memcpy( join_eui, tracker_ctx.join_eui, LORAWAN_JOIN_EUI_LEN );
        memcpy( app_key, tracker_ctx.app_key, LORAWAN_APP_KEY_LEN );
    }

    /* Init tracker context volatile parameters */
    tracker_ctx.accelerometer_move_history = 1;
    tracker_ctx.voltage                    = hal_mcu_get_vref_level( );
    tracker_ctx.gnss_scan_charge_nAh       = 0;
    tracker_ctx.wifi_scan_charge_nAh       = 0;
    tracker_ctx.reset_board_asked          = false;

    smtc_board_select_gnss_antenna( tracker_ctx.gnss_antenna_sel );

    /* Set the maximum authorized transmit output power supported by the board following the region */
    tracker_app_set_max_tx_power_supported( tracker_ctx.lorawan_region, tracker_ctx.lorawan_sub_region );

    /* Set the battery level */
    smtc_board_set_battery_level( tracker_get_battery_level( ) );
}

static void tracker_app_modem_configure_lorawan_params( void )
{
    smtc_modem_return_code_t rc = SMTC_MODEM_RC_OK;

    HAL_DBG_TRACE_INFO( "LoRaWAN parameters:\n" );

    rc = smtc_modem_get_chip_eui( stack_id, tracker_ctx.chip_eui );
    if( rc == SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_ARRAY( "ChipEIU", tracker_ctx.chip_eui, SMTC_MODEM_EUI_LENGTH );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "smtc_modem_get_chip_eui failed (%d)\n", rc );
    }

    rc = smtc_modem_set_deveui( stack_id, tracker_ctx.dev_eui );
    if( rc == SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_ARRAY( "DevEUI", tracker_ctx.dev_eui, SMTC_MODEM_EUI_LENGTH );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "smtc_modem_get_deveui failed (%d)\n", rc );
    }

    rc = smtc_modem_set_joineui( stack_id, tracker_ctx.join_eui );
    if( rc == SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_ARRAY( "JoinEUI", tracker_ctx.join_eui, SMTC_MODEM_EUI_LENGTH );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "smtc_modem_get_joineui failed (%d)\n", rc );
    }

    /* The Derive keys is done thought the smtc_modem_get_pin command */
    rc = smtc_modem_get_pin( stack_id, tracker_ctx.lorawan_pin );
    if( rc == SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_ARRAY( "PIN", tracker_ctx.lorawan_pin, 4 );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "smtc_modem_get_pin failed (%d)\n", rc );
    }

    if( tracker_ctx.use_semtech_join_server == false )
    {
        rc = smtc_modem_set_nwkkey( stack_id, tracker_ctx.app_key );
        if( rc == SMTC_MODEM_RC_OK )
        {
            HAL_DBG_TRACE_ARRAY( "AppKey", tracker_ctx.app_key, SMTC_MODEM_KEY_LENGTH );
        }
        else
        {
            HAL_DBG_TRACE_ERROR( "smtc_modem_set_nwkkey failed (%d)\n", rc );
        }
    }
    else
    {
        HAL_DBG_TRACE_MSG( "AppKey : Use Semtech Join Sever\n" );
    }

    ASSERT_SMTC_MODEM_RC( smtc_modem_set_region( stack_id, tracker_ctx.lorawan_region ) );
    modem_region_to_string( tracker_ctx.lorawan_region );
}

/*
 * -------------------------------------------------------------------------
 * --- TRACKER APP FUNCTION TYPES ------------------------------------------
 */

static void tracker_app_build_and_send_tracker_settings( const uint8_t* buffer, uint8_t len )
{
    uint8_t tx_max_payload = 0;

    ASSERT_SMTC_MODEM_RC( smtc_modem_get_next_tx_max_payload( stack_id, &tx_max_payload ) );
    HAL_DBG_TRACE_PRINTF( "tx_max_payload %d \n", tx_max_payload );

    HAL_DBG_TRACE_PRINTF( " - Tracker settings (%d bytes) : ", len + 2 );

    if( tx_max_payload < ( len + 2 ) )
    {
        HAL_DBG_TRACE_ERROR( "TX max payload < payload len\n" );
    }
    else
    {
        uint8_t payload_len = 0;
        uint8_t lorawan_payload[242];

        /* Add tracker settings value */
        lorawan_payload[payload_len++] = TLV_TRACKER_SETTINGS_TAG;  // Tracker settings TAG
        lorawan_payload[payload_len++] = len;                       // Tracker settings LEN is variable

        memcpy( lorawan_payload + payload_len, buffer, len );
        payload_len += len;

        HAL_DBG_TRACE_MSG( "Send data\n" );
        smtc_modem_request_uplink( stack_id, TRACKER_REQUEST_MSG_PORT, false, lorawan_payload, payload_len );
    }
}

static void tracker_app_read_and_send_sensors( void )
{
    uint8_t  tx_max_payload = 0;
    uint8_t  payload_len    = 0;
    uint8_t  lorawan_payload[242];
    uint32_t charge_mah = 0;

    /* SENSORS DATA */
    HAL_DBG_TRACE_INFO( "*** sensors collect ***\n" );

    /* Move history */
    HAL_DBG_TRACE_PRINTF( "Move history : %d\n", tracker_ctx.accelerometer_move_history );

    /* Temperature */
    tracker_ctx.temperature = smtc_modem_hal_get_temperature( );
    HAL_DBG_TRACE_PRINTF( "Temperature : %d *C\n", tracker_ctx.temperature );

    /* Modem charge */
    smtc_modem_get_charge( &charge_mah );
    HAL_DBG_TRACE_PRINTF( "LBM Charge value : %d mAh\n", charge_mah );
    HAL_DBG_TRACE_PRINTF( "GNSS scan charge value : %d nAh\n", tracker_ctx.gnss_scan_charge_nAh );
    HAL_DBG_TRACE_PRINTF( "Wi-Fi scan charge value : %d nAh\n", tracker_ctx.wifi_scan_charge_nAh );
    tracker_app_store_new_accumulated_charge( charge_mah + ( tracker_ctx.gnss_scan_charge_nAh / 1000000 ) +
                                              ( tracker_ctx.gnss_scan_charge_nAh / 1000000 ) );
    HAL_DBG_TRACE_PRINTF( "Accumulated charge value : %d mAh\n", tracker_ctx.accumulated_charge_mAh );

    /* Board voltage charge */
    tracker_ctx.voltage = smtc_modem_hal_get_voltage_mv( ) * 20;
    HAL_DBG_TRACE_PRINTF( "Board voltage : %d mV\n\n", tracker_ctx.voltage );

    ASSERT_SMTC_MODEM_RC( smtc_modem_get_next_tx_max_payload( stack_id, &tx_max_payload ) );
    HAL_DBG_TRACE_PRINTF( "tx_max_payload %d \n", tx_max_payload );

    /* Temperature */
    lorawan_payload[payload_len++] = tracker_ctx.temperature >> 8;
    lorawan_payload[payload_len++] = tracker_ctx.temperature;

    /* Accumulated Charge in mAh */
    lorawan_payload[payload_len++] = tracker_ctx.accumulated_charge_mAh >> 8;
    lorawan_payload[payload_len++] = tracker_ctx.accumulated_charge_mAh;

    /* Board Voltage */
    lorawan_payload[payload_len++] = tracker_ctx.voltage >> 8;
    lorawan_payload[payload_len++] = tracker_ctx.voltage;

    if( tx_max_payload < payload_len )
    {
        HAL_DBG_TRACE_ERROR( "TX max payload < payload len\n" );
    }
    else
    {
        HAL_DBG_TRACE_MSG( "Use classic uplink to send data\n" );
        smtc_modem_request_uplink( stack_id, TRACKER_APP_SENSOR_PORT, false, lorawan_payload, payload_len );
    }
}

static void tracker_app_store_new_accumulated_charge( uint32_t charge_mAh )
{
    static uint32_t previous_charge_mAh = 0;  // Previous modem charge before read from LBM plus scans, keep
                                              // the historic even after leave the function because of the static

    /* Store the new accumulated charge only if the modem charge has changed */
    if( charge_mAh != previous_charge_mAh )
    {
        tracker_ctx.accumulated_charge_mAh += charge_mAh - previous_charge_mAh;
        HAL_DBG_TRACE_MSG( "New accumulated charge stored\n" );
        tracker_store_app_ctx( );

        previous_charge_mAh = charge_mAh;

        /* Set the new battery level */
        smtc_board_set_battery_level( tracker_get_battery_level( ) );
    }
}

static bool tracker_app_is_tracker_in_static_mode( void )
{
    if( ( ( tracker_ctx.accelerometer_move_history & TRACKER_SEND_ONE_MORE_SCANS_ONCE_STATIC ) == 0 ) &&
        ( tracker_ctx.accelerometer_used == 1 ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

static void tracker_app_parse_downlink_frame( uint8_t port, const uint8_t* payload, uint8_t size )
{
    switch( port )
    {
    case TRACKER_REQUEST_MSG_PORT:
    {
        uint8_t tag           = 0;
        uint8_t len           = 0;
        uint8_t payload_index = 0;

        while( payload_index < size )
        {
            tag = payload[payload_index++];
            len = payload[payload_index++];

            switch( tag )
            {
            case GET_APP_TRACKER_SETTINGS_CMD:
            {
                uint8_t settings_buffer[240];
                uint8_t tracker_settings_payload_max_len = 0;
                memcpy( settings_buffer, payload + payload_index, len );

                ASSERT_SMTC_MODEM_RC(
                    smtc_modem_get_next_tx_max_payload( stack_id, &tracker_settings_payload_max_len ) );

                HAL_DBG_TRACE_INFO( "###### ===== TRACKER CONFIGURATION SETTINGS PAYLOAD RECEIVED ==== ######\n\n" );

                tracker_ctx.tracker_settings_payload_len =
                    tracker_parse_cmd( stack_id, settings_buffer, tracker_ctx.tracker_settings_payload,
                                       tracker_settings_payload_max_len, false );

                /* Store the new values here if it's asked */
                if( ( tracker_ctx.new_value_to_set ) == true )
                {
                    tracker_ctx.new_value_to_set = false;
                    tracker_store_app_ctx( );
                }

                if( ( tracker_ctx.lorawan_parameters_have_changed == true ) ||
                    ( tracker_ctx.reset_board_asked == true ) )
                {
                    /* reset device because of LoRaWAN Parameters */
                    HAL_DBG_TRACE_INFO( "###### ===== RESET TRACKER ==== ######\n\n" );
                    smtc_modem_hal_reset_mcu( );
                }

                tracker_app_build_and_send_tracker_settings( tracker_ctx.tracker_settings_payload,
                                                             tracker_ctx.tracker_settings_payload_len );

                break;
            }
            case GET_MODEM_DATE_CMD:
            {
                uint8_t        buffer[6];
                const uint32_t modem_date = apps_modem_common_get_utc_time( stack_id );

                buffer[0] = GET_MODEM_DATE_CMD;
                buffer[1] = GET_MODEM_DATE_ANSWER_LEN;
                buffer[2] = modem_date >> 24;
                buffer[3] = modem_date >> 16;
                buffer[4] = modem_date >> 8;
                buffer[5] = modem_date;

                /* Use the emergency TX to reduce the latency */
                smtc_modem_request_emergency_uplink( stack_id, port, false, buffer, 6 );

                HAL_DBG_TRACE_INFO( "###### ===== SEND LBM DATE IN EMERGENCY TX ==== ######\n\n" );

                break;
            }
            default:
                payload_index += len;
                break;
            }
        }
    }
    default:
        break;
    }
}

static bool tracker_app_lr11xx_check_firmware_version( void* context )
{
    lr11xx_status_t status;
    bool            has_expected_fw_version = false;
    uint8_t         lr11xx_busy_pin_state   = 0;

    smtc_board_reset_radio( context );

    /* Check if the LR1110 is in transceiver mode or Modem-E mode, busy pin in low for transceiver mode, high for
     Modem-E */
    lr11xx_busy_pin_state = smtc_board_read_busy_pin( context );

    if( lr11xx_busy_pin_state == 0 )
    {
        /* LR1110 is in transceiver mode */
        status = lr11xx_system_get_version( context, &tracker_ctx.lr11xx_fw_version );

        if( ( status == LR11XX_STATUS_OK ) && ( tracker_ctx.lr11xx_fw_version.type == 0x0001 ) )
        {
            tracker_ctx.has_lr11xx_trx_firmware = true;

            if( tracker_ctx.lr11xx_fw_version.fw < LR11XX_FW_VERSION )
            {
                /* LR11XX firmware version is not the expected one, sstay in BLE and update the LR11XX firmware */
                HAL_DBG_TRACE_ERROR( "Wrong LR11XX_FW_VERSION, current version is 0x%04X, expected is 0x%04X\n",
                                     tracker_ctx.lr11xx_fw_version.fw, LR11XX_FW_VERSION );
            }
            else
            {
                HAL_DBG_TRACE_PRINTF( "LR11XX FW : 0x%04X\n", tracker_ctx.lr11xx_fw_version.fw );
                has_expected_fw_version = true;
            }
        }
        else
        {
            HAL_DBG_TRACE_PRINTF( "NO FW\n" );
            /* LR1110 has no firmware */
            tracker_ctx.lr11xx_fw_version.fw = 0;  // Set the lr11xx_fw_version to 0 to be able to update it thought BLE
            tracker_ctx.has_lr11xx_trx_firmware = false;
            has_expected_fw_version             = false;
        }
    }
    else
    {
        HAL_DBG_TRACE_PRINTF( "Modem-E firmware\n" );
        tracker_ctx.lr11xx_fw_version.fw    = 0;  // Set the lr11xx_fw_version to 0 to be able to update it thought BLE
        tracker_ctx.has_lr11xx_trx_firmware = false;
        has_expected_fw_version             = false;
    }

    return has_expected_fw_version;
}

static void tracker_app_display_lbm_version( void )
{
    smtc_modem_return_code_t modem_response_code = SMTC_MODEM_RC_OK;

    modem_response_code = smtc_modem_get_modem_version( &tracker_ctx.firmware_version );
    if( modem_response_code == SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_INFO( "LoRa Basics Modem version: %.2x.%.2x.%.2x\n", tracker_ctx.firmware_version.major,
                            tracker_ctx.firmware_version.minor, tracker_ctx.firmware_version.patch );
    }
}

static void tracker_app_set_adr( void )
{
    /* SF9 = DR3 for EU868 / IN865 / RU864 / AU915 / CN470 /AS923 / KR920 */
    uint8_t adr_custom_list_freq_SF9_DR3[16] = { 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
                                                 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03 }; /* 125kHz - SF9 */
    /* SF9 = DR1 for US915 */
    uint8_t adr_custom_list_SF9_DR1[16] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                            0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 }; /* 125kHz - SF9 */
    uint8_t custom_nb_trans             = 3;

    /* Set the ADR profile once joined */
    switch( tracker_ctx.lorawan_region )
    {
    case SMTC_MODEM_REGION_EU_868:
    case SMTC_MODEM_REGION_IN_865:
    case SMTC_MODEM_REGION_RU_864:
    case SMTC_MODEM_REGION_AU_915:
    case SMTC_MODEM_REGION_AS_923_GRP1:
    case SMTC_MODEM_REGION_AS_923_GRP2:
    case SMTC_MODEM_REGION_AS_923_GRP3:
    case SMTC_MODEM_REGION_CN_470:
    case SMTC_MODEM_REGION_CN_470_RP_1_0:
    case SMTC_MODEM_REGION_KR_920:
        ASSERT_SMTC_MODEM_RC(
            smtc_modem_adr_set_profile( stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_freq_SF9_DR3 ) );
        ASSERT_SMTC_MODEM_RC( smtc_modem_set_nb_trans( stack_id, custom_nb_trans ) );
        break;
    case SMTC_MODEM_REGION_US_915:
        ASSERT_SMTC_MODEM_RC(
            smtc_modem_adr_set_profile( stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_SF9_DR1 ) );
        ASSERT_SMTC_MODEM_RC( smtc_modem_set_nb_trans( stack_id, custom_nb_trans ) );
        break;
    default:
        HAL_DBG_TRACE_ERROR( "Region not supported in this example, could not set custom ADR profile\n" );
        break;
    }
}

static void tracker_app_set_max_tx_power_supported( smtc_modem_region_t region, smtc_modem_sub_region_t sub_region )
{
    int8_t max_tx_power_supported = 22;

    switch( region )
    {
    case SMTC_MODEM_REGION_IN_865:
    case SMTC_MODEM_REGION_AU_915:
        max_tx_power_supported = 14;
        break;
    case SMTC_MODEM_REGION_AS_923_GRP1:
        if( sub_region == SMTC_MODEM_SUB_REGION_JAPAN )
        {
            max_tx_power_supported = 9;
        }
        break;
    case SMTC_MODEM_REGION_KR_920:
        max_tx_power_supported = 8;
        break;
    default:
        break;
    }

    smtc_board_set_max_tx_power_supported( max_tx_power_supported );
}

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER LORA BASICS MODEM EVENT FUNCTION TYPES --------------------------
 */

static void on_alarm_event( void )
{
    smtc_modem_alarm_start_timer( APP_ONE_DAY_IN_SEC );

    /* Trig Mac command */
    ASSERT_SMTC_MODEM_RC( smtc_modem_trig_lorawan_mac_request(
        stack_id, SMTC_MODEM_LORAWAN_MAC_REQ_LINK_CHECK | SMTC_MODEM_LORAWAN_MAC_REQ_DEVICE_TIME ) );
}

/*!
 * @brief LoRa Basics Modem event callbacks called by smtc_event_process function
 */

static void on_modem_reset( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem reset ==== ######\n\n" );

    if( smtc_board_is_ready( ) == true )
    {
        /* System reset */
        smtc_modem_hal_reset_mcu( );
    }
    else
    {
        smtc_board_set_ready( true );
    }
}

static void on_modem_network_joined( void )
{
    /* Stop network research notification */
    smtc_board_stop_periodic_led_pulse( );

    ASSERT_SMTC_MODEM_RC( smtc_modem_set_class( stack_id, LORAWAN_CLASS ) );
    modem_class_to_string( LORAWAN_CLASS );

    /* Set the ADR according to the region */
    tracker_app_set_adr( );

    /* Trig Mac command */
    ASSERT_SMTC_MODEM_RC( smtc_modem_trig_lorawan_mac_request(
        stack_id, SMTC_MODEM_LORAWAN_MAC_REQ_LINK_CHECK | SMTC_MODEM_LORAWAN_MAC_REQ_DEVICE_TIME ) );

    smtc_modem_alarm_start_timer( APP_ONE_DAY_IN_SEC );
}

static void on_modem_down_data( const uint8_t* payload, uint8_t size, smtc_modem_dl_metadata_t metadata,
                                uint8_t remaining_data_nb )
{
    if( size != 0 )
    {
        HAL_DBG_TRACE_INFO( "Payload size  = %d\n", size );
        HAL_DBG_TRACE_ARRAY( "Payload", payload, size );
        tracker_app_parse_downlink_frame( metadata.fport, payload, size );
    }

    smtc_board_led_pulse( smtc_board_get_led_rx_mask( ), true, LED_PERIOD_MS );
}

static void on_lorawan_mac_time( smtc_modem_event_mac_request_status_t status, uint32_t gps_time_s,
                                 uint32_t gps_fractional_s )
{
    apps_modem_common_get_utc_time( stack_id );
}

static void on_link_status( smtc_modem_event_mac_request_status_t status, uint8_t margin, uint8_t gw_cnt ) {}

static void on_gnss_scan_done( smtc_modem_gnss_event_data_scan_done_t data )
{
    HAL_DBG_TRACE_INFO( "on_gnss_scan_done\n" );

    /* Convert GPS timestamp to UTC timestamp */
    for( int i = 0; i < data.nb_scans_valid; i++ )
    {
        data.scans[i].timestamp = apps_modem_common_convert_gps_to_utc_time( data.scans[i].timestamp );
    }

    /* Store the consumption */
    tracker_ctx.gnss_scan_charge_nAh += data.power_consumption_nah;

    /* Store the scans duration */
    tracker_ctx.scans_duration = data.navgroup_duration_ms;

    /* timestamp the beginning ot the TX sequence */
    if( data.timestamp != 0 )
    {
        tracker_ctx.scans_timestamp = apps_modem_common_convert_gps_to_utc_time( data.timestamp );
    }
    else
    {
        tracker_ctx.scans_timestamp = apps_modem_common_get_utc_time( stack_id );
    }
    tracker_ctx.start_sequence_timestamp = tracker_ctx.scans_timestamp;

    /* Log results in internal memory */
    if( tracker_ctx.internal_log_enable )
    {
        tracker_store_gnss_in_internal_log( &data, tracker_ctx.scans_timestamp );
    }
}

static void on_gnss_scan_terminated( smtc_modem_gnss_event_data_terminated_t data )
{
    int32_t                  duty_cycle_status_ms = 0;
    smtc_modem_status_mask_t modem_status;

    ASSERT_SMTC_MODEM_RC( smtc_modem_get_duty_cycle_status( stack_id, &duty_cycle_status_ms ) );
    HAL_DBG_TRACE_PRINTF( "Remaining duty cycle %d ms\n", duty_cycle_status_ms );

    /* Led start for user notification */
    smtc_board_led_pulse( smtc_board_get_led_tx_mask( ), true, LED_PERIOD_MS );

    /* Has tracker moved ? */
    tracker_ctx.accelerometer_move_history =
        ( tracker_ctx.accelerometer_move_history << 1 ) + is_accelerometer_detected_moved( );

    if( ( data.nb_scans_sent == 0 ) || ( tracker_ctx.scan_priority == TRACKER_NO_PRIORITY ) )
    {
        HAL_DBG_TRACE_MSG( "Start Wi-Fi scan\n" );
        smtc_modem_wifi_scan( stack_id, 0 );
    }
    else
    {
        if( tracker_app_is_tracker_in_static_mode( ) == true )
        {
            /* Stop Hall Effect sensors while the tracker is static */
            smtc_board_hall_effect_enable( false );

            HAL_DBG_TRACE_MSG( "Switch in static mode\n" );
            smtc_modem_gnss_scan_aggregate( stack_id, true );
            smtc_modem_gnss_scan( stack_id, SMTC_MODEM_GNSS_MODE_STATIC, tracker_ctx.static_scan_interval );

            /* Send sensors values in static mode */
            smtc_modem_get_status( stack_id, &modem_status );
            if( ( modem_status >> SMTC_MODEM_STATUS_JOINED ) == 1 )
            {
                tracker_app_read_and_send_sensors( );
            }
        }
        else
        {
            HAL_DBG_TRACE_MSG( "Continue in mobile mode\n" );
            smtc_modem_gnss_scan_aggregate( stack_id, false );
            smtc_modem_gnss_scan( stack_id, SMTC_MODEM_GNSS_MODE_MOBILE, tracker_ctx.mobile_scan_interval );
        }
    }
}

static void on_gnss_almanac_demod_update( smtc_modem_almanac_demodulation_event_data_almanac_update_t data )
{
    if( ( data.status_gps == SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_COMPLETED ) &&
        ( data.status_beidou == SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_COMPLETED ) )
    {
        HAL_DBG_TRACE_INFO( "Almanac update completed\n" );
    }
}

static void on_wifi_scan_done( smtc_modem_wifi_event_data_scan_done_t data )
{
    HAL_DBG_TRACE_INFO( "on_wifi_scan_done\n" );

    // Store the consumption
    tracker_ctx.wifi_scan_charge_nAh += data.power_consumption_nah;

    if( tracker_ctx.internal_log_enable )
    {
        tracker_store_wifi_in_internal_log( &data, tracker_ctx.scans_timestamp );
    }
}

static void on_wifi_scan_terminated( smtc_modem_wifi_event_data_terminated_t data )
{
    int32_t                  duty_cycle_status_ms = 0;
    smtc_modem_status_mask_t modem_status;

    HAL_DBG_TRACE_INFO( "on_wifi_scan_terminated\n" );

    ASSERT_SMTC_MODEM_RC( smtc_modem_get_duty_cycle_status( stack_id, &duty_cycle_status_ms ) );
    HAL_DBG_TRACE_PRINTF( "Remaining duty cycle %d ms\n", duty_cycle_status_ms );

    // Led start for user notification
    smtc_board_led_pulse( smtc_board_get_led_tx_mask( ), true, LED_PERIOD_MS );

    if( ( data.nb_scans_sent == 0 ) || ( tracker_app_is_tracker_in_static_mode( ) == true ) )
    {
        HAL_DBG_TRACE_MSG( "No scan results good enough or keep alive frame, sensors values\n" );
        smtc_modem_get_status( stack_id, &modem_status );
        if( ( modem_status >> SMTC_MODEM_STATUS_JOINED ) == 1 )
        {
            tracker_app_read_and_send_sensors( );
        }
    }

    if( tracker_app_is_tracker_in_static_mode( ) == true )
    {
        // Stop Hall Effect sensors while the tracker is static
        smtc_board_hall_effect_enable( false );

        HAL_DBG_TRACE_MSG( "Switch static mode\n" );
        smtc_modem_gnss_scan_aggregate( stack_id, true );
        smtc_modem_gnss_scan( stack_id, SMTC_MODEM_GNSS_MODE_STATIC, tracker_ctx.static_scan_interval );
    }
    else
    {
        HAL_DBG_TRACE_MSG( "Continue in mobile mode\n" );
        smtc_modem_gnss_scan_aggregate( stack_id, false );
        smtc_modem_gnss_scan( stack_id, SMTC_MODEM_GNSS_MODE_MOBILE, tracker_ctx.mobile_scan_interval );
    }
}

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
