/*!
 * @file      apps_modem_event.c
 *
 * @brief     LoRa Basics Modem event manager implementation
 *
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
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "apps_modem_event.h"
#include "smtc_hal_dbg_trace.h"
#include "smtc_modem_api_str.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Lora Basics Modem event callback functions
 */
apps_modem_event_callback_t* apps_modem_event_callback;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void apps_modem_event_init( apps_modem_event_callback_t* event_callback )
{
    apps_modem_event_callback = event_callback;
}

void apps_modem_event_process( void )
{
    smtc_modem_event_t       current_event;
    smtc_modem_return_code_t return_code = SMTC_MODEM_RC_OK;
    uint8_t                  event_pending_count;
    uint8_t                  rx_payload[SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH] = { 0 };  // Buffer for rx payload
    uint8_t                  length       = 0;      // Size of the payload in the rx_payload buffer
    smtc_modem_dl_metadata_t rx_metadata  = { 0 };  // Metadata of downlink
    uint8_t                  rx_remaining = 0;      // Remaining downlink payload in modem
    uint32_t                 gps_time_s;
    uint32_t                 gps_fractional_s;
    uint8_t                  margin;
    uint8_t                  gw_cnt;
    smtc_modem_gnss_event_data_scan_done_t                      gnss_scan_done_data;
    smtc_modem_gnss_event_data_terminated_t                     gnss_terminated_data;
    smtc_modem_wifi_event_data_scan_done_t                      wifi_scan_done_data;
    smtc_modem_wifi_event_data_terminated_t                     wifi_terminated_data;
    smtc_modem_almanac_demodulation_event_data_almanac_update_t almanac_demodulation_update;

    do
    {
        /* Read modem event */
        return_code = smtc_modem_get_event( &current_event, &event_pending_count );

        if( return_code == SMTC_MODEM_RC_OK )
        {
            if( apps_modem_event_callback != NULL )
            {
                switch( current_event.event_type )
                {
                case SMTC_MODEM_EVENT_RESET:
                    HAL_DBG_TRACE_INFO( "###### ===== BASICS MODEM RESET EVENT ==== ######\n" );
                    if( apps_modem_event_callback->reset != NULL )
                    {
                        apps_modem_event_callback->reset( );
                    }
                    break;
                case SMTC_MODEM_EVENT_ALARM:
                    HAL_DBG_TRACE_INFO( "###### ===== ALARM EVENT ==== ######\n" );
                    if( apps_modem_event_callback->alarm != NULL )
                    {
                        apps_modem_event_callback->alarm( );
                    }
                    break;
                case SMTC_MODEM_EVENT_JOINED:
                    HAL_DBG_TRACE_INFO( "###### ===== JOINED EVENT ==== ######\n" );
                    if( apps_modem_event_callback->joined != NULL )
                    {
                        apps_modem_event_callback->joined( );
                    }
                    break;
                case SMTC_MODEM_EVENT_JOINFAIL:
                    HAL_DBG_TRACE_INFO( "###### ===== JOINED FAIL EVENT ==== ######\n" );
                    if( apps_modem_event_callback->join_fail != NULL )
                    {
                        apps_modem_event_callback->join_fail( );
                    }
                    break;
                case SMTC_MODEM_EVENT_TXDONE:
                    HAL_DBG_TRACE_INFO( "###### ===== TX DONE EVENT ==== ######\n" );
                    switch( current_event.event_data.txdone.status )
                    {
                    case SMTC_MODEM_EVENT_TXDONE_NOT_SENT:
                        HAL_DBG_TRACE_ERROR( "TX Done status: %s\n", smtc_modem_event_txdone_status_to_str(
                                                                         current_event.event_data.txdone.status ) );
                        break;
                    case SMTC_MODEM_EVENT_TXDONE_SENT:
                    case SMTC_MODEM_EVENT_TXDONE_CONFIRMED:
                    default:
                        HAL_DBG_TRACE_PRINTF( "TX Done status: %s\n", smtc_modem_event_txdone_status_to_str(
                                                                          current_event.event_data.txdone.status ) );
                        break;
                    }
                    if( apps_modem_event_callback->tx_done != NULL )
                    {
                        apps_modem_event_callback->tx_done( current_event.event_data.txdone.status );
                    }
                    break;
                case SMTC_MODEM_EVENT_DOWNDATA:
                    HAL_DBG_TRACE_INFO( "###### ===== DOWNLINK EVENT ==== ######\n" );
                    smtc_modem_get_downlink_data( rx_payload, &length, &rx_metadata, &rx_remaining );

                    HAL_DBG_TRACE_PRINTF( "Rx window: %s\n",
                                          smtc_modem_event_downdata_window_to_str( rx_metadata.window ) );
                    HAL_DBG_TRACE_PRINTF( "Rx port: %d\n", rx_metadata.fport );
                    HAL_DBG_TRACE_PRINTF( "Rx RSSI: %d\n", rx_metadata.rssi - 64 );
                    HAL_DBG_TRACE_PRINTF( "Rx SNR: %d\n", rx_metadata.snr / 4 );

                    if( apps_modem_event_callback->down_data != NULL )
                    {
                        apps_modem_event_callback->down_data( rx_payload, length, rx_metadata, rx_remaining );
                    }
                    break;
                case SMTC_MODEM_EVENT_UPLOAD_DONE:
                    HAL_DBG_TRACE_INFO( "###### ===== UPLOAD DONE EVENT ==== ######\n" );
                    HAL_DBG_TRACE_PRINTF( "Upload status: %s\n", smtc_modem_event_uploaddone_status_to_str(
                                                                     current_event.event_data.uploaddone.status ) );
                    if( apps_modem_event_callback->upload_done != NULL )
                    {
                        apps_modem_event_callback->upload_done( current_event.event_data.uploaddone.status );
                    }
                    break;
                case SMTC_MODEM_EVENT_DM_SET_CONF:
                    HAL_DBG_TRACE_INFO( "###### ===== SET CONF EVENT ==== ######\n" );
                    HAL_DBG_TRACE_PRINTF(
                        "Tag: %s", smtc_modem_event_setconf_opcode_to_str( current_event.event_data.setconf.opcode ) );
                    if( apps_modem_event_callback->set_conf != NULL )
                    {
                        apps_modem_event_callback->set_conf( current_event.event_data.setconf.opcode );
                    }
                    break;
                case SMTC_MODEM_EVENT_MUTE:
                    HAL_DBG_TRACE_INFO( "###### ===== MUTE EVENT ==== ######\n" );
                    HAL_DBG_TRACE_PRINTF( "Mute: %s\n",
                                          smtc_modem_event_mute_status_to_str( current_event.event_data.mute.status ) );
                    if( apps_modem_event_callback->mute != NULL )
                    {
                        apps_modem_event_callback->mute( current_event.event_data.mute.status );
                    }
                    break;
                case SMTC_MODEM_EVENT_STREAM_DONE:
                    HAL_DBG_TRACE_INFO( "###### ===== STREAM DONE EVENT ==== ######\n" );
                    if( apps_modem_event_callback->stream_done != NULL )
                    {
                        apps_modem_event_callback->stream_done( );
                    }
                    break;
                case SMTC_MODEM_EVENT_LORAWAN_MAC_TIME:
                    HAL_DBG_TRACE_INFO( "###### ===== LORAWAN MAC TIME EVENT ==== ######\n" );
                    HAL_DBG_TRACE_PRINTF( "LoRaWAN MAC Time status : %s\n",
                                          smtc_modem_event_mac_request_status_to_str(
                                              current_event.event_data.lorawan_mac_time.status ) );

                    smtc_modem_get_lorawan_mac_time( current_event.stack_id, &gps_time_s, &gps_fractional_s );

                    if( apps_modem_event_callback->lorawan_mac_time != NULL )
                    {
                        apps_modem_event_callback->lorawan_mac_time( current_event.event_data.lorawan_mac_time.status,
                                                                     gps_time_s, gps_fractional_s );
                    }

                    break;
                case SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE:
                    HAL_DBG_TRACE_INFO( "###### ===== FUOTA DONE EVENT ==== ######\n" );
                    if( apps_modem_event_callback->lorawan_fuota_done != NULL )
                    {
                        apps_modem_event_callback->lorawan_fuota_done( current_event.event_data.fmp.status );
                    }
                    break;

                case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C:
                    HAL_DBG_TRACE_INFO( "###### ===== NO MORE MULTICAST SESSION CLASS C EVENT ==== ######\n" );
                    if( apps_modem_event_callback->no_more_multicast_session_class_c != NULL )
                    {
                        apps_modem_event_callback->no_more_multicast_session_class_c( );
                    }
                    break;

                case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B:
                    HAL_DBG_TRACE_INFO( "###### ===== NO MORE MULTICAST SESSION CLASS B EVENT ==== ######\n" );
                    if( apps_modem_event_callback->no_more_multicast_session_class_b != NULL )
                    {
                        apps_modem_event_callback->no_more_multicast_session_class_b( );
                    }
                    break;

                case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C:
                    HAL_DBG_TRACE_INFO( "###### ===== NEW MULTICAST SESSION CLASS C EVENT ==== ######\n" );
                    HAL_DBG_TRACE_PRINTF( "Multicast session: %s\n",
                                          smtc_modem_event_new_multicast_group_session_to_str(
                                              current_event.event_data.new_multicast_class_c.group_id ) );
                    if( apps_modem_event_callback->new_multicast_session_class_c != NULL )
                    {
                        apps_modem_event_callback->new_multicast_session_class_c(
                            current_event.event_data.new_multicast_class_c.group_id );
                    }
                    break;

                case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B:
                    HAL_DBG_TRACE_INFO( "###### ===== NEW MULTICAST SESSION CLASS B EVENT ==== ######\n" );
                    HAL_DBG_TRACE_PRINTF( "Multicast session: %s\n",
                                          smtc_modem_event_new_multicast_group_session_to_str(
                                              current_event.event_data.new_multicast_class_b.group_id ) );
                    if( apps_modem_event_callback->new_multicast_session_class_b != NULL )
                    {
                        apps_modem_event_callback->new_multicast_session_class_b(
                            current_event.event_data.new_multicast_class_b.group_id );
                    }
                    break;

                case SMTC_MODEM_EVENT_LINK_CHECK:
                    HAL_DBG_TRACE_INFO( "###### ===== LINK CHECK EVENT ==== ######\n" );

                    smtc_modem_get_lorawan_link_check_data( current_event.stack_id, &margin, &gw_cnt );

                    HAL_DBG_TRACE_PRINTF( "Link status: %s\n", smtc_modem_event_mac_request_status_to_str(
                                                                   current_event.event_data.link_check.status ) );
                    HAL_DBG_TRACE_PRINTF( "Margin: %d dB\n", margin );
                    HAL_DBG_TRACE_PRINTF( "Number of gateways: %d\n", gw_cnt );
                    if( apps_modem_event_callback->link_status != NULL )
                    {
                        apps_modem_event_callback->link_status( current_event.event_data.link_check.status, margin,
                                                                gw_cnt );
                    }
                    break;
                case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
                    HAL_DBG_TRACE_INFO( "###### ===== CLASS B PING SLOT INFO EVENT ==== ######\n" );
                    HAL_DBG_TRACE_PRINTF( "Class B ping slot status: %s\n",
                                          smtc_modem_event_mac_request_status_to_str(
                                              current_event.event_data.class_b_ping_slot_info.status ) );
                    if( apps_modem_event_callback->class_b_ping_slot_info != NULL )
                    {
                        apps_modem_event_callback->class_b_ping_slot_info(
                            current_event.event_data.class_b_ping_slot_info.status );
                    }
                    break;
                case SMTC_MODEM_EVENT_CLASS_B_STATUS:
                    HAL_DBG_TRACE_INFO( "###### ===== CLASS B STATUS EVENT ==== ######\n" );
                    HAL_DBG_TRACE_PRINTF(
                        "Class B status: %s\n",
                        smtc_modem_event_class_b_status_to_str( current_event.event_data.class_b_status.status ) );
                    if( apps_modem_event_callback->class_b_status != NULL )
                    {
                        apps_modem_event_callback->class_b_status( current_event.event_data.class_b_status.status );
                    }
                    break;
                case SMTC_MODEM_EVENT_GNSS_SCAN_DONE:
                    HAL_DBG_TRACE_INFO( "###### ===== GNSS SCAN DONE EVENT ==== ######\n" );

                    smtc_modem_gnss_get_event_data_scan_done( current_event.stack_id, &gnss_scan_done_data );

                    if( apps_modem_event_callback->gnss_scan_done != NULL )
                    {
                        apps_modem_event_callback->gnss_scan_done( gnss_scan_done_data );
                    }
                    break;
                case SMTC_MODEM_EVENT_GNSS_TERMINATED:
                    HAL_DBG_TRACE_INFO( "###### ===== GNSS TERMINATED EVENT ==== ######\n" );

                    smtc_modem_gnss_get_event_data_terminated( current_event.stack_id, &gnss_terminated_data );

                    if( apps_modem_event_callback->gnss_terminated != NULL )
                    {
                        apps_modem_event_callback->gnss_terminated( gnss_terminated_data );
                    }
                    break;
                case SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE:
                    HAL_DBG_TRACE_INFO( "###### ===== ALMANAC DEMOD UPDATE EVENT ==== ######\n" );
                    smtc_modem_almanac_demodulation_get_event_data_almanac_update( current_event.stack_id,
                                                                                   &almanac_demodulation_update );
                    if( apps_modem_event_callback->gnss_almanac_demod_update != NULL )
                    {
                        apps_modem_event_callback->gnss_almanac_demod_update( almanac_demodulation_update );
                    }
                    break;
                case SMTC_MODEM_EVENT_WIFI_SCAN_DONE:
                    HAL_DBG_TRACE_INFO( "###### ===== WIFI SCAN DONE EVENT ==== ######\n" );

                    smtc_modem_wifi_get_event_data_scan_done( current_event.stack_id, &wifi_scan_done_data );

                    if( apps_modem_event_callback->wifi_scan_done != NULL )
                    {
                        apps_modem_event_callback->wifi_scan_done( wifi_scan_done_data );
                    }
                    break;
                case SMTC_MODEM_EVENT_WIFI_TERMINATED:
                    HAL_DBG_TRACE_INFO( "###### ===== WIFI TERMINATED EVENT ==== ######\n" );

                    smtc_modem_wifi_get_event_data_terminated( current_event.stack_id, &wifi_terminated_data );

                    if( apps_modem_event_callback->wifi_terminated != NULL )
                    {
                        apps_modem_event_callback->wifi_terminated( wifi_terminated_data );
                    }
                    break;
                case SMTC_MODEM_EVENT_MAX:
                    break;
                default:
                    HAL_DBG_TRACE_INFO( "###### ===== UNKNOWN EVENT %u ==== ######\n", current_event.event_type );
                    break;
                }
            }
            else
            {
                HAL_DBG_TRACE_ERROR( "lora_basics_modem_event_callback not defined\n" );
            }
        }
        else
        {
            if( return_code != SMTC_MODEM_RC_NO_EVENT )
            {
                HAL_DBG_TRACE_ERROR( "smtc_modem_get_event != SMTC_MODEM_RC_OK\nreturn_code: %s\n",
                                     smtc_modem_return_code_to_str( return_code ) );
            }
        }
    } while( ( return_code == SMTC_MODEM_RC_OK ) && ( current_event.event_type != SMTC_MODEM_EVENT_MAX ) );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
