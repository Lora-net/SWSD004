/*!
 * @file      apps_modem_event.h
 *
 * @brief     LoRa Basics Modem event manager definition
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

#ifndef APPS_MODEM_EVENT_H
#define APPS_MODEM_EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "smtc_modem_api.h"
#include "smtc_modem_geolocation_api.h"

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
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Lora Basics Modem callback functions
 */
typedef struct
{
    /*!
     * @brief  Reset callback prototype.
     */
    void ( *reset )( void );
    /*!
     * @brief  Alarm timer expired callback prototype.
     */
    void ( *alarm )( void );
    /*!
     * @brief  Attemp to join network failed callback prototype.
     */
    void ( *joined )( void );
    /*!
     * @brief  Joined callback prototype.
     */
    void ( *join_fail )( void );
    /*!
     * @brief  Tx done callback prototype.
     *
     * @param [in] status
     */
    void ( *tx_done )( smtc_modem_event_txdone_status_t status );
    /*!
     * @brief Downlink data received callback prototype.
     *
     * @param [in] payload Received buffer pointer
     * @param [in] length    Received buffer size
     * @param [in] metadata          Structure holding downlink metadata
     * @param [in] remaining_data_nb Number of downlink data remaining
     */
    void ( *down_data )( const uint8_t* payload, uint8_t length, smtc_modem_dl_metadata_t metadata,
                         uint8_t remaining_data_nb );
    /*!
     * @brief  File upload completed callback prototype.
     *
     * @param [in] status \see smtc_modem_event_uploaddone_status_t
     */
    void ( *upload_done )( smtc_modem_event_uploaddone_status_t status );
    /*!
     * @brief  Set conf changed by DM callback prototype.
     *
     * @param [in] opcode \see smtc_modem_event_setconf_opcode_t
     */
    void ( *set_conf )( smtc_modem_event_setconf_opcode_t opcode );
    /*!
     * @brief  Mute callback prototype.
     *
     * @param [in] status \see smtc_modem_event_mute_status_t
     */
    void ( *mute )( smtc_modem_event_mute_status_t status );
    /*!
     * @brief  Data stream fragments sent callback prototype.
     */
    void ( *stream_done )( void );
    /*!
     * @brief  Alc sync callback prototype.
     *
     * @param [in] gps_time_s       GPS time in seconds
     */
    void ( *time_updated_alc_sync )( uint32_t gps_time_s );
    /*!
     * @brief  LoRaWAN time callback prototype.
     *
     * @param [in] status       Mac request status
     * @param [in] gps_time_s       GPS time in seconds
     * @param [in] gps_fractional_s GPS fractional second
     */
    void ( *lorawan_mac_time )( smtc_modem_event_mac_request_status_t status, uint32_t gps_time_s,
                                uint32_t gps_fractional_s );
    /*!
     * @brief  LoRaWAN FUOTA done callback prototype.
     *
     * @param [in] status       Firmware Management Package status
     */
    void ( *lorawan_fuota_done )( smtc_modem_event_fmp_status_t status );
    /*!
     * @brief  No more multicast session class C callback prototype.
     */
    void ( *no_more_multicast_session_class_c )( void );
    /*!
     * @brief  No more multicast session class b callback prototype.
     */
    void ( *no_more_multicast_session_class_b )( void );
    /*!
     * @brief  No more multicast session class C callback prototype.
     *
     * @param [in] group_id       Multicast group identifier
     */
    void ( *new_multicast_session_class_c )( smtc_modem_mc_grp_id_t group_id );
    /*!
     * @brief  No more multicast session class B callback prototype.
     *
     * @param [in] group_id       Multicast group identifier
     */
    void ( *new_multicast_session_class_b )( smtc_modem_mc_grp_id_t group_id );
    /*!
     * @brief  Link Status request callback prototype.
     *
     * @param [in] status \see smtc_modem_event_mac_request_status_t
     * @param [in] margin The demodulation margin in dB
     * @param [in] gw_cnt number of gateways that received the most recent LinkCheckReq command
     */
    void ( *link_status )( smtc_modem_event_mac_request_status_t status, uint8_t margin, uint8_t gw_cnt );
    /*!
     * @brief  Almanac update callback prototype.
     *
     * @param [in] status \see smtc_modem_event_almanac_update_status_t
     */
    // void ( *almanac_update )( smtc_modem_event_almanac_update_status_t status );
    /*!
     * @brief  User radio access callback prototype.
     *
     * @param [in] timestamp_ms timestamp in ms of the radio irq
     * @param [in] status Interrupt status
     */
    // void ( *user_radio_access )( uint32_t timestamp_ms, smtc_modem_event_user_radio_access_status_t status );
    /*!
     * @brief  Class B ping slot status callback prototype
     *
     * @param [in] status Class B ping slot status
     */
    void ( *class_b_ping_slot_info )( smtc_modem_event_mac_request_status_t status );
    /*!
     * @brief  Class B status callback prototype
     *
     * @param [in] status Class B status
     */
    void ( *class_b_status )( smtc_modem_event_class_b_status_t status );
    /*!
     * @brief  GNSS Scan done callback prototype.
     *
     * @param [in] data Scan done data, see \ref smtc_modem_gnss_event_data_scan_done_t
     */
    void ( *gnss_scan_done )( smtc_modem_gnss_event_data_scan_done_t     data);
    /*!
     * @brief  GNSS terminated callback prototype.
     *
     * @param [in] data Scan terminated data, see \ref smtc_modem_gnss_event_data_terminated_t
     */
    void ( *gnss_terminated )( smtc_modem_gnss_event_data_terminated_t data );
    /*!
     * @brief  GNSS almanac demod callback prototype.
     *
     * @param [in] data Almanac demod update, see \ref smtc_modem_almanac_demodulation_event_data_almanac_update_t
     */
    void ( *gnss_almanac_demod_update )( smtc_modem_almanac_demodulation_event_data_almanac_update_t data );
    /*!
     * @brief  WIFI Scan done callback prototype.
     *
     * @param [in] data Scan done data, see \ref smtc_modem_wifi_event_data_scan_done_t
     */
    void ( *wifi_scan_done )( smtc_modem_wifi_event_data_scan_done_t data );
    /*!
     * @brief  WIFI terminated callback prototype.
     *
     * @param [in] data Scan terminated data, see \ref smtc_modem_wifi_event_data_terminated_t
     */
    void ( *wifi_terminated )( smtc_modem_wifi_event_data_terminated_t data );
} apps_modem_event_callback_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Init the Lora Basics Modem event callbacks
 *
 * @param [in] event_callback Lora basics modem event callback \ref apps_modem_event_callback_t
 */
void apps_modem_event_init( apps_modem_event_callback_t* event_callback );

/*!
 * @brief Process the analysis of lora basics modem event and calls callback functions
 *        depending on event. This callback is called every time an event ( see modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
void apps_modem_event_process( void );

#ifdef __cplusplus
}
#endif

#endif  // APPS_MODEM_EVENT_H

/* --- EOF ------------------------------------------------------------------ */
