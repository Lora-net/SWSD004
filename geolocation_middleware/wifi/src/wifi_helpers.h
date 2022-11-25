/**
 * @file      wifi_helpers.h
 *
 * @brief     Interface between the WI-Fi middleware and the LR11xx radio driver.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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

#ifndef __WIFI_HELPERS_H__
#define __WIFI_HELPERS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

#include "wifi_helpers_defs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initialise the settings for Wi-Fi scan
 *
 * @param [in] wifi_settings Wi-Fi settings \ref wifi_settings_t
 */
void smtc_wifi_settings_init( const wifi_settings_t* wifi_settings );

/*!
 * @brief Start a Wi-Fi scan
 *
 * @param [in] radio_context Chip implementation context
 *
 * @return a boolean: true for success, false otherwise
 */
bool smtc_wifi_start_scan( const void* radio_context );

/*!
 * @brief Fetch the results obtained during previous Wi-Fi scan
 *
 * @param [in] radio_context Chip implementation context
 * @param [out] result Scan results \ref wifi_scan_all_result_t
 *
 * @return a boolean: true for success, false otherwise
 */
bool smtc_wifi_get_results( const void* radio_context, wifi_scan_all_result_t* result );

/*!
 * @brief Tear down function for Wi-Fi scan termination actions
 *
 * This function is typically to be called when during the handling of the event of user radio access
 */
void smtc_wifi_scan_ended( void );

/*!
 * @brief Get the power consumption of the last scan
 *
 * @param [in] radio_context Chip implementation context
 * @param [out] power_consumption_uah Power consumption of the last scan in uAh
 *
 * @return a boolean: true for success, false otherwise
 */
bool smtc_wifi_get_power_consumption( const void* radio_context, uint32_t* power_consumption_uah );

#ifdef __cplusplus
}
#endif

#endif  // __WIFI_HELPERS_H__

/* --- EOF ------------------------------------------------------------------ */
