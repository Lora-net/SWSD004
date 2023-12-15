/*!
 * @file      apps_modem_common.c
 *
 * @brief     Common functions shared by the examples
 *
 * @copyright
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

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "apps_modem_common.h"
#include "apps_utilities.h"
#include "lorawan_key_config.h"
#include "smtc_modem_api.h"
#include "smtc_board.h"
#include "smtc_hal.h"
#include "smtc_modem_api_str.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * @brief Offset in second between GPS EPOCH and UNIX EPOCH time
 */
#define OFFSET_BETWEEN_GPS_EPOCH_AND_UNIX_EPOCH 315964800

/*!
 * @brief Number of sec in 1024 weeks
 */
#define ONE_WEEK_NUMBER_ROLLOVER_SEC 619315200

/*!
 * @brief Week number rollover between 2019 and 2038
 */
#define WEEK_NUMBER_ROLLOVER_2019_2038 2

/*!
 * @brief Number of leap seconds as of September 15th 2021
 */
#define OFFSET_LEAP_SECONDS 18

/*!
 * @brief Size of the buffer used as placeholder for the human-readable time
 */
#define TIME_BUFFER_SIZE 80

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

uint32_t apps_modem_common_get_utc_time( uint8_t stack_id )
{
    uint32_t gps_time_s       = 0;
    uint32_t gps_fractional_s = 0;
    time_t   time_utc         = 0;

    const smtc_modem_return_code_t status = smtc_modem_get_lorawan_mac_time( stack_id, &gps_time_s, &gps_fractional_s );

    switch( status )
    {
    case SMTC_MODEM_RC_OK:
    {
        time_utc = ( time_t )( gps_time_s + OFFSET_BETWEEN_GPS_EPOCH_AND_UNIX_EPOCH - OFFSET_LEAP_SECONDS );

        char             buf[TIME_BUFFER_SIZE];
        const struct tm* time = localtime( &time_utc );  // localtime() is used here instead of gmtime() because the
                                                         // latter is not implemented in the libraries used by Keil

        strftime( buf, TIME_BUFFER_SIZE, "%a %Y-%m-%d %H:%M:%S %Z", time );
        HAL_DBG_TRACE_INFO( "Current UTC time: %s\n", buf );

        break;
    }
    case SMTC_MODEM_RC_NO_TIME:
    {
        HAL_DBG_TRACE_WARNING( "No time available.\n" );
        time_utc = 0;
        break;
    }
    default:
    {
        HAL_DBG_TRACE_ERROR( "Cannot get time from modem\n" );
        time_utc = 0;
        break;
    }
    }

    return ( ( uint32_t ) time_utc );
}

uint32_t apps_modem_common_convert_gps_to_utc_time( uint32_t gps_time_s )
{
    return gps_time_s + ( ONE_WEEK_NUMBER_ROLLOVER_SEC * WEEK_NUMBER_ROLLOVER_2019_2038 ) +
           OFFSET_BETWEEN_GPS_EPOCH_AND_UNIX_EPOCH - OFFSET_LEAP_SECONDS;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
