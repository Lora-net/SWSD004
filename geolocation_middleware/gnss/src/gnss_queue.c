/**
 * @file      gnss_queue.c
 *
 * @brief     Implementation of the GNSS scan group queue.
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stddef.h>
#include <string.h>

#include "mw_dbg_trace.h"
#include "gnss_queue.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define GNSS_QUEUE_FEATURE_OFF 0
#define GNSS_QUEUE_FEATURE_ON !GNSS_QUEUE_FEATURE_OFF

#ifndef GNSS_QUEUE_DBG_TRACE
#define GNSS_QUEUE_DBG_TRACE GNSS_QUEUE_FEATURE_OFF /* Enable/Disable traces here */
#endif

#if( GNSS_QUEUE_DBG_TRACE == GNSS_QUEUE_FEATURE_ON )
#define GNSS_QUEUE_TRACE_MSG( msg ) \
    do                              \
    {                               \
        MW_DBG_TRACE_PRINTF( msg ); \
    } while( 0 )

#define GNSS_QUEUE_TRACE_PRINTF( ... )      \
    do                                      \
    {                                       \
        MW_DBG_TRACE_PRINTF( __VA_ARGS__ ); \
    } while( 0 )

#define GNSS_QUEUE_PRINT( queue )                                                                     \
    {                                                                                                 \
        GNSS_QUEUE_TRACE_PRINTF( "****************************************\n" );                      \
        GNSS_QUEUE_TRACE_PRINTF( "token:       0x%02X\n", queue->token );                             \
        GNSS_QUEUE_TRACE_PRINTF( "group_size:  %d\n", queue->scan_group_size );                       \
        GNSS_QUEUE_TRACE_PRINTF( "scan_valid:  %d\n", queue->nb_scans_valid );                        \
        GNSS_QUEUE_TRACE_PRINTF( "scan_total:  %d\n", queue->nb_scans_total );                        \
        GNSS_QUEUE_TRACE_PRINTF( "scan_sent:   %d\n", queue->nb_scans_sent );                         \
        GNSS_QUEUE_TRACE_PRINTF( "abort:       %d\n", queue->abort );                                 \
        if( queue->mode == GNSS_SCAN_GROUP_MODE_DEFAULT )                                             \
        {                                                                                             \
            GNSS_QUEUE_TRACE_PRINTF( "mode:        DEFAULT\n" );                                      \
            GNSS_QUEUE_TRACE_PRINTF( "min_sv:      %d\n", queue->nb_svs_threshold );                  \
        }                                                                                             \
        else                                                                                          \
        {                                                                                             \
            GNSS_QUEUE_TRACE_PRINTF( "mode:        SENSITIVITY\n" );                                  \
        }                                                                                             \
        GNSS_QUEUE_TRACE_PRINTF( "power_cons:  %d uah\n", queue->power_consumption_uah );             \
        for( uint8_t i = 0; i < queue->nb_scans_valid; i++ )                                          \
        {                                                                                             \
            GNSS_QUEUE_TRACE_PRINTF( "scans[%d]: %02d %02d %02d - ", i, queue->scans[i].detected_svs, \
                                     queue->scans[i].results_size, queue->scans[i].nav_valid );       \
            for( uint8_t j = 0; j < ( GNSS_SCAN_METADATA_SIZE + queue->scans[i].results_size ); j++ ) \
            {                                                                                         \
                GNSS_QUEUE_TRACE_PRINTF( "%02X ", queue->scans[i].results_buffer[j] );                \
            }                                                                                         \
            GNSS_QUEUE_TRACE_PRINTF( "\n" );                                                          \
        }                                                                                             \
        GNSS_QUEUE_TRACE_PRINTF( "****************************************\n" );                      \
    }

#else
#define GNSS_QUEUE_TRACE_MSG( msg )
#define GNSS_QUEUE_TRACE_PRINTF( ... )
#define GNSS_QUEUE_PRINT( queue )
#endif

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void gnss_scan_group_queue_reset_token( gnss_scan_group_queue_t* queue )
{
    if( queue != NULL )
    {
        queue->token = 0x7F; /* maximum value for 7-bits token */
    }
}

void gnss_scan_group_queue_increment_token( gnss_scan_group_queue_t* queue )
{
    if( queue != NULL )
    {
        queue->token = ( queue->token + 1 ) % 0x80; /* roll-over on 7-bits */
    }
}

bool gnss_scan_group_queue_new( gnss_scan_group_queue_t* queue, uint8_t scan_group_size, gnss_scan_group_mode_t mode,
                                uint8_t nb_svs_threshold )
{
    if( ( queue != NULL ) && ( scan_group_size <= GNSS_SCAN_GROUP_SIZE_MAX ) )
    {
        /* queue params */
        queue->scan_group_size = scan_group_size;
        queue->mode            = mode;
        if( queue->mode == GNSS_SCAN_GROUP_MODE_DEFAULT )
        {
            queue->nb_svs_threshold = nb_svs_threshold;
        }
        else
        {
            /* in SENSITIVITY mode, a scan is valid if there is more that 0 SV detected */
            queue->nb_svs_threshold = 1;
        }

        /* reset queue current status */
        queue->nb_scans_valid        = 0;
        queue->nb_scans_total        = 0;
        queue->nb_scans_sent         = 0;
        queue->power_consumption_uah = 0;
        queue->abort                 = false;

        /* reset queue buffers */
        memset( queue->scans, 0, sizeof queue->scans );

        GNSS_QUEUE_TRACE_PRINTF( "%s:\n", __FUNCTION__ );
        GNSS_QUEUE_PRINT( queue );

        return true;
    }

    return false;
}

bool gnss_scan_group_queue_is_full( gnss_scan_group_queue_t* queue )
{
    /* Several conditions will trigger the queue to be terminated:
        - an abort occurred because not enough SV detected (GNSS_SCAN_GROUP_MODE_DEFAULT mode)
        - the number of scan done reached group size
        */
    if( queue != NULL )
    {
        return ( ( queue->abort == true ) || ( queue->nb_scans_total == queue->scan_group_size ) );
    }

    return false;
}

bool gnss_scan_group_queue_is_valid( gnss_scan_group_queue_t* queue )
{
    if( queue != NULL )
    {
        if( queue->mode == GNSS_SCAN_GROUP_MODE_DEFAULT )
        {
            /* All scans of the group have been done, and all are valid */
            return ( queue->nb_scans_valid == queue->scan_group_size );
        }
        else /* GNSS_SCAN_GROUP_MODE_SENSITIVITY */
        {
            if( ( ( queue->nb_scans_valid == 1 ) && ( queue->scans[0].nav_valid == true ) ) ||
                ( queue->nb_scans_valid > 1 ) )
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    return false;
}

void gnss_scan_group_queue_push( gnss_scan_group_queue_t* queue, gnss_scan_t* scan )
{
    if( ( queue != NULL ) && ( scan != NULL ) )
    {
        /* Add the scan to the queue if valid */
        if( scan->detected_svs >= ( queue->nb_svs_threshold ) ) /* nb_sv_threshold set to 1 in HIGH_SENSITIVITY mode */
        {
            memcpy( &( queue->scans[queue->nb_scans_valid] ), scan, sizeof( gnss_scan_t ) );
            queue->nb_scans_valid += 1;
        }
        else
        {
            if( queue->mode == GNSS_SCAN_GROUP_MODE_DEFAULT )
            {
                /* If no enough SV detected, we abort the current scan group */
                queue->abort = true;
                GNSS_QUEUE_TRACE_PRINTF( "%s: scan not valid, abort scan group\n", __FUNCTION__ );
            }
        }

        queue->nb_scans_total += 1;

        GNSS_QUEUE_TRACE_PRINTF( "%s:\n", __FUNCTION__ );
        GNSS_QUEUE_PRINT( queue );
    }
}

bool gnss_scan_group_queue_pop( gnss_scan_group_queue_t* queue, uint8_t** buffer, uint8_t* buffer_size )
{
    if( ( queue != NULL ) && gnss_scan_group_queue_is_valid( queue ) &&
        ( queue->nb_scans_sent < queue->nb_scans_valid ) && ( queue->abort == false ) )
    {
        const uint8_t index   = queue->nb_scans_sent;
        const uint8_t is_last = ( queue->nb_scans_sent == ( queue->nb_scans_valid - 1 ) );

        /* Set scan group metadata
            | last NAV (1bit) | token (7bits) |
            - token: scan group identifier
            - last NAV: indicates if this is the last NAV message of a scan group
        */
        queue->scans[index].results_buffer[0] = ( is_last << 7 ) | ( queue->token & 0x7F );

        /* Update queue info */
        queue->nb_scans_sent += 1;

        /* Return a pointer to the buffer to be sent, and its size */
        *buffer      = queue->scans[index].results_buffer;
        *buffer_size = GNSS_SCAN_METADATA_SIZE + queue->scans[index].results_size;

        GNSS_QUEUE_TRACE_PRINTF( "%s:\n", __FUNCTION__ );
        GNSS_QUEUE_PRINT( queue );

        return true;
    }

    /* scan results to be sent */
    *buffer      = NULL;
    *buffer_size = 0;
    GNSS_QUEUE_TRACE_PRINTF( "%s: no scan result left in queue\n", __FUNCTION__ );

    return false;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */