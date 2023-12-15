/**
 * @file      almanac.h
 *
 * @brief     LoRaWAN template
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

#ifndef ALMANAC_H
#define ALMANAC_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include "lr1_stack_mac_layer.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#define ALMANAC_DW_SIZE 255
#ifndef ALMANAC_PERIOD_S
#define ALMANAC_PERIOD_S 86400  // 24h
#endif
#define ALMANAC_1SECOND 1
#define ALMANAC_UP_COUNT_INIT 3

#define SERVICE_LR11XX_GNSS_CONTEXT_STATUS_LENGTH 9

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief LoRaWAN template Object
 *
 * @struct almanac_s
 *
 */

typedef struct almanac_s
{
    uint8_t stack_id;
    uint8_t task_id;
    bool    enabled;
    bool    initialized;
    uint8_t almanac_dw_buffer[ALMANAC_DW_SIZE];
    uint8_t almanac_dw_buffer_size;
    uint8_t almanac_status_from_lr11xx[SERVICE_LR11XX_GNSS_CONTEXT_STATUS_LENGTH];
    bool    get_almanac_status_from_lr11xx;
    uint8_t up_delay;
    uint8_t up_count;
    uint8_t rp_hook_id;
} almanac_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

////////////////////////////////////////////////////////
//////////// Init service object ///////////////////////
////////////////////////////////////////////////////////

/**
 * @brief Init a new LoRaWAN template services object
 *
 * @param service_id
 * @param task_id
 * @param downlink_callback
 * @param on_launch_callback
 * @param on_update_callback
 * @return bool
 */
void almanac_services_init( uint8_t* service_id, uint8_t task_id,
                            uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                            void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                            void** context_callback );

/**
 * @brief Callback called at task launch
 *
 * @param context_callback
 */
void almanac_service_on_launch( void* context );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
void almanac_service_on_update( void* context );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
uint8_t almanac_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief add generic task
 *
 * @param stack_id
 */
void start_almanac_service( uint8_t stack_id );

void stop_almanac_service( uint8_t stack_id );

#ifdef __cplusplus
}
#endif

#endif  // almanac_H

/* --- EOF ------------------------------------------------------------------ */
