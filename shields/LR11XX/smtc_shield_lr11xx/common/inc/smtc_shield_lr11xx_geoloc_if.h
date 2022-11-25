/*!
 * @file      smtc_shield_lr11xx_geoloc_if.h
 *
 * @brief     LR11xx-based shield geolocation interface
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
#ifndef SMTC_SHIELD_LR11XX_GEOLOC_IF_H
#define SMTC_SHIELD_LR11XX_GEOLOC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_hal_trace.h"

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include "lr11xx_radio_types.h"
#include "lr11xx_system_types.h"

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
 * @brief Set up the shield to be ready to perform a GNSS scan
 */
void smtc_shield_lr11xx_handle_pre_gnss_scan( void );

/*!
 * @brief Set up the shield after a GNSS scan
 */
void smtc_shield_lr11xx_handle_post_gnss_scan( void );

/*!
 * @brief Set up the shield to be ready to perform a Wi-Fi scan
 */
void smtc_shield_lr11xx_handle_pre_wifi_scan( void );

/*!
 * @brief Set up the shield after a Wi-Fi scan
 */
void smtc_shield_lr11xx_handle_post_wifi_scan( void );

/**
 * @brief Initialize the GPIO driving the scan LED
 */
void smtc_shield_lr11xx_init_led_scan( void );

/**
 * @brief De-initialize the GPIO driving the scan LED
 */
void smtc_shield_lr11xx_deinit_led_scan( void );

/**
 * @brief Configure the GPIO to switch on the scan LED
 */
void smtc_shield_lr11xx_set_led_scan( void );

/**
 * @brief Configure the GPIO to switch off the scan LED
 */
void smtc_shield_lr11xx_reset_led_scan( void );

/**
 * @brief Return the GPIO state the TX LED
 */
uint32_t smtc_shield_lr11xx_get_led_scan_state( void );

/**
 * @brief Configure the GPIO to toggle the scan LED
 */
void smtc_shield_lr11xx_toggle_led_scan( void );

/**
 * @brief Initialize the GPIO driving the scan LNA
 */
void smtc_shield_lr11xx_init_lna_scan( void );

/**
 * @brief De-initialize the GPIO driving the scan LNA
 */
void smtc_shield_lr11xx_deinit_lna_scan( void );

/**
 * @brief Configure the GPIO to switch on the scan LNA
 */
void smtc_shield_lr11xx_set_lna_scan( void );

/**
 * @brief Configure the GPIO to switch off the scan LNA
 */
void smtc_shield_lr11xx_reset_lna_scan( void );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_SHIELD_LR11XX_GEOLOC_IF_H

/* --- EOF ------------------------------------------------------------------ */
