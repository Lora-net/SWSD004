/*!
 * @file      lr1110_trk1xks_board.h
 *
 * @brief     Target board LR11XX TRK1xKS tracker board driver definition
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

#ifndef LR11XX_TRK1XKS_BOARD_H
#define LR11XX_TRK1XKS_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "smtc_hal.h"
#include "modem_pinout.h"
#include "lis2de12.h"
#include "leds.h"
#include "hall_effect.h"
#include "usr_button.h"
#include "smtc_modem_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define BOARD_TX_POWER_OFFSET 0

/*!
 * @brief Define the battery capacity in mAh.
 */
#define TRACKER_BOARD_BATTERY_CAPACITY 2400

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum
{
    GNSS_PATCH_ANTENNA = 1,
    GNSS_PCB_ANTENNA,
} smtc_board_gnss_antenna_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Init on the LNA
 */
void smtc_board_lna_init( void );

/*!
 * @brief Deinit on the LNA
 */
void smtc_board_lna_deinit( void );

/*!
 * @brief turn on the LNA
 */
void smtc_board_lna_on( void );

/*!
 * @brief turn off the LNA
 */
void smtc_board_lna_off( void );

/*!
 * @brief Init the VCC sensors
 */
void smtc_board_vcc_sensors_init( void );

/*!
 * @brief Deinit the VCC sensors
 */
void smtc_board_vcc_sensors_deinit( void );

/*!
 * @brief Turn ON the VCC sensors
 */
void smtc_board_vcc_sensors_on( void );

/*!
 * @brief Turn Off the VCC sensors
 */
void smtc_board_vcc_sensors_off( void );

/*!
 * @brief Init the 2G4 SPDT
 */
void smtc_board_spdt_2g4_init( void );

/*!
 * @brief Deinit the 2G4 SPDT
 */
void smtc_board_spdt_2g4_deinit( void );

/*!
 * @brief Turn ON the 2G4 SPDT
 */
void smtc_board_spdt_2g4_on( void );

/*!
 * @brief Turn OFF the 2G4 SPDT
 */
void smtc_board_spdt_2g4_off( void );

/*!
 * @brief Select Wi-Fi path
 */
void smtc_board_set_wifi_antenna( void );

/*!
 * @brief Select BLE path
 */
void smtc_board_set_ble_antenna( void );

/*!
 * @brief Init the GNSS SPDT
 */
void smtc_board_spdt_gnss_init( void );

/*!
 * @brief Denit the GNSS SPDT
 */
void smtc_board_spdt_gnss_deinit( void );

/*!
 * @brief Set the GNSS patch antenna
 */
void smtc_board_set_gnss_patch_antenna( void );

/*!
 * @brief Set the GNSS PCB antenna
 */
void smtc_board_set_gnss_pcb_antenna( void );

/*!
 * @brief Measure the dropout voltage when the board drains batteries current
 *
 * @param [in] stack_id The stack identifier
 * @param [in] region LoRaWAN region used, ref \smtc_modem_region_t
 * @param [out] drop Voltage drop measured during the TX
 * @param [out] time_recovery  time taken to the supply rail to reach Vnom after the TX shutdown
 */
void smtc_board_measure_battery_drop( uint8_t stack_id, smtc_modem_region_t region, int32_t* drop,
                                      uint32_t* time_recovery );

/*!
 * @brief Enable or disable the hall effect sensor
 *
 * @param [in] enable Enable or Disable the hall effect sensor
 */
void smtc_board_hall_effect_enable( bool enable );

/*!
 * @brief Turn on the effect hall sensor for a given duration
 *
 * @param [in] duration_ms Duration of the pulse, in milliseconds
 */
void smtc_board_hall_effect_enable_for_duration( uint32_t duration_ms );

/*!
 * @brief Stop the effect hall timer and stop the sensor
 */
void smtc_board_hall_effect_stop_timer( void );

/**
 * @brief Reset the radio
 *
 * @param [in] context Radio abstraction
 */
void smtc_board_reset_radio( const void* context );

/**
 * @brief Set the radio in DFU mode
 *
 * @param [in] context Radio abstraction
 */
void smtc_board_set_radio_in_dfu( const void* context );

/**
 * @brief Read and return the state of the LR11XX busy pin
 *
 * @param [in] context Radio abstraction
 */
uint8_t smtc_board_read_busy_pin( const void* context );

/**
 * @brief Select the gnss antenna
 *
 * @param [in] antenna available antenna on the board, \ref smtc_board_gnss_antenna_t
 */
void smtc_board_select_gnss_antenna( smtc_board_gnss_antenna_t antenna );

/**
 * @brief Start a periodic led pulse
 *
 * @param [in] led_mask Led to use
 * @param [in] pulse_duration_ms Duration of the pulse
 * @param [in] period_ms Periodicity of the pulse
 */
void smtc_board_start_periodic_led_pulse( uint32_t led_mask, uint32_t pulse_duration_ms, uint32_t period_ms );

/**
 * @brief Stop the periodic led pulse
 */
void smtc_board_stop_periodic_led_pulse( void );

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_TRK1XKS_BOARD_H

/* --- EOF ------------------------------------------------------------------ */
