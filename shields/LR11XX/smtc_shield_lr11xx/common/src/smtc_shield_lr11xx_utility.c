/*!
 * @file      lr1110_mb1lxks_board.c
 *
 * @brief     Target board LR1110 MB1LxKS shield board board driver implementation
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

#include <stdlib.h>

#include "smtc_board.h"
#include "smtc_shield_lr11xx_common_if.h"
#include "smtc_shield_lr11xx_geoloc_if.h"
#include "smtc_lr11xx_board.h"

#include "smtc_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

typedef enum
{
    SMTC_SHIELD_LR11XX_LED_TX,
    SMTC_SHIELD_LR11XX_LED_RX,
    SMTC_SHIELD_LR11XX_LED_SCAN,
    SMTC_SHIELD_LR11XX_LED_COUNT
} smtc_shield_lr11xx_led_t;

/*!
 * @brief LED TX MASK
 */
#define SMTC_SHIELD_LR11XX_LED_TX_MASK ( 1 << SMTC_SHIELD_LR11XX_LED_TX )

/*!
 * @brief LED RX MASK
 */
#define SMTC_SHIELD_LR11XX_LED_RX_MASK ( 1 << SMTC_SHIELD_LR11XX_LED_RX )

/*!
 * @brief LED SCAN MASK
 */
#define SMTC_SHIELD_LR11XX_LED_SCAN_MASK ( 1 << SMTC_SHIELD_LR11XX_LED_SCAN )

/*!
 * @brief LED ALL MASK
 */
#define SMTC_SHIELD_LR11XX_LED_ALL_MASK \
    ( SMTC_SHIELD_LR11XX_LED_TX_MASK | SMTC_SHIELD_LR11XX_LED_RX_MASK | SMTC_SHIELD_LR11XX_LED_SCAN_MASK )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief LR11XX EVK LED context
 */
typedef struct
{
    timer_event_t led_timer;         /*!< @brief Pulse timer */
    bool          timer_initialized; /*!< @brief True if the pulse timer has been initialized, false otherwise */
} smtc_shield_lr11xx_led_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief LR11XX EVK LED context array
 */
static smtc_shield_lr11xx_led_ctx_t smtc_shield_lr11xx_leds[SMTC_SHIELD_LR11XX_LED_COUNT] = {
    { .timer_initialized = false }, { .timer_initialized = false }, { .timer_initialized = false }
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Pulse timer timeout callback
 *
 * @param context Context used to retrieve the index of the relevant LED.
 */
static void on_led_timer_event( void* context );

/*!
 * @brief Select and turn on Leds
 *
 * @param [in] leds Leds MASK to turn on leds
 */
void leds_on( uint8_t leds );

/*!
 * @brief Select and turn off Leds
 *
 * @param [in] leds Leds MASK to turn off leds
 */
void leds_off( uint8_t leds );

/*!
 * @brief Select and toggle Leds
 *
 * @param [in] leds Leds MASK to turn off leds
 */
void leds_toggle( uint8_t leds );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_board_led_set( uint32_t led_mask, bool turn_on )
{
    /* If a pulse timer is running on one of the requested LEDs, it
     *  must be stopped to avoid conflicting with the requested LED state. */
    smtc_shield_lr11xx_led_t led = SMTC_SHIELD_LR11XX_LED_TX;
    for( led = SMTC_SHIELD_LR11XX_LED_TX; led < SMTC_SHIELD_LR11XX_LED_COUNT; led++ )
    {
        if( led_mask & ( 1 << led ) )
        {
            if( ( smtc_shield_lr11xx_leds[led].timer_initialized ) &&
                ( timer_is_started( &smtc_shield_lr11xx_leds[led].led_timer ) ) )
            {
                timer_stop( &smtc_shield_lr11xx_leds[led].led_timer );
            }
        }
    }
    if( turn_on )
    {
        leds_on( led_mask );
    }
    else
    {
        leds_off( led_mask );
    }
}

uint32_t smtc_board_led_get( uint32_t led_mask )
{
    uint32_t leds_state = 0;

    if( led_mask & SMTC_SHIELD_LR11XX_LED_TX_MASK )
    {
        /* LED TX */
        leds_state += smtc_shield_lr11xx_get_led_tx_state( ) << SMTC_SHIELD_LR11XX_LED_TX_MASK;
    }
    if( led_mask & SMTC_SHIELD_LR11XX_LED_RX_MASK )
    {
        /* LED RX */
        leds_state += smtc_shield_lr11xx_get_led_rx_state( ) << SMTC_SHIELD_LR11XX_LED_RX_MASK;
    }
    if( led_mask & SMTC_SHIELD_LR11XX_LED_SCAN_MASK )
    {
        /* LED SCAN */
        leds_state += smtc_shield_lr11xx_get_led_scan_state( ) << SMTC_SHIELD_LR11XX_LED_SCAN_MASK;
    }

    return leds_state;
}

void smtc_board_led_pulse( uint32_t led_mask, bool turn_on, uint32_t duration_ms )
{
    smtc_shield_lr11xx_led_t led = SMTC_SHIELD_LR11XX_LED_TX;
    for( led = SMTC_SHIELD_LR11XX_LED_TX; led < SMTC_SHIELD_LR11XX_LED_COUNT; led++ )
    {
        if( led_mask & ( 1 << led ) )
        {
            if( smtc_shield_lr11xx_leds[led].timer_initialized )
            {
                if( timer_is_started( &smtc_shield_lr11xx_leds[led].led_timer ) )
                {
                    timer_stop( &smtc_shield_lr11xx_leds[led].led_timer );
                }
            }
            else
            {
                timer_init( &smtc_shield_lr11xx_leds[led].led_timer, on_led_timer_event );
                timer_set_context( &smtc_shield_lr11xx_leds[led].led_timer, ( void* ) led );
                smtc_shield_lr11xx_leds[led].timer_initialized = true;
            }
            timer_set_value( &smtc_shield_lr11xx_leds[led].led_timer, duration_ms );
            timer_start( &smtc_shield_lr11xx_leds[led].led_timer );
        }
    }
    if( turn_on )
    {
        leds_on( led_mask );
    }
    else
    {
        leds_off( led_mask );
    }
}

uint32_t smtc_board_get_led_tx_mask( void ) { return SMTC_SHIELD_LR11XX_LED_TX_MASK; }

uint32_t smtc_board_get_led_rx_mask( void ) { return SMTC_SHIELD_LR11XX_LED_RX_MASK; }

uint32_t smtc_board_get_led_all_mask( void ) { return SMTC_SHIELD_LR11XX_LED_ALL_MASK; }

void smtc_board_leds_blink( uint8_t led_mask, uint32_t delay, uint8_t nb_blink )
{
    uint8_t i = 0;

    leds_off( SMTC_SHIELD_LR11XX_LED_ALL_MASK );

    while( i < nb_blink )
    {
        i++;
        leds_on( led_mask );
        hal_mcu_delay_ms( delay / 2 );
        leds_off( led_mask );
        hal_mcu_delay_ms( delay / 2 );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void leds_on( uint8_t leds )
{
    if( leds & SMTC_SHIELD_LR11XX_LED_TX_MASK )
    {
        /* LED TX */
        smtc_shield_lr11xx_set_led_tx( );
    }
    if( leds & SMTC_SHIELD_LR11XX_LED_RX_MASK )
    {
        /* LED RX */
        smtc_shield_lr11xx_set_led_rx( );
    }
    if( leds & SMTC_SHIELD_LR11XX_LED_SCAN_MASK )
    {
        /* LED SCAN */
        smtc_shield_lr11xx_set_led_scan( );
    }
}

void leds_off( uint8_t leds )
{
    if( leds & SMTC_SHIELD_LR11XX_LED_TX_MASK )
    {
        /* LED TX */
        smtc_shield_lr11xx_reset_led_tx( );
    }
    if( leds & SMTC_SHIELD_LR11XX_LED_RX_MASK )
    {
        /* LED RX */
        smtc_shield_lr11xx_reset_led_rx( );
    }
    if( leds & SMTC_SHIELD_LR11XX_LED_SCAN_MASK )
    {
        /* LED SCAN */
        smtc_shield_lr11xx_reset_led_scan( );
    }
}

void leds_toggle( uint8_t leds )
{
    if( leds & SMTC_SHIELD_LR11XX_LED_TX_MASK )
    {
        /* LED TX */
        smtc_shield_lr11xx_toggle_led_tx( );
    }
    if( leds & SMTC_SHIELD_LR11XX_LED_RX_MASK )
    {
        /* LED RX */
        smtc_shield_lr11xx_toggle_led_rx( );
    }
    if( leds & SMTC_SHIELD_LR11XX_LED_SCAN_MASK )
    {
        /* LED SCAN */
        smtc_shield_lr11xx_toggle_led_scan( );
    }
}

uint32_t leds_get_state( uint8_t leds )
{
    uint32_t leds_state = 0;

    if( leds & SMTC_SHIELD_LR11XX_LED_TX_MASK )
    {
        /* LED TX */
        leds_state += smtc_shield_lr11xx_get_led_tx_state( ) << SMTC_SHIELD_LR11XX_LED_TX_MASK;
    }
    if( leds & SMTC_SHIELD_LR11XX_LED_RX_MASK )
    {
        /* LED RX */
        leds_state += smtc_shield_lr11xx_get_led_rx_state( ) << SMTC_SHIELD_LR11XX_LED_RX_MASK;
    }
    if( leds & SMTC_SHIELD_LR11XX_LED_SCAN_MASK )
    {
        /* LED SCAN */
        leds_state += smtc_shield_lr11xx_get_led_scan_state( ) << SMTC_SHIELD_LR11XX_LED_SCAN_MASK;
    }

    return leds_state;
}

void leds_blink( uint8_t leds, uint32_t delay, uint8_t nb_blink, bool reset_leds )
{
    uint8_t i = 0;

    if( reset_leds == true )
    {
        leds_off( SMTC_SHIELD_LR11XX_LED_ALL_MASK );
    }

    while( i < nb_blink )
    {
        i++;
        leds_on( leds );
        hal_mcu_delay_ms( delay / 2 );
        leds_off( leds );
        hal_mcu_delay_ms( delay / 2 );
    }
}

void on_led_timer_event( void* context )
{
    smtc_shield_lr11xx_led_t led      = ( smtc_shield_lr11xx_led_t ) context;
    uint32_t                 led_mask = 1 << led;
    leds_toggle( led_mask );
    timer_stop( &smtc_shield_lr11xx_leds[led].led_timer );
}

/* --- EOF ------------------------------------------------------------------ */
