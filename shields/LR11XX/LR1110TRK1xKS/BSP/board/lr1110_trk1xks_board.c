/*!
 * @file      lr11xx_trk1xks_board.c
 *
 * @brief     Target board LR11XX TRK1xKS tracker board driver implementation
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

#include "lr1110_trk1xks_board.h"
#include "smtc_lr11xx_board.h"
#include "smtc_board.h"
#include "smtc_board_ralf.h"
#include "ralf_lr11xx.h"
#include "lr11xx_hal_context.h"
#include "smtc_modem_test_api.h"
#include "smtc_modem_hal.h"
#include "lr11xx_hal.h"

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

/*!
 * @brief LR11XX Tracker Board LED context
 */
typedef struct
{
    timer_event_t led_timer;         /*!< @brief Pulse timer */
    bool          timer_initialized; /*!< @brief True if the pulse timer has been initialized, false otherwise */
} smtc_board_led_ctx_t;

/*!
 * @brief LR11XX Tracker Board Effect hall context
 */
typedef struct
{
    timer_event_t effect_hall_timer; /*!< @brief Effect hall timer */
    bool          timer_initialized; /*!< @brief True if the pulse timer has been initialized, false otherwise */
} smtc_board_effect_hall_ctx_t;

/*!
 * @brief LR11XX Tracker Periodic timer context
 */
typedef struct
{
    uint32_t      pulse_duration_ms;
    uint32_t      period_ms;
    uint32_t      led_mask;
    timer_event_t periodic_timer;    /*!< @brief periodic timer */
    bool          timer_initialized; /*!< @brief True if the pulse timer has been initialized, false otherwise */
} smtc_board_periodic_led_pulse_ctx_t;

/*!
 * @brief Define the max tx power supported
 */
int8_t max_tx_power_supported = 22;

/*!
 * @brief Define the battery level in percentage
 */
uint8_t battery_level = 100;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief smtc board ready flag
 */
static bool smtc_board_ready = false;

/*!
 * @brief smtc board gnss selected antenna
 */
static smtc_board_gnss_antenna_t smtc_board_gnss_selected_antenna = GNSS_PCB_ANTENNA;

/*!
 * @brief LR1110 Tracker Board LED context array
 */
static smtc_board_led_ctx_t smtc_board_leds[LR11XX_TRACKER_LED_COUNT] = { { .timer_initialized = false },
                                                                          { .timer_initialized = false } };

/*!
 * @brief LR1110 Tracker Board led pulse periodic timer context init
 */
static smtc_board_periodic_led_pulse_ctx_t smtc_board_periodic_led_pulse = { .timer_initialized = false };

/*!
 * @brief LR1110 Tracker Board effect hall context init
 */
static smtc_board_effect_hall_ctx_t smtc_board_effect_hall = { .timer_initialized = false };

static lr11xx_hal_context_t radio_context = {
    .nss    = SMTC_RADIO_NSS,
    .busy   = SMTC_RADIO_BUSY,
    .reset  = SMTC_RADIO_NRST,
    .spi_id = HAL_RADIO_SPI_ID,
};

/*!
 * @brief ralf_t object corresponding to the board
 */
static ralf_t local_ralf = { 0 };

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
 * @brief Periodic timer timeout callback
 *
 * @param context Context used to retrieve the index of the relevant LED.
 */
static void on_periodic_timer_event( void* context );

/*!
 * @brief Pulse timer timeout callback
 *
 * @param context Context not used here
 */
static void on_effect_hall_timer_event( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_board_init_periph( void )
{
    /* Init TX & RX Leds */
    leds_init( );

    /* Enable user button */
    usr_button_init( );

    /* LNA/SPDTs/SENSOR supplies/control */
    smtc_board_lna_init( );
    smtc_board_spdt_2g4_init( );
    smtc_board_vcc_sensors_init( );
    smtc_board_spdt_gnss_init( );

    /* LIS2DE12 accelerometer */
    accelerometer_init( INT_1 );

    /* Effect Hall sensor */
    smtc_board_hall_effect_enable( true );
}

void smtc_board_reinit_periph( void )
{
    /* Enable user button */
    usr_button_init( );

    /* LNA/SPDTs/SENSOR supplies/control */
    smtc_board_lna_init( );
    smtc_board_spdt_2g4_init( );
    smtc_board_spdt_gnss_init( );
}

void smtc_board_deinit_periph( void )
{
    /* Disable bothe user button */
    usr_button_deinit( );

    /* Disable LNA/SPDTs/SENSOR supplies/control */
    smtc_board_lna_deinit( );
    smtc_board_spdt_2g4_deinit( );
    smtc_board_spdt_gnss_deinit( );
}

bool smtc_board_get_usr_button( void )
{
    if( get_usr_button_irq_state( ) == true )
    {
        clear_usr_button_irq_state( );

        return true;
    }
    else
    {
        return false;
    }
}

void smtc_board_wifi_prescan( void )
{
    hal_mcu_partial_sleep_enable( true );
    smtc_board_spdt_2g4_on( );
    smtc_board_set_wifi_antenna( );
};

void smtc_board_wifi_postscan( void )
{
    hal_mcu_partial_sleep_enable( false );
    smtc_board_spdt_2g4_off( );
};

void smtc_board_gnss_prescan( void )
{
    if( ( smtc_board_gnss_selected_antenna & GNSS_PCB_ANTENNA ) == GNSS_PCB_ANTENNA )
    {
        HAL_DBG_TRACE_PRINTF( "Gnss scan PCB antenna\n" );
        smtc_board_set_gnss_pcb_antenna( );
    }
    else
    {
        HAL_DBG_TRACE_PRINTF( "Gnss scan PATCH antenna\n" );
        smtc_board_set_gnss_patch_antenna( );
    }

    hal_mcu_partial_sleep_enable( true );
    smtc_board_lna_on( );
}

void smtc_board_gnss_postscan( void )
{
    hal_mcu_partial_sleep_enable( false );
    smtc_board_lna_off( );
}

void smtc_board_lna_init( void ) { hal_gpio_init_out( RADIO_LNA_CTRL, 0 ); }

void smtc_board_lna_deinit( void ) { hal_gpio_deinit( RADIO_LNA_CTRL ); }

void smtc_board_lna_on( void ) { hal_gpio_set_value( RADIO_LNA_CTRL, 1 ); }

void smtc_board_lna_off( void ) { hal_gpio_set_value( RADIO_LNA_CTRL, 0 ); }

void smtc_board_spdt_2g4_init( void )
{
    hal_gpio_init_out( VCC_SWITCH_WIFI_BLE, 0 );
    hal_gpio_init_out( SWITCH_WIFI_BLE, 1 );  // Wi-Fi by default
}

void smtc_board_spdt_2g4_deinit( void )
{
    hal_gpio_deinit( VCC_SWITCH_WIFI_BLE );
    hal_gpio_deinit( SWITCH_WIFI_BLE );
}

void smtc_board_spdt_2g4_on( void ) { hal_gpio_set_value( VCC_SWITCH_WIFI_BLE, 1 ); }

void smtc_board_spdt_2g4_off( void ) { hal_gpio_set_value( VCC_SWITCH_WIFI_BLE, 0 ); }

void smtc_board_set_wifi_antenna( void )
{
    /* SWITCH_WIFI_BLE_Pin = 1 ==> Wi-Fi */
    hal_gpio_set_value( SWITCH_WIFI_BLE, 1 );
}

void smtc_board_set_ble_antenna( void )
{
    /* SWITCH_WIFI_BLE_Pin = 0 ==> BLE */
    hal_gpio_set_value( SWITCH_WIFI_BLE, 0 );
}

void smtc_board_vcc_sensors_init( void ) { hal_gpio_init_out( VCC_SENSORS_MCU, 0 ); }

void smtc_board_vcc_sensors_deinit( void ) { hal_gpio_deinit( VCC_SENSORS_MCU ); }

void smtc_board_vcc_sensors_on( void ) { hal_gpio_set_value( VCC_SENSORS_MCU, 1 ); }

void smtc_board_vcc_sensors_off( void ) { hal_gpio_set_value( VCC_SENSORS_MCU, 0 ); }

void smtc_board_spdt_gnss_init( void )
{
    hal_gpio_init_out( GPS_SWITCH, 0 );  // PCB Antenna by default
}

void smtc_board_spdt_gnss_deinit( void ) { hal_gpio_deinit( GPS_SWITCH ); }

void smtc_board_set_gnss_patch_antenna( void )
{
    /* GPS_SWITCH = 1 ==> PATCH */
    hal_gpio_set_value( GPS_SWITCH, 1 );
}

void smtc_board_set_gnss_pcb_antenna( void )
{
    /* GPS_SWITCH = 0 ==> PCB */
    hal_gpio_set_value( GPS_SWITCH, 0 );
}

void smtc_board_select_gnss_antenna( smtc_board_gnss_antenna_t antenna )
{
    if( antenna == GNSS_PCB_ANTENNA )
    {
        smtc_board_gnss_selected_antenna = GNSS_PCB_ANTENNA;
    }
    else
    {
        smtc_board_gnss_selected_antenna = GNSS_PATCH_ANTENNA;
    }
}

lr11xx_system_lfclk_cfg_t smtc_board_get_lf_clk( void ) { return LR11XX_SYSTEM_LFCLK_EXT; }

void smtc_board_hall_effect_enable( bool enable )
{
    HAL_DBG_TRACE_PRINTF( "hall effect sensor enable : %d\n", enable );

    if( enable == true )
    {
        smtc_board_vcc_sensors_init( );
        smtc_board_vcc_sensors_on( );
        hall_effect_init( HALL_EFFECT_IRQ_ON );
    }
    else
    {
        /* Stop Effect hall sensors while tracker is static */
        hall_effect_deinit( );
        smtc_board_vcc_sensors_off( );
        smtc_board_vcc_sensors_deinit( );
    }
}

void smtc_board_hall_effect_enable_for_duration( uint32_t duration_ms )
{
    if( smtc_board_effect_hall.timer_initialized )
    {
        if( timer_is_started( &smtc_board_effect_hall.effect_hall_timer ) )
        {
            timer_stop( &smtc_board_effect_hall.effect_hall_timer );
        }
    }
    else
    {
        timer_init( &smtc_board_effect_hall.effect_hall_timer, on_effect_hall_timer_event );
        timer_set_context( &smtc_board_effect_hall.effect_hall_timer, NULL );
        smtc_board_effect_hall.timer_initialized = true;
    }
    timer_set_value( &smtc_board_effect_hall.effect_hall_timer, duration_ms );
    timer_start( &smtc_board_effect_hall.effect_hall_timer );

    smtc_board_hall_effect_enable( true );
}

void smtc_board_hall_effect_stop_timer( void )
{
    if( smtc_board_effect_hall.timer_initialized )
    {
        if( timer_is_started( &smtc_board_effect_hall.effect_hall_timer ) )
        {
            timer_stop( &smtc_board_effect_hall.effect_hall_timer );
        }
    }

    smtc_board_hall_effect_enable( false );
}

bool smtc_board_is_ready( void ) { return smtc_board_ready; }

void smtc_board_set_ready( bool ready ) { smtc_board_ready = ready; }

void smtc_board_measure_battery_drop( uint8_t stack_id, smtc_modem_region_t region, int32_t* drop,
                                      uint32_t* time_recovery )
{
    smtc_modem_return_code_t modem_return_code = SMTC_MODEM_RC_OK;
    uint32_t                 relaxed_voltage = 0;
    uint32_t                 tick_vdrop      = 0;

    relaxed_voltage = smtc_modem_hal_get_voltage( ) * 20;

    /* Enter in test mode */
    modem_return_code = smtc_modem_test_start( );
    if( modem_return_code != SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_ERROR( "smtc_modem_test_start failed (%d)\n", modem_return_code );
    }

    switch( region )
    {
    case SMTC_MODEM_REGION_EU_868:
    case SMTC_MODEM_REGION_IN_865:
    case SMTC_MODEM_REGION_RU_864:
    {
        modem_return_code = smtc_modem_test_tx_cw( 865500000, 14 );
        break;
    }
    case SMTC_MODEM_REGION_US_915:
    case SMTC_MODEM_REGION_AU_915:
    case SMTC_MODEM_REGION_AS_923_GRP1:
    case SMTC_MODEM_REGION_AS_923_GRP2:
    case SMTC_MODEM_REGION_AS_923_GRP3:
    case SMTC_MODEM_REGION_KR_920:
    {
        modem_return_code = smtc_modem_test_tx_cw( 920900000, 14 );
        break;
    }
    default:
    {
        HAL_DBG_TRACE_ERROR( "This region is not covered by this test\n" );
        break;
    }
    }

    /* Wait the drop */
    hal_mcu_delay_ms( 2000 );

    /* Measure the drop */
    *drop = relaxed_voltage - smtc_modem_hal_get_voltage( ) * 20;

    /* Leave the test mode */
    smtc_modem_test_nop( );
    smtc_modem_test_stop( );

    HAL_DBG_TRACE_INFO( "Battery voltage drop = %d mV\n", *drop );

    if( *drop > 0 )
    {
        *time_recovery = 0;
        /* Get Start Tick*/
        tick_vdrop = hal_mcu_get_tick( );
        /* Wait 66% of drop recovery */
        while( ( smtc_modem_hal_get_voltage( ) * 20 < relaxed_voltage - ( *drop / 3 ) ) && ( *time_recovery < 10000 ) )
        {
            *time_recovery = hal_mcu_get_tick( ) - tick_vdrop;
        }

        HAL_DBG_TRACE_INFO( "Voltage recovery time = %d ms\n", *time_recovery );
    }
}

void smtc_board_led_set( uint32_t led_mask, bool turn_on )
{
    /* If a pulse timer is running on one of the requested LEDs, it
     *  must be stopped to avoid conflicting with the requested LED state. */
    lr11xx_tracker_led_t led = LR11XX_TRACKER_LED_TX;
    for( led = LR11XX_TRACKER_LED_TX; led < LR11XX_TRACKER_LED_COUNT; led++ )
    {
        if( led_mask & ( 1 << led ) )
        {
            if( ( smtc_board_leds[led].timer_initialized ) && ( timer_is_started( &smtc_board_leds[led].led_timer ) ) )
            {
                timer_stop( &smtc_board_leds[led].led_timer );
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

uint32_t smtc_board_led_get( uint32_t led_mask ) { return leds_get_state( led_mask ); }

void smtc_board_led_pulse( uint32_t led_mask, bool turn_on, uint32_t duration_ms )
{
    lr11xx_tracker_led_t led = LR11XX_TRACKER_LED_TX;
    for( led = LR11XX_TRACKER_LED_TX; led < LR11XX_TRACKER_LED_COUNT; led++ )
    {
        if( led_mask & ( 1 << led ) )
        {
            if( smtc_board_leds[led].timer_initialized )
            {
                if( timer_is_started( &smtc_board_leds[led].led_timer ) )
                {
                    timer_stop( &smtc_board_leds[led].led_timer );
                }
            }
            else
            {
                timer_init( &smtc_board_leds[led].led_timer, on_led_timer_event );
                timer_set_context( &smtc_board_leds[led].led_timer, ( void* ) led );
                smtc_board_leds[led].timer_initialized = true;
            }
            timer_set_value( &smtc_board_leds[led].led_timer, duration_ms );
            timer_start( &smtc_board_leds[led].led_timer );
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

void smtc_board_start_periodic_led_pulse( uint32_t led_mask, uint32_t pulse_duration_ms, uint32_t period_ms )
{
    if( period_ms > pulse_duration_ms )
    {
        smtc_board_periodic_led_pulse.led_mask          = led_mask;
        smtc_board_periodic_led_pulse.pulse_duration_ms = pulse_duration_ms;
        smtc_board_periodic_led_pulse.period_ms         = period_ms;

        smtc_board_led_pulse( led_mask, true, pulse_duration_ms );

        if( smtc_board_periodic_led_pulse.timer_initialized )
        {
            if( timer_is_started( &smtc_board_periodic_led_pulse.periodic_timer ) )
            {
                timer_stop( &smtc_board_periodic_led_pulse.periodic_timer );
            }
        }
        else
        {
            timer_init( &smtc_board_periodic_led_pulse.periodic_timer, on_periodic_timer_event );
            timer_set_context( &smtc_board_periodic_led_pulse.periodic_timer, NULL );
            smtc_board_periodic_led_pulse.timer_initialized = true;
        }
        timer_set_value( &smtc_board_periodic_led_pulse.periodic_timer, period_ms - pulse_duration_ms );
        timer_start( &smtc_board_periodic_led_pulse.periodic_timer );
    }
}

void smtc_board_stop_periodic_led_pulse( void ) { timer_stop( &smtc_board_periodic_led_pulse.periodic_timer ); }

uint32_t smtc_board_get_led_tx_mask( void ) { return LED_TX_MASK; }

uint32_t smtc_board_get_led_rx_mask( void ) { return LED_RX_MASK; }

uint32_t smtc_board_get_led_all_mask( void ) { return LED_ALL_MASK; }

void smtc_board_leds_blink( uint8_t leds, uint32_t delay, uint8_t nb_blink )
{
    leds_blink( leds, delay, nb_blink, true );
}

ralf_t* smtc_board_initialise_and_get_ralf( void )
{
    local_ralf = ( ralf_t ) RALF_LR11XX_INSTANTIATE( &radio_context );
    return &local_ralf;
}

int smtc_board_get_tx_power_offset( void ) { return BOARD_TX_POWER_OFFSET; }

void smtc_board_set_max_tx_power_supported( int8_t max_tx_power )
{
    max_tx_power_supported = max_tx_power;
}

int8_t smtc_board_get_max_tx_power_supported( void ) { return max_tx_power_supported; }

void smtc_board_set_battery_level( uint8_t battery_level_percentage ) { battery_level = battery_level_percentage; }

uint8_t smtc_board_get_battery_level( void ) { return battery_level; }

void smtc_board_reset_radio( const void* context ) { lr11xx_hal_reset( context ); }

void smtc_board_set_radio_in_dfu( const void* context )
{
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;

    /* Force dio0 to 0 */
    hal_gpio_init_out( lr11xx_context->busy, 0 );

    /* reset the chip */
    hal_gpio_set_value( lr11xx_context->reset, 0 );
    hal_mcu_wait_us( 5000 );
    hal_gpio_set_value( lr11xx_context->reset, 1 );

    /* wait 250ms */
    hal_mcu_wait_us( 250000 );

    /* reinit dio0 */
    hal_gpio_init_in( lr11xx_context->busy, HAL_GPIO_PULL_MODE_NONE, HAL_GPIO_IRQ_MODE_OFF, NULL );
}

uint8_t smtc_board_read_busy_pin( const void* context )
{
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;

    hal_mcu_wait_us( 1000000 );
    return ( uint8_t ) hal_gpio_get_value( lr11xx_context->busy );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void on_led_timer_event( void* context )
{
    lr11xx_tracker_led_t led      = ( lr11xx_tracker_led_t ) context;
    uint32_t             led_mask = 1 << led;
    leds_toggle( led_mask );
    timer_stop( &smtc_board_leds[led].led_timer );
}

void on_periodic_timer_event( void* context )
{
    smtc_board_start_periodic_led_pulse( smtc_board_periodic_led_pulse.led_mask,
                                         smtc_board_periodic_led_pulse.pulse_duration_ms,
                                         smtc_board_periodic_led_pulse.period_ms );
}

static void on_effect_hall_timer_event( void* context )
{
    HAL_DBG_TRACE_PRINTF( "Stop hall effect sensor\n" );
    smtc_board_hall_effect_enable( false );
    timer_stop( &smtc_board_effect_hall.effect_hall_timer );
}

/* --- EOF ------------------------------------------------------------------ */
