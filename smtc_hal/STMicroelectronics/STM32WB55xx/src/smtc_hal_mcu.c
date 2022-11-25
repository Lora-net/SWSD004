/*!
 * \file      smtc_hal_mcu.c
 *
 * \brief     MCU Hardware Abstraction Layer implementation
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_hal_mcu.h"
#include "smtc_hal_mcu_ex.h"
#include "modem_pinout.h"

#include "smtc_board.h"

#include "stm32wbxx_hal.h"
#include "stm32wbxx_ll_utils.h"

#include "smtc_hal.h"

#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * Watchdog counter reload value during sleep
 *
 * \remark The period must be lower than MCU watchdog period
 */
#define WATCHDOG_RELOAD_PERIOD_SECONDS 20
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Low Power options
 */
typedef enum low_power_mode_e
{
    LOW_POWER_ENABLE,
    LOW_POWER_DISABLE,
    LOW_POWER_DISABLE_ONCE
} low_power_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static volatile bool             exit_wait            = false;
static volatile low_power_mode_t lp_current_mode      = LOW_POWER_ENABLE;
static bool                      partial_sleep_enable = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief init the MCU clock tree
 */
static void hal_mcu_system_clock_config( void );

/*!
 * @brief init the GPIO
 */
static void hal_mcu_gpio_init( void );

/*!
 * @brief init the power voltage detector
 */
static void hal_mcu_pvd_config( void );

/*!
 * @brief reinit the MCU clock tree after a stop mode
 */
static void hal_mcu_system_clock_re_config_after_stop( void );

/*!
 * @brief Deinit the MCU
 */
static void hal_mcu_lpm_mcu_deinit( void );

/*!
 * @brief MCU enter in low power stop mode
 */
static void hal_mcu_lpm_enter_stop_mode( void );

/*!
 * @brief MCU exit in low power stop mode
 */
static void hal_mcu_lpm_exit_stop_mode( void );

/*!
 * @brief Function runing the low power mode handler
 */
static void hal_mcu_lpm_handler( void );

#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
/*!
 * @brief printf
 */
static void vprint( const char* fmt, va_list argp );
#endif

/*!
 * @brief Configure the STM32WB SMPS
 */
static void hal_mcu_smps_config( void );

/*!
 * @brief Activate the forward of the LSE on RCO pin
 *
 * @param enable Enable or disable the LSCO
 */
static void hal_mcu_system_clock_forward_LSE( bool enable );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_mcu_critical_section_begin( uint32_t* mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void hal_mcu_critical_section_end( uint32_t* mask ) { __set_PRIMASK( *mask ); }

void hal_mcu_disable_irq( void ) { __disable_irq( ); }

void hal_mcu_enable_irq( void ) { __enable_irq( ); }

void hal_mcu_delay_ms( uint32_t delay_ms ) { HAL_Delay( delay_ms ); }

uint32_t hal_mcu_get_tick( void ) { return HAL_GetTick( ); }

void hal_mcu_init( void )
{
    /* Initialize MCU HAL library */
    HAL_Init( );

    /* Initialize clocks */
    hal_mcu_system_clock_config( );

    // Initialize watchdog
#if( HAL_USE_WATCHDOG == HAL_FEATURE_ON )
    hal_watchdog_init( );
#endif  // HAL_USE_WATCHDOG == HAL_FEATURE_ON

    /* Initialize GPIOs */
    hal_mcu_gpio_init( );

    /* Initialize I2C */
    hal_i2c_init( HAL_I2C_ID, SMTC_I2C_SDA, SMTC_I2C_SCL );

    /* Initialize UART */
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_init( HAL_PRINTF_UART_ID, UART_TX, UART_RX );
#endif

    /* Initialize low power timer */
    hal_lp_timer_init( );

    /* Initialize the user flash */
    hal_flash_init( );

    /* Init power voltage voltage detector */
    hal_mcu_pvd_config( );

    /* Initialize SPI */
    hal_spi_init( HAL_RADIO_SPI_ID, SMTC_RADIO_SPI_MOSI, SMTC_RADIO_SPI_MISO, SMTC_RADIO_SPI_SCLK );

    /* Initialize RTC */
    hal_rtc_init( );

    /* Initialize ADC */
    hal_adc_init( );
}

void hal_mcu_reset( void )
{
    __disable_irq( );
    NVIC_SystemReset( );  // Restart system
}

void __attribute__( ( optimize( "O0" ) ) ) hal_mcu_wait_us( const int32_t microseconds )
{
    const uint32_t nb_nop = microseconds * 1000 / 363;
    for( uint32_t i = 0; i < nb_nop; i++ )
    {
        __NOP( );
    }
}

void hal_mcu_set_sleep_for_ms( const int32_t milliseconds )
{
    bool last_sleep_loop = false;

    if( milliseconds <= 0 )
    {
        return;
    }

    int32_t time_counter = milliseconds;

    hal_watchdog_reload( );

    if( lp_current_mode == LOW_POWER_ENABLE )
    {
        do
        {
            if( ( time_counter > ( WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 ) ) )
            {
                time_counter -= WATCHDOG_RELOAD_PERIOD_SECONDS * 1000;
                hal_rtc_wakeup_timer_set_ms( WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 );
            }
            else
            {
                hal_rtc_wakeup_timer_set_ms( time_counter );
                // if the sleep time is less than the wdog reload period, this is the last sleep loop
                last_sleep_loop = true;
            }
            hal_mcu_lpm_handler( );
            hal_watchdog_reload( );
        } while( ( hal_rtc_has_wut_irq_happened( ) == true ) && ( last_sleep_loop == false ) );
        if( ( last_sleep_loop == false ) || ( lp_current_mode == LOW_POWER_DISABLE_ONCE ) )
        {
            // in case sleep mode is interrupted by an other irq than the wake up timer, stop it and exit
            hal_rtc_wakeup_timer_stop( );
        }
    }
}

uint16_t hal_mcu_get_vref_level( void ) { return hal_adc_get_vref_int( ); }

int16_t hal_mcu_get_temperature( void ) { return hal_adc_get_temp( ); }

void hal_mcu_disable_low_power_wait( void )
{
    exit_wait       = true;
    lp_current_mode = LOW_POWER_DISABLE;
}

void hal_mcu_enable_low_power_wait( void )
{
    exit_wait       = false;
    lp_current_mode = LOW_POWER_ENABLE;
}

void hal_mcu_trace_print( const char* fmt, ... )
{
#if HAL_DBG_TRACE == HAL_FEATURE_ON
    va_list argp;
    va_start( argp, fmt );
    vprint( fmt, argp );
    va_end( argp );
#endif
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line
 * number where the assert_param error has occurred. Input          : - file:
 * pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    // User can add his own implementation to report the file name and line
    // number,
    // ex: printf("Wrong parameters value: file %s on line %lu\r\n", file, line)

    SMTC_HAL_TRACE_PRINTF( "Wrong parameters value: file %s on line %lu\r\n", ( const char* ) file, line );
    // Infinite loop
    while( 1 )
    {
    }
}
#endif

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
}

void HAL_MspInit( void )
{
    HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

    /* HSEM Clock enable */
    __HAL_RCC_HSEM_CLK_ENABLE( );

    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( MemoryManagement_IRQn, 0, 0 );
    /* BusFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( BusFault_IRQn, 0, 0 );
    /* UsageFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( UsageFault_IRQn, 0, 0 );
    /* SVCall_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SVCall_IRQn, 0, 0 );
    /* DebugMonitor_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( DebugMonitor_IRQn, 0, 0 );
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( PendSV_IRQn, 0, 0 );
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

void hal_mcu_partial_sleep_enable( bool enable ) { partial_sleep_enable = enable; }

void hal_mcu_smps_enable( bool enable )
{
    if( enable )
    {
        LL_PWR_SMPS_Enable( );
    }
    else
    {
        LL_PWR_SMPS_Disable( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void hal_mcu_system_clock_config( void )
{
    // Configure the main internal regulator output voltage
    RCC_OscInitTypeDef       rcc_osc_init_struct = { 0 };
    RCC_ClkInitTypeDef       rcc_clk_init_struct = { 0 };
    RCC_PeriphCLKInitTypeDef periph_clk_init     = { 0 };

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess( );
    __HAL_RCC_LSEDRIVE_CONFIG( RCC_LSEDRIVE_LOW );
    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );
    /** Initializes the CPU, AHB and APB busses clocks
     */
    rcc_osc_init_struct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    rcc_osc_init_struct.HSEState            = RCC_HSE_ON;
    rcc_osc_init_struct.LSEState            = RCC_LSE_ON;
    rcc_osc_init_struct.HSIState            = RCC_HSI_ON;
    rcc_osc_init_struct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    rcc_osc_init_struct.PLL.PLLState        = RCC_PLL_NONE;
    if( HAL_RCC_OscConfig( &rcc_osc_init_struct ) != HAL_OK )
    {
        mcu_panic( );
    }
    /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
     */
    rcc_clk_init_struct.ClockType = RCC_CLOCKTYPE_HCLK4 | RCC_CLOCKTYPE_HCLK2 | RCC_CLOCKTYPE_HCLK |
                                    RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    rcc_clk_init_struct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSE;
    rcc_clk_init_struct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    rcc_clk_init_struct.APB1CLKDivider = RCC_HCLK_DIV1;
    rcc_clk_init_struct.APB2CLKDivider = RCC_HCLK_DIV1;
    rcc_clk_init_struct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
    rcc_clk_init_struct.AHBCLK4Divider = RCC_SYSCLK_DIV1;
    if( HAL_RCC_ClockConfig( &rcc_clk_init_struct, FLASH_LATENCY_1 ) != HAL_OK )
    {
        mcu_panic( );
    }
    /** Initializes the peripherals clocks
     */
    periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_SMPS | RCC_PERIPHCLK_RFWAKEUP | RCC_PERIPHCLK_RTC |
                                           RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RNG |
                                           RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_LPTIM1;

    periph_clk_init.RTCClockSelection      = RCC_RTCCLKSOURCE_LSE;
    periph_clk_init.Lptim1ClockSelection   = RCC_LPTIM1CLKSOURCE_LSE;
    periph_clk_init.AdcClockSelection      = RCC_ADCCLKSOURCE_SYSCLK;
    periph_clk_init.RngClockSelection      = LL_RCC_RNG_CLKSOURCE_LSE;
    periph_clk_init.Usart1ClockSelection   = RCC_USART1CLKSOURCE_PCLK2;
    periph_clk_init.I2c1ClockSelection     = RCC_I2C1CLKSOURCE_PCLK1;
    periph_clk_init.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
    periph_clk_init.SmpsClockSelection     = RCC_SMPSCLKSOURCE_HSE;
    periph_clk_init.SmpsDivSelection       = RCC_SMPSCLKDIV_RANGE1;

    if( HAL_RCCEx_PeriphCLKConfig( &periph_clk_init ) != HAL_OK )
    {
        mcu_panic( );  // Initialization Error
    }

    /* Configure the SMPS */
    hal_mcu_smps_config( );

    /* Set the 32MHz accuracy, by default load capacitance is 12pF, min is 12pF and max is 16pF.
    Value is Between Min_Data = 0 and Max_Data = 63 ==> 4pF/63 = 0.06pF of step, 55 means load capacitance is 15.5pF */
    LL_RCC_HSE_SetCapacitorTuning( 55 );

    hal_mcu_system_clock_forward_LSE( true );
}

static void hal_mcu_smps_config( void )
{
    /*
     *   See AN5246 from ST
     *
     *   TX level of +0 dBm (Tx Code = 25) and a digital
     *   activity at 20 mA maximum, then we have to set the VFBSMPS at a voltage higher than
     *   1.4 V + 15 mV (load impact) + 10 mV (trimming accuracy), that is VFBSMPS > 1.425 V, which
     *   gives 1.450 V
     *
     *   SMPSVOS = (VFBSMPS � 1.5 V) / 50 mV + SMPS_coarse_engi_trim
     *
     *   uint8_t smps_coarse_engi_trim = (*( __IO uint32_t* ) 0x1FFF7559) & 0x0F;
     *   uint8_t smps_fine_engi_trim = (*( __IO uint32_t* ) 0x1FFF7549) & 0x0F;
     *   float vfbsmps = 1.45;
     *
     *   smpsvos = ( ( vfbsmps - 1.5 ) / 0.050 ) + smps_coarse_engi_trim;
     */

    /* Configure the SMPS */
    LL_PWR_SMPS_SetStartupCurrent( LL_PWR_SMPS_STARTUP_CURRENT_80MA );
    LL_PWR_SMPS_SetOutputVoltageLevel(
        LL_PWR_SMPS_OUTPUT_VOLTAGE_1V50 );  // smpsvos should be 6 meaning LL_PWR_SMPS_OUTPUT_VOLTAGE_1V50
    LL_PWR_SMPS_SetMode( LL_PWR_SMPS_BYPASS );
}

static void hal_mcu_pvd_config( void )
{
    PWR_PVDTypeDef sConfigPVD = { 0 };
    sConfigPVD.PVDLevel       = PWR_PVDLEVEL_1;
    sConfigPVD.Mode           = PWR_PVD_MODE_IT_RISING;
    if( HAL_PWR_ConfigPVD( &sConfigPVD ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    /* Enable PVD */
    HAL_PWR_EnablePVD( );

    /* Enable and set PVD Interrupt priority */
    HAL_NVIC_SetPriority( PVD_PVM_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( PVD_PVM_IRQn );
}

static void hal_mcu_gpio_init( void )
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE( );
    __HAL_RCC_GPIOB_CLK_ENABLE( );
    __HAL_RCC_GPIOC_CLK_ENABLE( );
    __HAL_RCC_GPIOE_CLK_ENABLE( );
    __HAL_RCC_GPIOH_CLK_ENABLE( );

#if( HAL_HW_DEBUG_PROBE == HAL_FEATURE_ON )
    /* Enable debug in sleep/stop/standby */
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
#else
    HAL_DBGMCU_DisableDBGSleepMode( );
    HAL_DBGMCU_DisableDBGStopMode( );
    HAL_DBGMCU_DisableDBGStandbyMode( );
#endif

    hal_gpio_init_out( SMTC_RADIO_NSS, 1 );
    hal_gpio_init_in( SMTC_RADIO_BUSY, HAL_GPIO_PULL_MODE_NONE, HAL_GPIO_IRQ_MODE_OFF, NULL );
    // Here init only the pin as an exti rising and the callback will be attached later
    hal_gpio_init_in( SMTC_RADIO_DIOX, HAL_GPIO_PULL_MODE_DOWN, HAL_GPIO_IRQ_MODE_RISING, NULL );
    hal_gpio_init_out( SMTC_RADIO_NRST, 1 );
}

/**
 * @brief Enters Low Power Stop Mode
 *
 * @note ARM exits the function when waking up
 *
 */
static void hal_mcu_lpm_enter_stop_mode( void )
{
    CRITICAL_SECTION_BEGIN( );

    // Deinit periph & enter Stop Mode
    if( partial_sleep_enable == true )
    {
        hal_mcu_lpm_mcu_deinit( );
    }
    else
    {
        smtc_board_deinit_periph( );
        hal_mcu_lpm_mcu_deinit( );
    }

    CRITICAL_SECTION_END( );

    /* In case of debugger probe attached, work-around of issue specified in "ES0394 - STM32WB55Cx/Rx/Vx device
       errata": 2.2.9 Incomplete Stop 2 mode entry after a wakeup from debug upon EXTI line 48 event "With the JTAG
       debugger enabled on GPIO pins and after a wakeup from debug triggered by an event on EXTI line 48 (CDBGPWRUPREQ),
       the device may enter in a state in which attempts to enter Stop 2 mode are not fully effective ..."
    */
    LL_EXTI_DisableIT_32_63( LL_EXTI_LINE_48 );
    LL_C2_EXTI_DisableIT_32_63( LL_EXTI_LINE_48 );

    LL_C2_PWR_SetPowerMode( LL_PWR_MODE_SHUTDOWN );

    /* Enter Stop Mode */
    HAL_PWREx_EnterSTOP2Mode( PWR_STOPENTRY_WFI );
}

/**
 * @brief Exists Low Power Stop Mode
 *
 */
static void hal_mcu_lpm_exit_stop_mode( void )
{
    // Disable IRQ while the MCU is not running on HSI
    CRITICAL_SECTION_BEGIN( );

    /* Reinitializes the MCU */
    hal_mcu_lpm_mcu_reinit( );

    if( partial_sleep_enable == false )
    {
        // Reinitializes the peripherals
        smtc_board_reinit_periph( );
    }

    CRITICAL_SECTION_END( );
}

/**
 * @brief Low power handler
 *
 */
static void hal_mcu_lpm_handler( void )
{
#if( HAL_LOW_POWER_MODE == HAL_FEATURE_ON )
    // stop systick to avoid getting pending irq while going in stop mode
    // Systick is automatically restart when going out of sleep
    HAL_SuspendTick( );

    __disable_irq( );
    // If an interrupt has occurred after __disable_irq( ), it is kept pending
    // and cortex will not enter low power anyway

    hal_mcu_lpm_enter_stop_mode( );
    hal_mcu_lpm_exit_stop_mode( );

    __enable_irq( );
    HAL_ResumeTick( );
#endif
}

/**
 * @brief De-init periph begore going in sleep mode
 *
 */
static void hal_mcu_lpm_mcu_deinit( void )
{
    hal_spi_deinit( HAL_RADIO_SPI_ID );

    /* Disable I2C */
    hal_i2c_deinit( HAL_I2C_ID );
    /* Disable UART */
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_deinit( HAL_PRINTF_UART_ID );
#endif
}

/**
 * @brief Re-init MCU clock after a wait in stop mode 2
 *
 */
void hal_mcu_lpm_mcu_reinit( void )
{
    /* Reconfig needed OSC and PLL */
    hal_mcu_system_clock_re_config_after_stop( );

    /* Initialize UART */
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_init( HAL_PRINTF_UART_ID, UART_TX, UART_RX );
#endif
    /* Initialize I2C */
    hal_i2c_init( HAL_I2C_ID, SMTC_I2C_SDA, SMTC_I2C_SCL );

    /* Initialize SPI */
    hal_spi_init( HAL_RADIO_SPI_ID, SMTC_RADIO_SPI_MOSI, SMTC_RADIO_SPI_MISO, SMTC_RADIO_SPI_SCLK );
}

static void hal_mcu_system_clock_re_config_after_stop( void )
{
    CRITICAL_SECTION_BEGIN( );

    /**Configure LSE Drive Capability
     */
    __HAL_RCC_LSEDRIVE_CONFIG( RCC_LSEDRIVE_LOW );

    /* Enable HSE */
    __HAL_RCC_HSE_CONFIG( RCC_HSE_ON );

    /* Wait till HSE is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSERDY ) == RESET )
    {
    }

    /* Enable HSI */
    __HAL_RCC_HSI_ENABLE( );

    /* Wait till HSI is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    {
    }

    /* Select HSE as system clock source */
    __HAL_RCC_SYSCLK_CONFIG( RCC_SYSCLKSOURCE_HSE );

    /* Wait till HSE is used as system clock source */
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_HSE )
    {
    }

    CRITICAL_SECTION_END( );
}

void hal_mcu_system_clock_forward_LSE( bool enable )
{
    if( enable )
    {
        HAL_RCCEx_EnableLSCO( RCC_LSCOSOURCE_LSE );
    }
    else
    {
        HAL_RCCEx_DisableLSCO( );
    }
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler( void )
{
    HAL_DBG_TRACE_ERROR( "HARDFAULT_Handler\n" );

    /* reset the board */
    hal_mcu_reset( );
}

#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
static void vprint( const char* fmt, va_list argp )
{
    char string[HAL_PRINT_BUFFER_SIZE];
    if( 0 < vsprintf( string, fmt, argp ) )  // build string
    {
        hal_uart_tx( HAL_PRINTF_UART_ID, ( uint8_t* ) string, strlen( string ) );
    }
}
#endif

/*!
 * @brief  This function handles PVD interrupt request.
 * @param  None
 * @returns None
 */
void PVD_PVM_IRQHandler( void )
{
    HAL_DBG_TRACE_ERROR( "PVD_PVM_IRQHandler\n" );
    /* Loop inside the handler to prevent the Cortex from using the Flash,
       allowing the flash interface to finish any ongoing transfer. */
    while( __HAL_PWR_GET_FLAG( PWR_FLAG_PVDO ) != RESET )
    {
    }
    /* Then reset the board */
    hal_mcu_reset( );
}

/* --- EOF ------------------------------------------------------------------ */
