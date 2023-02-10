/*!
 * @ingroup   tracker_application
 * @file      main_tracker_application.h
 *
 * @brief     LoRa Basics Modem LR11XX tracker application definition.
 *
 * @copyright
 * @parblock
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
 * @endparblock
 */

/*!
 * @addtogroup tracker_application
 * LoRa Basics Modem LR11XX tracker application
 * @{
 */

#ifndef MAIN_TRACKER_APPLICATION_H
#define MAIN_TRACKER_APPLICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- Application Configuration -----------------------------------------------
 */

/*!
 * @brief Defines the application mobile scan interval
 * when device has moved, 120s (2 min), value in [s].
 */
#define MOBILE_SCAN_INTERVAL_DEFAULT 120

/*!
 * @brief Defines the application static scan interval
 * when device doesn't move, 6 hours, 21600 seconds, value in [s].
 */
#define STATIC_SCAN_INTERVAL_DEFAULT 21600

/**
 * @brief Duration in second after last ALC sync response received to consider the local clock time invalid
 *
 * Set time valid for 7 day (to be fine tuned depending on board properties)
 */
#define APP_ALC_TIMING_INVALID ( APP_ALC_TIMING_INTERVAL * 7 )

/**
 * @brief Interval in second between two consecutive ALC sync requests
 *
 * 1 time sync requests per day
 */
#define APP_ALC_TIMING_INTERVAL ( 3600 * 24 )

/*!
 * @brief Defines the BLE thread advertisement timeout
 * when device doesn't connect to smartphone, 30000, value in [ms].
 */
#define TRACKER_ADV_TIMEOUT_MS 30000

/*!
 * @brief Hall effect sensor activation timeout, in milliseconds.
 * @note Only relevant in airplane mode. When not in airplane mode, the
 *        sensor is always enabled.
 */
#define TRACKER_AIRPLANE_HALL_EFFECT_TIMEOUT_MS 60000

/*!
 * @brief Define the drop out voltage in mV threshold where the tracker
 * stays in airplane mode.
 * @note Used on a design without supercapacitor.
 */
#define TRACKER_BOARD_DROPOUT_VOLTAGE_THRESHOLD 0

/*!
 * @brief Define the drop out voltage recovery time, in milliseconds, above which where the tracker
 * stays in airplane mode.
 * @note Used on a design with supercapacitor.
 */
#define TRACKER_BOARD_MAX_VOLTAGE_RECOVERY_TIME 6000

/*!
 * @brief Defines the application firmware version
 */
#define TRACKER_MAJOR_APP_VERSION 2
#define TRACKER_MINOR_APP_VERSION 2
#define TRACKER_SUB_MINOR_APP_VERSION 0

#define TRACKER_PCB_HW_NUMBER 595
#define TRACKER_MAJOR_PCB_HW_VERSION 1
#define TRACKER_MINOR_PCB_HW_VERSION 0

/**
 * @brief LR11XX radio firmware
 */
#define LR11XX_FW_VERSION 0x0307

/*!
 * @brief Time during which a LED is turned on when a TX or RX event occurs, in [ms]
 */
#define LED_PERIOD_MS 100

/*!
 * @brief Time during which a LED is turned on when the device is in join precedure, in [ms]
 */
#define LED_JOIN_PULSE_MS 25

/*!
 * @brief Time between 2 LED pulse when the device is in join precedure, in [ms]
 */
#define LED_JOIN_PERIOD_MS 3000

/*!
 * @brief LoRaWAN application TLV Tag
 */
#define TLV_TRACKER_SETTINGS_TAG 0x4C

/*!
 * @brief LoRaWAN port used to the gnss push solver messages
 */
#define GNSS_PUSH_SOLVER_MSG_PORT 150

/*!
 * @brief LoRaWAN port used to trigger tracker events
 */
#define TRACKER_REQUEST_MSG_PORT 151

/**
 * @brief LoRaWAN port use for sensors uplinks
 */
#define TRACKER_APP_SENSOR_PORT 193

/*!
 * @brief Define the number of time where the tracker send a scan result once static
 */
#define TRACKER_SEND_ONE_MORE_SCANS_ONCE_STATIC 0x03
#define TRACKER_SEND_TWO_MORE_SCANS_ONCE_STATIC 0x07
#define TRACKER_SEND_THREE_MORE_SCANS_ONCE_STATIC 0x0F

/*
 * -----------------------------------------------------------------------------
 * --- LoRaWAN Configuration ---------------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // MAIN_TRACKER_APPLICATION_H

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
