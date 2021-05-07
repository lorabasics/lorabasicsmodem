/*!
 * \file      wifi_scan.h
 *
 * \brief     Wifi scan definition
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef __WIFI_SCAN_H__
#define __WIFI_SCAN_H__

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr1110_wifi.h"
#include "lr1110_system.h"
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define WIFI_SINGLE_BEACON_LEN          7
#define WIFI_NBR_RETRIALS_DEFAULT       5
#define WIFI_MAX_RESULTS_DEFAULT        10
#define WIFI_TIMEOUT_IN_MS_DEFAULT      110
#define WIFI_MAX_RESULT_TOTAL           32

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct {
    bool                            enabled;
    lr1110_wifi_channel_mask_t      channels;
    lr1110_wifi_signal_type_scan_t  types;
    lr1110_wifi_mode_t              scan_mode;
    uint8_t                         nbr_retrials;
    uint8_t                         max_results;
    uint32_t                        timeout;
} wifi_settings_t;

typedef struct {
    lr1110_wifi_mac_address_t         mac_address;
    lr1110_wifi_channel_t             channel;
    lr1110_wifi_signal_type_result_t  type;
    int8_t                            rssi;
    uint8_t                           country_code[LR1110_WIFI_STR_COUNTRY_CODE_SIZE];
} wifi_scan_single_result_t;

typedef struct {
    bool                         error;
    uint8_t                      nbr_results;
    wifi_scan_single_result_t    results[WIFI_MAX_RESULT_TOTAL];
} wifi_scan_all_result_t;

typedef struct {
    wifi_settings_t             settings;

    lr1110_system_reg_mode_t    reg_mode;
    wifi_scan_all_result_t      results;
} wifi_t;


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Init for wifi scan
 *
 */
void wifi_init( void );

/*!
 * @brief Start a WiFi scan and stream data if scan is a success
 *
 * @param [in] stack_id      - Chosen stack id
 * @param [in] port          - The port used for streaming
 * @return bool              - Scan status, will return false and skip the streaming if no wifi ssid found
 *
 * @remarks modem_suspend_radio_access must be called before and modem_resume_radio_access after
 *
 */
bool scan_and_stream_wifi(uint8_t stack_id, uint8_t port);

/*!
 * @brief execute the wifi scan process
 *
 * @param [in] context       - Chip implementation context
 * @param [out] n_results    - The number of results available after the scan finished
 *
 */
void wifi_execute_scan( const void* context, uint8_t* n_results );

/*!
 * @brief Display the mac addresses scanned during the function wifi_execute_scan
 *
 */
void wifi_display_results( void );

/*!
 * @brief Send wifi results by streaming
 *
 * @param [in] nbr_results   - Chip implementation context
 * @param [in] stack_id      - Chosen stack id
 * @param [in] port          - The port used for streaming
 *
 */
void send_wifi_data(int nbr_results, uint8_t stack_id, uint8_t port);

#endif  // __WIFI_SCAN_H__

/* --- EOF ------------------------------------------------------------------ */
