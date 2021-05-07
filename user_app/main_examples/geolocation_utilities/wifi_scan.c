/*!
 * \file      wifi_scan.c
 *
 * \brief     Wifi scan implementation.
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

#include <stdio.h>
#include <string.h>
#include "wifi_scan.h"
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"

#define WIFI_SCAN_ABORT_ON_TIMEOUT ( true )
#define WIFI_MAX_RESULTS_PER_SCAN ( 20 )
#define WIFI_CONSUMPTION_DCDC_CORRELATION_MA ( 12 )
#define WIFI_CONSUMPTION_DCDC_DEMODULATION_MA ( 4 )
#define WIFI_CONSUMPTION_LDO_CORRELATION_MA ( 24 )
#define WIFI_CONSUMPTION_LDO_DEMODULATION_MA ( 8 )

#define TAG_WIFI 8

wifi_t wifi;

void wifi_init( void )
{
    wifi.results.nbr_results = 0;

    wifi.settings.enabled      = true;
    wifi.settings.channels     = 0x3FFF;
    wifi.settings.types        = LR1110_WIFI_TYPE_SCAN_B;
    wifi.settings.scan_mode    = LR1110_WIFI_SCAN_MODE_BEACON_AND_PKT;
    wifi.settings.nbr_retrials = WIFI_NBR_RETRIALS_DEFAULT;
    wifi.settings.max_results  = WIFI_MAX_RESULTS_DEFAULT;
    wifi.settings.timeout      = WIFI_TIMEOUT_IN_MS_DEFAULT;
}

// Start a WiFi scan and stream data if scan is a success
bool scan_and_stream_wifi( uint8_t stack_id, uint8_t port )
{
    uint8_t nbr_results = 0;

    wifi_execute_scan( NULL, &nbr_results );
    if( nbr_results > 0 )
    {
        wifi_display_results( );
        send_wifi_data( nbr_results, stack_id, port );
        return true;
    }

    return false;
}

static void wifi_add_scan_to_results( lr1110_system_reg_mode_t reg_mode, wifi_scan_all_result_t* results,
                                      lr1110_wifi_basic_complete_result_t* scan_result, uint8_t nbr_results )
{
    for( uint8_t index = 0; ( index < nbr_results ) && ( results->nbr_results < WIFI_MAX_RESULT_TOTAL ); index++ )
    {
        const lr1110_wifi_basic_complete_result_t* local_basic_result = &scan_result[index];

        results->results[results->nbr_results].channel =
            lr1110_wifi_extract_channel_from_info_byte( local_basic_result->channel_info_byte );

        results->results[results->nbr_results].type =
            lr1110_wifi_extract_signal_type_from_data_rate_info( local_basic_result->data_rate_info_byte );

        memcpy( results->results[results->nbr_results].mac_address, local_basic_result->mac_address,
                LR1110_WIFI_MAC_ADDRESS_LENGTH );

        results->results[results->nbr_results].rssi = local_basic_result->rssi;
        results->nbr_results++;
    }
}

// execute the wifi scan process
void wifi_execute_scan( const void* context, uint8_t* n_results )
{
    // do the scan
    lr1110_wifi_scan( context, wifi.settings.types, wifi.settings.channels, wifi.settings.scan_mode,
                      wifi.settings.max_results, wifi.settings.nbr_retrials, wifi.settings.timeout,
                      WIFI_SCAN_ABORT_ON_TIMEOUT );

    // copy the results
    uint8_t                             nbr_results                                      = 0;
    lr1110_wifi_basic_complete_result_t wifi_results_mac_addr[WIFI_MAX_RESULTS_PER_SCAN] = { 0 };

    lr1110_wifi_get_nb_results( context, &nbr_results );

    *n_results = ( nbr_results > WIFI_MAX_RESULTS_PER_SCAN ) ? WIFI_MAX_RESULTS_PER_SCAN : nbr_results;

    lr1110_wifi_read_basic_complete_results( context, 0, *n_results, wifi_results_mac_addr );

    wifi_add_scan_to_results( LR1110_SYSTEM_REG_MODE_DCDC, &wifi.results, wifi_results_mac_addr, *n_results );
}

// Send wifi results by streaming
void send_wifi_data( int nbr_results, uint8_t stack_id, uint8_t port )
{
    uint8_t wifi_index       = 0;
    uint8_t wifi_payload_len = 0;  // reset the payload len
    uint8_t wifi_payload[256];

    wifi_payload[wifi_payload_len++] = TAG_WIFI;                              // Wi-Fi TAG
    wifi_payload[wifi_payload_len++] = nbr_results * WIFI_SINGLE_BEACON_LEN;  // Wi-Fi Len

    wifi_index = wifi_payload_len;
    for( uint8_t i = 0; i < nbr_results; i++ )
    {
        wifi_payload[wifi_index] = wifi.results.results[i].rssi;
        memcpy( &wifi_payload[wifi_index + 1], wifi.results.results[i].mac_address, 6 );
        wifi_index += WIFI_SINGLE_BEACON_LEN;
    }
    wifi_payload_len += nbr_results * WIFI_SINGLE_BEACON_LEN;

    /* Push the Wi-Fi data in the FiFo stream */
    if( smtc_modem_stream_add_data( stack_id, port, wifi_payload, wifi_payload_len ) != RC_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to stream wifi data\n" );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_INFO( "stream wifi data done\n" );
    }
}

// Display the mac addresses scanned during the function wifi_execute_scan
void wifi_display_results( void )
{
    if( wifi.results.nbr_results != 0 )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "nb MAC scanned: %d\r\n", wifi.results.nbr_results );
        for( uint8_t i = 0; i < wifi.results.nbr_results; i++ )
        {
            // SMTC_MODEM_HAL_TRACE_ARRAY("MAC address:", wifi.results.results[i].mac_address, 6 );
            SMTC_MODEM_HAL_TRACE_MSG( "MAC addr: " );
            for( uint8_t j = 0; j < 6; j++ )
            {
                SMTC_MODEM_HAL_TRACE_PRINTF( "%#02X ", wifi.results.results[i].mac_address[j] );
            }
            SMTC_MODEM_HAL_TRACE_PRINTF( " -- Channel: %d", wifi.results.results[i].channel );
            SMTC_MODEM_HAL_TRACE_PRINTF( " -- Type: %d", wifi.results.results[i].type );
            SMTC_MODEM_HAL_TRACE_PRINTF( " -- RSSI: %d\r\n", wifi.results.results[i].rssi );
        }
        SMTC_MODEM_HAL_TRACE_MSG( "\r\n" );
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_MSG( "No MAC address found \r\n" );
    }
}
