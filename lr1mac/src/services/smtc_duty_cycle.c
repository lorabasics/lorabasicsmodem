/*!
 * \file      smtc_duty_cycle.c
 *
 * \brief     Duty Cycle implementation
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_hal.h"
#include "smtc_duty_cycle.h"

#include <string.h> //for memset
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Get in which band a frequency is
 *
 * @param dtc_obj                   Contains the duty cycle context
 * @param freq_hz                   Frequency requested
 * @return uint8_t                  Return the band index of the frequency
 */
static uint8_t smtc_duty_cycle_get_band( smtc_dtc_t* dtc_obj, uint32_t freq_hz );

/**
 * @brief Get the consumed Time On Air on a band
 *
 * @param dtc_obj                   Contains the duty cycle context
 * @param band                      Band requested
 * @return uint32_t                 Return the consumed Time On Air
 */
static uint32_t smtc_duty_cycle_get_band_consumed_time_ms( smtc_dtc_t* dtc_obj, uint8_t band );

/**
 * @brief Compute Index in array of TOA with the a given timestamp in millisecond
 *
 * @param timestamp_ms              Timestamp ms
 * @return uint8_t                  Return the corresponding index
 */
static inline uint8_t smtc_duty_cycle_compute_index( uint32_t timestamp_ms );

/**
 * @brief Compute Diff between the two timestamp and manage wrapping
 *
 * @param rtc_ms                    RTC ms
 * @param timestamp_ms              Timestamp ms
 * @return uint32_t                 Return the time diff
 */
static inline uint32_t smtc_duty_cycle_time_diff( uint32_t rtc_ms, uint32_t timestamp_ms );

/**
 * @brief Put band number in array
 *
 * @param dtc_obj                   Contains the duty cycle context
 * @param tmp_band                  Array to store the band
 * @param tmp_band_index            Index in Array
 */
static void smtc_duty_cycle_put_band_in_array( smtc_dtc_t* dtc_obj, uint8_t* tmp_band, uint8_t band, uint8_t* tmp_band_index );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void smtc_duty_cycle_init( smtc_dtc_t* dtc_obj )
{
    // Set to 0 the dtc_obj
    memset( dtc_obj, 0, sizeof( smtc_dtc_t ) );
}

void smtc_duty_cycle_config( smtc_dtc_t* dtc_obj, uint8_t number_of_bands, uint8_t band_idx,
                             uint16_t duty_cycle_regulation, uint32_t freq_min, uint32_t freq_max )
{
    if( number_of_bands > SMTC_DTC_BANDS_MAX )
    {
        smtc_modem_hal_mcu_panic( );
    }
    dtc_obj->number_of_bands = number_of_bands;
    if( ( band_idx >= dtc_obj->number_of_bands ) || ( dtc_obj->number_of_bands == 0 ) )
    {
        smtc_modem_hal_mcu_panic( );
    }
    dtc_obj->bands[band_idx].duty_cycle_regulation = duty_cycle_regulation;
    dtc_obj->bands[band_idx].freq_min              = freq_min;
    dtc_obj->bands[band_idx].freq_max              = freq_max;
}

uint8_t smtc_duty_cycle_enable_set( smtc_dtc_t* dtc_obj, bool enable )
{
    if( dtc_obj->number_of_bands == 0 )
    {
        return false;
    }
    dtc_obj->enabled = enable;
    return true;
}

void smtc_duty_cycle_sum( smtc_dtc_t* dtc_obj, uint32_t freq_hz, uint32_t toa_ms )
{
    if( dtc_obj->number_of_bands == 0 )
    {
        return;
    }
    uint32_t rtc_time_now = smtc_modem_hal_get_time_in_ms( );
    uint8_t  band         = smtc_duty_cycle_get_band( dtc_obj, freq_hz );
    uint8_t  idx_previous = smtc_duty_cycle_compute_index( dtc_obj->bands[band].toa_timestamp_ms );

    // to manage rtc_ms wrapping
    uint8_t idx_new = idx_previous + smtc_duty_cycle_compute_index( smtc_duty_cycle_time_diff(
                                         rtc_time_now, dtc_obj->bands[band].toa_timestamp_ms ) );
    idx_new %= SMTC_DTC_TOA_BUFF_SIZE;

    // Convert TOA to the resolution
    toa_ms = ( toa_ms < smtc_dtc_resolution_ms )
                 ? 1
                 : ( smtc_dtc_resolution_ms == 1 ) ? toa_ms : ( toa_ms / smtc_dtc_resolution_ms ) + 1;

    // More than SMTC_DTC_PERIOD_MS since the last timestamp
    if( smtc_duty_cycle_time_diff( rtc_time_now, dtc_obj->bands[band].toa_timestamp_ms ) >= SMTC_DTC_PERIOD_MS )
    {
        // Erase band cumulated TOA
        memset( dtc_obj->bands[band].toa_sum_ms, 0, sizeof( dtc_obj->bands[band].toa_sum_ms ) );
    }
    else
    {
        if( idx_new == idx_previous )
        {
            // Sum TOA in same buffer
            toa_ms += dtc_obj->bands[band].toa_sum_ms[idx_new];
        }
        else
        {
            // Erase obsolete data between last saved and the current
            uint8_t i = idx_previous;
            while( i != idx_new )
            {
                i++;
                if( i >= SMTC_DTC_TOA_BUFF_SIZE )
                {
                    i = 0;
                }
                dtc_obj->bands[band].toa_sum_ms[i] = 0;
            }
        }
    }
    // Save the new TOA
    dtc_obj->bands[band].toa_sum_ms[idx_new] = toa_ms;
    dtc_obj->bands[band].toa_timestamp_ms    = rtc_time_now;
}

void smtc_duty_cycle_update( smtc_dtc_t* dtc_obj )
{
    if( dtc_obj->number_of_bands == 0 )
    {
        return;
    }
    uint32_t rtc_time_now = smtc_modem_hal_get_time_in_ms( );
    for( uint8_t band = 0; band < dtc_obj->number_of_bands; band++ )
    {
        uint8_t idx_previous = smtc_duty_cycle_compute_index( dtc_obj->bands[band].toa_timestamp_ms );
        // to manage rtc_ms wrapping
        uint8_t idx_new = idx_previous + smtc_duty_cycle_compute_index( smtc_duty_cycle_time_diff(
                                             rtc_time_now, dtc_obj->bands[band].toa_timestamp_ms ) );
        idx_new %= SMTC_DTC_TOA_BUFF_SIZE;

        // More than SMTC_DTC_PERIOD_MS since the last timestamp
        if( smtc_duty_cycle_time_diff( rtc_time_now, dtc_obj->bands[band].toa_timestamp_ms ) >= SMTC_DTC_PERIOD_MS )
        {
            // Erase band cumulated TOA, it's been over 1h
            memset( dtc_obj->bands[band].toa_sum_ms, 0, sizeof( dtc_obj->bands[band].toa_sum_ms ) );
            dtc_obj->bands[band].toa_timestamp_ms = rtc_time_now;
        }
        else
        {
            // Erase obsolete data between last saved and the current
            uint8_t i = idx_previous;
            while( i != idx_new )
            {
                i++;
                if( i >= SMTC_DTC_TOA_BUFF_SIZE )
                {
                    i = 0;
                }
                // If equal do not erase the TOA
                if( i != idx_new )
                {
                    dtc_obj->bands[band].toa_sum_ms[i] = 0;
                }
            }
        }
    }
}

bool smtc_duty_cycle_is_toa_accepted( smtc_dtc_t* dtc_obj, uint32_t freq_hz, uint32_t toa_ms )
{
    if( ( dtc_obj->enabled == false ) || ( dtc_obj->number_of_bands == 0 ) )
    {
        return true;
    }

    uint8_t  band           = smtc_duty_cycle_get_band( dtc_obj, freq_hz );
    uint32_t toa_consummed  = smtc_duty_cycle_get_band_consumed_time_ms( dtc_obj, band );
    uint16_t duty_cycle     = dtc_obj->bands[band].duty_cycle_regulation;
    int32_t  remaining_time = ( int32_t )( ( SMTC_DTC_PERIOD_MS / duty_cycle ) - toa_consummed );

    if( remaining_time < 0 )
    {
        return false;
    }
    else
    {
        if( toa_ms > ( uint32_t ) remaining_time )
        {
            return false;
        }
    }
    return true;
}

int32_t smtc_duty_cycle_band_get_available_toa_ms( smtc_dtc_t* dtc_obj, uint8_t band )
{
    if( ( dtc_obj->enabled == false ) || ( dtc_obj->number_of_bands == 0 ) )
    {
        return true;
    }

    uint32_t toa_consummed = smtc_duty_cycle_get_band_consumed_time_ms( dtc_obj, band );
    uint16_t duty_cycle    = dtc_obj->bands[band].duty_cycle_regulation;
    int32_t  toa           = ( int32_t )( ( SMTC_DTC_PERIOD_MS / duty_cycle ) - toa_consummed );

    return toa;
}

bool smtc_duty_cycle_is_channel_free( smtc_dtc_t* dtc_obj, uint32_t freq_hz )
{
    if( ( dtc_obj->enabled == false ) || ( dtc_obj->number_of_bands == 0 ) )
    {
        return true;
    }

    uint8_t band = smtc_duty_cycle_get_band( dtc_obj, freq_hz );
    if( smtc_duty_cycle_band_get_available_toa_ms( dtc_obj, band ) > 0 )
    {
        return true;
    }
    return false;
}

bool smtc_duty_cycle_is_band_free( smtc_dtc_t* dtc_obj, uint8_t band )
{
    if( ( dtc_obj->enabled == false ) || ( dtc_obj->number_of_bands == 0 ) )
    {
        return true;
    }

    if( smtc_duty_cycle_band_get_available_toa_ms( dtc_obj, band ) > 0 )
    {
        return true;
    }
    return false;
}

int32_t smtc_duty_cycle_get_next_free_time_ms( smtc_dtc_t* dtc_obj, uint8_t number_of_tx_freq, uint32_t* tx_freq_list )
{
    if( ( dtc_obj->enabled == false ) || ( dtc_obj->number_of_bands == 0 ) )
    {
        return 0;
    }

    int32_t ret                     = 0;
    uint8_t tmp_band_dtc_free_index = 0;
    uint8_t tmp_band_dtc_full_index = 0;

    uint8_t tmp_band_dtc_free[SMTC_DTC_BANDS_MAX];
    uint8_t tmp_band_dtc_full[SMTC_DTC_BANDS_MAX];

    // 0xFF is to avoid to have a true value,
    // tmp_band_dtc_free and tmp_band_dtc_full must contains band number (ex: 0 to 5 for EU868) after the newt for loop
    memset( tmp_band_dtc_free, 0xFF, SMTC_DTC_BANDS_MAX );
    memset( tmp_band_dtc_full, 0xFF, SMTC_DTC_BANDS_MAX );

    // Update duty-cycle timing
    smtc_duty_cycle_update( dtc_obj );

    for( uint8_t i = 0; i < number_of_tx_freq; i++ )
    {
        uint8_t band = smtc_duty_cycle_get_band( dtc_obj, tx_freq_list[i] );

        if( smtc_duty_cycle_is_band_free( dtc_obj, band ) == true )
        {
            // Put unique band in free array
            smtc_duty_cycle_put_band_in_array( dtc_obj, tmp_band_dtc_free, band, &tmp_band_dtc_free_index );
        }
        else
        {
            // Put unique band in full array
            smtc_duty_cycle_put_band_in_array( dtc_obj, tmp_band_dtc_full, band, &tmp_band_dtc_full_index );
        }
    }

    if( ( tmp_band_dtc_full_index == 0 ) && ( tmp_band_dtc_free_index == 0 ) )
    {
        smtc_modem_hal_mcu_panic( "Empty frequency list\n" );
    }

    if( tmp_band_dtc_free_index > 0 )
    {
        // Time is available, compute how much by sum TOA of all bands
        // return negative value if time available
        for( uint8_t i = 0; i < tmp_band_dtc_free_index; i++ )
        {
            ret -= smtc_duty_cycle_band_get_available_toa_ms( dtc_obj, tmp_band_dtc_free[i] );
        }
    }
    else
    {
        // All bands reached the max available TOA, search for the next nearest TOA available slot
        int32_t  toa_tmp      = smtc_duty_cycle_band_get_available_toa_ms( dtc_obj, tmp_band_dtc_full[0] );
        uint32_t timestamp_ms = dtc_obj->bands[tmp_band_dtc_full[0]].toa_timestamp_ms;

        for( uint8_t i = 0; i < tmp_band_dtc_full_index; i++ )
        {
            int32_t tmp = smtc_duty_cycle_band_get_available_toa_ms( dtc_obj, tmp_band_dtc_full[i] );
            if( toa_tmp < tmp )
            {
                toa_tmp      = tmp;
                timestamp_ms = dtc_obj->bands[tmp_band_dtc_full[i]].toa_timestamp_ms;
            }
        }

        ret = ( int32_t )( SMTC_DTC_PERIOD_MS - ( int32_t )( smtc_modem_hal_get_time_in_ms( ) - timestamp_ms ) );
    }

    return ret;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint8_t smtc_duty_cycle_get_band( smtc_dtc_t* dtc_obj, uint32_t freq_hz )
{
    for( uint8_t i = 0; i < dtc_obj->number_of_bands; i++ )
    {
        if( ( freq_hz >= dtc_obj->bands[i].freq_min ) && ( freq_hz < dtc_obj->bands[i].freq_max ) )
        {
            return i;
        }
    }
    return 0;
}

static uint32_t smtc_duty_cycle_get_band_consumed_time_ms( smtc_dtc_t* dtc_obj, uint8_t band )
{
    uint32_t toa_ms = 0;
    for( uint8_t i = 0; i < SMTC_DTC_TOA_BUFF_SIZE; i++ )
    {
        toa_ms += dtc_obj->bands[band].toa_sum_ms[i];
    }

    // Convert to the resolution
    toa_ms *= smtc_dtc_resolution_ms;
    return toa_ms;
}

static inline uint8_t smtc_duty_cycle_compute_index( uint32_t timestamp_ms )
{
    return ( ( ( timestamp_ms / 60000UL ) % ( ( ( SMTC_DTC_PERIOD_MS / 1000UL ) + SMTC_DTC_SECONDS_BY_UNIT ) / 60 ) ) /
             ( SMTC_DTC_SECONDS_BY_UNIT / 60 ) );
}

static inline uint32_t smtc_duty_cycle_time_diff( uint32_t rtc_ms, uint32_t timestamp_ms )
{
    return rtc_ms - timestamp_ms;
}

static void smtc_duty_cycle_put_band_in_array( smtc_dtc_t* dtc_obj, uint8_t* tmp_band, uint8_t band,
                                               uint8_t* tmp_band_index )
{
    bool is_present = false;

    for( uint8_t i = 0; i < dtc_obj->number_of_bands; i++ )
    {
        if( tmp_band[i] == band )
        {
            is_present = true;
            break;
        }
    }
    if( is_present == false )
    {
        tmp_band[*tmp_band_index] = band;
        ( *tmp_band_index )++;
    }
}
/* --- EOF ------------------------------------------------------------------ */
