/*!
 * \file      smtc_modem_hal_dbg_trace.h
 *
 * \brief     Modem Hardware Abstraction Layer trace features.
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
#ifndef __SMTC_MODEM_HAL_DBG_TRACE_H__
#define __SMTC_MODEM_HAL_DBG_TRACE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#if( UNIT_TEST_DBG )
#include <stdio.h>
#endif
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_hal.h"
#include "smtc_modem_hal_options.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

// clang-format off
#if ( MODEM_HAL_DBG_TRACE_COLOR == MODEM_HAL_FEATURE_ON )
    #define MODEM_HAL_DBG_TRACE_COLOR_BLACK               "\x1B[0;30m"
    #define MODEM_HAL_DBG_TRACE_COLOR_RED                 "\x1B[0;31m"
    #define MODEM_HAL_DBG_TRACE_COLOR_GREEN               "\x1B[0;32m"
    #define MODEM_HAL_DBG_TRACE_COLOR_YELLOW              "\x1B[0;33m"
    #define MODEM_HAL_DBG_TRACE_COLOR_BLUE                "\x1B[0;34m"
    #define MODEM_HAL_DBG_TRACE_COLOR_MAGENTA             "\x1B[0;35m"
    #define MODEM_HAL_DBG_TRACE_COLOR_CYAN                "\x1B[0;36m"
    #define MODEM_HAL_DBG_TRACE_COLOR_WHITE               "\x1B[0;37m"
    #define MODEM_HAL_DBG_TRACE_COLOR_DEFAULT             "\x1B[0m"
#else
    #define MODEM_HAL_DBG_TRACE_COLOR_BLACK   ""
    #define MODEM_HAL_DBG_TRACE_COLOR_RED     ""
    #define MODEM_HAL_DBG_TRACE_COLOR_GREEN   ""
    #define MODEM_HAL_DBG_TRACE_COLOR_YELLOW  ""
    #define MODEM_HAL_DBG_TRACE_COLOR_BLUE    ""
    #define MODEM_HAL_DBG_TRACE_COLOR_MAGENTA ""
    #define MODEM_HAL_DBG_TRACE_COLOR_CYAN    ""
    #define MODEM_HAL_DBG_TRACE_COLOR_WHITE   ""
    #define MODEM_HAL_DBG_TRACE_COLOR_DEFAULT ""
#endif

#if ( MODEM_HAL_DBG_TRACE ) && !defined (PERF_TEST_ENABLED)

    #if ( UNIT_TEST_DBG )
        #define SMTC_MODEM_HAL_TRACE_PRINTF( ... )  printf (  __VA_ARGS__ )
    #else
        #define SMTC_MODEM_HAL_TRACE_PRINTF( ... )  smtc_modem_hal_print_trace (  __VA_ARGS__ )
    #endif

    #define SMTC_MODEM_HAL_TRACE_MSG( msg )                                           \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );                   \
        SMTC_MODEM_HAL_TRACE_PRINTF( msg );                                           \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_MSG_COLOR( msg, color )                              \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_TRACE_PRINTF( color );                                         \
        SMTC_MODEM_HAL_TRACE_PRINTF( msg );                                           \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );                   \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_INFO( ... )                                          \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_GREEN );                     \
        SMTC_MODEM_HAL_TRACE_PRINTF( "INFO : " );                                     \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                   \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );                   \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_WARNING( ... )                                       \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_YELLOW );                    \
        SMTC_MODEM_HAL_TRACE_PRINTF( "WARN : " );                                     \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                   \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );                   \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_ERROR( ... )                                         \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_RED );                       \
        SMTC_MODEM_HAL_TRACE_PRINTF( "ERROR: " );                                     \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                   \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );                   \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_ARRAY( msg, array, len )                             \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_TRACE_PRINTF("%s - (%lu bytes):\n", msg, ( uint32_t )len );    \
        for( uint32_t i = 0; i < ( uint32_t )len; i++ )                        \
        {                                                                      \
            if( ( ( i % 16 ) == 0 ) && ( i > 0 ) )                             \
            {                                                                  \
                SMTC_MODEM_HAL_TRACE_PRINTF("\n");                                    \
            }                                                                  \
            SMTC_MODEM_HAL_TRACE_PRINTF( " %02X", array[i] );                         \
        }                                                                      \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" );                                          \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_PACKARRAY( msg, array, len )   \
    do                                                   \
    {                                                    \
        for( uint32_t i = 0; i < ( uint32_t ) len; i++ ) \
        {                                                \
            SMTC_MODEM_HAL_TRACE_PRINTF( "%02X", array[i] );    \
        }                                                \
    } while( 0 );

#else
    #define SMTC_MODEM_HAL_TRACE_PRINTF( ... )
    #define SMTC_MODEM_HAL_TRACE_MSG( msg )
    #define SMTC_MODEM_HAL_TRACE_MSG_COLOR( msg, color )
    #define SMTC_MODEM_HAL_TRACE_INFO( ... )
    #define SMTC_MODEM_HAL_TRACE_WARNING( ... )
    #define SMTC_MODEM_HAL_TRACE_ERROR( ... )
    #define SMTC_MODEM_HAL_TRACE_ARRAY( msg, array, len )
    #define SMTC_MODEM_HAL_TRACE_PACKARRAY( ... )

#endif

#if defined (PERF_TEST_ENABLED)
    #define SMTC_MODEM_HAL_PERF_TEST_TRACE_PRINTF( ... )  smtc_modem_hal_print_trace (  __VA_ARGS__ )
#else
    #define SMTC_MODEM_HAL_PERF_TEST_TRACE_PRINTF( ... )
#endif



#if ( MODEM_HAL_DBG_TRACE_RP == MODEM_HAL_FEATURE_ON )
    #define SMTC_MODEM_HAL_RP_TRACE_MSG( msg )                                        \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_TRACE_PRINTF( "    " );                                        \
        SMTC_MODEM_HAL_TRACE_PRINTF( msg );                                           \
    } while( 0 )

    #define SMTC_MODEM_HAL_RP_TRACE_PRINTF( ... )                                     \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_TRACE_PRINTF( "    " );                                        \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                   \
    } while( 0 )

#else
    #define SMTC_MODEM_HAL_RP_TRACE_MSG( msg )
    #define SMTC_MODEM_HAL_RP_TRACE_PRINTF( ... )
#endif
// clang-format on

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

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_MODEM_HAL_DBG_TRACE_H__

/* --- EOF ------------------------------------------------------------------ */
