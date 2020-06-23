/*!
 * \file      smtc_bsp_dbg_trace.h
 *
 * \brief     Board specific package debug log API definition.
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
#ifndef __SMTC_BSP_DBG_TRACE_H__
#define __SMTC_BSP_DBG_TRACE_H__

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

#include "smtc_bsp_options.h"
#include "smtc_bsp_mcu.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

// clang-format off
#if( BSP_DBG_TRACE_COLOR == BSP_FEATURE_ON )
    #define BSP_DBG_TRACE_COLOR_BLACK               "\x1B[0;30m"
    #define BSP_DBG_TRACE_COLOR_RED                 "\x1B[0;31m"
    #define BSP_DBG_TRACE_COLOR_GREEN               "\x1B[0;32m"
    #define BSP_DBG_TRACE_COLOR_YELLOW              "\x1B[0;33m"
    #define BSP_DBG_TRACE_COLOR_BLUE                "\x1B[0;34m"
    #define BSP_DBG_TRACE_COLOR_MAGENTA             "\x1B[0;35m"
    #define BSP_DBG_TRACE_COLOR_CYAN                "\x1B[0;36m"
    #define BSP_DBG_TRACE_COLOR_WHITE               "\x1B[0;37m"
    #define BSP_DBG_TRACE_COLOR_DEFAULT             "\x1B[0m"
#else
    #define BSP_DBG_TRACE_COLOR_BLACK   ""
    #define BSP_DBG_TRACE_COLOR_RED     ""
    #define BSP_DBG_TRACE_COLOR_GREEN   ""
    #define BSP_DBG_TRACE_COLOR_YELLOW  ""
    #define BSP_DBG_TRACE_COLOR_BLUE    ""
    #define BSP_DBG_TRACE_COLOR_MAGENTA ""
    #define BSP_DBG_TRACE_COLOR_CYAN    ""
    #define BSP_DBG_TRACE_COLOR_WHITE   ""
    #define BSP_DBG_TRACE_COLOR_DEFAULT ""
#endif

#if ( BSP_DBG_TRACE ) && !defined (PERF_TEST_ENABLED)

    #if ( UNIT_TEST_DBG )
        #define BSP_DBG_TRACE_PRINTF( ... )  printf (  __VA_ARGS__ )
    #else
        #define BSP_DBG_TRACE_PRINTF( ... )  bsp_trace_print (  __VA_ARGS__ )
    #endif

    #define BSP_DBG_TRACE_MSG( msg )                                           \
    do                                                                         \
    {                                                                          \
        BSP_DBG_TRACE_PRINTF( BSP_DBG_TRACE_COLOR_DEFAULT );                   \
        BSP_DBG_TRACE_PRINTF( msg );                                           \
    } while ( 0 );

    #define BSP_DBG_TRACE_MSG_COLOR( msg, color )                              \
    do                                                                         \
    {                                                                          \
        BSP_DBG_TRACE_PRINTF( color );                                         \
        BSP_DBG_TRACE_PRINTF( msg );                                           \
        BSP_DBG_TRACE_PRINTF( BSP_DBG_TRACE_COLOR_DEFAULT );                   \
    } while ( 0 );

    #define BSP_DBG_TRACE_INFO( ... )                                          \
    do                                                                         \
    {                                                                          \
        BSP_DBG_TRACE_PRINTF( BSP_DBG_TRACE_COLOR_GREEN );                     \
        BSP_DBG_TRACE_PRINTF( "INFO : " );                                     \
        BSP_DBG_TRACE_PRINTF( __VA_ARGS__ );                                   \
        BSP_DBG_TRACE_PRINTF( BSP_DBG_TRACE_COLOR_DEFAULT );                   \
    } while ( 0 );

    #define BSP_DBG_TRACE_WARNING( ... )                                       \
    do                                                                         \
    {                                                                          \
        BSP_DBG_TRACE_PRINTF( BSP_DBG_TRACE_COLOR_YELLOW );                    \
        BSP_DBG_TRACE_PRINTF( "WARN : " );                                     \
        BSP_DBG_TRACE_PRINTF( __VA_ARGS__ );                                   \
        BSP_DBG_TRACE_PRINTF( BSP_DBG_TRACE_COLOR_DEFAULT );                   \
    } while ( 0 );

    #define BSP_DBG_TRACE_ERROR( ... )                                         \
    do                                                                         \
    {                                                                          \
        BSP_DBG_TRACE_PRINTF( BSP_DBG_TRACE_COLOR_RED );                       \
        BSP_DBG_TRACE_PRINTF( "ERROR: " );                                     \
        BSP_DBG_TRACE_PRINTF( __VA_ARGS__ );                                   \
        BSP_DBG_TRACE_PRINTF( BSP_DBG_TRACE_COLOR_DEFAULT );                   \
    } while ( 0 );

    #define BSP_DBG_TRACE_ARRAY( msg, array, len )                             \
    do                                                                         \
    {                                                                          \
        BSP_DBG_TRACE_PRINTF("%s - (%lu bytes):\n", msg, ( uint32_t )len );    \
        for( uint32_t i = 0; i < ( uint32_t )len; i++ )                        \
        {                                                                      \
            if( ( ( i % 16 ) == 0 ) && ( i > 0 ) )                             \
            {                                                                  \
                BSP_DBG_TRACE_PRINTF("\n");                                    \
            }                                                                  \
            BSP_DBG_TRACE_PRINTF( " %02X", array[i] );                         \
        }                                                                      \
        BSP_DBG_TRACE_PRINTF( "\n" );                                          \
    } while ( 0 );

    #define BSP_DBG_TRACE_PACKARRAY( msg, array, len )   \
    do                                                   \
    {                                                    \
        for( uint32_t i = 0; i < ( uint32_t ) len; i++ ) \
        {                                                \
            BSP_DBG_TRACE_PRINTF( "%02X", array[i] );    \
        }                                                \
    } while( 0 );

#else
    #define BSP_DBG_TRACE_PRINTF( ... )
    #define BSP_DBG_TRACE_MSG( msg )
    #define BSP_DBG_TRACE_MSG_COLOR( msg, color )
    #define BSP_DBG_TRACE_INFO( ... )
    #define BSP_DBG_TRACE_WARNING( ... )
    #define BSP_DBG_TRACE_ERROR( ... )
    #define BSP_DBG_TRACE_ARRAY( msg, array, len )
    #define BSP_DBG_TRACE_PACKARRAY( ... )

#endif

#if defined (PERF_TEST_ENABLED)
    #define BSP_PERF_TEST_TRACE_PRINTF( ... )  bsp_trace_print (  __VA_ARGS__ )
#else
    #define BSP_PERF_TEST_TRACE_PRINTF( ... )
#endif



#if ( BSP_DBG_TRACE_RP == BSP_FEATURE_ON )
    #define BSP_DBG_TRACE_MSG_RP( msg )                                        \
    do                                                                         \
    {                                                                          \
        BSP_DBG_TRACE_PRINTF( "    " );                                        \
        BSP_DBG_TRACE_PRINTF( msg );                                           \
    } while( 0 )

    #define BSP_DBG_TRACE_PRINTF_RP( ... )                                     \
    do                                                                         \
    {                                                                          \
        BSP_DBG_TRACE_PRINTF( "    " );                                        \
        BSP_DBG_TRACE_PRINTF( __VA_ARGS__ );                                   \
    } while( 0 )

#else
    #define BSP_DBG_TRACE_MSG_RP( msg )
    #define BSP_DBG_TRACE_PRINTF_RP( ... )
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

#endif  // __SMTC_BSP_DBG_TRACE_H__

/* --- EOF ------------------------------------------------------------------ */
