/*!
 * \file      bsp_lr1mac.h
 *
 * \brief     board and user specific definitions for lr1mac lorawan stack
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

#ifndef __BSP_LR1MAC_H__
#define __BSP_LR1MAC_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

// Crystal error of the MCU to fine adjust the rx window for lorawan ( ex: set 30 for a crystal error = 0.3%)
#if defined( SX1280 )
#define BSP_CRYSTAL_ERROR 0
#else
#define BSP_CRYSTAL_ERROR 2
#endif

// Delay introduce by the mcu Have to fine tune to adjust the window rx for lorawan
#if defined( SX1280 )
#define BSP_BOARD_DELAY_RX_SETTING_MS 1
#else
#define BSP_BOARD_DELAY_RX_SETTING_MS 7
#endif

// Rx timeout windows is superior or equal at BSP_MIN_RX_TIMEOUT_DELAY_MS
#if defined( SX1280 )
#define BSP_MIN_RX_TIMEOUT_DELAY_MS 6
#endif

// start flash address to store lorawan context
#define BSP_LORAWAN_CONTEXT_ADDR_OFFSET 0

// The Lorawan context is stored in memory with a period equal to FLASH_UPDATE_PERIOD packets transmitted
#define BSP_USER_NUMBER_OF_RETRANSMISSION 1

// Only used in case of user defined darate distribution strategy refereed to doc that explain this value
#define BSP_USER_DR_DISTRIBUTION_PARAMETERS 0x10000000

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

#endif  // __BSP_LR1MAC_H__

/* --- EOF ------------------------------------------------------------------ */
