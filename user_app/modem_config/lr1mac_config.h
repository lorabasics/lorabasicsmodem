/*!
 * \file      lr1mac_config.h
 *
 * \brief     LR1MAC config file
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

#ifndef __LR1MAC_CONFIG_H__
#define __LR1MAC_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#if defined( SX128X )

// TCXO Startup delay if applicable, if no tcxo is used put 0
#define BSP_LR1MAC_TCXO_STARTUP_DELAY_MS 0
// Crystal error of the MCU to fine adjust the rx window for lorawan ( ex: set 30 for a crystal error = 0.3%)
#define BSP_CRYSTAL_ERROR 0
// Delay introduce by the mcu Have to fine tune to adjust the window rx for lorawan
#define BSP_BOARD_DELAY_WAKE_UP 1
// Rx timeout windows is superior or equal at BSP_MIN_RX_TIMEOUT_DELAY_MS
#define BSP_MIN_RX_TIMEOUT_DELAY_MS 6
// rx fine auto find tune option
#define BSP_LR1MAC_DISABLE_FINE_TUNE 1

#elif defined( LR1110 )

// TCXO Startup delay if applicable, if no tcxo is used put 0
#define BSP_LR1MAC_TCXO_STARTUP_DELAY_MS 5
// Crystal error of the MCU to fine adjust the rx window for lorawan ( ex: set 30 for a crystal error = 0.3%)
#define BSP_CRYSTAL_ERROR 0
// Delay introduce by the mcu Have to fine tune to adjust the window rx for lorawan
#define BSP_BOARD_DELAY_WAKE_UP 1

#elif defined( SX126X )

// TCXO Startup delay if applicable, if no tcxo is used put 0
#define BSP_LR1MAC_TCXO_STARTUP_DELAY_MS 0
// Crystal error of the MCU to fine adjust the rx window for lorawan ( ex: set 30 for a crystal error = 0.3%)
#define BSP_CRYSTAL_ERROR 0
// Delay introduce by the mcu Have to fine tune to adjust the window rx for lorawan
#define BSP_BOARD_DELAY_WAKE_UP 7

#else

// TCXO Startup delay if applicable, if no tcxo is used put 0
#define BSP_LR1MAC_TCXO_STARTUP_DELAY_MS 0
// Crystal error of the MCU to fine adjust the rx window for lorawan ( ex: set 30 for a crystal error = 0.3%)
#define BSP_CRYSTAL_ERROR 0
// Delay introduce by the mcu Have to fine tune to adjust the window rx for lorawan
#define BSP_BOARD_DELAY_WAKE_UP 0

#endif

#define BSP_BOARD_DELAY_RX_SETTING_MS ( BSP_BOARD_DELAY_WAKE_UP + BSP_LR1MAC_TCXO_STARTUP_DELAY_MS )

// The Lorawan context is stored in memory with a period equal to FLASH_UPDATE_PERIOD packets transmitted
#define BSP_USER_NUMBER_OF_RETRANSMISSION 1

// Only used in case of user defined darate distribution strategy refereed to doc that explain this value
#define BSP_USER_DR_DISTRIBUTION_PARAMETERS 0x10000000

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // __LR1MAC_CONFIG_H__

/* --- EOF ------------------------------------------------------------------ */
