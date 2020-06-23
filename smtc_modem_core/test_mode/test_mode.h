/*!
 * \file      test_mode.h
 *
 * \brief     command parser for hw modem over soft modem
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

#ifndef __TEST_MODE_H__
#define __TEST_MODE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "ral.h"
#include "modem_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * \typedef test_mode_tx_mode_t
 * \brief   Test mode TX type
 */
typedef enum test_mode_tx_mode_e
{
    TEST_MODE_TX_SINGLE = 0,  //!< Transmit a single packet
    TEST_MODE_TX_CONTINUE     //!< Continuously transmit packets
} test_mode_tx_mode_t;

/* clang-format off */
/*!
 * \typedef host_cmd_test_sf_t
 * \brief   Test mode Spreading Factor type
 */
typedef enum host_cmd_test_sf_e
{
    TST_FSK = 0,    //!< FSK
    TST_SF_7,       //!< SF7
    TST_SF_8,       //!< SF8
    TST_SF_9,       //!< SF9
    TST_SF_10,      //!< SF10
    TST_SF_11,      //!< SF11
    TST_SF_12,      //!< SF12
    TST_SF_5,       //!< SF5
    TST_SF_6,       //!< SF6
    TST_SF_MAX
} host_cmd_test_sf_t;

static const uint8_t host_cmd_test_sf_convert[TST_SF_MAX] = {
    [TST_FSK]   = RAL_LORA_SF6,
    [TST_SF_7]  = RAL_LORA_SF7,
    [TST_SF_8]  = RAL_LORA_SF8,
    [TST_SF_9]  = RAL_LORA_SF9,
    [TST_SF_10] = RAL_LORA_SF10,
    [TST_SF_11] = RAL_LORA_SF11,
    [TST_SF_12] = RAL_LORA_SF12,
    [TST_SF_5]  = RAL_LORA_SF5,
    [TST_SF_6]  = RAL_LORA_SF6,
};

/*!
 * \typedef host_cmd_test_bw_t
 * \brief   Test mode Bandwith type
 */
typedef enum host_cmd_test_bw_e
{
    TST_BW_125 = 0, //!< BW125
    TST_BW_250,     //!< BW250
    TST_BW_500,     //!< BW500
    TST_BW_200,     //!< BW200
    TST_BW_400,     //!< BW400
    TST_BW_800,     //!< BW800
    TST_BW_1600,    //!< BW1600
    TST_BW_MAX
} host_cmd_test_bw_t;

static const uint8_t host_cmd_test_bw_convert[TST_BW_MAX] = {
    [TST_BW_125]  = RAL_LORA_BW_125_KHZ,
    [TST_BW_250]  = RAL_LORA_BW_250_KHZ,
    [TST_BW_500]  = RAL_LORA_BW_500_KHZ,
    [TST_BW_200]  = RAL_LORA_BW_200_KHZ,
    [TST_BW_400]  = RAL_LORA_BW_400_KHZ,
    [TST_BW_800]  = RAL_LORA_BW_800_KHZ,
    [TST_BW_1600] = RAL_LORA_BW_1600_KHZ,
};
/*!
 * \typedef host_cmd_test_cr_t
 * \brief   Test mode Coding Rate type
 */
typedef enum host_cmd_test_cr_e
{
    TST_CR_4_5 = 0, //!< CR 4/5
    TST_CR_4_6,     //!< CR 4/6
    TST_CR_4_7,     //!< CR 4/7
    TST_CR_4_8,     //!< CR 4/8
    TST_CR_LI_4_5,  //!< CR 4/5 long interleaved
    TST_CR_LI_4_6,  //!< CR 4/6 long interleaved
    TST_CR_LI_4_8,  //!< CR 4/8 long interleaved
    TST_CR_MAX
} host_cmd_test_cr_t;

static const uint8_t host_cmd_test_cr_convert[TST_CR_MAX] = {
    [TST_CR_4_5]    = RAL_LORA_CR_4_5,
    [TST_CR_4_6]    = RAL_LORA_CR_4_6,
    [TST_CR_4_7]    = RAL_LORA_CR_4_7,
    [TST_CR_4_8]    = RAL_LORA_CR_4_8,
    [TST_CR_LI_4_5] = RAL_LORA_CR_LI_4_5,
    [TST_CR_LI_4_6] = RAL_LORA_CR_LI_4_6,
    [TST_CR_LI_4_8] = RAL_LORA_CR_LI_4_8,
};

/* clang-format on */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief   Get the modem Test mode activated status
 *          Once test mode is active, all other modem commands are disabled.
 *
 * \retval [out]    return                  - is_tst_mode_enabled : true / false
 */
uint8_t test_mode_enabled_is( void );

/*!
 * \brief   Start the modem Test mode
 * \remark  Test mode can only be activated if the modem has not yet received a command that results
 *          in a radio operation.
 *          Once test mode is active, all other modem commands are disabled.
 *
 * \retval [out]    return                  - modem_return_code_t
 */
modem_return_code_t test_mode_start( void );

/*!
 * \brief   No operation Test mode
 * \remark  This command may be used to terminate an ongoing continuous TX operation.
 *
 * \retval [out]    return                  - modem_return_code_t
 */
modem_return_code_t test_mode_nop( void );

/*!
 * \brief   Test mode TX single or continue
 * \remark  Transmit a single packet or continuously transmit packets as fast as possible.
 *
 * \param  [in]     frequency               - Frequency in MHz
 * \param  [in]     pwr_in_dbm              - Power in dbm
 * \param  [in]     sf                      - spreading factor following host_cmd_test_sf_t definition
 * \param  [in]     bw                      - spreading factor following host_cmd_test_bw_t definition
 * \param  [in]     cr                      - spreading factor following host_cmd_test_cr_t definition
 * \param  [in]     payload_length          - Number of byte that will be sent
 * \param  [in]     tx_mode                 - TEST_MODE_TX_SINGLE / TEST_MODE_TX_CONTINUE
 *
 * \retval [out]    return                  - modem_return_code_t
 */
modem_return_code_t test_mode_tx( uint32_t frequency, int8_t pwr_in_dbm, host_cmd_test_sf_t sf, host_cmd_test_bw_t bw,
                                  host_cmd_test_cr_t cr, uint8_t payload_length, test_mode_tx_mode_t tx_mode );

/*!
 * \brief   Test mode transmit a continuous wave.
 * \remark
 *
 * \param  [in]     frequency               - Frequency in MHz
 * \param  [in]     pwr_in_dbm              - Power in dbm
 *
 * \retval [out]    return                  - modem_return_code_t
 */
modem_return_code_t test_mode_cw( uint32_t frequency, int8_t pwr_in_dbm );

/*!
 * \brief   Test mode RX continue
 * \remark  Continuously receive packets.
 *
 * \param  [in]     frequency               - Frequency in MHz
 * \param  [in]     sf                      - spreading factor following host_cmd_test_sf_t definition
 * \param  [in]     bw                      - spreading factor following host_cmd_test_bw_t definition
 * \param  [in]     cr                      - spreading factor following host_cmd_test_cr_t definition
 *
 * \retval [out]    return                  - modem_return_code_t
 */
modem_return_code_t test_mode_rx_cont( uint32_t frequency, host_cmd_test_sf_t sf, host_cmd_test_bw_t bw,
                                       host_cmd_test_cr_t cr );

/*!
 * \brief   Test mode radio reset
 * \remark  Reset the radio.
 *
 * \retval [out]    return                  - modem_return_code_t
 */
modem_return_code_t test_mode_radio_reset( void );

/*!
 * \brief   Test mode exit
 * \remark  Exit test mode and reset modem.
 *
 * \retval [out]    return                  - modem_return_code_t
 */
modem_return_code_t test_mode_exit( void );

/*!
 * \brief   Test mode radio write
 * \remark  Send SPI commands directly to the SX12xx radio.
 *
 * \param  [in]     command*                - radio command
 * \param  [in]     command_length          - radio command length
 * \param  [in]     data*                   - related data to the command
 * \param  [in]     data_length             - data length
 *
 * \retval [out]    return                  - modem_return_code_t
 */
modem_return_code_t test_mode_radio_write( uint8_t* command, uint16_t command_length, uint8_t* data,
                                           uint16_t data_length );

/*!
 * \brief   Test mode radio read
 * \remark  Receive SPI commands directly to the SX12xx radio.
 *
 * \param  [in]     command*                - radio command
 * \param  [in]     command_length          - radio command length
 * \param  [out]    data*                   - output of the requested command
 * \param  [in]     data_length             - return data length must be set by the user
 *
 * \retval [out]    return                  - modem_return_code_t
 */
modem_return_code_t test_mode_radio_read( uint8_t* command, uint16_t command_length, uint8_t* data,
                                          uint16_t data_length );
#ifdef __cplusplus
}
#endif

#endif  //__TEST_MODE_H__
