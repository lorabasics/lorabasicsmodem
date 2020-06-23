/*!
 * \file      smtc_bsp_nvm.h
 *
 * \brief     Board specific package NVM API definition.
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
#ifndef __SMTC_BSP_NVM_H__
#define __SMTC_BSP_NVM_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include <string.h>  // memcpy

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

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
/*!
 * Restores the data context from a nvm device.
 *
 * \remark This method invokes memcpy - reads number of bytes from the address
 *
 *  \param addr   Flash address to begin reading from
 *  \param buffer Buffer pointer to write to
 *  \param size   Buffer size to read in bytes
 *  \retval       0 on success, negative error code on failure
 */
int32_t bsp_nvm_context_restore( const uint32_t addr, uint8_t* buffer, const uint32_t size );

/*!
 * Stores the data context to a nvm device
 *
 *  \remark To be safer this function has to implement a read/check data
 *          sequence after programming
 *
 *  \param addr   Flash address to begin writing to
 *  \param buffer Buffer pointer to write from
 *  \param size   Buffer size to be written in bytes
 *  \retval       0 on success, negative error code on failure
 */
int32_t bsp_nvm_context_store( const uint32_t addr, const uint8_t* buffer, const uint32_t size );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_BSP_NVM_H__
