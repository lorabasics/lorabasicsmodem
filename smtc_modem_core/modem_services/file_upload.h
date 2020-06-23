/*!
 * \file      file_upload.h
 *
 * \brief     File upload API
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

#ifndef __FILE_UPLOAD_H__
#define __FILE_UPLOAD_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>  // C99 types

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define MODE_0 0
#define FILE_UPLOAD_HEADER_SIZE 12
#define FILE_UPLOAD_DIRECTION 0x40

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * \typedef file_upload_encrypt_mode_t
 * \brief   File Upload Encrypt Mode
 */
typedef enum file_upload_encrypt_mode
{
    FILE_UPLOAD_NOT_ENCRYPTED = 0x00,  //!< File Upload not encrypted
    FILE_UPLOAD_ENCRYPTED     = 0x01   //!< File Upload encrypted
} file_upload_encrypt_mode_t;
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief   create upload
 * \remark
 *
 * \param  [in]     sid                     - session ID
 * \param  [in]     pdata                   - data
 * \param  [in]     sz                      - size of data
 * \param  [in]     average_delay           - delay between each uplink frame
 * \param  [in]     port                    - applicative where the data will be forwarded
 * \param  [in]     encryption              - file upload encrypted or not
 * \retval          int32_t                 - unique session ID counter, -1 in case of error
 */
int32_t file_upload_create( uint32_t sid, uint32_t** pdata, uint32_t sz, uint16_t average_delay, uint8_t port,
                            uint8_t encryption );

/*!
 * \brief   uplink fragment generation
 * \remark
 *
 * \param  [out]    buf*                    - buffer that contains the fragment
 * \param  [out]    bufsz                   - buffer size
 * \param  [in]     sid                     - session ID
 * \param  [in]     fcnt                    - frame counter
 * \retval          int32_t                 - Return the number of pending byte(s)
 */
int32_t file_upload_gen_uplink( unsigned char* buf, int32_t bufsz, uint32_t sid, uint32_t fcnt );

/*!
 * \brief   uplink fragment generation
 * \remark
 *
 * \param  [in]     sid                     - session ID
 * \retval          uint8_t                 - Return the session counter ID
 */
uint8_t file_upload_get_session_counter( uint32_t sid );
/*!
 * \brief   set hash key in header byte 1
 * \remark
 *
 * \param  [in]     hash0                    - fileupload hash 0
 * \param  [in]     hash1                    - fileupload hash 1
 * \retval          void
 */
void file_upload_set_hash( uint32_t hash0, uint32_t hash1 );
/*!
 * \brief   return the encrypted mode
 * \remark
 *
 * \retval          file_upload_encrypt_mode_t
 */
file_upload_encrypt_mode_t file_upload_get_encryption_mode( void );

/*!
 * \brief   return file_upload_attach_payload_buffer
 * \remark
 *
 * \param  [in]     file*
 * \retval          revoidturn
 */
void file_upload_attach_payload_buffer( uint8_t* file );

#ifdef __cplusplus
}
#endif

#endif  // __FILE_UPLOAD_H__

/* --- EOF ------------------------------------------------------------------ */
