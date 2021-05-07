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
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
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

#include "file_upload_defs.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */
#define FILE_UPLOAD_HEADER_SIZE 12

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
 * TODO make this enum and file_upload_s in private, and only deal with file
 * upload session IDs
 */

enum
{
    BUFSZ       = ( ( 2 * 1024 ) + FILE_UPLOAD_HEADER_SIZE ),  // buffer size + header
    NSESS       = 1,                                           // number of concurrent sessions
    CHUNK_NW    = 2,                                           // number of words per chunk
    MAXOVERHEAD = 2,                                           // max factor of chunks to send
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

typedef struct file_upload_s
{
    uint32_t  active;  // active session
    uint32_t  header[3];
    uint32_t* wordbuf;                 // data buffer
    uint16_t  cct[NSESS];              // chunk count
    uint8_t   session_counter[NSESS];  // session counter
    uint16_t  cntx[NSESS];             // chunk transmission count
    uint8_t   fntx[NSESS];             // frame transmission count
    uint16_t  average_delay[NSESS];    // average frame transmission rate/delay

} file_upload_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief   create upload
 * \remark
 *
 * \param  [in]     file_upload*            - Pointer to File Upload context
 * \param  [in]     sid                     - session ID
 * \param  [in]     pdata                   - data
 * \param  [in]     sz                      - size of data
 * \param  [in]     average_delay           - delay between each uplink frame
 * \param  [in]     port                    - applicative where the data will be
 * forwarded \param  [in]     encryption              - file upload encrypted or
 * not \retval          int32_t                 - unique session ID counter, -1
 * in case of error
 */
int32_t file_upload_create( file_upload_t* file_upload, uint32_t sid, uint32_t** pdata, uint32_t sz,
                            uint16_t average_delay, uint8_t port, uint8_t encryption );

/*!
 * \brief   uplink fragment generation
 * \remark
 *
 * \param  [in]     file_upload*            - Pointer to File Upload context
 * \param  [out]    buf*                    - buffer that contains the fragment
 * \param  [out]    bufsz                   - buffer size
 * \param  [in]     sid                     - session ID
 * \param  [in]     fcnt                    - frame counter
 * \retval          int32_t                 - Return the number of pending
 * byte(s)
 */
int32_t file_upload_gen_uplink( file_upload_t* file_upload, unsigned char* buf, int32_t bufsz, uint32_t sid,
                                uint32_t fcnt );

/*!
 * \brief   uplink fragment generation
 * \remark
 *
 * \param  [in]     file_upload*            - Pointer to File Upload context
 * \param  [in]     sid                     - session ID
 * \retval          uint8_t                 - Return the session counter ID
 */
uint8_t file_upload_get_session_counter( file_upload_t* file_upload, uint32_t sid );

/*!
 * \brief   set hash key in header byte 1
 * \remark
 *
 * \param  [in]     file_upload*             - Pointer to File Upload context
 * \param  [in]     hash0                    - fileupload hash 0
 * \param  [in]     hash1                    - fileupload hash 1
 * \retval          void
 */
void file_upload_set_hash( file_upload_t* file_upload, uint32_t hash0, uint32_t hash1 );
/*!
 * \brief   return the encrypted mode
 * \remark
 *
 * \param  [in]     file_upload*             - Pointer to File Upload context
 * \retval          file_upload_encrypt_mode_t
 */
file_upload_encrypt_mode_t file_upload_get_encryption_mode( file_upload_t* file_upload );

/*!
 * \brief   return file_upload_attach_payload_buffer
 * \remark
 *
 * \param  [in]     file_upload*             - Pointer to File Upload context
 * \param  [in]     file*
 * \retval          revoidturn
 */
void file_upload_attach_payload_buffer( file_upload_t* file_upload, uint8_t* file );

#ifdef __cplusplus
}
#endif

#endif  // __FILE_UPLOAD_H__

/* --- EOF ------------------------------------------------------------------ */
