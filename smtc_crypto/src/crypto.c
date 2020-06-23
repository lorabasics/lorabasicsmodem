/*!
 * \file      crypto.c
 *
 * \brief     Implements crypto funtions.
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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "lr1mac_utilities.h"
#include "lr1mac_defs.h"
#include "aes.h"
#include "cmac.h"
#include "crypto.h"
#define FileId 5
/*!
 * CMAC/AES Message Integrity Code (MIC) Block B0 size
 */
#define LORAMAC_MIC_BLOCK_B0_SIZE 16

/*!
 * MIC field computation initial data
 */
static uint8_t MicBlockB0[] = {0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*!
 * Contains the computed MIC field.
 *
 * \remark Only the 4 first bytes are used
 */
static uint8_t Mic[16];

/*!
 * Encryption aBlock and sBlock
 */
static uint8_t aBlock[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t sBlock[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*!
 * AES computation context variable
 */
static aes_context AesContext;

/*!
 * CMAC computation context variable
 */
static AES_CMAC_CTX AesCmacCtx[1];

/*!
 * \brief Computes the LoRaMAC frame MIC field
 *
 * \param [IN]  buffer          Data buffer
 * \param [IN]  size            Data buffer size
 * \param [IN]  key             AES key to be used
 * \param [IN]  address         Frame address
 * \param [IN]  dir             Frame direction [0: uplink, 1: downlink]
 * \param [IN]  sequenceCounter Frame sequence counter
 * \param [OUT] mic Computed MIC field
 */

void compute_mic(const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint32_t *mic)
{

    MicBlockB0[5] = dir;

    MicBlockB0[6] = (address)&0xFF;
    MicBlockB0[7] = (address >> 8) & 0xFF;
    MicBlockB0[8] = (address >> 16) & 0xFF;
    MicBlockB0[9] = (address >> 24) & 0xFF;

    MicBlockB0[10] = (sequenceCounter)&0xFF;
    MicBlockB0[11] = (sequenceCounter >> 8) & 0xFF;
    MicBlockB0[12] = (sequenceCounter >> 16) & 0xFF;
    MicBlockB0[13] = (sequenceCounter >> 24) & 0xFF;

    MicBlockB0[15] = size & 0xFF;

    AES_CMAC_Init(AesCmacCtx);

    AES_CMAC_SetKey(AesCmacCtx, key);

    AES_CMAC_Update(AesCmacCtx, MicBlockB0, LORAMAC_MIC_BLOCK_B0_SIZE);

    AES_CMAC_Update(AesCmacCtx, buffer, size & 0xFF);

    AES_CMAC_Final(Mic, AesCmacCtx);

    *mic = (uint32_t)((uint32_t)Mic[3] << 24 | (uint32_t)Mic[2] << 16 | (uint32_t)Mic[1] << 8 | (uint32_t)Mic[0]);
}

void lora_crypto_payload_encrypt(const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *encBuffer)
{

    uint16_t i;
    uint8_t bufferIndex = 0;
    uint16_t ctr = 1;

    memset1(AesContext.ksch, '\0', 240);
    aes_set_key(key, 16, &AesContext);

    aBlock[5] = dir;

    aBlock[6] = (address)&0xFF;
    aBlock[7] = (address >> 8) & 0xFF;
    aBlock[8] = (address >> 16) & 0xFF;
    aBlock[9] = (address >> 24) & 0xFF;

    aBlock[10] = (sequenceCounter)&0xFF;
    aBlock[11] = (sequenceCounter >> 8) & 0xFF;
    aBlock[12] = (sequenceCounter >> 16) & 0xFF;
    aBlock[13] = (sequenceCounter >> 24) & 0xFF;

    while (size >= 16)
    {
        aBlock[15] = ((ctr)&0xFF);
        ctr++;
        aes_encrypt(aBlock, sBlock, &AesContext);
        for (i = 0; i < 16; i++)
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
        size -= 16;
        bufferIndex += 16;
    }

    if (size > 0)
    {
        aBlock[15] = ((ctr)&0xFF);
        aes_encrypt(aBlock, sBlock, &AesContext);
        for (i = 0; i < size; i++)
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
    }
}

void payload_decrypt(const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *decBuffer)
{

    lora_crypto_payload_encrypt(buffer, size, key, address, dir, sequenceCounter, decBuffer);
}

void join_compute_mic(const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t *mic)
{

    AES_CMAC_Init(AesCmacCtx);

    AES_CMAC_SetKey(AesCmacCtx, key);

    AES_CMAC_Update(AesCmacCtx, buffer, size & 0xFF);

    AES_CMAC_Final(Mic, AesCmacCtx);

    *mic = (uint32_t)((uint32_t)Mic[3] << 24 | (uint32_t)Mic[2] << 16 | (uint32_t)Mic[1] << 8 | (uint32_t)Mic[0]);
}

void join_decrypt(const uint8_t *buffer, uint16_t size, const uint8_t *key, uint8_t *decBuffer)
{

    memset1(AesContext.ksch, '\0', 240);
    aes_set_key(key, 16, &AesContext);
    aes_encrypt(buffer, decBuffer, &AesContext);
    // Check if optional CFList is included
    if (size >= 16)
    {
        aes_encrypt(buffer + 16, decBuffer + 16, &AesContext);
    }
}

void join_compute_skeys(const uint8_t *key, const uint8_t *appNonce, uint16_t devNonce, uint8_t *nwkSKey, uint8_t *appSKey)
{

    uint8_t nonce[16];
    uint8_t *pDevNonce = (uint8_t *)&devNonce;

    memset1(AesContext.ksch, '\0', 240);
    aes_set_key(key, 16, &AesContext);

    memset1(nonce, 0, sizeof(nonce));
    nonce[0] = 0x01;
    memcpy1(nonce + 1, appNonce, 6);
    memcpy1(nonce + 7, pDevNonce, 2);
    aes_encrypt(nonce, nwkSKey, &AesContext);

    memset1(nonce, 0, sizeof(nonce));
    nonce[0] = 0x02;
    memcpy1(nonce + 1, appNonce, 6);
    memcpy1(nonce + 7, pDevNonce, 2);
    aes_encrypt(nonce, appSKey, &AesContext);
}

void lora_crypto_add_mic(uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter)
{

    uint32_t mic;
    compute_mic(buffer, size, key, address, dir, sequenceCounter, &mic);
    memcpy(&buffer[size], (uint8_t *)&mic, 4);
}

int check_mic(uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint32_t sequenceCounter, uint32_t micIn)
{
    uint32_t mic;
    int status = -1;
    compute_mic(buffer, size, key, address, 1, sequenceCounter, &mic);
    if (mic == micIn)
    {
        status = 0;
    }
    return (status);
}
int check_join_mic(const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t micIn)
{
    uint32_t mic;
    int status = -1;
    join_compute_mic(buffer, size, key, &mic);
    if (mic == micIn)
    {
        status = 0;
    }
    return (status);
}
