/*
Notice:
This software is provided exclusively for evaluation purposes. It is experimental and does not satisfy 
Semtech's software quality standards. 

All Rights Reserved. There is no license provided with this software.

Disclaimer:
“THE SEMTECH SOFTWARE AND THE DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR
IMPLIED WARRANTY OF ANY KIND FROM SEMTECH OR FROM ANY OTHER PERSON OR ENTITY, INCLUDING
WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, OR FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL SEMTECH BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.”
The content of this repository is provided under the GitHub Terms of Services.
Copyright Semtech Corporation 2021. All rights reserved, with the exception of files that
have an explicit license on top of their directory or in their open source code in which
case the later prevails.
*/
#ifndef _LORAGW_H
#define _LORAGW_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */
#include "sx126x-hal.h"

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

/*!
 * \brief Gateway configuration definition
 */
typedef struct {
    uint32_t channel_freq;                  /* Channel frequency */
    RadioLoRaSpreadingFactors_t sf;         /* Channel datarate */
    RadioLoRaBandwidths_t bw;               /* Channel bandwidth */
    int8_t tx_power;                        /* TX power */
    const char* server_address;             /* IP address of the Network Server */
    const int server_port;                  /* UDP port of the Network Server */
    bool antenna_switch_alternate;          /* Indicates if the gateway has 2 antennas and has to alternate */
} gateway_conf_t;

/*!
 * \brief Gateway channel definition (for auto config of the channel)
 */
typedef struct {
    char mac_addr[18];                      /* MAC address of the gateway */
    uint32_t channel_freq;                  /* Channel frequency to associate with the MAC address */
} gateway_freq_conf_t;

#endif /* _LORAGW_H */