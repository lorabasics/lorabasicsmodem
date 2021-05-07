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
#ifndef _USERCONF_H
#define _USERCONF_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include "loragw.h"

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define CHANNEL_FREQ    470900000UL

/*!
 * \brief Gateway configuration to be applied
 */
static gateway_conf_t gw_conf = {
    .channel_freq               = CHANNEL_FREQ,   /* Default channel, if MAC address is unknown */
    .sf                         = LORA_SF7,
    .bw                         = LORA_BW_125,
    .tx_power                   = 15,
    .server_address             = "192.168.1.1",  /* change to ip of NS */
    .server_port                = 1700,                 
    .antenna_switch_alternate   = false
};

/*!
 * \brief Association table MAC address / channel frequency
 */
#define GW_FREQ_CONF_NB 1
static gateway_freq_conf_t gw_freq_conf[GW_FREQ_CONF_NB] = {
    {
        /* GW 1 - Hybrid Single Channel Gateway */
        .mac_addr = "00:80:E1:36:00:2C",
        .channel_freq = CHANNEL_FREQ
    }
};

#endif /* _USERCONF_H */
