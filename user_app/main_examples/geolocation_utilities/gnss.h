/*!
 * \file      gnss.h
 *
 * \brief     GNSS definition
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

#ifndef __GNSS_H__
#define __GNSS_H__

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"

#include "device_management_defs.h"
#include "modem_utilities.h"
#include "lr1110_types.h"
#include "lr1110_gnss.h"
#include "lr1110_system.h"
/*
import base64
import requests

YOUR_TOKEN = 'Type your token, available at https://www.loracloud.com/portal/geolocation/token_management'

url = 'https://gls.loracloud.com/api/v3/almanac/full'
my_header = {'Ocp-Apim-Subscription-Key': YOUR_TOKEN}
res = requests.get(url, headers=my_header)
raw_bytes = bytes(base64.b64decode(res.json()["result"]["almanac_image"]))
my_almanac_in_hex = "static const lr1110_gnss_almanac_full_update_bytestream_t full_almanac = { "
my_almanac_in_hex += ", ".join("0x{:02X}".format(byt) for byt in raw_bytes)
my_almanac_in_hex += " };"
print(my_almanac_in_hex)
*/
//#include "almanac_2020_07_03.h"
#include "almanac_2021_04_06.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define TAG_NAV                6
#define DM_INTERVAL_USER_MIN   60


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Init for gnss scan
 *
 * @param assistance_position  - the latitude and longitude values of assistance position
 *
 */
bool gnss_init( const lr1110_gnss_solver_assistance_position_t assistance_position );

/*!
 * @brief Trigger an almanac update with local almanac
 *
 * @remarks modem_suspend_radio_access must be called before and modem_resume_radio_access after
 */
void update_local_almanac( void );

/*!
 * @brief Start a GNSS scan and stream data if scan is a success
 *
 * @param [in] stack_id      - Chosen stack id
 * @param [in] port          - The port used for streaming
 * @return bool              - Scan status, will return false and skip the streaming if no statellites found
 *
 * @remarks modem_suspend_radio_access must be called before and modem_resume_radio_access after
 */
bool scan_and_stream_gnss( uint8_t stack_id, uint8_t port );


#endif  // __GNSS_H__

/* --- EOF ------------------------------------------------------------------ */
