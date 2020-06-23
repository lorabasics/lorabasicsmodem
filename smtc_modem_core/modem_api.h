/*!
 * \file      modem_api.h
 *
 * \brief     soft modem API description
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

#ifndef __MODEM_API_H__
#define __MODEM_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "modem_context.h"
#include "lr1mac_defs.h"
#include "file_upload.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define FW_VERSION 0x00010000

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * \typedef modem_return_code_t
 * \brief   Modem Return Codes
 */
typedef enum modem_return_code
{
    RC_OK              = 0,    //!< command executed without errors
    RC_UNKNOWN         = 1,    //!< command code unknown
    RC_NOT_IMPLEMENTED = 2,    //!< command not yet implemented
    RC_NOT_INIT        = 3,    //!< command not initialized
    RC_INVALID         = 4,    //!< command parameters invalid
    RC_BUSY            = 5,    //!< command cannot be executed now
    RC_FAIL            = 6,    //!< command execution failed
    RC_BAD_FMT         = 7,    //!< format check failed
    RC_BAD_CRC         = 8,    //!< crc check failed
    RC_BAD_SIG         = 9,    //!< signature verification failed
    RC_BAD_SIZE        = 0xA,  //!< size check failed
    RC_RESERVED1       = 0xb,  //!< RFU
    RC_RESERVED2       = 0xc,  //!< RFU
    RC_RESERVED3       = 0xd,  //!< RFU
    RC_RESERVED4       = 0xe,  //!< RFU
    RC_FRAME_ERROR     = 0xf,  //!< serial port framing error
} modem_return_code_t;

/*!
 * \typedef e_tx_mode_t
 * \brief   Tx mode Confirmed / Unconfirmed
 */
typedef enum e_tx_mode
{
    TX_UNCONFIRMED = 0x00,  //!< Tx packet in Unconfirmed mode
    TX_CONFIRMED   = 0x01   //!< Tx packet in Confirmed mode
} e_tx_mode_t;

/*!
 * \typedef e_emergency_tx_t
 * \brief   Emergency definition
 */
typedef enum e_emergency_tx
{
    TX_EMERGENCY_OFF = 0x00,  //!< Normal Tx
    TX_EMERGENCY_ON  = 0x01   //!< Emergency Tx
} e_emergency_tx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief    Initialize the modem
 * \remark   This command must be called once in main
 *
 * \param  [in]     callback*               - Callback that will be called if an event is available
 */

void modem_init( void ( *callback )( void ) );

/*!
 * \brief    Run the modem engine
 * \remark   This function MUST be called at least at each time the delay_time_ms returned by it
 *
 * \retval  delay_time_ms           - The maximum delay in ms at which time the engine MUST be recalled
 */

uint32_t modem_run_engine( void );

/*!
 * \brief    Get the modem event
 * \remark   This command can be used to retrieve pending events from the modem.
 *           return error in the following cases :
 *              1) number of asynch message > number of type of asynch msg (should be impossible)
 *
 * \param  [out]    type*                   - Return the event type
 * \param  [out]    count*                  - Return number of missed events of this type in case of overrun.
 * \param  [out]    event_data*             - Return event specific data
 * \param  [out]    event_data_length*      - Return event specific data length
 * \param  [out]    asynchronous_msgnumber* - Return number of pending event(s)
 *
 * \retval  modem_return_code_t
 */

modem_return_code_t modem_get_event( modem_rsp_event_t* type, uint8_t* count, uint8_t* event_data,
                                     uint8_t* event_data_length, uint8_t* asynchronous_msgnumber );

/*!
 * \brief   Get the modem versions
 * \remark  This command returns the version of the bootloader and the version of the installed firmware
 *          plus the version of the implemented LoRaWAN standard (BCD).
 *
 * \param  [out]    bootloader*             - Return the bootloader version
 * \param  [out]    firmware*               - Return the firmware version
 * \param  [out]    lorawan*                - Return the LoRaWAN version
 *
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_version( uint32_t* bootloader, uint32_t* firmware, uint16_t* lorawan );

/*!
 * \brief   Reset the modem
 * \remark  This command performs a reset of the modem MCU.
 *          All transient state (including session information) will be lost
 *          and the modem needs to join the network again
 *
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_reset( void );

/*!
 * \brief   Factory Reset the modem
 * \remark  This command performs a factory reset. In addition to the MCU reset all persistent settings
 *          are reset back to their factory state
 *
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_factory_reset( void );

/*!
 * \brief   Get the modem charge
 * \remark  This command returns the total charge counter of the modem in mAh.
 *
 * \param  [out]    charge*                 - Return accumulated charge
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_charge( uint32_t* charge );

/*!
 * \brief   Reset the modem charge
 * \remark  This command resets the accumulated charge counter to zero.
 *
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_reset_charge( void );

/*!
 * \brief   Get the Tx power offset
 * \remark  This command gets the board-specific correction offset for transmission power to be used
 *          (signed integer in dB)
 *
 * \param  [out]    tx_pwr_offset*          - Return Tx power offset in dB
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_tx_power_offset( int8_t* tx_pwr_offset );

/*!
 * \brief   Set the Tx power offset
 * \remark  This command sets the board-specific correction offset for transmission power to be used
 *
 * \param  [in]     tx_pwr_offset           - Tx power offset in dB
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_tx_power_offset( int8_t tx_pwr_offset );

/*!
 * \brief   Get GPS wall time
 * \remark  The returned time specifies the seconds since GPS epoch (00:00:00, Sunday 6th of January 1980).
 *          If the device is not yet synchronized to GPS time then the returned value is zero.
 *          In case of time wrapping, the time could be really equal to zero so the value is incremented of 1 to avoid 0
 *
 * \param  [out]    gps_time_s*             - Return GPS time in seconds
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_time( uint32_t* gps_time_s );

/*!
 * \brief   Get the modem status
 * \remark  This command returns the modem status which may indicate one or more notification conditions
 *
 * \param  [out]    status*                 - Return the modem status
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_status( uint8_t* status );

/*!
 * \brief   Set Alarm timer
 * \remark  This command sets a application alarm timer (in seconds).
 *          When the timer has expired an Alarm event is generated.
 *
 * \param  [in]     alarm                   - Alarm in seconds
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_alarm_timer_s( uint32_t alarm );

/*!
 * \brief   Get the JoinEUI
 * \remark  This command returns the join EUI.
 *
 * \param  [out]    joineui*                - Return the JoinEUI
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_joineui( uint8_t* joineui );

/*!
 * \brief   Set the JoinEUI
 * \remark  This command set the join EUI.
 *
 * \param  [in]     joineui*                - JoinEUI
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_joineui( uint8_t joineui[8] );

/*!
 * \brief   Get the DevEUI
 * \remark  This command returns the DevEUI.
 *
 * \param  [out]    deveui*                 - Return the DevEUI
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_deveui( uint8_t* deveui );

/*!
 * \brief   Set the DevEUI
 * \remark  This command set the DevEUI.
 *
 * \param  [in]     deveui*                 - DevEUI
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_deveui( uint8_t deveui[8] );

/*!
 * \brief   Set the Network Key (for 1.0.3 set the App Key)
 * \remark  This command set the Network Key (for 1.0.3 set the App Key)
 *
 * \param  [in]     nwkkey*                 - NwkKey
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_nwkkey( uint8_t nwkkey[16] );

/*!
 * \brief   Get the device Class
 * \remark  This command gets the LoRaWAN device class.
 *
 * \param  [out]    class*                  - Return the class
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_class( modem_class_t* class );

/*!
 * \brief   Set the device Class
 * \remark  This command set the LoRaWAN device class.
 *
 * \param  [in]     class                   - class
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_class( modem_class_t class );

/*!
 * \brief   Get the region
 * \remark  This command returns the regulatory region.
 *
 * \param  [out]    region*                 - Return the region
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_region( uint8_t* region );

/*!
 * \brief   Set the region
 * \remark  This command sets the regulatory region
 *
 * \param  [in]     region                  - region
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_region( uint8_t region );

/*!
 * \brief   List region
 * \remark  This command returns the regulatory regions supported by the modem.
 *
 * \param  [out]    region_list*            - Return a region list
 * \param  [out]    nb_region*              - Return the number of region in list
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_list_region( uint8_t* region_list, uint8_t* nb_region );

/*!
 * \brief   Get ADR profile
 * \remark  This command returns the ADR profile type.
 *
 * \param  [out]    adr_profile*            - Return adr profile
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_adr_profile( dr_strategy_t* adr_profile );

/*!
 * \brief   Set ADR profile
 * \remark  This command sets the ADR profile and parameters.
 *
 * \param  [in]     adr_profile             - ADR profile
 * \param  [in]     adr_custom_data*        - ADR custom profile data
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_adr_profile( dr_strategy_t adr_profile, uint8_t adr_custom_data[16] );

/*!
 * \brief   Get DM port
 * \remark  This command gets the device management port.
 *
 * \param  [out]    dm_port*                - Return the DM port
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_dm_port( uint8_t* dm_port );

/*!
 * \brief   Set DM port
 * \remark  This command sets the device management port.
 *
 * \param  [in]     dm_port                 - DM port
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_dm_port( uint8_t dm_port );

/*!
 * \brief   Get DM info interval
 * \remark  This command returns the device management reporting interval.
 *          The periodic status reporting interval field is encoded in one
 *          byte where the two top-most bits specify the unit :
 *          (00 - sec, 01 - day, 10 - hour, 11 - min),
 *          and the lower six bits the value 0-63.
 *          A value of zero disables the periodic status reporting
 *
 * \param  [out]    interval*               - Return interval
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_dm_info_interval( uint8_t* interval );

/*!
 * \brief   Set DM info interval
 * \remark  This command sets the device management reporting interval.
 *          The periodic status reporting interval field is encoded in one
 *          byte where the two top-most bits specify the unit :
 *          (00 - sec, 01 - day, 10 - hour, 11 - min),
 *          and the lower six bits the value 0-63.
 *          A value of zero disables the periodic status reporting
 *
 * \param  [in]     interval                - interval
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_dm_info_interval( uint8_t interval );

/*!
 * \brief   Get DM info fields
 * \remark  This command lists the info fields to be included in the periodic DM status messages.
 *
 * \param  [out]    dm_fields_payload*      - Return the dm fields
 * \param  [out]    dm_field_length*        - Return the dm field length
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_dm_info_fields( uint8_t* dm_fields_payload, uint8_t* dm_field_length );

/*!
 * \brief
 * \remark
 *
 * \param  [in]     dm_fields_payload*      - The dm fields
 * \param  [in]     dm_field_length         - The dm field length
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_dm_info_fields( uint8_t* dm_fields_payload, uint8_t dm_field_length );

/*!
 * \brief   Request a DM status now
 * \remark  This command sends the specified set of information fields
 *          in one or more DM status messages immediately.
 *
 * \param  [in]     dm_fields_payload*      - The dm fields
 * \param  [in]     dm_field_length         - The dm field length
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_send_dm_status( uint8_t* dm_fields_payload, uint8_t dm_field_length );

/*!
 * \brief   Set application-specific status in DM
 * \remark  This commands sets application-specific status information to be reported to the DM service.
 *
 * \param  [in]     app_status*             - App status payload
 * \param  [in]     app_status_length       - App status length, must be 8
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_set_app_status( uint8_t app_status[dm_info_field_sz[e_inf_appstatus]],
                                          uint8_t app_status_length );

/*!
 * \brief   Join the network
 * \remark  This command starts joining the network
 *
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_join( void );

/*!
 * \brief   Leave the Network
 * \remark  This command leaves the network if already joined, or cancels an ongoing join process.
 *          After leaving the network, no further transmissions can occur.
 *
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_leave_network( void );

/*!
 * \brief   Suspend the modem communication
 * \remark  This command temporarily suspends or resumes the modem’s radio operations.
 *          Operations are suspended with parameter value 0x01 and resumed with parameter value 0x00.
 *
 * \param  [in]     suspend
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_suspend_modem_comm( e_modem_suspend_t suspend );

/*!
 * \brief   Get next Tx max payload size
 * \remark  This command returns the maximum application payload size possible according
 *          to the LoRaWAN regional parameters for the next transmission using the current data rate
 *
 * \param  [out]    tx_max_payload*         - Return the tx_max_payload
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_get_next_tx_max_payload( uint8_t* tx_max_payload );

/*!
 * \brief   Request Tx LoRaWAN
 * \remark  This command requests sending the given data on the specified port
 *          as an unconfirmed or confirmed frame.
 *          return error in the following cases :
 *              Device in ABP mode
 *              Device already join or join on going
 *
 * \param  [in]     f_port                  - Frame port
 * \param  [in]     msg_type                - Confirmed / Unconfirmed
 * \param  [in]     payload*                - Data
 * \param  [in]     payload_length          - Data length
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_request_tx( uint8_t f_port, e_tx_mode_t msg_type, uint8_t* payload, uint8_t payload_length );

/*!
 * \brief   Emergency Tx LoRaWAN
 * \remark  This command requests sending the given data on the specified port
 *          as an unconfirmed or confirmed frame immediately. It has higher priority
 *          than all other services and does not take duty cycle.
 *          return error in the following cases :
 *              Device in ABP mode
 *              Device already join or join on going
 *
 * \param  [in]     f_port                  - Frame port
 * \param  [in]     msg_type                - Confirmed / Unconfirmed
 * \param  [in]     payload*                - Data
 * \param  [in]     payload_length          - Data length
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_emergency_tx( uint8_t f_port, e_tx_mode_t msg_type, uint8_t* payload,
                                        uint8_t payload_length );

/*!
 * \brief   Create the upload_init
 * \remark  This command prepares a fragmented file upload.
 *
 * \param  [in]     f_port                  - Frame port
 * \param  [in]     encryption_mode         - 0x00: no encrypted,
 *                                            0x01: encrypted using a 128-bit AES key derived from the AppSKey
 * \param  [in]     size
 * \param  [in]     average_delay
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_upload_init( uint8_t f_port, file_upload_encrypt_mode_t encryption_mode, uint16_t size,
                                       uint16_t average_delay );

/*!
 * \brief   Create the upload_start
 * \remark  After all data bytes indicated to UploadInit have been provided
 *          this command can be issued to actually start the transmission stream
 *
 * \param  [in]     payload*                - data fragment
 * \param  [in]     payload_length          - data fragment size
 * \retval  modem_return_code_t
 */
modem_return_code_t modem_upload_start( uint8_t* payload, uint16_t payload_length );

/*!
 * \brief  return the average delay set during the last file init
 * \remark
 *
 * \retval  uint16_t
 */
uint16_t modem_upload_avgdelay_get( void );

/*!
 * \brief  return the pointer on radio_planner
 * \remark
 *
 * \retval  radio_planner_t *
 */
radio_planner_t* modem_get_radio_planner( void );

#ifdef __cplusplus
}
#endif

#endif  // __MODEM_API_H__

/* --- EOF ------------------------------------------------------------------ */
