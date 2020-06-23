/*!
 * \file      modem_context.h
 *
 * \brief     share functions + context of the soft modem .
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

#ifndef __SOFT_MODEM_CONTEXT_H__
#define __SOFT_MODEM_CONTEXT_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_bsp.h"
#include "device_management_defs.h"
#include "modem_supervisor.h"
#include "lr1mac_defs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define DEFAULT_DM_PORT 199
#define DEFAULT_DM_REPORTING_INTERVAL 0x81  // 1h
#define DEFAULT_DM_REPORTING_FIELDS 0x7B    // status, charge, temp, signal, uptime, rxtime
#define DEFAULT_DM_MUTE_DAY 0
#define UPLOAD_SID 0

#define DM_STATUS_NOW_MIN_TIME 2
#define DM_STATUS_NOW_MAX_TIME 5

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief  Init events data
 * \retval void
 */
void modem_event_init( void );

/*!
 * \brief  for each type of asynchronous message have to manage a counter in case of overrun
 * \param   [in]  event_type                - type of asynchronous message
 * \retval int8_t                    - Return the number of asynchronous event for this type
 *                                            of event since the last get event cmd
 */
uint8_t get_modem_event_count( modem_rsp_event_t event_type );

/*!
 * \brief set a value in the counter of a type of asynchronous event
 * \param   [in]  event_type                - type of asynchronous message
 * \param   [in]  value                     - value's type of asynchronous message
 * \retval void
 */
void set_modem_event_count( modem_rsp_event_t event_type, uint8_t value );

/*!
 * \brief increment the counter of a type of asynchronous event
 * \param   [in]  event_type                - type of asynchronous message
 * \retval void
 */
void increment_modem_event_count( modem_rsp_event_t event_type );

/*!
 * \brief  for each type of asynchronous message have to manage a status
 * \remark
 * \param   [in]  event_type                - type of asynchronous message
 * \retval int8_t                    - return the last status of asynchronous event for this type
 *                                            of event since the last get event cmd
 */
uint8_t get_modem_event_status( modem_rsp_event_t event_type );

/*!
 * \brief set a value in the status of a type of asynchronous event
 * \remark
 * \param   [in]  event_type                    - type of asynchronous message
 * \param   [in]  status                        - status of asynchronous message
 * \retval void
 */
void set_modem_event_status( modem_rsp_event_t event_type, uint8_t status );

/*!
 * \brief decrement the asynchronous message number
 * \retval void
 */
void decrement_asynchronous_msgnumber( void );

/*!
 * \brief increment the asynchronous message number
 * \param   [in] event_type                     - type of asynchronous message
 * \param   [in] status                         - status of asynchronous message
 * \retval void
 */
void increment_asynchronous_msgnumber( modem_rsp_event_t event_type, uint8_t status );

/*!
 * \brief get the last message event
 * \retval modem_rsp_event_t             - Return the last modem_rsp_event_t
 */
modem_rsp_event_t get_last_msg_event( void );

/*!
 * \brief get asynchronous message number
 * \retval uint8_t                       - Return the number of asynchronous message
 */
uint8_t get_asynchronous_msgnumber( void );

/*!
 * \brief set modem dm interval
 * \remark  This command sets the device management reporting interval.
 *          The periodic status reporting interval field is encoded in one
 *          byte where the two top-most bits specify the unit :
 *          (00 - sec, 01 - day, 10 - hour, 11 - min),
 *          and the lower six bits the value 0-63.
 *          A value of zero disables the periodic status reporting
 *
 * \param   [in]  interval                      - Set DM interval
 * \retval e_set_error_t                        - Return e_set_error_t
 */
e_set_error_t set_modem_dm_interval( uint8_t interval );

/*!
 * \brief get modem dm interval
 * \remark  This command returns the device management reporting interval.
 *          The periodic status reporting interval field is encoded in one
 *          byte where the two top-most bits specify the unit :
 *          (00 - sec, 01 - day, 10 - hour, 11 - min),
 *          and the lower six bits the value 0-63.
 *          A value of zero disables the periodic status reporting
 *
 * \retval uint8_t                       - Return the DM interval
 */
uint8_t get_modem_dm_interval( void );

/*!
 * \brief get modem dm interval in second
 * \remark
 * \retval   uint32_t                    - Return the DM interval in seconds
 */
uint32_t get_modem_dm_interval_second( void );

/*!
 * \brief   Set the modem LoRaWAN Class
 * \remark  This command set the LoRaWAN device class.
 *
 * \param   [in]    LoRaWAN_class       - modem_class_t
 * \retval   e_set_error_t
 */
e_set_error_t set_modem_class( modem_class_t LoRaWAN_class );

/*!
 * \brief   Get the modem LoRaWAN Class
 * \remark  This command gets the LoRaWAN device class.
 *
 * \retval   modem_class_t
 */
modem_class_t get_modem_class( void );

/*!
 * \brief   Set modem DM port
 * \remark  This command sets the device management port.
 *
 * \param   [in]    port                        - DM port
 * \retval   e_set_error_t
 */
e_set_error_t set_modem_dm_port( uint8_t port );

/*!
 * \brief   Get DM port
 * \remark  This command gets the device management port.
 *
 * \retval [out]    return                      - DM port
 */
uint8_t get_modem_dm_port( void );

/*!
 * \brief   Set ADR profile
 * \remark  This command sets the ADR profile and parameters.
 *
 * \param  [in]     user_dr                     - dr_strategy_t
 * \param  [in]     adr_custom_data             - ADR custom profile data
 * \param  [in]     adr_custom_length           - ADR custom profile data length
 * \retval [out]    e_set_error_t
 */
e_set_error_t set_modem_adr_profile( dr_strategy_t user_dr, uint8_t* adr_custom_data, uint8_t adr_custom_length );

/*!
 * \brief   Get ADR mode
 * \remark  This command returns the ADR profile mode.
 *
 * \retval [out]    dr_strategy_t               - Return adr profile
 */
dr_strategy_t get_modem_adr( void );

/*!
 * \brief   merge the join status of the stack and the "join on going" state of the modem
 * \remark
 * \retval   eModemJoinState_t           - return join state
 */
eModemJoinState_t get_join_state( void );

/*!
 * \brief   Set application-specific status in DM
 * \remark  This commands sets application-specific status information to be reported to the DM service.
 *
 * \param   [out]   app_status*                 - App status payload
 \retval void
 */
void set_modem_appstatus( uint8_t* app_status );

/*!
 * \brief   Get application-specific status in DM
 * \remark  This commands gets application-specific status information
 *
 * \param   [out]   app_status*                 - App status payload
 * \retval   void
 */
void get_modem_appstatus( uint8_t* app_status );

/*!
 * \brief   Reset the modem charge
 * \remark  This command resets the accumulated charge counter to zero.
 *
 * \retval void
 */
void reset_modem_charge( void );

/*!
 * \brief   Get the modem charge mAs
 * \remark  This command returns the total charge counter of the modem in mAs.
 *
 * \retval   uint32_t                    - Return accumulated charge
 */
uint32_t get_modem_charge_ma_s( void );

/*!
 * \brief   Get the modem charge mAh
 * \remark  This command returns the total charge counter of the modem in mAh.
 *
 * \retval   uint32_t                    - Return accumulated charge
 */
uint32_t get_modem_charge_ma_h( void );

/*!
 * \brief   Get the modem voltage
 * \remark  This command returns the modem voltage
 *
 * \retval   uint8_t                    - Return modem voltage
 */
uint8_t get_modem_voltage( void );

/*!
 * \brief   Get the modem temperature
 * \remark  This command returns the modem temperature
 *
 * \retval  uint8_t                     - Return modem temperature
 */
uint8_t get_modem_temp( void );

/*!
 * \brief   return the modem status
 * \remark
 * \retval  uint8_t      bit 5 : file upload in progress
 *                       bit 4 : radio suspend
 *                       bit 3 : modem join
 *                       bit 2 : modem mute
 *                       bit 1 : reset after panic
 *                       bit 0 : reset after brownout
 * */
uint8_t get_modem_status( void );

/*!
 * \brief   Set the region
 * \remark  This command sets the regulatory region
 *
 * \param   [in]    region                      - region
 * \retval   return                      - e_set_error_t
 */
e_set_error_t set_modem_region( uint8_t region );

/*!
 * \brief   Get the region
 * \remark  This command returns the current regulatory region.
 *
 * \retval   return                     - Return the region
 */
uint8_t get_modem_region( void );

/*!
 * \brief   Get the modem status after a crash
 * \remark  From bitfield status
 *
 * \retval   return                      - Return crash in bit field status
 */
bool get_modem_status_reset_after_crash( void );

/*!
 * \brief   Get the modem status file upload
 * \remark  From bitfield status
 *
 * \retval   return                      - Return file upload in bit field status
 */
bool get_modem_status_file_upload( void );

/*!
 * \brief   Get the modem status joining
 * \remark  From bitfield status
 *
 * \retval   return                      - Return joining in bit field status
 */
bool get_modem_status_joining( void );

/*!
 * \brief   Get the modem status streaming
 * \remark  From bitfield status
 *
 * \retval  [out]   return                      - Return streaming in bit field status
 */
bool get_modem_status_streaming( void );

/*!
 * \brief   Set the modem status file upload
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set file upload in bit field status
 */
void set_modem_status_file_upload( bool value );

/*!
 * \brief   Set the modem status joining
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set joining in bit field status
 */
void set_modem_status_joining( bool value );

/*!
 * \brief   Set the modem status streaming
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set streaming in bit field status
 */
void set_modem_status_streaming( bool value );

/*!
 * \brief   Set the modem status radio suspend
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set radio suspend in bit field status
 */
void set_modem_status_radio_suspend( bool value );

/*!
 * \brief   Set the modem status join
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set join in bit field status
 */
void set_modem_status_modem_joined( bool value );

/*!
 * \brief   Set the modem status mute
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set mute in bit field status
 */
void set_modem_status_modem_mute( bool value );

/*!
 * \brief   Set the modem status reset after panic
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set reset after panic in bit field status
 */
void set_modem_status_reset_after_crash( bool value );

/*!
 * \brief   Set the modem status reset after brownout
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set reset after brownout in bit field status
 */
void set_modem_status_reset_after_brownout( bool value );

/*!
 * \brief   Set the Downlink frame in modem context
 * \remark  Retrieve the LoRaWAN downlink from stack to set in modem context
 *          Be careful: MUST BE called only on event
 *
 * \retval void
 */
void set_modem_downlink_frame( void );

/*!
 * \brief   Get the Downlink frame in modem context
 * \remark  This function must be called after set_modem_downlink_frame()
 *
 * \param   [in]    modem_dwn*                  - s_modem_dwn_t
 */
void get_modem_downlink_frame( s_modem_dwn_t* modem_dwn );

/*!
 * \brief   Set DM retrieve pending downlink frame
 * \remark  This function set the modem supervisor to create downlink opportunities
 *
 * \param   [in]    up_count                    - number of requested uplink
 * \param   [in]    up_delay                    - Delay in second between each uplink
 */
void set_dm_retrieve_pending_dl( uint8_t up_count, uint8_t up_delay );

/*!
 * \brief   Get DM retrieve pending downlink frame
 * \remark  This function get requested downlink opportunities configuration
 *
 * \param   [in]    pending_dl                  - s_dm_retrieve_pending_dl_t
 */
void get_dm_retrieve_pending_dl( s_dm_retrieve_pending_dl_t* pending_dl );

/*!
 * \brief   check DM Info cmd Size
 *
 * \param   [in]  cmd                           - Current dm info code that must be checked
 * \param   [in]  length                        - Length of the tested requested dm info code
 * \retval e_dm_cmd_length_valid         - Return valid length or not
 */
e_dm_cmd_length_valid dm_check_dminfo_size( e_dm_info_t cmd, uint8_t length );

/*!
 * \brief   DM SetConf
 * \param   [in]  tag                           - e_dm_info_t that will be handle
 * \param   [in]  data *                        - Current dm info code that must be checked
 * \param   [in]  length                        - Length of the tested requested dm info code+data
 * \retval e_dm_error_t                         - Return valid or not
 */
e_dm_error_t dm_set_conf( e_dm_info_t tag, uint8_t* data, uint8_t length );

/*!
 * \brief   Check the modem mute state
 *
 * \retval e_modem_mute_t                - Return Modem Muted state
 */
e_modem_mute_t get_modem_muted( void );

/*!
 * \brief   Get the number of muted days when the modem will send a status message anyway
 * \remark  0x00: applicative message allowed
 *          0x01: to 0xFE: number of muted day(s) with status message (applicative message not allowed)
 *          0xFF: never send status message or applicative message allowed
 *
 * \retval uint8_t                       - Return a number of day(s)
 */
uint8_t dm_get_number_of_days_mute( void );

/*!
 * \brief   set the number of muted days when the modem will send a status message anyway
 * \remark  0x00: applicative message allowed
 *          0x01: to 0xFE: number of muted day(s) with status message (applicative message not allowed)
 *          0xFF: never send status message or applicative message allowed
 *
 * \param   [in]  days                          - Set a number of day(s)
 * \retval void
 */
void dm_set_number_of_days_mute( uint8_t days );

/*!
 * \brief   get the DM fields included in the periodic DM status messages
 *
 * \param   [in]  dm *                          - Returned array that contains fields included in the periodic
 *                                                DM status messages.
 * \param   [in]  flag
 *                                              - e_dm_info_rate_t : If DM_INFO_NOW: set bitfield given by the user
 *                                                                  with GetInfo command,
 *                                                               Else: set bitfield for saved context with SetDmInfo
 * \retval len                           - Return the len of the dm payload
 */
uint8_t get_dm_info_tag_list( uint8_t* dm, e_dm_info_rate_t flag );

/*!
 * \brief   This command sets the default info fields to be included in the periodic DM status messages.
 *          The set is specified as list of field codes as defined in Uplink Message Format.
 *          An empty set is valid and will effectively disable the DM status message.
 *
 * \param   [in]  requested_info_list         - Array of bytes with requested DM code in each bytes
 * \param   [in]  len                         - Number of byte that composed requested_info_list
 * \param   [in]  flag                        - e_dm_info_rate_t: If DM_INFO_NOW: set bitfield given by the user with
 *                                                                  GetInfo command,
 *                                                              Else: set bitfield for saved context with SetDmInfo
 * \retval e_set_error_t               - Return SET_ERROR in case of failure, else false SET_OK
 */
e_set_error_t set_dm_info( uint8_t* requested_info_list, uint8_t len, e_dm_info_rate_t flag );

/*!
 * \brief   DM status messages
 *
 * \param   [out] dm_uplink_message *         - Returned array that contains one or more concatenated device
 *                                              information fields.
 * \param   [out] dm_uplink_message_len *     - Returned array length
 * \param   [in]  max_size                    - max payload size that must be returned
 * \param   [in]  flag                        - e_dm_info_rate_t: If DM_INFO_NOW: set bitfield given by the user with
 *                                                                  GetInfo command,
 *                                                              Else: set bitfield for saved context with SetDmInfo
 * \retval bool                               - Return true if there are pending message(s) else false
 */
bool dm_status_payload( uint8_t* dm_uplink_message, uint8_t* dm_uplink_message_len, uint8_t max_size,
                        e_dm_info_rate_t flag );

/*!
 * \brief   DM ALC Sync uplink payload
 *
 * \param   [in]  alc_sync_time             - Indicate the time when the payload will be really send
 * \param   [in]  app_time_ans_required     - True/False Set AnsRequired in AppTimeReq if AppTimeReq is present in
 *                                            uplink
 * \param   [in]  force_resync_status       - If true the NbTransmission set by ForceDeviceResyncReq must be
 * \param   [in]  max_payload_length        - final max length of the constructed payload
 * \param   [out] dm_uplink_message *       - Returned array that contains one or more concatenated ALC Sync data
 * \param   [out] dm_uplink_message_len *   - Returned array length
 * checked \retval void
 */
void dm_alc_sync_uplink_payload( uint32_t alc_sync_time, uint8_t app_time_ans_required, uint8_t force_resync_status,
                                 uint8_t max_payload_length, uint8_t* dm_uplink_message,
                                 uint8_t* dm_uplink_message_len );
/*!
 * \brief    add a join task in scheduler
 * \remark
 */
void modem_supervisor_add_task_join( void );

/*!
 * \brief    add a DM Status task in scheduler
 * \remark
 */
void modem_supervisor_add_task_dm_status( uint32_t next_execute );

/*!
 * \brief    add a DM Status Now task in scheduler
 * \remark
 */
void modem_supervisor_add_task_dm_status_now( void );

/*!
 * \brief    add a ALC Sync time request task in scheduler
 * \remark
 */
void modem_supervisor_add_task_alc_sync_time_req( uint32_t next_execute );

/*!
 * \brief    add a ALC Sync answer task in scheduler
 * \remark
 */
void modem_supervisor_add_task_alc_sync_ans( uint32_t next_execute );

/*!
 * \brief    add a DM Mute Task to decrement the number of muted day(s)
 * \remark
 */
void modem_supervisor_add_task_modem_mute( void );

/*!
 * \brief    add a task to retrieve all pending downlink
 * \remark
 */
void modem_supervisor_add_task_retrieve_dl( uint32_t next_execute );

/*!
 * \brief    add a stream task in scheduler
 * \remark
 */
void modem_supervisor_add_task_stream( void );

/*!
 * \brief    Set modem Suspend
 * \remark
 * \param   [in]  suspend               - True: Suspend modem, False: un-suspend modem
 * \retval e_set_error_t                   - Return SET_ERROR in case of failure, else false SET_OK
 */
e_set_error_t set_modem_suspend( e_modem_suspend_t suspend );

/*!
 * \brief    Get modem Suspend status
 * \remark
 * \retval e_modem_suspend_t               - Return suspend type
 */
e_modem_suspend_t get_modem_suspend( void );

/*!
 * \brief    Get uptime since last reset in seconds
 * \remark
 * \retval uint32_t                          - Return modem uptime since last reset
 */
uint32_t get_modem_uptime_s( void );

/*!
 * \brief    Set modem start time to compute uptime
 * \remark
 * \param   [in]  time                          - Modem start time
 * \retval  void
 */
void set_modem_start_time_s( uint32_t time );

/*!
 * \brief       Save modem context in non volatile memory
 * \remark
 * \retval   void
 */
void modem_store_context( void );

/*!
 * \brief    load modem context in non volatile memory
 * \remark
 * \retval   void
 */
void modem_load_context( void );

/*!
 * \brief    store the context factory of the modem in the non volatile memory
 * \retval   void
 */
void modem_context_factory_reset( void );

/*!
 * \brief    set the upload session counter variable
 * \param   [in]  session_counter         - session counter
 * \retval   void
 */
void modem_set_dm_upload_sctr( uint8_t session_counter );

/*!
 * \brief    get the upload state
 * \retval e_modem_upload_state_t
 */
e_modem_upload_state_t modem_get_upload_state( void );

/*!
 * \brief    set the upload state
 * \param   [in]  upload_state
 * \retval void
 */
void modem_set_upload_state( e_modem_upload_state_t upload_state );

/*!
 * \brief    get the stream state
 * \param   [in]  void
 * \retval  [out] e_modem_stream_state_t
 */
e_modem_stream_state_t modem_get_stream_state( void );

/*!
 * \brief    get the stream port
 * \param   [in]  void
 * \retval  [out] port
 */
uint8_t modem_get_stream_port( void );

/*!
 * \brief    get the stream encryption
 * \param   [in]  void
 * \retval  [out] encryption
 */
bool modem_get_stream_encryption( void );

/*!
 * \brief    set the stream state
 * \param   [in]  e_modem_stream_state_t
 * \param   [out] void
 */
void modem_set_stream_state( e_modem_stream_state_t stream_state );

/*!
 * \brief    set the stream port
 * \param   [in]  port
 * \param   [out] void
 */
void modem_set_stream_port( uint8_t port );

/*!
 * \brief    set the stream encryption
 * \param   [in]  encryption
 * \param   [out] void
 */
void modem_set_stream_encryption( bool enc );

/*!
 * \brief    get the upload session counter variable
 * \retval return session_counter value
 */
uint8_t modem_get_dm_upload_sctr( void );

/*!
 * \brief    set info_bitfield_periodic
 * \param   [in]  value
 * \retval void
 */
void modem_set_dm_info_bitfield_periodic( uint32_t value );

/*!
 * \brief    get info_bitfield_periodic
 * \remark return bit field for periodic DM status
 * \retval uint32_t
 */
uint32_t modem_get_dm_info_bitfield_periodic( void );
/*!
 * \brief    get info_bitfield_periodic
 * \remark   return alarm value in seconds
 * \retval   uint32_t
 */
uint32_t modem_get_user_alarm( void );
/*!
 * \brief   set_user_alarm
 * \param   [in]  alarm value in seconds
 * \retval void
 */
void modem_set_user_alarm( uint32_t alarm );

/*!
 * \brief   Get modem is requested a reset
 * \retval bool          - true if reset is requested
 */
bool get_modem_reset_requested( void );

/*!
 * \brief   Set modem is requested a reset
 * \param   [in]  reset_req     - true if reset is requested
 * \retval  void
 */
void set_modem_reset_requested( bool reset_req );

#ifdef __cplusplus
}
#endif

#endif  // __SOFT_MODEM_CONTEXT_H__

/* --- EOF ------------------------------------------------------------------ */
