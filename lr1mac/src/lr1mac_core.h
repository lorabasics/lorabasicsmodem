/*!
 * \file      lr1_mac_core.h
 *
 * \brief     LoRaWan core definition
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

#ifndef __LR1MAC_CORE_H__
#define __LR1MAC_CORE_H__
/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include "stdint.h"
#include "lr1mac_defs.h"
#include "lr1_stack_mac_layer.h"

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------------
 */

/*!
 * \class LoraWanObject
 * \brief An absolutely minimalistic LoRaWAN Class A stack implementation .
 * \remark  In future implementation the constructor will contain :
 *            APPSKey, NWSKey, DevAdrr mandatory for ABP devices
 *            DevEui, AppEUI, APPKey mandatory for OTAA devices
 *            OTAA or ABP Flag.
 *         In future implementation A Radio objet will be also a parameter of this class.
 */

void lr1mac_core_init( radio_planner_t* rp, lorawan_keys_t* lorawan_keys, smtc_real_t* smtc_region );

/*!
     * \brief Sends an uplink
     * \param [IN] uint8_t           fPort          Uplink Fport
     * \param [IN] const uint8_t*    dataInFport    User Payload
     * \param [IN] const uint8_t     sizeIn         User Payload Size
     * \param [IN] const uint8_t     PacketType     User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,

     * \param [OUT] lr1mac_states_t         Current state of the LoraWan stack :
     * \param                                            => return LWPSATE_SEND if all is ok
     * \param                                            => return Error in case of payload too long
     * \param                                            => return Error In case of the Lorawan stack previous state is
   not equal to idle
     */

/*!
     * \brief  Receive Applicative Downlink
     * \param [IN] uint8_t*          UserRxFport          Downlinklink Fport
     * \param [IN] uint8_t*          UserRxPayload        Applicative Downlink Payload
     * \param [IN] uint8_t*          UserRxPayloadSize    Applicative Downlink Payload Size
     * \param [IN] const uint8_t     PacketType     User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,

     * \param [OUT] status_lorawan_t   Return an error if No Packet available.
     */
status_lorawan_t lr1mac_core_payload_receive( uint8_t* UserRxFport, uint8_t* UserRxPayload,
                                              uint8_t* UserRxPayloadSize );

/*!
 * \brief to Send a Join request
 * \param [] None
 * \param [OUT] lr1mac_states_t         Current state of the LoraWan stack :
 *                                                 => return LWPSATE_SEND if all is ok
 *                                                 => return Error In case of the Lorawan stack previous state is not
 * equal to idle
 */
lr1mac_states_t lr1mac_core_join( uint32_t target_time_ms );

/*!
 * \brief Returns the join state
 * \param [] None
 * \param [OUT] Returns the join state         NOT_JOINED: the device is joined to a network
 *                                             JOINED: the device is not connected
 *                                             Always returns JOINED for ABP devices
 */
join_status_t lr1mac_core_is_joined( void );

/*!
 * \brief Reset the join status to NotJoined
 * \param [] None
 * \param [OUT] None
 */
void lr1mac_core_join_status_clear( void );

/*!
 * \brief SetDataRateStrategy of the devices
 * \remark Refereed to the dedicated chapter in Wiki page for detailed explanation about
 *         implemented data rate choice (distribution data rate).
 * \remark The current implementation support 4 different dataRate Strategy :
 *            STATIC_ADR_MODE                  for static Devices with ADR managed by the Network
 *            MOBILE_LONGRANGE_DR_DISTRIBUTION for Mobile Devices with strong Long range requirement
 *            MOBILE_LOWPER_DR_DISTRIBUTION    for Mobile Devices with strong Low power requirement
 *            JOIN_DR_DISTRIBUTION             Dedicated for Join requests
 *
 * \param [IN]  dr_strategy_t              DataRate Mode (describe above)
 * \param [OUT] None
 */
void          lr1mac_core_dr_strategy_set( dr_strategy_t adrModeSelect );
dr_strategy_t lr1mac_core_dr_strategy_get( void );
void          lr1mac_core_dr_custom_set( uint32_t DataRateCustom );

/*!
 * \brief   Runs the MAC layer state machine.
 *          Must be called periodically by the application. Not timing critical. Can be interrupted.
 * \remark  Not timing critical. Can be interrupted.
 *
 * \param [IN]  AvailableRxPacket *            Return if an applicative packet is available
 * \param [OUT] lr1mac_states_t        return the lorawan state machine state
 */
lr1mac_states_t lr1mac_core_process( user_rx_packet_type_t* AvailableRxPacket );

/*!
 * \brief   Return the state of the Radio
 * \param [IN]  none
 * \param [OUT] return the state of the radio (Not yet finalized will be replace by an enum)
 */
uint8_t lr1mac_core_radio_state_get( void );

/*!
 * \brief   Reload the LoraWAN context saved in the flash
 * \param [IN]  none
 * \param [OUT] none
 */
status_lorawan_t lr1mac_core_context_load( void );
/*!
 * \brief   Save The LoraWAN context  in the flash
 * \param [IN]  none
 * \param [OUT] none
 */
void lr1mac_core_context_save( void );
/*!
 * \brief   Get the snr of the last user receive packet
 * \param [IN]  none
 * \param [OUT] Int 16 last snr
 */
int16_t lr1mac_core_last_snr_get( void );
/*!
 * \brief   Get the snr of the last user receive packet
 * \param [IN]  none
 * \param [OUT] Int 16 last snr
 */
int16_t lr1mac_core_last_rssi_get( void );
/*!
 * \brief   Store provisioning informations in flash
 * \param [IN]  none
 * \param [OUT] none
 */
void lr1mac_core_keys_set( lorawan_keys_t LoRaWanKeys );

/*!
 * \brief   Reload the factory Config in the LoraWAN Stack
 * \param [IN]  none
 * \param [OUT] none
 */
void lr1mac_core_factory_reset( void );  // load factory MAC context from constructor

/*!
 * \brief Set Device type : OTAA or ABP
 * \param [IN]  none
 * \param [OUT] enum Ota Device Type     OTAA_DEVICE or ABP_DEVICE useful for certification,
 */
void lr1mac_core_otaa_set( type_otaa_abp_t deviceType );

/*!
 * \brief   Return the Max payload length allowed for the next transmit
 * \remark  DataRate + FOPTS + region  dependant ( )
 * \remark  In any case if user set a too long payload, the send method will answer by an error status
 * \param [IN]  none
 * \param [OUT] Return max payload length for next Transmission
 */
uint32_t lr1mac_core_next_max_payload_length_get( void );

/*!
 * \brief   Call this function to set the loraWan join variable in NOT_JOINED state
 * \param [IN]  none
 * \param [OUT] none
 */
void lr1mac_core_new_join( void );

/*!
 * \brief   Return the DevAddr of the device
 * \param [IN]  none
 * \param [OUT] return DevAddr
 */
uint32_t lr1mac_core_devaddr_get( void );

/*!
 * \brief   Return the DevEUI of the device
 * \param [IN]  DevEUI *       Return the device EUI
 * \param [OUT] none
 */
void lr1mac_core_deveui_get( uint8_t* DevEui );

/*!
 * \brief   Set the DevEUI of the device
 * \param [IN]  DevEUI *       Device EUI
 * \param [OUT] none
 */
void lr1mac_core_deveui_set( uint8_t* DevEui );

/*!
 * \brief   Set the AppKey of the device
 * \param [IN]  AppKey *
 * \param [OUT] none
 */
void lr1mac_core_app_key_set( uint8_t* AppKey );

/*!
 * \brief   Get the AppEui of the device
 * \param [IN]  AppEui *
 * \param [OUT] none
 */
void lr1mac_core_appeui_key_get( uint8_t* AppEui );

/*!
 * \brief   Set the AppEui of the device
 * \param [IN]  AppEui *
 * \param [OUT] none
 */
void lr1mac_core_appeui_key_set( uint8_t* AppEui );

/*!
 * \brief   Return the next transmission power
 * \remark
 * \param [IN]  none
 * \param [OUT] return the next transmission power
 */
uint8_t lr1mac_core_next_power_get( void );

/*!
 * \brief   Return the returns the next data rate
 * \remark
 * \param [IN]  none
 * \param [OUT] return the next transmission power
 */
uint8_t lr1mac_core_next_dr_get( void );

/*!
 * \brief   Return the returns the next Tx Frequency
 * \remark
 * \param [IN]  none
 * \param [OUT] return the next transmission power
 */

uint32_t lr1mac_core_next_frequency_get( void );

/*!
 * \brief   Return the returns the max data rate of all enabled channels
 * \remark
 * \param [IN]  none
 * \param [OUT] return the max data rate
 */
uint8_t lr1mac_core_max_dr_get( void );

/*!
 * \brief   Return the returns the min data rate of all enabled channels
 * \remark
 * \param [IN]  none
 * \param [OUT] return the min data rate
 */

uint8_t lr1mac_core_min_dr_get( void );

/*!
 * \brief   returns the current state of the MAC layer.
 * \remark  If the MAC is not in the idle state, the user cannot call any methods except the LoraWanProcess() method and
 * the GetLorawanProcessState() method \param [IN]  none \param [OUT] return the next transmission power
 */

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
lr1mac_states_t lr1mac_core_state_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
type_otaa_abp_t lr1mac_core_is_otaa_device( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint16_t lr1mac_core_nb_reset_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint16_t lr1mac_core_devnonce_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1mac_core_apps_key_get( uint8_t* key );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
join_status_t lr1_mac_joined_status_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
lr1mac_states_t lr1mac_core_payload_send( uint8_t fPort, const uint8_t* dataIn, const uint8_t sizeIn,
                                          uint8_t PacketType, uint32_t target_time_ms );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
lr1mac_states_t lr1mac_core_payload_send_at_time( uint8_t fport, const uint8_t* data_in, const uint8_t size_in,
                                                  uint8_t packet_type, uint32_t target_time_ms );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
radio_planner_t* lr1mac_core_rp_get( void );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
int8_t lr1mac_core_tx_power_offset_get( void );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1mac_core_tx_power_offset_set( int8_t power_off );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
receive_win_t lr1mac_core_rx_window_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t lr1mac_core_fcnt_up_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t lr1mac_core_next_join_time_second_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
int32_t lr1mac_core_next_free_duty_cycle_ms_get( void );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1mac_core_duty_cycle_enable_set( uint8_t enable );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t lr1mac_core_version_get( void );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t lr1mac_core_rx_ack_bit_get( void );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
lr1_stack_mac_t* lr1mac_core_stack_mac_get( void );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t lr1mac_core_is_supported_region( smtc_real_region_types_t region_type );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
smtc_real_region_types_t lr1mac_core_get_region( void );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t lr1mac_core_set_region( smtc_real_region_types_t region_type );
#endif
