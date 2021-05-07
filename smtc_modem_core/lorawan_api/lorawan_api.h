/*!
 * \file      lorawan_api.h
 *
 * \brief     Lorawan abstraction layer definitions.
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

#ifndef __LORAWAN_API_H__
#define __LORAWAN_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "lr1mac_defs.h"
#include "lr1_stack_mac_layer.h"
#include "smtc_real_defs.h"
#include "radio_planner.h"
#include "fifo_ctrl.h"

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
 * \brief Init the LoRaWAN stack
 */
void lorawan_api_init( radio_planner_t* rp );

/*!
 * \brief
 */
void lorawan_rp_callback_api( radio_planner_t* rp );

/*!
 * \brief Set the LoRaWAN regional parameters
 * \param [out] smtc_real_region_types_t* Region
 */
smtc_real_region_types_t lorawan_api_get_region( void );

/*!
 * \brief Set the LoRaWAN regional parameters
 * \param [in] smtc_real_region_types_t Region
 */
status_lorawan_t lorawan_api_set_region( smtc_real_region_types_t region_type );

/*!
 * \brief Sends an uplink when it's possible
 * \param [in] uint8_t           fPort          Uplink Fport
 * \param [in] const uint8_t*    dataInFport    User Payload
 * \param [in] const uint8_t     sizeIn         User Payload Size
 * \param [in] const uint8_t     PacketType     User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,
 * \param [in] uint32_t          TargetTimeMs   RTC time when the packet must be sent
 * \param [out] lr1mac_states_t         Current state of the LoraWan stack :
 * \param                                            => return LWPSATE_SEND if all is ok
 * \param                                            => return Error in case of payload too long
 * \param                                            => return Error In case of the Lorawan stack previous state is not
 *                                                      equal to idle
 */
lr1mac_states_t lorawan_api_payload_send( uint8_t fPort, const uint8_t* dataIn, const uint8_t sizeIn,
                                          uint8_t PacketType, uint32_t TargetTimeMs );

/*!
 * \brief Sends an uplink at time
 * \param [in] uint8_t           fPort          Uplink Fport
 * \param [in] const uint8_t*    dataInFport    User Payload
 * \param [in] const uint8_t     sizeIn         User Payload Size
 * \param [in] const uint8_t     PacketType     User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,
 * \param [in] uint32_t          TargetTimeMs   RTC time when the packet must be sent
 * \param [out] lr1mac_states_t         Current state of the LoraWan stack :
 * \param                                            => return LWPSATE_SEND if all is ok
 * \param                                            => return Error in case of payload too long
 * \param                                            => return Error In case of the Lorawan stack previous state is not
 *                                                      equal to idle
 */
lr1mac_states_t lorawan_api_payload_send_at_time( uint8_t fPort, const uint8_t* dataIn, const uint8_t sizeIn,
                                                  uint8_t PacketType, uint32_t TargetTimeMs );
/*!
 * \brief  Receive Applicative Downlink
 * \param [in] uint8_t*          UserRxFport            Downlinklink Fport
 * \param [in] uint8_t*          UserRxPayload          Applicative Downlink Payload
 * \param [in] uint8_t*          UserRxPayloadSize      Applicative Downlink Payload Size
 * \param [in] const uint8_t     PacketType             User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,

 * \param [out] eStatusLoRaWan   Return an error if No Packet available.
 */
status_lorawan_t lorawan_api_payload_receive( uint8_t* UserRxFport, uint8_t* UserRxPayload,
                                              uint8_t* UserRxPayloadSize );

/*!
 * \brief to Send a Join request
 * \param [] None
 * \param [out] lr1mac_states_t         Current state of the LoraWan stack :
 *                                                 => return LWPSATE_SEND if all is ok
 *                                                 => return Error In case of the Lorawan stack previous state is not
 *                                                      equal to idle
 */
lr1mac_states_t lorawan_api_join( uint32_t target_time_ms );

/*!
 * \brief Returns the join state
 * \param [] None
 * \param [out] Returns the join state         NOT_JOINED: the device is joined to a network
 *                                             JOINED: the device is not connected
 *                                             Always returns JOINED for ABP devices
 */
join_status_t lorawan_api_isjoined( void );

/*!
 * \brief Rreset the join status to NotJoined
 * \param [] None
 * \param [out] None
 */
void lorawan_api_join_status_clear( void );

/*!
 * \brief SetDataRateStrategy of the devices
 * \remark Refered to the dedicated chapter in Wiki page for detailed explanation about
 *         implemented data rate choice (distribution data rate).
 * \remark The current implementation support 4 different dataRate Strategy :
 *            STATIC_ADR_MODE                   for static Devices with ADR managed by the Network
 *            MOBILE_LONGRANGE_DR_DISTRIBUTION  for Mobile Devices with strong Long range requirement
 *            MOBILE_LOWPER_DR_DISTRIBUTION     for Mobile Devices with strong Low power requirement
 *            JOIN_DR_DISTRIBUTION              Dedicated for Join requests
 *
 * \param [in]  dr_strategy_t                   DataRate Mode (describe above)
 * \param [out] None
 */
status_lorawan_t lorawan_api_dr_strategy_set( dr_strategy_t adrModeSelect );
dr_strategy_t    lorawan_api_dr_strategy_get( void );
void             lorawan_api_dr_custom_set( uint32_t DataRateCustom );

/*!
 * \brief   Runs the MAC layer state machine.
 *          Must be called periodically by the application. Not timing critical. Can be interrupted.
 * \remark  Not timing critical. Can be interrupted.
 *
 * \param [in]  AvailableRxPacket *             Return if an applicative packet is available
 * \param [out] lr1mac_states_t                 return the lorawan state machine state
 */
lr1mac_states_t lorawan_api_process( user_rx_packet_type_t* AvailableRxPacket );

/*!
 * \brief   Return the state of the Radio
 * \param [in]  none
 * \param [out] return the state of the radio (Not yet finalized will be replace by an enum)
 */
uint8_t lorawan_api_GetRadioState( void );

/*!
 * \brief   Reload the LoraWAN context saved in the flash
 * \param [in]  none
 * \param [out] none
 */
void lorawan_api_context_load( void );
/*!
 * \brief   Save The LoraWAN context in the flash
 * \param [in]  none
 * \param [out] none
 */
void lorawan_api_context_save( void );
/*!
 * \brief   Get the snr of the last user receive packet
 * \param [in]  none
 * \param [out] Int 16 last snr
 */
int16_t lorawan_api_last_snr_get( void );
/*!
 * \brief   Get the snr of the last user receive packet
 * \param [in]  none
 * \param [out] Int 16 last snr
 */
int16_t lorawan_api_last_rssi_get( void );
/*!
 * \brief   Store provisioning informations in flash
 * \param [in]  none
 * \param [out] none
 */
void lorawan_api_keys_set( lorawan_keys_t LoRaWanKeys );

/*!
 * \brief   Reload the factory Config in the LoraWAN Stack
 * \param [in]  none
 * \param [out] none
 */
void lorawan_api_factory_reset( void );

/*!
 * \brief   Get Device type : OTA or APB
 * \param [in]  none
 * \param [out] enum Ota Device Type     OTA_DEVICE or ABP_DEVICE,
 */
type_otaa_abp_t lorawan_api_is_ota_device( void );

/*!
 * \brief Set Device type : OTA or APB
 * \param [in]  none
 * \param [out] enum Ota Device Type     OTA_DEVICE or ABP_DEVICE useful for certification,
 */
void lorawan_api_ota_set( type_otaa_abp_t deviceType );

/*!
 * \brief   Return the Max payload length allowed for the next transmit
 * \remark  DataRate + FOPTS + region  dependant ( )
 * \remark  In any case if user set a too long payload, the send method will answer by an error status
 * \param [in]  none
 * \param [out] Return max payload length for next Transmission
 */
uint32_t lorawan_api_next_max_payload_length_get( void );

/*!
 * \brief   Call this function to set the loraWan join variable in NOT_JOINED state
 * \param [in]  none
 * \param [out] none
 */
void lorawan_api_new_join( void );

/*!
 * \brief   Return the DevAddr of the device
 * \param [in]  none
 * \param [out] return DevAddr
 */
uint32_t lorawan_api_devaddr_get( void );

/*!
 * \brief   Return the DevEUI of the device
 * \param [in]  DevEUI *       Return the device EUI
 * \param [out] none
 */
void lorawan_api_deveui_get( uint8_t* DevEui );

/*!
 * \brief   Set the DevEUI of the device
 * \param [in]  DevEUI *       Device EUI
 * \param [out] none
 */
void lorawan_api_deveui_set( const uint8_t* DevEui );

/*!
 * \brief   Set the AppKey of the device
 * \param [in]  AppKey *
 * \param [out] none
 */
void lorawan_api_app_key_set( const uint8_t* AppKey );

/*!
 * \brief   Get the AppEui of the device
 * \param [in]  AppEui *
 * \param [out] none
 */
void lorawan_api_appeui_key_get( uint8_t* AppEui );

/*!
 * \brief   Set the AppEui of the device
 * \param [in]  AppEui *
 * \param [out] none
 */
void lorawan_api_appeui_key_set( const uint8_t* AppEui );

/*!
 * \brief   Return the next transmission power
 * \remark
 * \param [in]  none
 * \param [out] return the next transmission power
 */
uint8_t lorawan_api_next_power_get( void );

/*!
 * \brief   Return the returns the next data rate
 * \remark
 * \param [in]  none
 * \param [out] return the next transmission power
 */
uint8_t lorawan_api_next_dr_get( void );

/*!
 * \brief   Return the returns the next Tx Frequency
 * \remark
 * \param [in]  none
 * \param [out] return the next transmission power
 */

uint32_t lorawan_api_next_frequency_get( void );

/*!
 * \brief   Return the returns the max data rate of all enabled channels
 * \remark
 * \param [in]  none
 * \param [out] return the max data rate
 */
uint8_t lorawan_api_max_tx_dr_get( void );
/*!
 * \brief   Return the returns the min data rate of all enabled channels
 * \remark
 * \param [in]  none
 * \param [out] return the min data rate
 */
uint8_t lorawan_api_min_tx_dr_get( void );

/*!
 * \brief   Return the returns the current data rate mask of all enabled channels
 * \remark
 * \param [IN]  none
 * \param [OUT] return the mask data rate
 */
uint16_t lorawan_api_mask_tx_dr_channel_up_dwell_time_check( void );

/*!
 * \brief   returns the current state of the MAC layer.
 * \remark  If the MAC is not in the idle state, the user cannot call any methods except the LoraWanProcess()
 *          method and the GetLorawanProcessState() method
 * \param [in]  none
 * \param [out] return the next transmission power
 */
lr1mac_states_t lorawan_api_state_get( void );
/*!
 * \brief   returns the current AppsKey.
 * \remark
 * \param [in]  none
 * \param [out] return the AppsKey
 */
uint8_t* lorawan_api_apps_key_get( void );
/*!
 * \brief   returns the number of reset
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint16_t lorawan_api_nb_reset_get( void );
/*!
 * \brief   returns the last devnonce
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint16_t lorawan_api_devnonce_get( void );
/*!
 * \brief   returns the Rx window used by the downlink
 * \remark
 * \param [in]  none
 * \param [out] return
 */
receive_win_t lorawan_api_rx_window_get( void );
/*!
 * \brief   returns the min time to perform a new join request
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint32_t lorawan_api_next_join_time_second_get( void );
/*!
 * \brief   when > 0, returns the min time to perform a new uplink request
 * \remark
 * \param [in]  none
 * \param [out] return
 */
int32_t lorawan_api_next_free_duty_cycle_ms_get( void );
/*!
 * \brief   Enable / disable the dutycycle
 * \remark
 * \param [in]  bool enable
 * \param [out] status_lorawan_t
 */
status_lorawan_t lorawan_api_duty_cycle_enable_set( bool enable );
/*!
 * \brief   return the last uplink frame counter
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint32_t lorawan_api_fcnt_up_get( void );
/*!
 * \brief   Get the LoRaWAN hook ID in radio planner
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint8_t lorawan_api_rp_hook_id_get( void );

/*!
 * \brief   Enable/disable class C
 * \remark
 * \param [in]  uint8_t
 * \param [out] none
 */
void lorawan_api_class_c_enabled( bool enable );
/*!
 * \brief   Start class C
 * \remark
 * \param [in]  uint8_t
 * \param [out] none
 */
void lorawan_api_class_c_start( void );
/*!
 * \brief   Stop class C
 * \remark
 * \param [in]  uint8_t
 * \param [out] none
 */
void lorawan_api_class_c_stop( void );
/*!
 * \brief   Get the downlink frame ACK bit state
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint8_t lorawan_api_rx_ack_bit_get( void );

/*!
 * \brief   Set the number uplink without downlink before reset stack
 * \remark
 * \param  [in]  uint16_t no_rx_packet_count
 * \retval [out] status_lorawan_t
 */
status_lorawan_t lorawan_api_no_rx_packet_count_config_set( uint16_t no_rx_packet_count );

/*!
 * \brief   Get the configured number of uplink without downlink before reset stack
 * \remark
 * \retval [out] uint16_t
 */
uint16_t lorawan_api_no_rx_packet_count_config_get( void );

/**
 * @brief  Get the current number of uplink without downlink before reset stack
 *
 * @return uint16_t
 */
uint16_t lorawan_api_no_rx_packet_count_current_get( void );

/*!
 * \brief   Get the number uplink without downlink in mobile mode
 * \remark
 * \retval [out] uint16_t
 */
uint16_t lorawan_api_no_rx_packet_count_in_mobile_mode_get( void );

/*!
 * \brief   Set the current counter of number uplink without downlink in mobile mode
 * \remark
 * \retval [in] uint32_t
 */
void lorawan_api_no_rx_packet_count_in_mobile_mode_set( uint16_t no_rx_packet_count );

/*!
 * \brief   Set the status of the Modem LoRaWAN certification
 * \remark  To authorized LoRaWAN certification in modem
 * \param [in]  uint8_t true/false
 * \param [out] return
 */
void lorawan_api_modem_certification_set( uint8_t enable );

/*!
 * \brief   Get the status of the Modem LoRaWAN certification
 * \remark  Is certification is authorized in modem
 * \param [in]  none
 * \param [out] return uint8_t
 */
uint8_t lorawan_api_modem_certification_is_enabled( void );

/*!
 * \brief   Get the status of the LoRaWAN certification
 * \remark  Is enabled by the Test Tool
 * \param [in]  none
 * \param [out] return uint8_t
 */
uint8_t lorawan_api_certification_is_enabled( void );
/*!
 * \brief   Api to run the LoRaWAN certification
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint8_t lorawan_api_certification( uint8_t new_cmd, uint8_t fport_in, uint8_t* user_fport, uint8_t* msg_type,
                                   uint8_t* user_rx_payload, uint8_t* user_rx_payload_size, uint8_t* user_payload,
                                   uint8_t* user_payload_size );

/*!
 * \brief   Api to choose the lorawan key in case of a crc error
 * \remark  a crc error is present at the first start
 * \param [in]  device key
 * \param [out] none
 */
void lorawan_api_set_default_key( uint8_t default_app_key[16], uint8_t default_dev_eui[8],
                                  uint8_t default_join_eui[8] );

/*!
 * \brief  return true if stack receive a link adr request
 * \remark reset the flag automatically each time the upper layer call this function
 * \param [in]  void
 * \param [out] bool
 */
bool lorawan_api_available_link_adr_get( void );

/*!
 * \brief  return the current stack obj
 * \remark
 * \param [in]  void
 * \param [out] lr1_stack_mac_t*
 */
lr1_stack_mac_t* lorawan_api_stack_mac_get( void );

/**
 * @brief
 *
 * @return fifo_ctrl_t*
 */
fifo_ctrl_t* lorawan_api_get_fifo_obj( void );

/**
 * @brief set network type
 *
 * @param network_type true : public, false : private
 */
void lorawan_api_set_network_type( bool network_type );
/**
 * @brief  get network type
 *
 * @return true public network
 * @return false private network
 */
bool lorawan_api_get_network_type( void );

/*!
 * @brief lr1_stack_nb_trans_get
 * @remark
 * @return nb_trans
 */
uint8_t lorawan_api_nb_trans_get( void );

/*!
 * @brief lorawan_api_nb_trans_set
 * @remark
 * @param   nb_trans have to be smaller than 16
 * @return status_lorawan_t
 */
status_lorawan_t lorawan_api_nb_trans_set( uint8_t nb_trans );

/**
 * @brief get the current crystal error
 *
 */
uint32_t lorawan_api_get_crystal_error( void );

/**
 * @brief set the crystal error
 *
 * @param crystal_error
 */
void lorawan_api_set_crystal_error( uint32_t crystal_error );

#ifdef __cplusplus
}
#endif

#endif  // __LORAWAN_API_H__

/* --- EOF ------------------------------------------------------------------ */
