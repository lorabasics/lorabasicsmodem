/**
 * @file      smtc_modem_api.h
 *
 * @brief     Generic Modem API description
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

#ifndef SMTC_MODEM_API_H__
#define SMTC_MODEM_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief EUI (joinEUI, devEUI, etc.) length in byte
 */
#define SMTC_MODEM_EUI_LENGTH 8

/**
 * @brief Cryptographic key length in byte
 */
#define SMTC_MODEM_KEY_LENGTH 16

/**
 * @brief Application-defined user data length in byte
 */
#define SMTC_MODEM_DM_USER_DATA_LENGTH 8

/**
 * @brief ADR custom configuration length in byte
 */
#define SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH 16

/**
 * @defgroup SMTC_MODEM_EVENT_DEF Event codes definitions
 * @{
 */
#define SMTC_MODEM_EVENT_RESET 0x00                //!< Modem has been reset
#define SMTC_MODEM_EVENT_ALARM 0x01                //!< Alarm timer expired
#define SMTC_MODEM_EVENT_JOINED 0x02               //!< Network successfully joined
#define SMTC_MODEM_EVENT_TXDONE 0x03               //!< Frame transmitted
#define SMTC_MODEM_EVENT_DOWNDATA 0x04             //!< Downlink data received
#define SMTC_MODEM_EVENT_FILEDONE 0x05             //!< File upload completed
#define SMTC_MODEM_EVENT_SETCONF 0x06              //!< Configuration has been changed by the Device Management
#define SMTC_MODEM_EVENT_MUTE 0x07                 //!< Modem has been muted or un-muted by the Device Management
#define SMTC_MODEM_EVENT_STREAMDONE 0x08           //!< Stream upload completed (stream data buffer depleted)
#define SMTC_MODEM_EVENT_LINKSTATUS 0x09           //!< Network connectivity status changed
#define SMTC_MODEM_EVENT_JOINFAIL 0x0A             //!< Attempt to join network failed
#define SMTC_MODEM_EVENT_WIFI 0x0B                 //!< Wi-Fi scan done
#define SMTC_MODEM_EVENT_GNSS 0x0C                 //!< GNSS scan done
#define SMTC_MODEM_EVENT_TIME_UPDATED 0x0D         //!< Time has been updated by Alcsync
#define SMTC_MODEM_EVENT_TIMEOUT_ADR_CHANGED 0x0E  //!< ADR profile was switched to network controlled
#define SMTC_MODEM_EVENT_NEW_LINK_ADR 0x0F         //!< New link ADR requested by network
#define SMTC_MODEM_NO_EVENT 0xFF                   //!< No event available
/**
 * @}
 */

/**
 * @defgroup SMTC_MODEM_DM_INFO_DEF DM info fields codes
 * @{
 */
#define SMTC_MODEM_DM_FIELD_STATUS 0x00          //!< modem status
#define SMTC_MODEM_DM_FIELD_CHARGE 0x01          //!< charge counter [mAh]
#define SMTC_MODEM_DM_FIELD_VOLTAGE 0x02         //!< supply voltage [1/50 V]
#define SMTC_MODEM_DM_FIELD_TEMPERATURE 0x03     //!< junction temperature [deg Celsius]
#define SMTC_MODEM_DM_FIELD_SIGNAL 0x04          //!< strength of last downlink (RSSI [dBm]+64, SNR [0.25 dB])
#define SMTC_MODEM_DM_FIELD_UP_TIME 0x05         //!< duration since last reset [h]
#define SMTC_MODEM_DM_FIELD_RX_TIME 0x06         //!< duration since last downlink [h]
#define SMTC_MODEM_DM_FIELD_ADR_MODE 0x08        //!< ADR profile (0-3)
#define SMTC_MODEM_DM_FIELD_JOIN_EUI 0x09        //!< JoinEUI
#define SMTC_MODEM_DM_FIELD_INTERVAL 0x0A        //!< reporting interval [values 0-63, units s/m/h/d]
#define SMTC_MODEM_DM_FIELD_REGION 0x0B          //!< regulatory region
#define SMTC_MODEM_DM_FIELD_RST_COUNT 0x0F       //!< modem reset count
#define SMTC_MODEM_DM_FIELD_DEV_EUI 0x10         //!< DevEUI
#define SMTC_MODEM_DM_FIELD_SESSION 0x12         //!< session id / join nonce
#define SMTC_MODEM_DM_FIELD_CHIP_EUI 0x13        //!< ChipEUI
#define SMTC_MODEM_DM_FIELD_APP_STATUS 0x16      //!< application-specific status
#define SMTC_MODEM_DM_FIELD_ALMANAC_STATUS 0x18  //!< almanac status
/**
 * @}
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Modem Return Codes
 */
typedef enum smtc_modem_return_code_e
{
    RC_OK              = 0x00,  //!< command executed without errors
    RC_UNKNOWN         = 0x01,  //!< command code unknown
    RC_NOT_IMPLEMENTED = 0x02,  //!< command not yet implemented
    RC_NOT_INIT        = 0x03,  //!< command not initialized
    RC_INVALID         = 0x04,  //!< command parameters invalid
    RC_BUSY            = 0x05,  //!< command cannot be executed now
    RC_FAIL            = 0x06,  //!< command execution failed
    RC_BAD_FORMAT      = 0x07,  //!< format check failed
    RC_BAD_CRC         = 0x08,  //!< crc check failed
    RC_BAD_SIGNATURE   = 0x09,  //!< signature verification failed
    RC_BAD_SIZE        = 0x0A,  //!< size check failed
    RC_FRAME_ERROR     = 0x0F,  //!< serial port framing error
    RC_NO_TIME         = 0x10,  //!< no time available, either via network synchronization or set by the host
} smtc_modem_return_code_t;

/**
 * @brief Modem Datarate Profiles
 */
typedef enum smtc_modem_adr_profile_e
{
    SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED = 0x00,  //!< Network Server controlled for static devices
    SMTC_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE  = 0x01,  //!< Long range distribution for mobile devices
    SMTC_MODEM_ADR_PROFILE_MOBILE_LOW_POWER   = 0x02,  //!< Low power distribution for mobile devices
    SMTC_MODEM_ADR_PROFILE_CUSTOM             = 0x03,  //!< User defined distribution
} smtc_modem_adr_profile_t;

/**
 * @brief Modem status
 */
enum smtc_modem_status_mask_e
{
    SMTC_MODEM_STATUS_BROWNOUT = ( 1 << 0 ),
    SMTC_MODEM_STATUS_CRASH    = ( 1 << 1 ),
    SMTC_MODEM_STATUS_MUTE     = ( 1 << 2 ),
    SMTC_MODEM_STATUS_JOINED   = ( 1 << 3 ),
    SMTC_MODEM_STATUS_SUSPEND  = ( 1 << 4 ),
    SMTC_MODEM_STATUS_UPLOAD   = ( 1 << 5 ),
    SMTC_MODEM_STATUS_JOINING  = ( 1 << 6 ),
    SMTC_MODEM_STATUS_STREAM   = ( 1 << 7 ),
};

/**
 * @brief   Modem Status masks
 */
typedef uint32_t smtc_modem_status_mask_t;

/**
 * @brief Modem class enumeration
 */
typedef enum smtc_modem_class_e
{
    SMTC_MODEM_CLASS_A = 0x00,  //!< Modem class A
    SMTC_MODEM_CLASS_C = 0x01,  //!< Modem class C
} smtc_modem_class_t;

/**
 * @brief Cipher mode for file upload service
 */
typedef enum smtc_modem_file_upload_cipher_mode_e
{
    SMTC_MODEM_FILE_UPLOAD_NO_CIPHER,         //!< Do not encrypt file
    SMTC_MODEM_FILE_UPLOAD_AES_WITH_APPSKEY,  //!< Encrypt file using AES with appskey
} smtc_modem_file_upload_cipher_mode_t;

/**
 * @brief Cipher mode for stream service
 */
typedef enum smtc_modem_stream_cipher_mode_e
{
    SMTC_MODEM_STREAM_NO_CIPHER,         //!< Do not encrypt stream
    SMTC_MODEM_STREAM_AES_WITH_APPSKEY,  //!< Encrypt stream using AES with appskey
} smtc_modem_stream_cipher_mode_t;

/**
 * @brief Version structure definition
 */
typedef struct smtc_modem_version_s
{
    uint8_t major;  //!< Major value
    uint8_t minor;  //!< Minor value
    uint8_t patch;  //!< Patch value
} smtc_modem_version_t;

/**
 * @brief DM uplink reporting internal format
 */
typedef enum smtc_modem_dm_info_interval_format_e
{
    SMTC_MODEM_DM_INFO_INTERVAL_IN_SECOND = 0x00,
    SMTC_MODEM_DM_INFO_INTERVAL_IN_DAY    = 0x01,
    SMTC_MODEM_DM_INFO_INTERVAL_IN_HOUR   = 0x02,
    SMTC_MODEM_DM_INFO_INTERVAL_IN_MINUTE = 0x03,
} smtc_modem_dm_info_interval_format_t;

/**
 * @brief Modem region ID
 */
typedef enum smtc_modem_region_e
{
    SMTC_MODEM_REGION_EU_868        = 1,
    SMTC_MODEM_REGION_AS_923        = 2,
    SMTC_MODEM_REGION_US_915        = 3,
    SMTC_MODEM_REGION_AU_915        = 4,
    SMTC_MODEM_REGION_CN_470        = 5,
    SMTC_MODEM_REGION_WW2G4         = 6,
    SMTC_MODEM_REGION_AS_923_GRP2   = 7,
    SMTC_MODEM_REGION_AS_923_GRP3   = 8,
    SMTC_MODEM_REGION_IN_865        = 9,
    SMTC_MODEM_REGION_KR_920        = 10,
    SMTC_MODEM_REGION_RU_864        = 11,
    SMTC_MODEM_REGION_CN_470_RP_1_0 = 12,
} smtc_modem_region_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Get the modem event
 *
 * @remark This command can be used to retrieve pending events from the modem.
 *
 * @param [out] type              The event type (This parameter can be a value of @ref SMTC_MODEM_EVENT_DEF)
 * @param [out] count             The number of event(s) of this type, including missed event(s), in case of overrun
 * @param [in]  stack_id          The stack identifier
 * @param [out] data              The event specific data
 * @param [out] data_length       The event specific data length
 * @param [out] pending_count     The number of pending event(s)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */

smtc_modem_return_code_t smtc_modem_get_event( uint8_t* type, uint8_t* count, uint8_t* stack_id, uint8_t* data,
                                               uint16_t* data_length, uint8_t* pending_count );

/**
 * @brief Get the firmware and the LoRaWAN versions
 *
 * @param [out] firmware_version The firmware version
 * @param [out] lorawan_version  The LoRaWAN version
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_version( smtc_modem_version_t* firmware_version,
                                                 smtc_modem_version_t* lorawan_version );

/**
 * @brief Reset the modem
 *
 * @remark All transient state (including session information) will be lost - the modem needs to join the network again
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_reset( void );

/**
 * @brief Reset the modem to its original state
 *
 * @remark In addition to the MCU reset all persistent settings are reset back to their factory state
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_factory_reset( void );

/**
 * @brief Get the total charge counter of the modem in mAh
 *
 * @param [out] charge_mah The accumulated charge in mAh
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_charge( uint32_t* charge_mah );

/**
 * @brief Reset the total charge counter of the modem
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_reset_charge( void );

/**
 * @brief Get the Tx power offset in dB
 *
 * @param [in]  stack_id         The stack identifier
 * @param [out] tx_pwr_offset_db The Tx power offset in dB
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_tx_power_offset_db( uint8_t stack_id, int8_t* tx_pwr_offset_db );

/**
 * @brief Set the Tx power offset in dB
 *
 * @param [in] stack_id         The stack identifier
 * @param [in] tx_pwr_offset_db The Tx power offset in dB
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_tx_power_offset_db( uint8_t stack_id, int8_t tx_pwr_offset_db );

/**
 * @brief Get the Rx power offset in dB
 *
 * @param [in]  stack_id     The stack identifier
 * @param [out] rx_offset_db The Rx power offset in dB
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_rx_power_offset_db( uint8_t stack_id, int16_t* rx_offset_db );

/**
 * @brief Set the Rx power offset in dB
 *
 * @param [in] stack_id      The stack identifier
 * @param [in] rx_offset_db  The Rx power offset in dB
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_rx_power_offset_db( uint8_t stack_id, int16_t rx_offset_db );

/**
 * @brief Enable / disable the Application Layer Clock Synchronization (ALCSync) service
 *
 * @param [in] mode The ALCSync configuration to apply
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_alcsync_mode( uint8_t mode );

/**
 * @brief Get the current state of the Application Layer Clock Synchronization (ALCSync) service
 *
 * @param [out] mode The current ALCSync configuration
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_alcsync_mode( uint8_t* mode );

/**
 * @brief Get GPS epoch time
 *
 * @remark The returned time specifies the number of seconds elapsed since GPS epoch (00:00:00, Sunday 6th of January
 * 1980). If the device is not yet synchronized to GPS time then the returned value is zero. In case of time wrapping,
 * the time could be really equal to zero so the value is incremented of 1 to avoid 0
 *
 * @param [out] gps_time_s The GPS time in seconds
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_time( uint32_t* gps_time_s );

/**
 * @brief Set Application Layer Clock Synchronization (ALCSync) service FPort
 *
 * @remark An alc_sync_fport value of 0 or equal to DM FPort will encapsulate the ALCSync requests into DM frames, to be
 * used with DAS ALCSync service.
 *
 * @param [in] alc_sync_fport The ALCSync FPort.
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_alc_sync_fport( uint8_t alc_sync_fport );

/**
 * @brief Get Application Layer Clock Synchronization (ALCSync) service FPort
 *
 * @param [out] alc_sync_fport The ALCSync FPort
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_alc_sync_fport( uint8_t* alc_sync_fport );

/**
 * @brief Get the modem status
 *
 * @param [in]  stack_id    The stack identifier
 * @param [out] status_mask The modem status defined in @ref smtc_modem_status_mask_e
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_status( uint8_t stack_id, smtc_modem_status_mask_t* status_mask );

/**
 * @brief Set and start the alarm timer
 *
 * @remark When the timer expires, an alarm event is generated
 *
 * @param [in] alarm_timer_in_s The alarm timer in second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_start_alarm_timer( uint32_t alarm_timer_in_s );

/**
 * @brief Stop and clear alarm timer
 *
 * @remark If there is no alarm running, the function will return @ref RC_NOT_INIT
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_clear_alarm_timer( void );

/**
 * @brief Get the number of seconds remaining before the alarm triggers an event
 *
 * @remark If there is no alarm running, the number of seconds remaining is 0, and return code is RC_NOT_INIT.
 *
 * @param [out] remaining_time_in_s The number of seconds remaining before the alarm triggers an event
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_alarm_remaining_time( uint32_t* remaining_time_in_s );

/**
 * @brief Get the JoinEUI
 *
 * @param [in]  stack_id The stack identifier
 * @param [out] joineui  The current JoinEUI
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_joineui( uint8_t stack_id, uint8_t joineui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Set the JoinEUI
 *
 * @param [in]  stack_id The stack identifier
 * @param [in]  joineui  The JoinEUI to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_joineui( uint8_t stack_id, const uint8_t joineui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Get the DevEUI
 *
 * @param [in]  stack_id The stack identifier
 * @param [out] deveui   The current DevEUI
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_deveui( uint8_t stack_id, uint8_t deveui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Set the DevEUI
 *
 * @param [in]  stack_id The stack identifier
 * @param [in]  deveui   The DevEUI to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_deveui( uint8_t stack_id, const uint8_t deveui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Set the Network Key (aka Application Key in LoRaWAN v1.0.x)
 *
 * @param [in] stack_id The stack identifier
 * @param [in] nwkkey   The key to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_nwkkey( uint8_t stack_id, const uint8_t nwkkey[SMTC_MODEM_KEY_LENGTH] );

/**
 * @brief Get the current LoRaWAN class
 *
 * @param [in]  stack_id      The stack identifier
 * @param [out] lorawan_class The current LoRaWAN class
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_class( uint8_t stack_id, smtc_modem_class_t* lorawan_class );

/**
 * @brief Set the LoRaWAN class
 *
 * @param [in] stack_id      The stack identifier
 * @param [in] lorawan_class The LoRaWAN class to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_class( uint8_t stack_id, smtc_modem_class_t lorawan_class );

/**
 * @brief Get the current LoRaWAN region
 *
 * @param [in]  stack_id The stack identifier
 * @param [out] region   The current LoRaWAN region
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_region( uint8_t stack_id, smtc_modem_region_t* region );

/**
 * @brief Set the LoRaWAN region
 *
 * @param [in]  stack_id The stack identifier
 * @param [in]  region   The LoRaWAN region to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_region( uint8_t stack_id, smtc_modem_region_t region );

/**
 * @brief Get the current ADR profile
 *
 * @param [in]  stack_id    The stack identifier
 * @param [out] adr_profile The current ADR profile
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_adr_profile( uint8_t stack_id, smtc_modem_adr_profile_t* adr_profile );

/**
 * @brief Set the ADR profile
 *
 * @remark If @ref SMTC_MODEM_ADR_PROFILE_CUSTOM is selected, custom data are taken into account
 *
 * @param [in] stack_id        The stack identifier
 * @param [in] adr_profile     The ADR profile to be configured
 * @param [in] adr_custom_data The definition of the custom ADR profile
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_adr_profile( uint8_t stack_id, smtc_modem_adr_profile_t adr_profile,
                                                     const uint8_t adr_custom_data[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] );

/**
 * @brief Get the configured custom ADR profile data
 *
 * @param [in]  stack_id        The stack identifier
 * @param [out] adr_custom_data The definition of the custom ADR profile
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_adr_custom_data( uint8_t stack_id,
                                                         uint8_t adr_custom_data[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] );

/**
 * @brief Get the current available Datarate in regards of Uplink ChMash and DwellTime
 *
 * @param [in]  stack_id                 The stack identifier
 * @param [out] available_datarates_mask The available datarates, described in a bitfield
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_available_datarates( uint8_t stack_id, uint16_t* available_datarates_mask );

/**
 * @brief Get the Device Management LoRaWAN FPort
 *
 * @param [out] dm_fport The FPort on which the DM info is sent
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_dm_fport( uint8_t* dm_fport );

/**
 * @brief Set the Device Management LoRaWAN FPort
 *
 * @param [in] dm_fport The FPort on which the DM info is sent. This value must be in the range [1:223]
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_dm_fport( uint8_t dm_fport );

/**
 * @brief Get the interval between 2 DM info field messages
 *
 * @param [out] format   The reporting interval format
 * @param [out] interval The interval in unit defined in format
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_dm_info_interval( smtc_modem_dm_info_interval_format_t* format,
                                                          uint8_t*                              interval );

/**
 * @brief Set the interval between 2 DM info field messages
 *
 * @remark A value set to 0 disables the feature - no matter the format.
 *
 * @param [in] format   The reporting interval format
 * @param [in] interval The interval in unit defined in format
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_dm_info_interval( smtc_modem_dm_info_interval_format_t format,
                                                          uint8_t                              interval );

/**
 * @brief Get DM info fields
 *
 * @param [out] dm_fields_payload The DM fields  (defined in @ref SMTC_MODEM_DM_INFO_DEF)
 * @param [out] dm_field_length   The DM field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_dm_info_fields( uint8_t* dm_fields_payload, uint8_t* dm_field_length );

/**
 * @brief Set the DM info fields to be sent on a regular basis
 *
 * @remark The interval between two DM info field messages is defined with @ref smtc_modem_set_dm_info_interval
 *
 * @param [in] dm_fields_payload The DM fields  (defined in @ref SMTC_MODEM_DM_INFO_DEF)
 * @param [in] dm_field_length   The DM field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_dm_info_fields( const uint8_t* dm_fields_payload, uint8_t dm_field_length );

/**
 * @brief Request an immediate DM status
 *
 * @remark The content is indepandent from the configuration set with @ref smtc_modem_set_dm_info_fields
 *
 * @param [in] dm_fields_payload The DM fields  (defined in @ref SMTC_MODEM_DM_INFO_DEF)
 * @param [in] dm_field_length   The DM field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_request_single_dm_uplink( const uint8_t* dm_fields_payload,
                                                              uint8_t        dm_field_length );

/**
 * @brief Set user-specific data to be reported by Device Management frames
 *
 * @remark This field will be sent only if it is selected in @ref smtc_modem_set_dm_info_fields or @ref
 * smtc_modem_request_single_dm_uplink
 *
 * @param [in] user_data User-specific data
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_dm_user_data( const uint8_t user_data[SMTC_MODEM_DM_USER_DATA_LENGTH] );

/**
 * @brief Get user-specific data to be reported by Device Management frames
 *
 * @param [out] user_data User-specific data
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_dm_user_data( uint8_t user_data[SMTC_MODEM_DM_USER_DATA_LENGTH] );

/**
 * @brief Join the network
 *
 * @param [in] stack_id The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_join_network( uint8_t stack_id );

/**
 * @brief Leave an already joined network or cancels on ongoing join process
 *
 * @param [in] stack_id The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_leave_network( uint8_t stack_id );

/**
 * @brief Suspend the radio communications initiated by the modem
 *
 * @param [in] suspend The configuration to be applied (true: suspend communications / false: resume communications)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_suspend_radio_communications( bool suspend );

/**
 * @brief Get the maximum payload size that can be used for the next uplink
 *
 * @remark This value depends on the LoRaWAN regional parameters for the next transmission using the current data rate
 *
 * @param [in]  stack_id            The stack identifier
 * @param [out] tx_max_payload_size The maximum payload size in byte
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_next_tx_max_payload( uint8_t stack_id, uint8_t* tx_max_payload_size );

/**
 * @brief Request a LoRaWAN uplink
 *
 * @remark LoRaWAN NbTrans parameter can be set in mobiles and custom ADR modes with @ref smtc_modem_set_nb_trans
 *
 * @param [in] stack_id       The stack identifier
 * @param [in] fport          The LoRaWAN FPort on which the uplink is done
 * @param [in] confirmed      The message type (true: confirmed, false: unconfirmed)
 * @param [in] payload        The data to be sent
 * @param [in] payload_length The number of bytes from payload to be sent
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_request_uplink( uint8_t stack_id, uint8_t fport, bool confirmed,
                                                    const uint8_t* payload, uint8_t payload_length );

/**
 * @brief Request an immediate LoRaWAN uplink
 *
 * @remark It has higher priority than all other services and is not subject to duty cycle restrictions, if any
 * @remark LoRaWAN NbTrans parameter can be set in mobiles and custom ADR modes with @ref smtc_modem_set_nb_trans
 *
 * @param [in] stack_id       The stack identifier
 * @param [in] fport          The LoRaWAN FPort on which the uplink is done
 * @param [in] confirmed      The message type (true: confirmed, false: unconfirmed)
 * @param [in] payload        The data to be sent
 * @param [in] payload_length The number of bytes from payload to be sent
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_request_emergency_uplink( uint8_t stack_id, uint8_t fport, bool confirmed,
                                                              const uint8_t* payload, uint8_t payload_length );

/**
 * @brief Create and initialize a file upload session
 *
 * @param [in] stack_id        The stack identifier
 * @param [in] index           The index on which the upload is done
 * @param [in] cipher_mode     The cipher mode
 * @param [in] file            The file buffer
 * @param [in] file_length     The file size
 * @param [in] average_delay_s The minimal delay between two file upload fragments, in seconds (from the end of an
 *                             uplink to the start of the next one)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_file_upload_init( uint8_t stack_id, uint8_t index,
                                                      smtc_modem_file_upload_cipher_mode_t cipher_mode,
                                                      const uint8_t* file, uint16_t file_length,
                                                      uint32_t average_delay_s );

/**
 * @brief Start the file upload towards the DAS
 *
 * @param [in] stack_id The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_file_upload_start( uint8_t stack_id );

/**
 * @brief Reset the file upload session
 *
 * @remark If there is an ongoing upload, this function will stop it
 *
 * @param [in] stack_id The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_file_upload_reset( uint8_t stack_id );

/**
 * @brief Create and initialize a data stream
 *
 * @param [in] stack_id    The stack identifier
 * @param [in] fport       The FPort on which the stream is sent
 * @param [in] cipher_mode The cipher mode
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_stream_init( uint8_t stack_id, uint8_t fport,
                                                 smtc_modem_stream_cipher_mode_t cipher_mode );

/**
 * @brief Add data to the stream
 *
 * @remark If no stream has been initialized,  open a new unencrypted stream on the FPort given as parameter
 *
 * @param [in] stack_id The stack identifier
 * @param [in] fport    The FPort on which the stream is sent
 * @param [in] data     The data to be added to the stream
 * @param [in] len      The number of bytes from data to be added to the stream
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_stream_add_data( uint8_t stack_id, uint8_t fport, const uint8_t* data,
                                                     uint8_t len );

/**
 * @brief Return the current stream status
 *
 * @param [in]  stack_id The stack identifier
 * @param [in]  fport    The FPort on which the stream is sent
 * @param [out] pending  The length of pending data for transmission
 * @param [out] free     The length of free space in the buffer
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_stream_status( uint8_t stack_id, uint8_t fport, uint16_t* pending, uint16_t* free );

/**
 * @brief Enable / disable the certification mode
 *
 * @param [in] stack_id The stack identifier
 * @param [in] enable   The certification mode state (default: disabled)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_certification_mode( uint8_t stack_id, bool enable );

/**
 * @brief Get the current state of the certification mode
 *
 * @param [in]  stack_id The stack identifier
 * @param [out] enable   The certification mode state
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_certification_mode( uint8_t stack_id, bool* enable );

/**
 * @brief Set the connection timeout thresholds
 *
 * @remark The value 0 deactivates the function
 * @remark It is recommended to have nb_of_uplinks_before_network_controlled smaller than nb_of_uplink_before_reset
 *
 * @param [in] stack_id                                The stack identifier
 * @param [in] nb_of_uplinks_before_network_controlled The number of uplinks without downlink before ADR profile
 *                                                     switches to network controlled
 * @param [in] nb_of_uplinks_before_reset              The number of uplinks without downlink before reset
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_connection_timeout_thresholds( uint8_t  stack_id,
                                                                       uint16_t nb_of_uplinks_before_network_controlled,
                                                                       uint16_t nb_of_uplinks_before_reset );

/**
 * @brief Get the configured connection timeout thresholds
 *
 * @param [in]  stack_id                                The stack identifier
 * @param [out] nb_of_uplinks_before_network_controlled The number of uplinks without downlink before ADR profile
 *                                                      switches to network controlled
 * @param [out] nb_of_uplinks_before_reset              The number of uplinks without downlink before reset
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_connection_timeout_thresholds(
    uint8_t stack_id, uint16_t* nb_of_uplinks_before_network_controlled, uint16_t* nb_of_uplinks_before_reset );

/**
 * @brief Get the current status of the connection timeouts
 *
 * @param [in]  stack_id                                 The stack identifier
 * @param [out] nb_of_uplinks_before_network_controlled  The number of remaining uplinks without downlink before ADR
 *                                                       profile switches to network controlled
 * @param [out] nb_of_uplinks_before_reset               The number of remaining uplinks without downlink before reset
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_current_connection_timeout( uint8_t   stack_id,
                                                                    uint16_t* nb_of_uplinks_before_network_controlled,
                                                                    uint16_t* nb_of_uplinks_before_reset );

/**
 * @brief Get the current status of the duty cycle
 *
 * @remark If the returned value is positive, it is the time still available. A negative value indicates the time to
 * wait until band availability
 *
 * @param [out] duty_cycle_status_ms The status of the duty cycle in milliseconds
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_duty_cycle_status( int32_t* duty_cycle_status_ms );

/**
 * @brief Prevent the stack from accessing the radio
 *
 * @remark The user must call this function before performing operations requiring a direct access to the radio (e.g.
 * test modes). Otherwise, undefined behavior may occurs.
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_suspend_radio_access( void );

/**
 * @brief Allow the stack to get access to the radio
 *
 * @remark The user must call this function after performing operations requiring a direct access to the radio (e.g.
 * test modes). Otherwise, all stack-related tasks remain pending.
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_resume_radio_access( void );

/**
 * @brief Configure network type to private or public
 *
 * @param [in]  stack_id      The stack identifier
 * @param [in]  network_type  The configuration to be applied (true: public network / false: private network)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_network_type( uint8_t stack_id, bool network_type );

/**
 * @brief Get the configured network type
 *
 * @param [in]  stack_id      The stack identifier
 * @param [in]  network_type  The configuration to be applied (true: public network / false: private network)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_network_type( uint8_t stack_id, bool* network_type );

/**
 * @brief Configure the LBT feature (Listen Before Talk)
 *
 * @param [in]  stack_id            The stack identifier
 * @param [in]  listen_duration_ms  The listen duration in ms to be configured
 * @param [in]  threshold_dbm       The lbt threshold in dbm to be configured
 * @param [in]  bw_hz               The lbt bandwith in hertz to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_lbt_configuration( uint8_t stack_id, uint32_t listen_duration_ms,
                                                           int16_t threshold_dbm, uint32_t bw_hz );

/**
 * @brief Get the configured LBT (Listen Before Talk)
 *
 * @param [in]  stack_id            The stack identifier
 * @param [out] listen_duration_ms  The current listen duration in ms
 * @param [out] threshold_dbm       The current lbt threshold in dbm
 * @param [out] bw_hz               The current lbt bandwith in hertz
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_lbt_configuration( uint8_t stack_id, uint32_t* listen_duration_ms,
                                                           int16_t* threshold_dbm, uint32_t* bw_hz );

/**
 * @brief Start LBT feature (Listen Before Talk)
 * @remark The configuration function @ref smtc_modem_set_lbt_config must be called before starting the LBT feature
 *
 * @param [in]  stack_id            The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_start_lbt( uint8_t stack_id );

/**
 * @brief Stop LBT feature (Listen Before Talk)
 *
 * @param [in]  stack_id            The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_stop_lbt( uint8_t stack_id );

/**
 * @brief Set the number of transmissions (nb_trans) in case of unconfirmed uplink
 *
 * @param [in]  stack_id  The stack identifier
 * @param [in]  nb_trans  The number of transmissions ( 0 < value < 16 )
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_nb_trans( uint8_t stack_id, uint8_t nb_trans );

/**
 * @brief Get the configured number of transmissions (nb_trans) in case of unconfirmed uplink
 *
 * @param [in]  stack_id  The stack identifier
 * @param [out] nb_trans  The number of transmissions
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_nb_trans( uint8_t stack_id, uint8_t* nb_trans );

/**
 * @brief Get the configured stream redundancy radio (rr)
 *
 * @param [in]  stack_id   The stack identifier
 * @param [out] stream_rr  The stream redundancy ratio
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_stream_rr( uint8_t stack_id, uint8_t* stream_rr );

/**
 * @brief Set the stream redundancy ratio
 *
 * @param [in]  stack_id   The stack identifier
 * @param [in]  redundancy_ratio_percent  The stream redundancy ratio. Accepted value [0; 110%]
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_stream_redundancy_ratio( uint8_t stack_id, uint8_t redundancy_ratio_percent );

/**
 * @brief Set modem crystal error
 *
 * @param [in] crystal_error_ppm The crystal error in ppm
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_set_crystal_error( uint32_t crystal_error_ppm );

/**
 * @brief Get the modem crystal error
 *
 * @param [out] crystal_error_ppm The crystal error in ppm
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_crystal_error( uint32_t* crystal_error_ppm );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_API_H__

/* --- EOF ------------------------------------------------------------------ */
