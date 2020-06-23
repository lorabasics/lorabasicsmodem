/*!
 * \file      lr1_stack_mac_layer.h
 *
 * \brief     LoRaWan stack mac layer definition
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

#ifndef __LR1_STACK_MAC_LAYER_H__
#define __LR1_STACK_MAC_LAYER_H__
#ifdef __cplusplus
extern "C" {
#endif
/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */
#include "lr1mac_defs.h"
#include "radio_planner.h"
#include "smtc_real_defs.h"

/*
 *-----------------------------------------------------------------------------------
 * --- PUBLIC TYPES -----------------------------------------------------------------
 */
typedef struct lr1_stack_mac_s
{
    smtc_real_t* real;  // Region Abstraction Layer
    uint16_t     nb_of_reset;
    /* LoraWan Context */
    /* Only 16 ch mask => ChMaskCntl not used */
    /* Duty cycle is not managed */

    /*******************************************/
    /*      Update by Link ADR command         */
    /*******************************************/
    uint8_t  tx_data_rate;
    uint8_t  tx_data_rate_adr;
    int8_t   tx_power;
    int8_t   tx_power_offset;
    uint16_t channel_mask;
    uint8_t  nb_trans;
    uint8_t  nb_trans_cpt;
    /********************************************/
    /*     Update by RxParamaSetupRequest       */
    /********************************************/
    uint8_t  rx2_data_rate;
    uint32_t rx2_frequency;
    uint8_t  rx1_dr_offset;
    /********************************************/
    /*     Update by NewChannelReq command      */
    /********************************************/
    // NbOfActiveChannel value are valid

    /********************************************/
    /*   Update by RxTimingSetupReq command     */
    /********************************************/
    int rx1_delay_s;
    /********************************************/
    /*   Update by TxParamSetupReq command      */
    /********************************************/
    uint8_t max_eirp_dbm;
    bool    uplink_dwell_time;
    bool    downlink_dwell_time;
    /********************************************/
    /*   Update by DutyCycleReq command         */
    /********************************************/
    uint32_t max_duty_cycle_index;
    /********************************************/
    /*   Other Data To store                    */
    /********************************************/
    uint32_t fcnt_up;
    uint32_t fcnt_dwn;
    uint32_t dev_addr;
    uint8_t  nwk_skey[16];
    uint8_t  app_skey[16];
    uint8_t  app_key[16];
    uint8_t  dev_eui[8];
    uint8_t  app_eui[8];
    bool     otaa_device;
    /*******************************************/
    /* what about keys: AppEUI:Nwskey:AppSkey  */
    /*******************************************/

    // LoRaWan Mac Data for uplink
    uint8_t tx_fport;
    uint8_t tx_mtype;
    uint8_t tx_major_bits;
    uint8_t tx_fctrl;
    uint8_t tx_ack_bit;
    uint8_t app_payload_size;
    uint8_t tx_payload_size;
    uint8_t tx_payload[255];
    uint8_t tx_fopts_length;
    uint8_t tx_fopts_data[15];
    uint8_t tx_fopts_lengthsticky;
    uint8_t tx_fopts_datasticky[240];
    uint8_t tx_fopts_current_length;
    uint8_t tx_fopts_current_data[15];
    // LoRaWan Mac Data for downlin
    uint8_t               rx_fport;
    uint8_t               rx_mtype;
    uint8_t               rx_major;
    uint8_t               rx_fctrl;
    uint8_t               rx_ack_bit;
    uint8_t               rx_fopts_length;
    uint8_t               rx_fopts[16];
    uint8_t               rx_payload_size;  //@note Have to by replace by a fifo objet to manage class c
    uint8_t               rx_payload[255];  //@note Have to by replace by a fifo objet to manage class c
    uint8_t               rx_payload_empty;
    user_rx_packet_type_t available_app_packet;

    // LoRaWan Mac Data for duty-cycle
    uint32_t tx_duty_cycle_time_off_ms;
    uint32_t tx_duty_cycle_timestamp_ms;

    // LoRaWan Mac Data for join
    uint16_t dev_nonce;
    uint8_t  cf_list[16];

    // LoRaWan Mac Data for nwk Ans
    uint8_t nwk_payload[255];  //@note resize this buffer
    uint8_t nwk_payload_size;

    uint8_t nwk_ans[255];  //@note reuse user payload data or at least reduce size or use opt byte
    uint8_t nwk_ans_size;

    // LoraWan Config
    int           adr_ack_cnt;
    int           adr_ack_cnt_confirmed_frame;
    int           adr_ack_limit;
    int           adr_ack_delay;
    uint8_t       adr_ack_req;
    uint8_t       adr_enable;
    dr_strategy_t adr_mode_select;
    uint32_t      adr_custom;

    // Join Duty cycle management
    uint32_t next_time_to_join_seconds;
    uint32_t retry_join_cpt;
    uint32_t first_join_timestamp;

    uint8_t            tx_sf;
    modulation_type_t  tx_modulation_type;
    lr1mac_bandwidth_t tx_bw;
    uint32_t           tx_frequency;
    uint32_t           rx1_frequency;
    uint8_t            rx1_sf;
    lr1mac_bandwidth_t rx1_bw;
    modulation_type_t  rx1_modulation_type;
    uint8_t            rx2_sf;
    lr1mac_bandwidth_t rx2_bw;
    modulation_type_t  rx2_modulation_type;
    uint8_t            sync_word;

    // initially implemented in phy layer
    lr1mac_radio_state_t radio_process_state;
    radio_planner_t*     rp;
    uint32_t             rx_timeout_ms;
    uint16_t             rx_window_symb;
    join_status_t        join_status;
    rp_status_t          planner_status;
    int16_t              rx_snr;
    int16_t              rx_rssi;
    uint32_t             isr_radio_timestamp;
    int32_t              rx_offset_ms;
    uint32_t             timestamp_failsafe;
    uint32_t             dev_addr_isr;
    uint8_t              type_of_ans_to_send;
    uint8_t              nwk_payload_index;
    lr1mac_states_t      lr1_process;
    uint32_t             rtc_target_timer_ms;
    uint8_t              send_at_time;
} lr1_stack_mac_t;

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------------
 */
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_init( lr1_stack_mac_t* lr1_mac, lorawan_keys_t* lorawan_keys, smtc_real_t* real );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_session_init( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_tx_frame_build( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_tx_frame_encrypt( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_tx_radio_start( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_rx_radio_start( lr1_stack_mac_t* lr1_mac, const rx_win_type_t type, const uint32_t time_to_start );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
int lr1_stack_mac_downlink_check_under_it( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_rp_callback( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
int lr1_stack_mac_radio_state_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_rx_timer_configure( lr1_stack_mac_t* lr1_mac, const rx_win_type_t type );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
rx_packet_type_t lr1_stack_mac_rx_frame_decode( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_update( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t lr1_stack_mac_cmd_parse( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_join_request_build( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_mac_join_accept( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t lr1_stack_mac_min_dr_get( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t lr1_stack_mac_max_dr_get( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_rx1_join_delay_set( lr1_stack_mac_t* lr1_mac );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1_stack_rx2_join_dr_set( lr1_stack_mac_t* lr1_mac );

/*!
 * \brief lr1_stack_network_next_free_duty_cycle_ms_get
 * \remark duty cycle time off left about the network duty-cycle request
 * \param [IN]  lr1_stack_mac_t
 * \return duty-cycle time-off left, 0 if no pending limitation
 */
int32_t lr1_stack_network_next_free_duty_cycle_ms_get( lr1_stack_mac_t* lr1_mac );

#ifdef __cplusplus
}
#endif

#endif  // __LR1_STACK_MAC_LAYER_H__
