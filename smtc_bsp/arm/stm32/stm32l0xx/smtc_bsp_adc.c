/*!
 * \file      smtc_bsp_adc.c
 *
 * \brief     Implements the ADC BSP functions.
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "stm32l0xx_hal.h"
#include "smtc_bsp_mcu.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct bsp_adc_s
{
    ADC_TypeDef*           interface;
    ADC_HandleTypeDef      handle;
    ADC_ChannelConfTypeDef channel;
} bsp_adc_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

#if defined( __GNUC__ )
#pragma GCC diagnostic ignored "-Wmissing-braces"
#endif
static bsp_adc_t bsp_adc[] = { 
    [0] =
        {
            .interface = ADC1,
        },
}
;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void bsp_adc_init( const uint32_t id )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( bsp_adc_t ) ) );
    uint32_t local_id = id - 1;

    bsp_adc[local_id].handle.Instance = bsp_adc[local_id].interface;

    bsp_adc[local_id].handle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
    bsp_adc[local_id].handle.Init.Resolution            = ADC_RESOLUTION12b;
    bsp_adc[local_id].handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    bsp_adc[local_id].handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    bsp_adc[local_id].handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    bsp_adc[local_id].handle.Init.EOCSelection          = EOC_SINGLE_CONV;
    bsp_adc[local_id].handle.Init.SamplingTime          = ADC_SAMPLETIME_160CYCLES_5;
    bsp_adc[local_id].handle.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    bsp_adc[local_id].handle.Init.LowPowerAutoWait      = ENABLE;
    bsp_adc[local_id].handle.Init.ScanConvMode          = DISABLE;
    bsp_adc[local_id].handle.Init.ContinuousConvMode    = DISABLE;
    bsp_adc[local_id].handle.Init.DiscontinuousConvMode = DISABLE;
    bsp_adc[local_id].handle.Init.LowPowerAutoPowerOff  = DISABLE;
    bsp_adc[local_id].handle.Init.LowPowerFrequencyMode = DISABLE;
    bsp_adc[local_id].handle.Init.OversamplingMode      = DISABLE;
    bsp_adc[local_id].handle.Init.DMAContinuousRequests = DISABLE;

    bsp_adc[local_id].channel.Channel = ADC_CHANNEL_TEMPSENSOR;
    bsp_adc[local_id].channel.Rank    = 1;

    if( HAL_ADC_Init( &bsp_adc[local_id].handle ) != HAL_OK )
    {
        bsp_mcu_panic( );
    }

    if( HAL_ADC_ConfigChannel( &bsp_adc[local_id].handle, &bsp_adc[local_id].channel ) != HAL_OK )
    {
        bsp_mcu_panic( );
    }

    if( HAL_ADC_Start( &bsp_adc[local_id].handle ) != HAL_OK )
    {
        bsp_mcu_panic( );
    }
}

uint32_t bsp_adc_get_value( const uint32_t id, const uint32_t timeout_ms )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( bsp_adc ) ) );
    uint32_t local_id = id - 1;

    // Wait for end of conversion
    if( HAL_ADC_PollForConversion( &bsp_adc[local_id].handle, timeout_ms ) != HAL_OK )
    {
        bsp_mcu_panic( );
    }

    // Read ADC Value
    return HAL_ADC_GetValue( &bsp_adc[local_id].handle );
}

void bsp_adc_deinit( const uint32_t id )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( bsp_adc ) ) );
    uint32_t local_id = id - 1;

    HAL_ADC_DeInit( &bsp_adc[local_id].handle );
}

void HAL_ADC_MspInit( ADC_HandleTypeDef* adc_handle )
{
    if( adc_handle->Instance == bsp_adc[0].interface )
    {
        __HAL_RCC_ADC1_CLK_ENABLE( );
    }
    else
    {
        bsp_mcu_panic( );
    }
}

void HAL_ADC_MspDeInit( ADC_HandleTypeDef* adc_handle )
{
    if( adc_handle->Instance == bsp_adc[0].interface )
    {
        __HAL_RCC_ADC1_CLK_DISABLE( );
    }
    else
    {
        bsp_mcu_panic( );
    }
}
