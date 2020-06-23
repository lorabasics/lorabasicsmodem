/*!
 * \file      smtc_bsp_uart.c
 *
 * \brief     Board specific package UART API implementation.
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
#include "smtc_bsp_gpio_pin_names.h"
#include "smtc_bsp_mcu.h"
#include "smtc_bsp_uart.h"
#include "smtc_bsp_options.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static DMA_HandleTypeDef hdma_usart1_rx;

static UART_HandleTypeDef huart2;
static UART_HandleTypeDef huart1;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void bsp_uart1_init( void )
{
    __HAL_RCC_DMA1_CLK_ENABLE( );
    HAL_NVIC_SetPriority( DMA1_Channel2_3_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( DMA1_Channel2_3_IRQn );

    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = 115200;
    huart1.Init.WordLength             = UART_WORDLENGTH_8B;
    huart1.Init.StopBits               = UART_STOPBITS_1;
    huart1.Init.Parity                 = UART_PARITY_NONE;
    huart1.Init.Mode                   = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if( HAL_UART_Init( &huart1 ) != HAL_OK )
    {
        bsp_mcu_panic( );
    }
}

void bsp_uart1_deinit( void )
{
    HAL_UART_DeInit( &huart1 );
}

void bsp_uart2_init( void )
{
    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = 115200;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if( HAL_UART_Init( &huart2 ) != HAL_OK )
    {
        bsp_mcu_panic( );
    }
}

void bsp_uart2_deinit( void )
{
    HAL_UART_DeInit( &huart2 );
}

void bsp_uart1_dma_start_rx( uint8_t* buff, uint16_t size )
{
    HAL_UART_DMAStop( &huart1 );
    HAL_UART_Receive_DMA( &huart1, buff, size );
}

void bsp_uart1_dma_stop_rx( void )
{
    HAL_UART_DMAStop( &huart1 );
}

void bsp_uart1_tx( uint8_t* buff, uint8_t len )
{
    HAL_UART_Transmit( &huart1, ( uint8_t* ) buff, len, 0xffffff );
}

void bsp_uart2_tx( uint8_t* buff, uint8_t len )
{
    HAL_UART_Transmit( &huart2, ( uint8_t* ) buff, len, 0xffffff );
}

void HAL_UART_MspInit( UART_HandleTypeDef* huart )
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if( huart->Instance == USART1 )
    {
        __HAL_RCC_USART1_CLK_ENABLE( );

        GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
        GPIO_InitStruct.Pin       = ( 1 << ( HW_MODEM_RX_LINE & 0x0F ) ) | ( 1 << ( HW_MODEM_TX_LINE & 0x0F ) );
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

        hdma_usart1_rx.Instance                 = DMA1_Channel3;
        hdma_usart1_rx.Init.Request             = DMA_REQUEST_3;
        hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart1_rx.Init.Mode                = DMA_NORMAL;
        hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_LOW;

        if( HAL_DMA_Init( &hdma_usart1_rx ) != HAL_OK )
        {
            bsp_mcu_panic( );
        }
        __HAL_LINKDMA( huart, hdmarx, hdma_usart1_rx );
    }
    else if( huart->Instance == USART2 )
    {
        __HAL_RCC_USART2_CLK_ENABLE( );

        GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
        GPIO_InitStruct.Pin       = ( 1 << ( DEBUG_UART_TX & 0x0F ) ) | ( 1 << ( DEBUG_UART_RX & 0x0F ) );
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

        __HAL_RCC_GPIOA_CLK_ENABLE( );
        HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );
    }
    else
    {
        bsp_mcu_panic( );
    }
}

void HAL_UART_MspDeInit( UART_HandleTypeDef* huart )
{
    if( huart->Instance == USART1 )
    {
        __HAL_RCC_USART1_CLK_DISABLE( );
        HAL_GPIO_DeInit( GPIOA, ( 1 << ( HW_MODEM_TX_LINE & 0x0F ) ) );
        HAL_GPIO_DeInit( GPIOA, ( 1 << ( HW_MODEM_RX_LINE & 0x0F ) ) );

        HAL_DMA_DeInit( &hdma_usart1_rx );

        __HAL_RCC_DMA1_CLK_DISABLE( );
    }
    if( huart->Instance == USART2 )
    {
        __HAL_RCC_USART2_CLK_DISABLE( );
        HAL_GPIO_DeInit( GPIOA, ( 1 << ( DEBUG_UART_TX & 0x0F ) ) );
        HAL_GPIO_DeInit( GPIOA, ( 1 << ( DEBUG_UART_RX & 0x0F ) ) );
    }
}

void DMA1_Channel2_3_IRQHandler( void )
{
    HAL_DMA_IRQHandler( huart1.hdmarx );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
