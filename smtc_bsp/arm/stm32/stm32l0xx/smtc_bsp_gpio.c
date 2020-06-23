/*!
 * \file      smtc_bsp_gpio.c
 *
 * \brief     Implements the gpio BSP functions
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
#include "smtc_bsp_gpio.h"

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

/*!
 * GPIO setup data structure
 */
typedef struct bsp_gpio_s
{
    bsp_gpio_pin_names_t pin;
    uint32_t             mode;
    uint32_t             pull;
    uint32_t             speed;
    uint32_t             alternate;
} bsp_gpio_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * Array holding attached IRQ gpio data context
 */
static bsp_gpio_irq_t const* gpio_irq[16];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/*!
 * Generic gpio initialization
 *
 * \param [in/out] gpio  Holds MCU gpio parameters
 * \param [in]     value Initial MCU pit value
 * \param [in/out] irq   Pointer to IRQ data context.
 *                         NULL when setting gpio as output
 */
static void bsp_gpio_init( const bsp_gpio_t* gpio, const uint32_t value, const bsp_gpio_irq_t* irq );

//
// MCU output pin Handling
//

void bsp_gpio_init_out( const bsp_gpio_pin_names_t pin, const uint32_t value )
{
    bsp_gpio_t gpio = {
        .pin = pin, .mode = GPIO_MODE_OUTPUT_PP, .pull = GPIO_NOPULL, .speed = GPIO_SPEED_FREQ_LOW, .alternate = 0
    };
    bsp_gpio_init( &gpio, ( value != 0 ) ? GPIO_PIN_SET : GPIO_PIN_RESET, NULL );
}

//
// MCU input pin Handling
//

void bsp_gpio_init_in( const bsp_gpio_pin_names_t pin, const gpio_pull_mode_t pull_mode, const gpio_irq_mode_t irq_mode,
                       bsp_gpio_irq_t* irq )
{
    const uint32_t modes[] = { GPIO_MODE_INPUT, GPIO_MODE_IT_RISING, GPIO_MODE_IT_FALLING,
                               GPIO_MODE_IT_RISING_FALLING };
    const uint32_t pulls[] = { GPIO_NOPULL, GPIO_PULLUP, GPIO_PULLDOWN };

    bsp_gpio_t gpio = {
        .pin = pin, .mode = modes[irq_mode], .pull = pulls[pull_mode], .speed = GPIO_SPEED_FREQ_LOW, .alternate = 0
    };

    if( irq != NULL )
    {
        irq->pin = pin;
    }

    bsp_gpio_init( &gpio, GPIO_PIN_RESET, irq );
}

void bsp_gpio_irq_attach( const bsp_gpio_irq_t* irq )
{
    if( ( irq != NULL ) && ( irq->callback != NULL ) )
    {
        gpio_irq[( irq->pin ) & 0x0F] = irq;
    }
}

void bsp_gpio_irq_deatach( const bsp_gpio_irq_t* irq )
{
    if( irq != NULL )
    {
        gpio_irq[( irq->pin ) & 0x0F] = NULL;
    }
}

void bsp_gpio_irq_enable( void )
{
    HAL_NVIC_EnableIRQ( EXTI0_1_IRQn );
    HAL_NVIC_EnableIRQ( EXTI2_3_IRQn );
    HAL_NVIC_EnableIRQ( EXTI4_15_IRQn );
}

void bsp_gpio_irq_disable( void )
{
    HAL_NVIC_DisableIRQ( EXTI0_1_IRQn );
    HAL_NVIC_DisableIRQ( EXTI2_3_IRQn );
    HAL_NVIC_DisableIRQ( EXTI4_15_IRQn );
}

//
// MCU pin state control
//

void bsp_gpio_set_value( const bsp_gpio_pin_names_t pin, const uint32_t value )
{
    GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( IOPPERIPH_BASE + ( ( pin & 0xF0 ) << 6 ) );

    HAL_GPIO_WritePin( gpio_port, ( 1 << ( pin & 0x0F ) ), ( value != 0 ) ? GPIO_PIN_SET : GPIO_PIN_RESET );
}

void bsp_gpio_toggle( const bsp_gpio_pin_names_t pin )
{
    GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( IOPPERIPH_BASE + ( ( pin & 0xF0 ) << 6 ) );

    HAL_GPIO_TogglePin( gpio_port, ( 1 << ( pin & 0x0F ) ) );
}

uint32_t bsp_gpio_get_value( const bsp_gpio_pin_names_t pin )
{
    GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( IOPPERIPH_BASE + ( ( pin & 0xF0 ) << 6 ) );

    return ( HAL_GPIO_ReadPin( gpio_port, ( ( 1 << ( pin & 0x0F ) ) ) ) != GPIO_PIN_RESET ) ? 1 : 0;
}

bool bsp_gpio_is_pending_irq( void )
{
    return ( ( NVIC_GetPendingIRQ( EXTI0_1_IRQn ) == 1 ) || ( NVIC_GetPendingIRQ( EXTI2_3_IRQn ) == 1 ) ||
             ( NVIC_GetPendingIRQ( EXTI4_15_IRQn ) == 1 ) )
               ? true
               : false;
}
//
// MCU pin control private functions
//

static void bsp_gpio_init( const bsp_gpio_t* gpio, const uint32_t value, const bsp_gpio_irq_t* irq )
{
    GPIO_InitTypeDef gpio_local;
    GPIO_TypeDef*    gpio_port = ( GPIO_TypeDef* ) ( IOPPERIPH_BASE + ( ( gpio->pin & 0xF0 ) << 6 ) );

    gpio_local.Pin       = ( 1 << ( gpio->pin & 0x0F ) );
    gpio_local.Mode      = gpio->mode;
    gpio_local.Pull      = gpio->pull;
    gpio_local.Speed     = gpio->speed;
    gpio_local.Alternate = gpio->alternate;

    HAL_GPIO_WritePin( gpio_port, gpio_local.Pin, ( GPIO_PinState ) value );
    HAL_GPIO_Init( gpio_port, &gpio_local );

    if( ( gpio->mode == GPIO_MODE_IT_RISING ) || ( gpio->mode == GPIO_MODE_IT_FALLING ) ||
        ( gpio->mode == GPIO_MODE_IT_RISING_FALLING ) )
    {
        bsp_gpio_irq_attach( irq );
        switch( gpio->pin & 0x0F )
        {
        case 0:
        case 1:
            HAL_NVIC_SetPriority( EXTI0_1_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI0_1_IRQn );
            break;
        case 2:
        case 3:
            HAL_NVIC_SetPriority( EXTI2_3_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI2_3_IRQn );
            break;
        default:
            HAL_NVIC_SetPriority( EXTI4_15_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI4_15_IRQn );
            break;
        }
    }
}

//
// MCU interrupt handlers
//

void EXTI0_1_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_0 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_1 );
}

void EXTI2_3_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_2 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_3 );
}

void EXTI4_15_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_4 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_5 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_6 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_7 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_8 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_9 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_10 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_11 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_12 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_13 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_14 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_15 );
}

void HAL_GPIO_EXTI_Callback( uint16_t gpio_pin )
{
    uint8_t callback_index = 0;

    if( gpio_pin > 0 )
    {
        while( gpio_pin != 0x01 )
        {
            gpio_pin = gpio_pin >> 1;
            callback_index++;
        }
    }

    if( ( gpio_irq[callback_index] != NULL ) && ( gpio_irq[callback_index]->callback != NULL ) )
    {
        gpio_irq[callback_index]->callback( gpio_irq[callback_index]->context );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
