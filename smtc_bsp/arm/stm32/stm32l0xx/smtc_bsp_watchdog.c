/*!
 * \file      smtc_bsp_watchdog.h
 *
 * \brief     Board specific package WATCHDOG management API definition.
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
#include "smtc_bsp_watchdog.h"

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

static IWDG_HandleTypeDef iwdg_handle;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void bsp_watchdog_init( void )
{
    iwdg_handle.Instance       = IWDG;
    iwdg_handle.Init.Prescaler = IWDG_PRESCALER_256;
    iwdg_handle.Init.Window    = IWDG_WINDOW_DISABLE;
    iwdg_handle.Init.Reload    = 0xFFF;

    // Enable IWDG. LSI is turned on automaticaly.
    __HAL_IWDG_START( &iwdg_handle );

    // Enable write access to IWDG_PR, IWDG_RLR and IWDG_WINR registers by
    // writing 0x5555 in KR
    IWDG_ENABLE_WRITE_ACCESS( &iwdg_handle );
    // Write to IWDG registers the Prescaler & Reload values to work with.
    iwdg_handle.Instance->PR  = iwdg_handle.Init.Prescaler;
    iwdg_handle.Instance->RLR = iwdg_handle.Init.Reload;

    // Wait for register to be updated.
    while( iwdg_handle.Instance->SR != RESET )
    {
    }

    // If window parameter is different than current value, modify window
    // register.
    if( iwdg_handle.Instance->WINR != iwdg_handle.Init.Window )
    {
        // Write to IWDG WINR the IWDG_Window value to compare with. In any
        // case, even if window feature is disabled, Watchdog will be reloaded
        // by writing windows register.
        iwdg_handle.Instance->WINR = iwdg_handle.Init.Window;
    }
    else
    {
        // Reload IWDG counter with value defined in the reload register.
        __HAL_IWDG_RELOAD_COUNTER( &iwdg_handle );
    }
}

void bsp_watchdog_reload( void )
{
    __HAL_IWDG_RELOAD_COUNTER( &iwdg_handle );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
