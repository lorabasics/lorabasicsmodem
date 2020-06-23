/*!
 * \file      smtc_bsp_mcu.c
 *
 * \brief     Board specific package MCU API implementation.
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
#include "stm32l0xx_ll_utils.h"
#include "smtc_bsp.h"

#if( BSP_DBG_TRACE == BSP_FEATURE_ON )
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
// STM32L073 temperature calibration information
#define TEMP130_CAL_ADDR ( ( uint16_t* ) ( ( uint32_t ) 0x1FF8007E ) )
#define TEMP30_CAL_ADDR ( ( uint16_t* ) ( ( uint32_t ) 0x1FF8007A ) )
#define VDD_CALIB ( ( uint16_t )( 3000 ) )
#define VDD_APPLI ( ( uint16_t )( 3000 ) )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

// Low Power options
typedef enum low_power_mode_e
{
    LOW_POWER_ENABLE,
    LOW_POWER_DISABLE,
    LOW_POWER_DISABLE_ONCE
} low_power_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static volatile bool             bsp_exit_wait       = false;
static volatile low_power_mode_t bsp_lp_current_mode = LOW_POWER_ENABLE;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void bsp_system_clock_config( void );
static void bsp_mcu_start_systick( void );
static void bsp_mcu_stop_systick( void );
static void bsp_mcu_gpio_init( void );

#if( BSP_LOW_POWER_MODE == BSP_FEATURE_ON )
static void bsp_mcu_deinit( void );
static void bsp_mcu_reinit( void );
static void bsp_system_clock_re_config_after_stop( void );
static void bsp_low_power_handler( void );
#else
static bool bsp_mcu_no_low_power_wait( const int32_t milliseconds );
#endif

#if( BSP_DBG_TRACE == BSP_FEATURE_ON )
static void vprint( const char* fmt, va_list argp );
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void bsp_mcu_critical_section_begin( uint32_t* mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void bsp_mcu_critical_section_end( uint32_t* mask )
{
    __set_PRIMASK( *mask );
}

void bsp_mcu_disable_periph_irq( void )
{
    bsp_gpio_irq_disable( );
    bsp_tmr_irq_disable( );
}

void bsp_mcu_enable_periph_irq( void )
{
    bsp_gpio_irq_enable( );
    bsp_tmr_irq_enable( );
}

void bsp_mcu_init( void )
{
    // Initialize MCU HAL library
    HAL_Init( );
    // Initialize clocks
    bsp_system_clock_config( );
    // Initialize GPIOs
    bsp_mcu_gpio_init( );

    // Initialize low power timer
    bsp_tmr_init( );

    // Initialize UART
#if( BSP_USE_USER_UART == BSP_FEATURE_ON )
    bsp_uart1_init( );
    // bsp_uart_init( BSP_USER_UART_ID, 115200 );
#endif
#if( BSP_USE_PRINTF_UART == BSP_FEATURE_ON )
    // bsp_uart_init( BSP_PRINTF_UART_ID, 115200 );
    bsp_uart2_init( );

    // Change stdio default buffer to \ref stdio_buffer
    // setvbuf( stdout, stdio_buffer, _IOLBF, sizeof( stdio_buffer ) );
#endif

    // Initialize SPI

    bsp_spi_init( BSP_RADIO_SPI_ID, RADIO_SPI_MOSI, RADIO_SPI_MISO, RADIO_SPI_SCLK );

    // Initialize RTC
    bsp_rtc_init( );

    // Initialize watchdog
#if( BSP_USE_WATCHDOG == 1 )
    bsp_watchdog_init( );
#endif
}

void bsp_disable_irq( void )
{
    __disable_irq( );
}

void bsp_enable_irq( void )
{
    __enable_irq( );
}

void bsp_mcu_reset( void )
{
    __disable_irq( );

    // Restart system
    NVIC_SystemReset( );
}

void bsp_mcu_panic( void )
{
    CRITICAL_SECTION_BEGIN( );

    BSP_DBG_TRACE_ERROR( "%s\n", __func__ );
    BSP_DBG_TRACE_ERROR( "PANIC" );

    while( 1 )
    {
    }
}

void bsp_mcu_handle_lr1mac_issue( void )
{
    CRITICAL_SECTION_BEGIN( );
    BSP_DBG_TRACE_ERROR( "%s\n", __func__ );
    BSP_DBG_TRACE_ERROR( "LR1MAC PANIC" );

    while( 1 )
    {
    }
}

void bsp_mcu_modem_need_reset( void )
{
    __disable_irq( );

    // Restart system
    NVIC_SystemReset( );
}

void bsp_mcu_set_sleep_for_ms( const int32_t milliseconds )
{
    if( milliseconds <= 0 )
    {
        return;
    }

    if( bsp_lp_current_mode == LOW_POWER_DISABLE_ONCE )
    {
        bsp_lp_current_mode = LOW_POWER_ENABLE;
        return;
    }
    int32_t time_counter = milliseconds;

    bsp_watchdog_reload( );

#if( BSP_LOW_POWER_MODE == BSP_FEATURE_ON )
    while( ( time_counter > ( BSP_WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 ) ) &&
           ( bsp_lp_current_mode == LOW_POWER_ENABLE ) )
    {
        time_counter -= BSP_WATCHDOG_RELOAD_PERIOD_SECONDS * 1000;
        bsp_rtc_wakeup_timer_set_ms( BSP_WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 );
        bsp_low_power_handler( );
        bsp_watchdog_reload( );
    }
    if( bsp_lp_current_mode == LOW_POWER_ENABLE )
    {  // have to wake up on modem cmd
        bsp_rtc_wakeup_timer_set_ms( time_counter );
        bsp_low_power_handler( );
        bsp_watchdog_reload( );
    }
#else
    while( ( time_counter > ( BSP_WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 ) ) &&
           ( bsp_lp_current_mode == LOW_POWER_ENABLE ) )
    {
        time_counter -= BSP_WATCHDOG_RELOAD_PERIOD_SECONDS * 1000;
        if( ( bsp_mcu_no_low_power_wait( BSP_WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 ) == true ) ||
            ( bsp_lp_current_mode != LOW_POWER_ENABLE ) )
        {
            // wait function was interrupted, inturrupt here also
            bsp_watchdog_reload( );
            return;
        }
        bsp_watchdog_reload( );
    }
    if( bsp_lp_current_mode == LOW_POWER_ENABLE )
    {
        bsp_mcu_no_low_power_wait( time_counter );
        bsp_watchdog_reload( );
    }
#endif
}

void __attribute__( ( optimize( "O0" ) ) ) bsp_mcu_wait_us( const int32_t microseconds )
{
    const uint32_t nb_nop = microseconds * 1000 / 561;
    for( uint32_t i = 0; i < nb_nop; i++ )
    {
        __NOP( );
    }
}

uint8_t bsp_mcu_get_battery_level( void )
{
    return 254;
}

void bsp_mcu_disable_low_power_wait( void )
{
    bsp_exit_wait       = true;
    bsp_lp_current_mode = LOW_POWER_DISABLE;
}

void bsp_mcu_enable_low_power_wait( void )
{
    bsp_exit_wait       = false;
    bsp_lp_current_mode = LOW_POWER_ENABLE;
}

void bsp_mcu_disable_once_low_power_wait( void )
{
    bsp_exit_wait       = true;
    bsp_lp_current_mode = LOW_POWER_DISABLE_ONCE;
}

int32_t bsp_mcu_get_mcu_temperature( void )
{
    // Configure ADC1 to read MCU internal temperature
    bsp_adc_init( 1 );
    uint32_t measure = bsp_adc_get_value( 1, 1000 );
    bsp_adc_deinit( 1 );

    int32_t temperature = 0;

    // Convert ADC value to celsius using STM32 calibration
    temperature = ( ( measure * VDD_APPLI / VDD_CALIB ) - ( int32_t ) *TEMP30_CAL_ADDR );
    temperature = temperature * ( int32_t )( 130 - 30 );
    temperature = temperature / ( int32_t )( *TEMP130_CAL_ADDR - *TEMP30_CAL_ADDR );
    temperature = temperature + 30;

    return temperature;
}

uint8_t bsp_mcu_get_mcu_voltage( void )
{
    // to be implemented by user
    return 0x98; //0x98 means 3v ()
}

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
}

void bsp_trace_print( const char* fmt, ... )
{
#if BSP_DBG_TRACE == BSP_FEATURE_ON
    va_list argp;
    va_start( argp, fmt );
    vprint( fmt, argp );
    va_end( argp );
#endif
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line
 * number where the assert_param error has occurred. Input          : - file:
 * pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    // User can add his own implementation to report the file name and line
    // number,
    // ex: printf("Wrong parameters value: file %s on line %lu\r\n", file, line)

    BSP_DBG_TRACE_PRINTF( "Wrong parameters value: file %s on line %lu\r\n", ( const char* ) file, line );
    // Infinite loop
    while( 1 )
    {
    }
}
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void bsp_system_clock_config( void )
{
    RCC_OscInitTypeDef       rcc_osc_init;
    RCC_ClkInitTypeDef       rcc_clk_init;
    RCC_PeriphCLKInitTypeDef periph_clk_init;

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    // Initializes the CPU, AHB and APB busses clocks
    rcc_osc_init.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
    rcc_osc_init.HSEState            = RCC_HSE_OFF;
    rcc_osc_init.HSIState            = RCC_HSI_ON;
    rcc_osc_init.LSEState            = RCC_LSE_ON;
    rcc_osc_init.LSIState            = RCC_LSI_OFF;
    rcc_osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    rcc_osc_init.PLL.PLLState        = RCC_PLL_ON;
    rcc_osc_init.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    rcc_osc_init.PLL.PLLMUL          = RCC_PLLMUL_6;
    rcc_osc_init.PLL.PLLDIV          = RCC_PLLDIV_3;
    if( HAL_RCC_OscConfig( &rcc_osc_init ) != HAL_OK )
    {
    }

    // Initializes the CPU, AHB and APB busses clocks
    rcc_clk_init.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    rcc_clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
    rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

    if( HAL_RCC_ClockConfig( &rcc_clk_init, FLASH_LATENCY_1 ) != HAL_OK )
    {
    }

    periph_clk_init.PeriphClockSelection =
        RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_LPTIM1 | RCC_PERIPHCLK_USART1;
    periph_clk_init.LptimClockSelection  = RCC_LPTIM1CLKSOURCE_LSE;
    periph_clk_init.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
    periph_clk_init.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    periph_clk_init.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if( HAL_RCCEx_PeriphCLKConfig( &periph_clk_init ) != HAL_OK )
    {
    }

    bsp_mcu_start_systick( );
}

static void bsp_mcu_start_systick( void )
{
    // Configure the Systick interrupt time
    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );

    // Configure the Systick
    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

static void bsp_mcu_stop_systick( void )
{
    SysTick->CTRL = 0;
}

static void bsp_mcu_gpio_init( void )
{
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE( );
    __HAL_RCC_GPIOB_CLK_ENABLE( );
    __HAL_RCC_GPIOC_CLK_ENABLE( );
    __HAL_RCC_GPIOD_CLK_ENABLE( );
    __HAL_RCC_GPIOH_CLK_ENABLE( );

#if( BSP_HW_DEBUG_PROBE == BSP_FEATURE_ON )
    // Enable debug in sleep/stop/standby
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
#endif

    bsp_gpio_init_out( RADIO_NSS, 1 );
    bsp_gpio_init_in( RADIO_BUSY_PIN, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_OFF, NULL );
    // Here init only the pin as an exti rising and the callback will be attached later
    bsp_gpio_init_in( RADIO_DIOX, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_RISING, NULL );
    bsp_gpio_init_out( RADIO_NRST, 1 );

    bsp_gpio_init_out( RADIO_ANTENNA_SWITCH, 1 );
}

void HAL_MspInit( void )
{
    __HAL_RCC_SYSCFG_CLK_ENABLE( );
    __HAL_RCC_PWR_CLK_ENABLE( );

    // System interrupt init
    // SVC_IRQn interrupt configuration
    HAL_NVIC_SetPriority( SVC_IRQn, 0, 0 );
    // PendSV_IRQn interrupt configuration
    HAL_NVIC_SetPriority( PendSV_IRQn, 0, 0 );
    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

#if( BSP_LOW_POWER_MODE == BSP_FEATURE_ON )

/**
 * \brief Enters Low Power Stop Mode
 *
 * \note ARM exits the function when waking up
 */
static void bsp_lpm_enter_stop_mode( void )
{
    CRITICAL_SECTION_BEGIN( );

    bsp_mcu_deinit( );

    // Disable the Power Voltage Detector
    HAL_PWR_DisablePVD( );

    // Clear wake up flag
    SET_BIT( PWR->CR, PWR_CR_CWUF );

    // Enable Ultra low power mode
    HAL_PWREx_EnableUltraLowPower( );

    // Enable the fast wake up from Ultra low power mode
    HAL_PWREx_EnableFastWakeUp( );

    CRITICAL_SECTION_END( );

    // Enter Stop Mode
    HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
}

/*!
 * \brief Exists Low Power Stop Mode
 */
static void bsp_lpm_exit_stop_mode( void )
{
    // Disable IRQ while the MCU is not running on HSI
    CRITICAL_SECTION_BEGIN( );

    // Initializes the peripherals
    bsp_mcu_reinit( );

    CRITICAL_SECTION_END( );
}

/*!
 * \brief handler low power (TODO: put in a new smtc_bsp_lpm with option)
 */
static void bsp_low_power_handler( void )
{
    // first stop systick to avoid getting pending irq while going in stop mode
    bsp_mcu_stop_systick( );

    __disable_irq( );
    /*!
     * If an interrupt has occurred after __disable_irq( ), it is kept pending
     * and cortex will not enter low power anyway
     */

    bsp_lpm_enter_stop_mode( );
    bsp_lpm_exit_stop_mode( );

    __enable_irq( );

    // re start systick
    bsp_mcu_start_systick( );
}

static void bsp_mcu_deinit( void )
{
    bsp_spi_deinit( BSP_RADIO_SPI_ID );
    bsp_uart2_deinit( );
    bsp_uart1_deinit( );
}

static void bsp_mcu_reinit( void )
{
    // Reconfig needed OSC and PLL
    bsp_system_clock_re_config_after_stop( );

    // Initialize UART
#if( BSP_USE_PRINTF_UART == BSP_FEATURE_ON )
    bsp_uart1_init( );
    bsp_uart2_init( );
#endif

    // Initialize SPI
    bsp_spi_init( BSP_RADIO_SPI_ID, RADIO_SPI_MOSI, RADIO_SPI_MISO, RADIO_SPI_SCLK );
}

static void bsp_system_clock_re_config_after_stop( void )
{
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    // Enable HSI
    __HAL_RCC_HSI_CONFIG( RCC_HSI_ON );

    // Wait till HSI is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    {
    }

    // Enable PLL
    __HAL_RCC_PLL_ENABLE( );

    // Wait till PLL is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    {
    }

    // Select PLL as system clock source
    __HAL_RCC_SYSCLK_CONFIG( RCC_SYSCLKSOURCE_PLLCLK );

    // Wait till PLL is used as system clock source
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
    {
    }
}

#else  // ie BSP_LOW_POWER_MODE == BSP_FEATURE_OFF

static bool bsp_mcu_no_low_power_wait( const int32_t milliseconds )
{
    uint32_t start_time = bsp_rtc_get_time_ms( );

    while( bsp_rtc_get_time_ms( ) < ( start_time + milliseconds ) )
    {
        // interruptible wait for 10ms
        HAL_Delay( 10 );
        if( bsp_exit_wait == true )
        {
            // stop wait/lp function and return immediatly
            bsp_exit_wait = false;
            return true;
        }
    }
    return false;
}
#endif

#if( BSP_DBG_TRACE == BSP_FEATURE_ON )
static void vprint( const char* fmt, va_list argp )
{
    char string[BSP_PRINT_BUFFER_SIZE];
    if( 0 < vsprintf( string, fmt, argp ) )  // build string
    {
        bsp_uart2_tx( ( uint8_t* ) string, strlen( string ) );
    }
}
#endif

/* --- EOF ------------------------------------------------------------------ */
