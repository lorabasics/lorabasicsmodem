/*
Notice:
This software is provided exclusively for evaluation purposes. It is experimental and does not satisfy 
Semtech's software quality standards. 

All Rights Reserved. There is no license provided with this software.

Disclaimer:
“THE SEMTECH SOFTWARE AND THE DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR
IMPLIED WARRANTY OF ANY KIND FROM SEMTECH OR FROM ANY OTHER PERSON OR ENTITY, INCLUDING
WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, OR FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL SEMTECH BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.”
The content of this repository is provided under the GitHub Terms of Services.
Copyright Semtech Corporation 2021. All rights reserved, with the exception of files that
have an explicit license on top of their directory or in their open source code in which
case the later prevails.
*/
#include "sx126x-hal.h"

/*!
 * \brief Helper macro to create Interrupt objects only if the pin name is
 *        different from NC
 */
#define CreateDioPin( pinName, dio )                 \
            if( pinName == NC )                      \
            {                                        \
                dio = NULL;                          \
            }                                        \
            else                                     \
            {                                        \
                dio = new InterruptIn( pinName );    \
            }

/*!
 * \brief Helper macro to avoid duplicating code for setting dio pins parameters
 */
#define DioAssignCallback( dio, pinMode, callback )                    \
            if( dio != NULL )                                          \
            {                                                          \
                dio->mode( pinMode );                                  \
                dio->rise( this, static_cast <Trigger>( callback ) );  \
            }

/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 */
#define WaitOnBusy( )             while( BUSY == 1 ){ }

/*!
 * \brief Used to block execution to give enough time to Busy to go up
 *        in order to respect Tsw, see datasheet ยง8.3.1
 */
#define WaitOnCounter( )          for( uint8_t counter = 0; counter < 15; counter++ ) \
                                  {  __NOP( ); }

                                  
// This code handles cases where assert_param is undefined
#ifndef assert_param
#define assert_param( ... )
#endif

SX126xHal::SX126xHal( PinName mosi, PinName miso, PinName sclk, PinName nss,
                      PinName busy, PinName dio1, PinName dio2, PinName dio3, PinName rst,
                      PinName freqSel, PinName deviceSelect, PinName antSwPower, RadioCallbacks_t *callbacks )
        :   SX126x( callbacks ),
            RadioNss( nss ),
            RadioReset( rst ),
            BUSY( busy ),
            FreqSelect( freqSel ),
            DeviceSelect( deviceSelect ),
            antSwitchPower( antSwPower )
{
    CreateDioPin( dio1, DIO1 );
    CreateDioPin( dio2, DIO2 );
    CreateDioPin( dio3, DIO3 );
    RadioSpi = new SPI( mosi, miso, sclk );

    RadioNss = 1;
    RadioReset = 1;
}

SX126xHal::~SX126xHal( void )
{
    if( this->RadioSpi != NULL )
    {
        delete RadioSpi;
    }
    if( DIO1 != NULL )
    {
        delete DIO1;
    }
    if( DIO2 != NULL )
    {
        delete DIO2;
    }
    if( DIO3 != NULL )
    {
        delete DIO3;
    }
};

void SX126xHal::SpiInit( void )
{
    RadioNss = 1;
    RadioSpi->format( 8, 0 );
    RadioSpi->frequency( SX126x_SPI_FREQ_DEFAULT );

    wait( 0.1 );
}


void SX126xHal::IoIrqInit( DioIrqHandler irqHandler )
{
    assert_param( RadioSpi != NULL );
    if( RadioSpi != NULL )
    {
        SpiInit( );
    }

    BUSY.mode( PullNone );
    DioAssignCallback( DIO1, PullNone, irqHandler );
    DioAssignCallback( DIO2, PullNone, irqHandler );
    DioAssignCallback( DIO3, PullNone, irqHandler );
}

void SX126xHal::Reset( void )
{
    __disable_irq( );
    wait_ms( 20 );
    RadioReset.output( );
    RadioReset = 0;
    wait_ms( 50 );
    RadioReset = 1;
    RadioReset.input( ); // Using the internal pull-up
    wait_ms( 20 );
    __enable_irq( );
}

void SX126xHal::Wakeup( void )
{
    __disable_irq( );

    //Don't wait for BUSY here

    if( RadioSpi != NULL )
    {
        RadioNss = 0;
        RadioSpi->write( RADIO_GET_STATUS );
        RadioSpi->write( 0 );
        RadioNss = 1;
    }

    // Wait for chip to be ready.
    WaitOnBusy( );

    __enable_irq( );
    
    AntSwOn( );
}

void SX126xHal::WriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
#ifdef ADV_DEBUG
    printf("cmd: 0x%02x", command );
    for( uint8_t i = 0; i < size; i++ )
    {
        printf("-%02x", buffer[i] );
    }
    printf("\n\r");
#endif
    
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        RadioNss = 0;
        RadioSpi->write( ( uint8_t )command );
        for( uint16_t i = 0; i < size; i++ )
        {
            RadioSpi->write( buffer[i] );
        }
        RadioNss = 1;
    }
    WaitOnCounter( );
}

void SX126xHal::ReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        RadioNss = 0;
        RadioSpi->write( ( uint8_t )command );
        RadioSpi->write( 0 );
        for( uint16_t i = 0; i < size; i++ )
        {
             buffer[i] = RadioSpi->write( 0 );
        }
        RadioNss = 1;
    }
}

void SX126xHal::WriteRegister( uint16_t address, uint8_t *buffer, uint16_t size )
{
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        RadioNss = 0;
        RadioSpi->write( RADIO_WRITE_REGISTER );
        RadioSpi->write( ( address & 0xFF00 ) >> 8 );
        RadioSpi->write( address & 0x00FF );
        for( uint16_t i = 0; i < size; i++ )
        {
            RadioSpi->write( buffer[i] );
        }
        RadioNss = 1;
    }
}

void SX126xHal::WriteRegister( uint16_t address, uint8_t value )
{
    WriteRegister( address, &value, 1 );
}

void SX126xHal::ReadRegister( uint16_t address, uint8_t *buffer, uint16_t size )
{
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        RadioNss = 0;
        RadioSpi->write( RADIO_READ_REGISTER );
        RadioSpi->write( ( address & 0xFF00 ) >> 8 );
        RadioSpi->write( address & 0x00FF );
        RadioSpi->write( 0 );
        for( uint16_t i = 0; i < size; i++ )
        {
            buffer[i] = RadioSpi->write( 0 );
        }
        RadioNss = 1;
    }
}

uint8_t SX126xHal::ReadRegister( uint16_t address )
{
    uint8_t data;

    ReadRegister( address, &data, 1 );
    return data;
}

void SX126xHal::WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        RadioNss = 0;
        RadioSpi->write( RADIO_WRITE_BUFFER );
        RadioSpi->write( offset );
        for( uint16_t i = 0; i < size; i++ )
        {
            RadioSpi->write( buffer[i] );
        }
        RadioNss = 1;
    }
}

void SX126xHal::ReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    WaitOnBusy( );

    if( RadioSpi != NULL )
    {
        RadioNss = 0;
        RadioSpi->write( RADIO_READ_BUFFER );
        RadioSpi->write( offset );
        RadioSpi->write( 0 );
        for( uint16_t i = 0; i < size; i++ )
        {
            buffer[i] = RadioSpi->write( 0 );
        }
        RadioNss = 1;
    }
}

uint8_t SX126xHal::GetDioStatus( void )
{
    return ( *DIO3 << 3 ) | ( *DIO2 << 2 ) | ( *DIO1 << 1 ) | ( BUSY << 0 );
}

uint8_t SX126xHal::GetDeviceType( void )
{
    uint16_t val = 0;
    val = DeviceSelect.read_u16( );

    if( val <= 0x2000 )
    {
        return( SX1262 );
    }
    else if( val <= 0xA000 )
    {
        return( SX1268 );
    }
    else 
    {
        return( SX1261 );
    }
}

uint8_t SX126xHal::GetFreqSelect( void )
{
    uint16_t val = 0;
    val = FreqSelect.read_u16( );

    if( val < 100 )
    {
        return( MATCHING_FREQ_915 );
    }
    else if( val <= 0x3000 )
    {
        return( MATCHING_FREQ_780 );
    }
    else if( val <= 0x4900 )       // 0x4724
    {
        return( MATCHING_FREQ_490 );
    }
    else if( val <= 1 )
    {
        return( MATCHING_FREQ_434 );
    }
    else if( val <= 1 )
    {
        return( MATCHING_FREQ_280 );
    }
    else if( val <= 0xF000 )
    {
        return( MATCHING_FREQ_169 );
    }
    else
    {
        return( MATCHING_FREQ_868 );
    }
}

void SX126xHal::AntSwOn( void )
{
    antSwitchPower = 1;
}

void SX126xHal::AntSwOff( void )
{
    antSwitchPower = 0;
}
