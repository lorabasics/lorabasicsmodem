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
#ifndef __SX126x_HAL_H__
#define __SX126x_HAL_H__

#include "sx126x.h"

/*!
 * \brief The default value of SPI clock
 */
#define SX126x_SPI_FREQ_DEFAULT                  16000000 

/*!
 * \brief Actual implementation of a SX126x radio
 */
class SX126xHal : public SX126x
{
public:
    /*!
     * \brief Constructor for SX126xHal with SPI support
     *
     * Represents the physical connectivity with the radio and set callback functions on radio interrupts
     */
    SX126xHal( PinName mosi, PinName miso, PinName sclk, PinName nss,
               PinName busy, PinName dio1, PinName dio2, PinName dio3, PinName rst,
               PinName freqSel, PinName deviceSelect, PinName antSwPower, RadioCallbacks_t *callbacks );

    /*!
     * \brief Destructor for SX126xHal
     *
     * Take care of the correct destruction of the communication objects
     */
    virtual ~SX126xHal( void );

    /*!
     * \brief Soft resets the radio
     */
    virtual void Reset( void );

    /*!
     * \brief Wakes up the radio
     */
    virtual void Wakeup( void );

    /*!
     * \brief Send a command that write data to the radio
     *
     * \param [in]  opcode        Opcode of the command
     * \param [in]  buffer        Buffer to be send to the radio
     * \param [in]  size          Size of the buffer to send
     */
    virtual void WriteCommand( RadioCommands_t opcode, uint8_t *buffer, uint16_t size );

    /*!
     * \brief Send a command that read data from the radio
     *
     * \param [in]  opcode        Opcode of the command
     * \param [out] buffer        Buffer holding data from the radio
     * \param [in]  size          Size of the buffer
     */
    virtual void ReadCommand( RadioCommands_t opcode, uint8_t *buffer, uint16_t size );

    /*!
     * \brief Write data to the radio memory
     *
     * \param [in]  address       The address of the first byte to write in the radio
     * \param [in]  buffer        The data to be written in radio's memory
     * \param [in]  size          The number of bytes to write in radio's memory
     */
    virtual void WriteRegister( uint16_t address, uint8_t *buffer, uint16_t size );

    /*!
     * \brief Write a single byte of data to the radio memory
     *
     * \param [in]  address       The address of the first byte to write in the radio
     * \param [in]  value         The data to be written in radio's memory
     */
    virtual void WriteRegister( uint16_t address, uint8_t value );

    /*!
     * \brief Read data from the radio memory
     *
     * \param [in]  address       The address of the first byte to read from the radio
     * \param [out] buffer        The buffer that holds data read from radio
     * \param [in]  size          The number of bytes to read from radio's memory
     */
    virtual void ReadRegister( uint16_t address, uint8_t *buffer, uint16_t size );

    /*!
     * \brief Read a single byte of data from the radio memory
     *
     * \param [in]  address       The address of the first byte to write in the
     *                            radio
     *
     * \retval      value         The value of the byte at the given address in
     *                            radio's memory
     */
    virtual uint8_t ReadRegister( uint16_t address );

    /*!
     * \brief Write data to the buffer holding the payload in the radio
     *
     * \param [in]  offset        The offset to start writing the payload
     * \param [in]  buffer        The data to be written (the payload)
     * \param [in]  size          The number of byte to be written
     */
    virtual void WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );

    /*!
     * \brief Read data from the buffer holding the payload in the radio
     *
     * \param [in]  offset        The offset to start reading the payload
     * \param [out] buffer        A pointer to a buffer holding the data from the radio
     * \param [in]  size          The number of byte to be read
     */
    virtual void ReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );

    /*!
     * \brief Returns the status of DIOs pins
     *
     * \retval      dioStatus     A byte where each bit represents a DIO state:
     *                            [ DIO3 | DIO2 | DIO1 | BUSY ]
     */
    virtual uint8_t GetDioStatus( void );

    /*!
     * \brief Returns the device type
     *
     * \retval      0: SX1261, 1: SX1262, 2: SX1268
     */
    virtual uint8_t GetDeviceType( void );

    /*!
     * \brief Returns the matching frequency
     *
     * \retval      1: 868 MHz
     *              0: 915 MHz
     */
    virtual uint8_t GetFreqSelect( void );

    /*!
     * \brief RF Switch power on
     */
    virtual void AntSwOn( void );

    /*!
     * \brief RF Switch power off
     */
    virtual void AntSwOff( void );

protected:

    SPI *RadioSpi;                                  //!< The SPI object used to communicate with the radio
    DigitalOut RadioNss;                            //!< The pin connected to Radio chip select (active low)
    DigitalInOut RadioReset;                        //!< The reset pin connected to the radio

    DigitalIn    BUSY;                              //!< The pin connected to BUSY
    InterruptIn *DIO1;                              //!< The pin connected to DIO1
    InterruptIn *DIO2;                              //!< The pin connected to DIO2
    InterruptIn *DIO3;                              //!< The pin connected to DIO3
    AnalogIn     FreqSelect;                        //!< The pin connected to a pull-up/down to select Frequency of the matching
    AnalogIn     DeviceSelect;                      //!< The pin connected to a pull-up/down to select device
    DigitalOut   antSwitchPower;                    //!< The pin connected to the RF Switch Power

    /*!
     * \brief Initializes SPI object used to communicate with the radio
     */
    virtual void SpiInit( void );

    /*!
     * \brief Sets the callback functions to be run on DIO1..3 interrupt
     *
     * \param [in]  irqHandler    A function pointer of the function to be run on every DIO interrupt
     */
    virtual void IoIrqInit( DioIrqHandler irqHandler );
};

#endif // __SX126x_HAL_H__
