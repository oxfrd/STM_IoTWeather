/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic SX1276 driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __SX1276_H__
#define __SX1276_H__

#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"

/*!
 * Radio wakeup time from SLEEP mode
 */
#define RADIO_WAKEUP_TIME                           1000 // [us]

/*!
 * Radio driver supported modems
 */
typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,
}RadioModems_t;

/*!
 * Radio driver internal state machine states definition
 */
typedef enum
{
    RF_IDLE = 0,
    RF_RX_RUNNING,
    RF_TX_RUNNING,
    RF_CAD,
}RadioState_t;

/*!
 * \brief Radio driver callback functions
 */
typedef struct
{
    /*!
     * \brief  Tx Done callback prototype.
     */
    void    ( *TxDone )( void );
    /*!
     * \brief  Tx Timeout callback prototype.
     */
    void    ( *TxTimeout )( void );
    /*!
     * \brief Rx Done callback prototype.
     *
     * \param [IN] payload Received buffer pointer
     * \param [IN] size    Received buffer size
     * \param [IN] rssi    RSSI value computed while receiving the frame [dBm]
     * \param [IN] snr     Raw SNR value given by the radio hardware
     *                     FSK : N/A ( set to 0 )
     *                     LoRa: SNR value in dB
     */
    void    ( *RxDone )( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
    /*!
     * \brief  Rx Timeout callback prototype.
     */
    void    ( *RxTimeout )( void );
    /*!
     * \brief Rx Error callback prototype.
     */
    void    ( *RxError )( void );
    /*!
     * \brief  FHSS Change Channel callback prototype.
     *
     * \param [IN] currentChannel   Index number of the current channel
     */
    void ( *FhssChangeChannel )( uint8_t currentChannel );

    /*!
     * \brief CAD Done callback prototype.
     *
     * \param [IN] channelDetected    Channel Activity detected during the CAD
     */
    void ( *CadDone ) ( bool channelActivityDetected );
}RadioEvents_t;

/*!
 * Radio FSK modem parameters
 */
typedef struct
{
    int8_t   Power;
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
}RadioFskSettings_t;

/*!
 * Radio FSK packet handler state
 */
typedef struct
{
    uint8_t  PreambleDetected;
    uint8_t  SyncWordDetected;
    int8_t   RssiValue;
    int32_t  AfcValue;
    uint8_t  RxGain;
    uint16_t Size;
    uint16_t NbBytes;
    uint8_t  FifoThresh;
    uint8_t  ChunkSize;
}RadioFskPacketHandler_t;

/*!
 * Radio LoRa modem parameters
 */
typedef struct
{
    int8_t   Power;
    uint32_t Bandwidth;
    uint32_t Datarate;
    bool     LowDatarateOptimize;
    uint8_t  Coderate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     FreqHopOn;
    uint8_t  HopPeriod;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
}RadioLoRaSettings_t;

/*!
 * Radio LoRa packet handler state
 */
typedef struct
{
    int8_t SnrValue;
    int16_t RssiValue;
    uint8_t Size;
}RadioLoRaPacketHandler_t;

/*!
 * Radio Settings
 */
typedef struct
{
    RadioState_t             State;
    RadioModems_t            Modem;
    uint32_t                 Channel;
    RadioFskSettings_t       Fsk;
    RadioFskPacketHandler_t  FskPacketHandler;
    RadioLoRaSettings_t      LoRa;
    RadioLoRaPacketHandler_t LoRaPacketHandler;
}RadioSettings_t;

/*!
 * Radio hardware and global parameters
 */
typedef struct SX1276_s
{
    Gpio_t        Reset;
    Gpio_t        DIO0;
    Gpio_t        DIO1;
    Gpio_t        DIO2;
    Gpio_t        DIO3;
    Gpio_t        DIO4;
    Gpio_t        DIO5;
    Spi_t         Spi;
    uint8_t       RxTx;
    RadioSettings_t Settings;

    /*!
    * Antenna switch GPIO pins objects
    */
    Gpio_t        AntSwitchLf;
    Gpio_t        AntSwitchHf;
}SX1276_t;

//extern SX1276_t SX1276;

/*!
 * Hardware IO IRQ callback function definition
 */
typedef void (DioIrqHandler)(void);

/*!
 * SX1276 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

#define RX_BUFFER_SIZE                              256

/*!
 * ============================================================================
 * Public functions prototypes
 * ============================================================================
 */

/*!
 * \brief Initializes the radio
 *
 * \param [IN] events Structure containing the driver callback functions
 */
void SX1276Init(RadioEvents_t *events);

/*!
 * Return current radio status
 *
 * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
RadioState_t SX1276GetStatus(void);

/*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
void SX1276SetModem(RadioModems_t modem);

/*!
 * \brief Sets the channels configuration
 *
 * \param [IN] freq         Channel RF frequency
 */
void SX1276SetChannel(uint32_t freq);

/*!
 * \brief Sets the channels configuration
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] freq       Channel RF frequency
 * \param [IN] rssiThresh RSSI threshold
 *
 * \retval isFree         [true: Channel is free, false: Channel is not free]
 */
bool SX1276IsChannelFree(RadioModems_t modem, uint32_t freq, int16_t rssiThresh);

/*!
 * \brief Generates a 32 bits random value based on the RSSI readings
 *
 * \remark This function sets the radio in LoRa modem mode and disables
 *         all interrupts.
 *         After calling this function either SX1276SetRxConfig or
 *         SX1276SetTxConfig functions must be called.
 *
 * \retval randomValue    32 bits random value
 */
uint32_t SX1276Random(void);

/*!
 * \brief Sets the reception parameters
 *
 * \remark When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A ( set to 0 )
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] symbTimeout  Sets the RxSingle timeout value (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: timeout in symbols
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed lenght is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols bewteen each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
void SX1276SetRxConfig(RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
                         bool iqInverted, bool rxContinuous);

/*!
 * \brief Sets the transmission parameters
 *
 * \remark When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] power        Sets the output power [dBm]
 * \param [IN] fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols bewteen each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] timeout      Transmission timeout [us]
 */
void SX1276SetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool FreqHopOn,
                        uint8_t HopPeriod, bool iqInverted, uint32_t timeout);

/*!
 * \brief Computes the packet time on air in us for the given payload
 *
 * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] pktLen     Packet payload length
 *
 * \retval airTime        Computed airTime (us) for the given packet payload length
 */
uint32_t SX1276GetTimeOnAir(RadioModems_t modem, uint8_t pktLen);

/*!
 * \brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void SX1276Send(uint8_t *buffer, uint8_t size);

/*!
 * \brief Sets the radio in sleep mode
 */
void SX1276SetSleep(void);

/*!
 * \brief Sets the radio in standby mode
 */
void SX1276SetStby(void);

/*!
 * \brief Sets the radio in reception mode for the given time
 * \param [IN] timeout Reception timeout [us] [0: continuous, others timeout]
 */
void SX1276SetRx(uint32_t timeout);

/*!
 * \brief Start a Channel Activity Detection
 */
void SX1276StartCad(void);

/*!
 * \brief Reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
int16_t SX1276ReadRssi(RadioModems_t modem);

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void SX1276Write(uint8_t addr, uint8_t data);

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \retval data Register value
 */
uint8_t SX1276Read(uint8_t addr);

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void SX1276WriteBuffer(uint8_t addr, uint8_t *buffer, uint8_t size);

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void SX1276ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size);




/*!
 * \brief Initializes the radio I/Os pins interface
 */
void SX1276IoInit(void);

/*!
 * \brief Initializes DIO IRQ handlers
 *
 * \param [IN] irqHandlers Array containing the IRQ callback functions
 */
void SX1276IoIrqInit(DioIrqHandler **irqHandlers);

/*!
 * \brief De-initializes the radio I/Os pins interface.
 *
 * \remark Useful when going in MCU lowpower modes
 */
void SX1276IoDeInit(void);

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] channel Channel frequency in Hz
 * \retval PaSelect RegPaConfig PaSelect value
 */
uint8_t SX1276GetPaSelect(uint32_t channel);

/*!
 * \brief Set the RF Switch I/Os pins in Low Power mode
 *
 * \param [IN] status enable or disable
 */
void SX1276SetAntSwLowPower(bool status);

/*!
 * \brief Initializes the RF Switch I/Os pins interface
 */
void SX1276AntSwInit(void);

/*!
 * \brief De-initializes the RF Switch I/Os pins interface
 *
 * \remark Needed to decrease the power consumption in MCU lowpower modes
 */
void SX1276AntSwDeInit(void);

/*!
 * \brief Controls the antena switch if necessary.
 *
 * \remark see errata note
 *
 * \param [IN] rxTx [1: Tx, 0: Rx]
 */
void SX1276SetAntSw(uint8_t rxTx);

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool SX1276CheckRfFrequency(uint32_t frequency);

#endif // __SX1276_H__
