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
#if !FEATURE_LWIP
    #error [NOT_SUPPORTED] LWIP not supported for this target
#endif

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <assert.h>

#include "mbed.h"
#include "EthernetInterface.h"
#include "UDPSocket.h"
#include "mbedtls/base64.h"

#include "jsmn.h"

#include "loragw.h"
#include "userconf.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/*!
 * \brief Timing debug
 */
#define DEBUG_TIME 0
#define DEBUG_VERBOSE 0

#if DEBUG_TIME == 1
    #define DEBUG_TIME_START start_dbg = gateway_count_us.read_us();
    #define DEBUG_TIME_STOP   stop_dbg = gateway_count_us.read_us();
    #define DEBUG_TIME_PRINT(x) printf("__DBG: %s: %" PRIu32 "\n", x, stop_dbg - start_dbg);
static uint32_t start_dbg;
static uint32_t stop_dbg;
#else
    #define DEBUG_TIME_START
    #define DEBUG_TIME_STOP
    #define DEBUG_TIME_PRINT(x)
#endif

#if DEBUG_VERBOSE == 1
    #define DEBUG_MSG(str)                printf(str)
    #define DEBUG_PRINTF(fmt, args...)    printf(fmt, args)
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define PROTOCOL_VERSION    2           /* v1.3 */
#define PKT_PUSH_DATA   0
#define PKT_PUSH_ACK    1
#define PKT_PULL_DATA   2
#define PKT_PULL_RESP   3
#define PKT_PULL_ACK    4
#define PKT_TX_ACK      5

#define NB_PKT_MAX      1
#define STATUS_SIZE     200
#define TX_BUFF_SIZE    ((540 * NB_PKT_MAX) + 30 + STATUS_SIZE)

#define DEFAULT_KEEPALIVE   10.0

#define BUFFER_SIZE         255

#define _1HOUR 3600000000U
static uint8_t FAILSAFE_CNT = 0;

/*!
 * \brief Number of tick size steps for tx timeout
 */
#define TX_TIMEOUT_VALUE                            1000 // ms

/*!
 * \brief Number of tick size steps for rx timeout
 */
#define RX_TIMEOUT_VALUE                            100 // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_1000_US

/*!
 * \brief Mask of IRQs to listen to in rx mode
 */
uint16_t RxIrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;

/*!
 * \brief Mask of IRQs to listen to in tx mode
 */
uint16_t TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

static bool CrcError = false;
static bool NewPacketReceived = false;
static uint32_t NewPacketTimestamp = false;
static bool KeepAlive = true;
static bool TxDone = false;
static bool AntennaSwitch = false;

static uint8_t tx_size = 0;
static uint8_t tx_data[255];

static uint32_t tx_freq = 0;
static RadioLoRaSpreadingFactors_t tx_dr;
static char sf_str[10];

static uint8_t BufferSize = BUFFER_SIZE;
static uint8_t Buffer[BUFFER_SIZE];

static char buff_up[512];
static uint8_t buff_req[12]; /* buffer to compose pull requests */
static uint8_t buff_down[1000]; /* buffer to receive acknowledges and downlinks */

static uint32_t meas_nb_rx_rcv = 0; /* count packets received */
static uint32_t meas_nb_rx_ok = 0; /* count packets received with PAYLOAD CRC OK */
static uint32_t meas_nb_rx_bad = 0; /* count packets received with PAYLOAD CRC ERROR */
static uint32_t meas_nb_tx_ok = 0; /* count packets emitted successfully */

/* Concentrator counter */
static Timer gateway_count_us;

/* JSON parser */
jsmn_parser p;
jsmntok_t t[128]; /* We expect no more than 128 tokens */

/* HW Watchdog */
IWDG_HandleTypeDef hiwdg;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( IrqErrorCode_t );

/*!
 * \brief All the callbacks are stored in a structure
 */
RadioCallbacks_t callbacks =
{
    &OnTxDone,        // txDone
    &OnRxDone,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout,     // txTimeout
    &OnRxTimeout,     // rxTimeout
    &OnRxError,       // rxError
    NULL,             // rxPreambleDetect
    NULL,             // cadDone
};

void WatchDogStart ( void );

void WatchDogRelease ( void );

static void baud( int baudrate );

static int init_gateway( void );

static int sf_enum2nb(RadioLoRaSpreadingFactors_t sf);

static RadioLoRaSpreadingFactors_t sf_nb2enum(int nb);

static int bw_enum2nb(RadioLoRaBandwidths_t bw);

static int configure_gateway( bool ipol, uint8_t size, RadioLoRaSpreadingFactors_t sf );

static void keep_alive( );

static void tx_trigger_callback( );

static int jsoneq(const char *json, jsmntok_t *tok, const char *s);

/* -------------------------------------------------------------------------- */
/* --- HW CONFIGURATION ----------------------------------------------------- */

// mosi, miso, sclk, nss, busy, dio1, dio2, dio3, rst, freqSel, deviceSelect, antSwPower, callbacks...
SX126xHal Radio( D11, D12, D13, D7, D3, D5, NC, NC, A0, A1, A2, D8, &callbacks );

DigitalOut ANT_SW( D8 );
DigitalOut TxLed( A4 );
DigitalOut RxLed( A5 );

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main( void )
{
    WatchDogStart( );

    int ret, i, j;

    PacketStatus_t PacketStatus;

    /* data buffers */
    int buff_index = 0;

    /* protocol variables */
    uint8_t token_h; /* random token for acknowledgement matching */
    uint8_t token_l; /* random token for acknowledgement matching */
    /* gateway <-> MAC protocol variables */
    uint8_t mac_address[6];

    /* local timekeeping variables */
    Timeout timeout_keepalive;

    /* Concentrator counter */
    Timeout tx_trigger;
    uint32_t current_count = 0;
    uint32_t Failsafe_count_init = 0;

    /* mote info variables */
    uint32_t mote_addr = 0;
    uint16_t mote_fcnt = 0;

    /* network/socket variables */
    EthernetInterface eth;
    UDPSocket td_sock_up;
    UDPSocket td_sock_down;
    SocketAddress td_addr_up( gw_conf.server_address, gw_conf.server_port );
    SocketAddress td_addr_down( gw_conf.server_address, gw_conf.server_port );
    uint32_t net_mac_h; /* Most Significant Nibble, network order */
    uint32_t net_mac_l; /* Least Significant Nibble, network order */

    baud( 921600 );

    printf( "\n*********************************************\n" );
    printf(   "* LoRa Edge Hybrid Single Channel Gateway *\n" );
    printf(   "*********************************************\n" );

    TxLed = 0;
    RxLed = 0;

    printf( "Trying to get an IP address from DHCP server...\n" );

    /* Ethernet/UDP socket setup */
    if( eth.connect( ) != 0 )
    {
        printf( "ERROR: failed to connect to the Ethernet Interface\n" );
        NVIC_SystemReset();
        return -1; // Not Reach
    }

    printf( "The gateway IP address is '%s' (MAC: %s)\n", eth.get_ip_address(), eth.get_mac_address() );
    printf( "            netmask is '%s'\n", eth.get_netmask() );
    printf( "            gateway is '%s'\n", eth.get_gateway() );

    /* UDP uplink socket to network server */
    if( td_sock_up.open( &eth ) != 0 )
    {
        printf( "ERROR: failed to open UDP socket for Network Server\n" );
        NVIC_SystemReset();
        return -1; // Not Reach
    }
    td_sock_up.set_timeout( 100 ); /* milliseconds */
    printf( "Socket to the Network Server opened and configured\n" );

    /* UDP downlink socket to network server */
    if( td_sock_down.open( &eth ) != 0 )
    {
        printf( "ERROR: failed to open UDP socket for Network Server\n" );
        NVIC_SystemReset();
        return -1; // Not Reach
    }
    td_sock_down.set_timeout( 100 ); /* milliseconds */
    printf( "Socket to the Network Server opened and configured\n" );

    /* Select the channel to be used for this gateway based on its MAC address */
    for( i = 0; i < GW_FREQ_CONF_NB; i++ ) {
        //printf( "%s %s\n", eth.get_mac_address(), gw_freq_conf[i].mac_addr );
        if (strncasecmp(eth.get_mac_address(), gw_freq_conf[i].mac_addr, 17) == 0) {
            printf( "INFO: select channel %" PRIu32 " for this Gateway\n", gw_freq_conf[i].channel_freq );
            gw_conf.channel_freq = gw_freq_conf[i].channel_freq;
            break; /* we have found it, exit */
        }
    }
    if( i == GW_FREQ_CONF_NB ) {
        printf("WARNING: UNKNOWN Gateway, selecting default channel %" PRIu32 "\n", gw_conf.channel_freq );
    }

    /* Prepare for receiving LoRa packets */
    gateway_count_us.reset();
    gateway_count_us.start();
    init_gateway( );
    configure_gateway( false, 0, gw_conf.sf );

    /* Init Antenna Switch */
    ANT_SW = 1;

    /* Set RX mode */
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRxBoosted( ( TickTime_t ) { (RadioTickSizes_t)0xFF, 0xFFFF } ); /* RX continuous */

    /* pre-fill the data buffer with fixed fields */
    sscanf( eth.get_mac_address(), "%" PRIx8 ":%" PRIx8 ":%" PRIx8 ":%" PRIx8 ":%" PRIx8 ":%" PRIx8,
            &mac_address[0],
            &mac_address[1],
            &mac_address[2],
            &mac_address[3],
            &mac_address[4],
            &mac_address[5] );
    printf( "Parsed MAC is %02X:%02X:%02X:%02X:%02X:%02X\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5] );
    net_mac_h = (lwip_htonl( (mac_address[0] << 24) | (mac_address[1] << 16) | (mac_address[2] << 8) | (0xFF << 0) )); /* network byte order */
    net_mac_l = (lwip_htonl( (0xFE << 24) | (mac_address[3] << 16) | (mac_address[4] << 8) | (mac_address[5] << 0) )); /* network byte order */
    printf( "Gateway_ID: %02X%02X%02XFFFE%02X%02X%02X\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5] );
    buff_up[0] = PROTOCOL_VERSION;
    buff_up[3] = PKT_PUSH_DATA;
    *(uint32_t *)(buff_up + 4) = net_mac_h;
    *(uint32_t *)(buff_up + 8) = net_mac_l;

    /* Main loop */
    Failsafe_count_init = gateway_count_us.read_us();
    while( 1 )
    {
        WatchDogRelease ();
        if ( ( gateway_count_us.read_us() - Failsafe_count_init ) > _1HOUR )
        {
            FAILSAFE_CNT += 1;
            Failsafe_count_init = gateway_count_us.read_us();
            printf( "FAILSAFE_CNT:%u\n", FAILSAFE_CNT );
            if ( FAILSAFE_CNT >= 6 ) /* every 6 hours */
            {
                printf( "Failsafe RESET\n" );
                NVIC_SystemReset();
            }
        }
        if ( TxDone == true )
        {
DEBUG_TIME_START;
            /* Go back to RX after a downlink */
            TxDone = false;
            configure_gateway( false, 0, gw_conf.sf );
            Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
            Radio.SetRxBoosted( ( TickTime_t ) { (RadioTickSizes_t) 0xFF, 0xFFFF } ); /* RX continuous */
DEBUG_TIME_STOP;
DEBUG_TIME_PRINT("TX->RX");
        }
        if ( NewPacketReceived == true )
        {
            if( gw_conf.antenna_switch_alternate == true ) {
                /* Switch antenna for each packet received */
                AntennaSwitch = !AntennaSwitch;
                ANT_SW = (AntennaSwitch == true) ? 1 : 0;
                printf( "Set Antenna Switch to %d\n", (AntennaSwitch == true) ? 1 : 0 );
            }

            /* Get received packet data/information */
            Radio.GetPacketStatus( &PacketStatus );
            memset( Buffer, 0, sizeof Buffer );
            Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
            if( CrcError == false )
            {
                meas_nb_rx_ok += 1;
                DEBUG_MSG( "[CRC OK] " );
            }
            else
            {
                meas_nb_rx_bad += 1;
                DEBUG_MSG( "\n[CRC ERR] " );
            }
            DEBUG_PRINTF( "Received packet (size:%d) with rssi: %d; snr: %d\n", BufferSize, PacketStatus.LoRa.RssiPkt, PacketStatus.LoRa.SnrPkt );

            /* update GW status report */
            meas_nb_rx_rcv += 1;

            if( CrcError == true ) {
                DEBUG_MSG( "\n" );
                /* update status */
                NewPacketReceived = false;
                CrcError = false;
                RxLed = 0;
                continue;
            }

            /* Get mote information from current packet (addr, fcnt) */
            /* FHDR - DevAddr */
            mote_addr  = Buffer[1];
            mote_addr |= Buffer[2] << 8;
            mote_addr |= Buffer[3] << 16;
            mote_addr |= Buffer[4] << 24;
            /* FHDR - FCnt */
            mote_fcnt  = Buffer[6];
            mote_fcnt |= Buffer[7] << 8;

            printf( "\nINFO: Received pkt from mote: %08" PRIx32" (fcnt=%" PRId16 ")\n", mote_addr, mote_fcnt );

            /* start composing datagram with the header */
            token_h = (uint8_t)rand(); /* random token */
            token_l = (uint8_t)rand(); /* random token */
            buff_up[1] = token_h;
            buff_up[2] = token_l;
            buff_index = 12; /* 12-byte header */

DEBUG_TIME_START;
            /* start of JSON structure */
            memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[", 9);
            buff_index += 9;

            /* Start of packet, add inter-packet separator if necessary */
            buff_up[buff_index] = '{';
            ++buff_index;

           /* RAW timestamp, 8-17 useful chars */
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, "\"tmst\":%" PRIu32, NewPacketTimestamp);
            if (j > 0) {
                buff_index += j;
            } else {
                printf("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
            }

            /* Packet concentrator channel, RF chain & RX frequency, 34-36 useful chars */

            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"chan\":%1u,\"rfch\":%1u,\"freq\":%.6lf", 0, 0, gw_conf.channel_freq / 1e6);

            if (j > 0) {
                buff_index += j;
            } else {
                printf("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
            }

            /* Packet status, 9-10 useful chars */
            if( CrcError == false )
            {
                memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
                buff_index += 9;
            }
            else
            {
                memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":-1", 10);
                buff_index += 10;
            }

            /* Packet modulation */
            memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
            buff_index += 14;

            /* Lora datarate */
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"datr\":\"SF%uBW%d\"", sf_enum2nb(gw_conf.sf), bw_enum2nb(gw_conf.bw));

            if (j > 0) {
                buff_index += j;
            } else {
                printf("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
            }

            /* Packet ECC coding rate, 11-13 useful chars */

            memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/5\"", 13);

            buff_index += 13;

            /* Lora SNR */
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"lsnr\":%.1f", (float)PacketStatus.LoRa.SnrPkt);
            if (j > 0) {
                buff_index += j;
            } else {
                printf("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
            }

            /* Packet RSSI, payload size */
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"rssi\":%.0f,\"size\":%u", (float)PacketStatus.LoRa.RssiPkt, BufferSize);
            if (j > 0) {
                buff_index += j;
            } else {
                printf("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
            }

            /* Packet base64-encoded payload, 14-350 useful chars */
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"data\":\"");
            if (j > 0) {
                buff_index += j;
            } else {
                printf("ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
            }
            mbedtls_base64_encode( (unsigned char *)(buff_up + buff_index), sizeof buff_up, (size_t *)&j, Buffer, BufferSize );
            if (j > 0) {
                buff_index += j;
            } else {
                printf("ERROR: [up] base64 encode failed\n");
            }
            /* End of base64 string */
            buff_up[buff_index] = '"';
            ++buff_index;

            /* End of packet serialization */
            buff_up[buff_index] = '}';
            ++buff_index;

            /* end of packet array */
            buff_up[buff_index] = ']';
            ++buff_index;

            /* end of JSON datagram payload */
            buff_up[buff_index] = '}';
            ++buff_index;
            buff_up[buff_index] = 0; /* add string terminator, for safety */
DEBUG_TIME_STOP;
DEBUG_TIME_PRINT("Build rxpk");

            /* update status */
            NewPacketReceived = false;
            CrcError = false;
            RxLed = 0;

            printf("\nJSON up at %d: %s\n", gateway_count_us.read_us(), (char *)(buff_up + 12)); /* DEBUG: display JSON payload */

            /* Send packet over UDP socket to the Network Server*/
            printf("[NS] [UP] PUSH_DATA to %s:%d\n", gw_conf.server_address, gw_conf.server_port );
DEBUG_TIME_START;
            ret = td_sock_up.sendto(td_addr_up, &buff_up, buff_index);
DEBUG_TIME_STOP;
DEBUG_TIME_PRINT("[NS] [UP] PUSH_DATA UDP sendto");
            if (ret < 0)
            {
                printf("[NS] PUSH_DATA failed (%d)\n", ret);
            }

DEBUG_TIME_START;
            /* Try to receive a datagram from the Network Server uplink socket */
            memset(buff_down, 0, sizeof buff_down);
            ret = td_sock_up.recvfrom( NULL, &buff_down, sizeof buff_down );
DEBUG_TIME_STOP;
DEBUG_TIME_PRINT("[NS] [UP] UDP recvfrom");
            if( ret == NSAPI_ERROR_WOULD_BLOCK )
            {
                /* No data, do nothing */
            }
            else if( ret < 0 )
            {
                printf("[NS] [UP] Failed to read data from socket (%d)\n", ret);
            }
            else
            {
                /* Received datagram */
#if 0
                printf("[NS] [UP] Received datagram with size %d\n", ret);
                for( i = 0; i < ret; i++)
                {
                    printf("%02X ", buff_down[i]);
                }
                printf("\n");
#endif
                if ((ret < 4) || (buff_down[0] != PROTOCOL_VERSION) )
                {
                    printf( "[NS] [UP] WARNING: [up] ignored invalid packet\n" );
                    for( i = 0; i < ret; i++ )
                    {
                        printf( "%02X ", buff_down[i] );
                    }
                    printf("\n");
                }
                else
                {
                    switch( buff_down[3] )
                    {
                        case PKT_PUSH_ACK:
                            printf("[NS] [UP] PUSH_ACK received\n");
                            break;
                        default:
                            printf("** ERROR [UP]: PUSH_ACK was expected, received 0x%02X\n", buff_down[3] );
                            break;
                    }
                }
            }
        }
        else
        {
            if( KeepAlive == true )
            {
                /* Send PULL_DATA for keep_alive */
                buff_req[0] = PROTOCOL_VERSION;
                buff_req[3] = PKT_PULL_DATA;
                *(uint32_t *)(buff_req + 4) = net_mac_h;
                *(uint32_t *)(buff_req + 8) = net_mac_l;
                token_h = (uint8_t)rand(); /* random token */
                token_l = (uint8_t)rand(); /* random token */
                buff_req[1] = token_h;
                buff_req[2] = token_l;

                printf("[NS] [DN] PULL_DATA to %s:%d\n", gw_conf.server_address, gw_conf.server_port );
DEBUG_TIME_START;
                ret = td_sock_down.sendto(td_addr_down, &buff_req, sizeof buff_req);
DEBUG_TIME_STOP;
DEBUG_TIME_PRINT("[NS] [DN] PULL_DATA UDP sendto");
                if (ret < 0)
                {
                    printf("[NS] [DN] PULL_DATA failed (%d)\n", ret);
                    NVIC_SystemReset();
                }

                /* Program next keep alive in 30 seconds */
                KeepAlive = false;
                timeout_keepalive.attach( &keep_alive, DEFAULT_KEEPALIVE );
            }
            else
            {
DEBUG_TIME_START;
                /* Try ot receive a datagram from the Network Server from downlink socket */
                memset(buff_down, 0, sizeof buff_down);
                ret = td_sock_down.recvfrom( NULL, &buff_down, sizeof buff_down );
DEBUG_TIME_STOP;
DEBUG_TIME_PRINT("[NS] [DN] UDP recvfrom");
                if( ret == NSAPI_ERROR_WOULD_BLOCK )
                {
                    /* No data, do nothing */
                }
                else if( ret < 0 )
                {
                    printf("[NS] [DN] Failed to read data from socket (%d)\n", ret);
                }
                else
                {
                    /* Received datagram */
#if 0
                    printf("[NS] [DN] Received datagram with size %d\n", ret);
                    for( i = 0; i < ret; i++)
                    {
                        printf("%02X ", buff_down[i]);
                    }
                    printf("\n");
#endif
                    if ((ret < 4) || (buff_down[0] != PROTOCOL_VERSION) )
                    {
                        printf( "[NS] [DN]  WARNING: ignored invalid packet\n" );
                        for( i = 0; i < ret; i++ )
                        {
                            printf( "%02X ", buff_down[i] );
                        }
                        printf("\n");
                    }
                    else
                    {
                        switch( buff_down[3] )
                        {
                            case PKT_PULL_ACK:
                                printf("[NS] [DN] PULL_ACK received\n");
                                break;
                            case PKT_PULL_RESP:
                                printf("[NS] [DN] PULL_RESP received\n");
                                {
                                    bool send_immediate = false;
                                    uint32_t count_us = 0;

                                    /* JSMN variables */
                                    int r;
                                    char * JSON_STRING;

                                    /* Base64 variables */
                                    size_t olen = 0;

                                    printf("\nJSON down at %d: %s\n", gateway_count_us.read_us(), (char *)(buff_down + 4));

DEBUG_TIME_START;
                                    /* Parse the JSON string */
                                    JSON_STRING = (char *)(buff_down + 4);
                                    jsmn_init(&p);
                                    r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), t, sizeof(t)/sizeof(t[0]));
                                    if (r < 0) {
                                        printf("Failed to parse JSON: %d\n", r);
                                        break;
                                    }
                                    printf("found %d tokens\n", r);

                                    /* Assume the top-level element is a txpk object */
                                    if (r < 1 || t[0].type != JSMN_OBJECT || jsoneq(JSON_STRING, &t[1], "txpk")) {
                                        printf("ERROR: txpk object expected\n");
                                        break;
                                    }

                                    for (i = 2; i < r; i++) {
                                        if (jsoneq(JSON_STRING, &t[i], "imme") == 0) {
                                            if (strncmp(JSON_STRING + t[i+1].start, "true", t[i+1].end-t[i+1].start) == 0) {
                                                /* TX procedure: send immediately */
                                                send_immediate = true;
                                                printf("INFO: [down] a packet will be sent in \"immediate\" mode\n");
                                            } else {
                                                send_immediate = false;
                                            }
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "tmst") == 0) {
                                            /* TX procedure: send on timestamp value */
                                            count_us = (uint32_t)strtoul(JSON_STRING + t[i+1].start, NULL, 10);
                                            printf("INFO: [down] a packet will be sent on timestamp value %" PRIu32 " and current is %d\n", count_us, gateway_count_us.read_us());
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "tmms") == 0) {
#if 0
                                            DEBUG_PRINTF("IGNORED: %.*s: %.*s\n",
                                                        t[i].end-t[i].start, JSON_STRING + t[i].start,
                                                        t[i+1].end-t[i+1].start, JSON_STRING + t[i+1].start);
#endif
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "freq") == 0) {
                                            tx_freq = (uint32_t)((double)(1.0e6) * atof(JSON_STRING + t[i+1].start));
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "rfch") == 0) {
#if 0
                                            DEBUG_PRINTF("IGNORED: %.*s: %.*s\n",
                                                        t[i].end-t[i].start, JSON_STRING + t[i].start,
                                                        t[i+1].end-t[i+1].start, JSON_STRING + t[i+1].start);
#endif
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "powe") == 0) {
#if 0
                                            DEBUG_PRINTF("IGNORED: %.*s: %.*s\n",
                                                        t[i].end-t[i].start, JSON_STRING + t[i].start,
                                                        t[i+1].end-t[i+1].start, JSON_STRING + t[i+1].start);
#endif
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "modu") == 0) {
#if 0
                                            DEBUG_PRINTF("IGNORED: %.*s: %.*s\n",
                                                        t[i].end-t[i].start, JSON_STRING + t[i].start,
                                                        t[i+1].end-t[i+1].start, JSON_STRING + t[i+1].start);
#endif
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "datr") == 0) {
                                            strncpy(sf_str, JSON_STRING + t[i+1].start + 2, t[i+1].end-t[i+1].start);
                                            if (sf_str[0] == '1') { /* SF >= 10, 2 chars */
                                                sf_str[2] = '\0';
                                            } else {
                                                sf_str[1] = '\0';
                                            }
                                            tx_dr = sf_nb2enum(atoi(sf_str));
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "codr") == 0) {
#if 0
                                            DEBUG_PRINTF("IGNORED: %.*s: %.*s\n",
                                                        t[i].end-t[i].start, JSON_STRING + t[i].start,
                                                        t[i+1].end-t[i+1].start, JSON_STRING + t[i+1].start);
#endif
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "ipol") == 0) {
#if 0
                                            DEBUG_PRINTF("IGNORED: %.*s: %.*s\n",
                                                        t[i].end-t[i].start, JSON_STRING + t[i].start,
                                                        t[i+1].end-t[i+1].start, JSON_STRING + t[i+1].start);
#endif
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "prea") == 0) {
#if 0
                                            DEBUG_PRINTF("IGNORED: %.*s: %.*s\n",
                                                        t[i].end-t[i].start, JSON_STRING + t[i].start,
                                                        t[i+1].end-t[i+1].start, JSON_STRING + t[i+1].start);
#endif
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "size") == 0) {
                                            tx_size = (uint8_t)strtoul(JSON_STRING + t[i+1].start, NULL, 10);
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "data") == 0) {
                                            if( mbedtls_base64_decode(tx_data, sizeof tx_data, &olen, (const unsigned char *)(JSON_STRING + t[i+1].start), t[i+1].end-t[i+1].start) == 0 )
                                            {
                                                for( int k=0; k<(int)olen; k++ )
                                                {
                                                    DEBUG_PRINTF("%02X ", tx_data[k] );
                                                }
                                                DEBUG_MSG("\n");
                                            }
                                            else
                                            {
                                                printf("ERROR: failed to parse \"txpk.data\" (got:%d expected:%d)\n", olen, t[i+1].end-t[i+1].start);
                                            }
                                            if (olen != tx_size) {
                                                printf("WARNING: [down] mismatch between .size (%u) and .data size (%u) once converter to binary\n", tx_size, olen);
                                            }
                                            i++;
                                        } else if (jsoneq(JSON_STRING, &t[i], "ncrc") == 0) {
#if 0
                                            DEBUG_PRINTF("IGNORED: %.*s: %.*s\n",
                                                        t[i].end-t[i].start, JSON_STRING + t[i].start,
                                                        t[i+1].end-t[i+1].start, JSON_STRING + t[i+1].start);
#endif
                                            i++;
                                        }
                                    }
DEBUG_TIME_STOP;
DEBUG_TIME_PRINT("Parse txpk");
                                    /* Send downlink */
                                    if( tx_size > 0 )
                                    {
                                        uint32_t tx_count = 0;
                                        if( send_immediate == true )
                                        {
                                            tx_count = 1000; /* 1ms later */
                                            tx_trigger.attach_us( &tx_trigger_callback, tx_count );
                                            printf( "TX: callback called in %" PRIu32 "us\n", tx_count );
                                        }
                                        else
                                        {
                                            current_count = (uint32_t)gateway_count_us.read_us();
                                            tx_count = count_us - current_count;
                                            /* TODO: handle counter rollover */
                                            if (current_count < count_us) {
                                                tx_trigger.attach_us( &tx_trigger_callback, tx_count );
                                                printf( "TX: callback called in %" PRIu32 "us (count_us:%" PRIu32 ", current_count:%" PRIu32 ")\n", tx_count, count_us, current_count );
                                            } else {
                                                printf("ERROR: TOO LATE TO SEND DOWNLINK, SKIP\n");
                                                NVIC_SystemReset();
                                            }
                                        }
                                    }
                                }
                                break;
                            default:
                                printf(" ERROR [DN]: Received UNKNOWN datagram 0x%02X\n", buff_down[3] );
                                break;
                        }
                    }
                }
            }
        }
    }

    eth.disconnect();

    return 0;
}

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/*!
 * Watch Dog Init And start with a period befor ereset set to 32 seconds
*/
void WatchDogStart ( void ) {
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    //hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    hiwdg.Init.Reload = 0xFFF;

    /* Enable IWDG. LSI is turned on automaticaly */
    __HAL_IWDG_START(&hiwdg);

    IWDG_ENABLE_WRITE_ACCESS(&hiwdg);
    /* Write to IWDG registers the Prescaler & Reload values to work with */
    (&hiwdg)->Instance->PR = hiwdg.Init.Prescaler;
    (&hiwdg)->Instance->RLR = hiwdg.Init.Reload;
    /* Check pending flag, if previous update not done, return timeout */

    /* Wait for register to be updated */
    while((&hiwdg)->Instance->SR != RESET) {
    }
    __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
    /* Return function status */
}

void WatchDogRelease ( void ) {
    __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
};

/*!
 * \brief Specify serial datarate for UART debug output
 */
static void baud( int baudrate )
{
    Serial s( USBTX, USBRX );
    s.baud( baudrate );
}

static int sf_enum2nb(RadioLoRaSpreadingFactors_t sf)
{
    switch( sf )
    {
        case LORA_SF5:
            return 5;
        case LORA_SF6:
            return 6;
        case LORA_SF7:
            return 7;
        case LORA_SF8:
            return 8;
        case LORA_SF9:
            return 9;
        case LORA_SF10:
            return 10;
        case LORA_SF11:
            return 11;
        case LORA_SF12:
            return 12;
        default:
            return -1;
    }
}

static RadioLoRaSpreadingFactors_t sf_nb2enum(int nb)
{
    switch( nb )
    {
        case 5:
            return LORA_SF5;
        case 6:
            return LORA_SF6;
        case 7:
            return LORA_SF7;
        case 8:
            return LORA_SF8;
        case 9:
            return LORA_SF9;
        case 10:
            return LORA_SF10;
        case 11:
            return LORA_SF11;
        case 12:
            return LORA_SF12;
        default:
            printf("ERROR: wrong SF number: %d\n", nb);
            return LORA_SF12;
    }
}

static int bw_enum2nb(RadioLoRaBandwidths_t bw)
{
    switch( bw )
    {
        case LORA_BW_500:
            return 500;
        case LORA_BW_250:
            return 250;
        case LORA_BW_125:
            return 125;
        default:
            return -1;
    }
}

/* Gateway radio initialization */
static int init_gateway( void )
{
    Radio.Init( );
    Radio.SetRegulatorMode( USE_LDO ); // Can also be set in LDO mode but consume more power
    Radio.SetStandby( STDBY_RC );
    Radio.SetPacketType( PACKET_TYPE_LORA );
    Radio.SetBufferBaseAddresses( 0x00, 0x00 );
    return 0;
}

/* Configure gateway modulation parameters */
static int configure_gateway( bool ipol, uint8_t size, RadioLoRaSpreadingFactors_t sf )
{
    PacketParams_t PacketParams;
    ModulationParams_t modulationParams;

    modulationParams.PacketType                  = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = sf;
    modulationParams.Params.LoRa.Bandwidth       = gw_conf.bw;
    modulationParams.Params.LoRa.CodingRate      = LORA_CR_4_5;

    PacketParams.PacketType                 = PACKET_TYPE_LORA;
    PacketParams.Params.LoRa.PreambleLength = 0x08;
    PacketParams.Params.LoRa.HeaderType     = LORA_PACKET_VARIABLE_LENGTH;
    PacketParams.Params.LoRa.PayloadLength  = size;
    PacketParams.Params.LoRa.Crc            = LORA_CRC_ON;
    PacketParams.Params.LoRa.InvertIQ       = (ipol == false) ? LORA_IQ_NORMAL : LORA_IQ_INVERTED;

    Radio.SetModulationParams( &modulationParams );
    Radio.SetPacketParams( &PacketParams );

    Radio.SetRfFrequency( gw_conf.channel_freq );

    if( ipol == false )
    {
        printf( "Configuring Gateway for RX: Freq:%luHz SF%d BW%d\n", gw_conf.channel_freq, sf_enum2nb(sf), bw_enum2nb(gw_conf.bw) );
    }
    else
    {
        printf( "Configuring Gateway for TX: SF%d BW%d\n", sf_enum2nb(sf), bw_enum2nb(gw_conf.bw) );
    }

    return 0;
}

/* Callback for the Keep Alive timer */
static void keep_alive( )
{
    KeepAlive = true;
}

/* Callback for the TX trigger */
static void tx_trigger_callback( )
{
    TxLed = 1;
    configure_gateway( true, tx_size, tx_dr );
    Radio.SetRfFrequency( tx_freq );
    Radio.SetTxParams( gw_conf.tx_power, RADIO_RAMP_200_US );
    Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SendPayload( tx_data, tx_size, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );

    /* Update status report */
    meas_nb_tx_ok += 1;
}


/* JSON parsing helper */
static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
    if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}

void OnTxDone( void )
{
    TxLed = 0;
    TxDone = true;
}

void OnRxDone( void )
{
    if (NewPacketReceived == false)
    {
        RxLed = 1;
        NewPacketTimestamp = (uint32_t)(gateway_count_us.read_us());
        NewPacketReceived = true;
    }
}

void OnTxTimeout( void )
{
}

void OnRxTimeout( void )
{
}

void OnRxError( IrqErrorCode_t errorCode )
{
    if (errorCode == IRQ_CRC_ERROR_CODE )
    {
        if (NewPacketReceived == false)
        {
            RxLed = 1;
            NewPacketReceived = true;
            CrcError = true;
        }
    }
}
