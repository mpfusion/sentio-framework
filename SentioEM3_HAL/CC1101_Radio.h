/*
 *  CC1101_Radio.h
 *
 *  Created on: 10/01/2013
 *      Author: Linlin, Sebastian
 */

#ifndef CC1101_RADIO_H_
#define CC1101_RADIO_H_

#include "SystemConfig.h"
#include "efm32_emu.h"
#include "efm32_cmu.h"
#include "efm32_usart.h"
#include "efm32_gpio.h"

#include "DebugInterface.h"
#include "Statemachine.h"
#include "System.h"

#define _CC1101_USART           USART0
#define _CC1101_Location        USART_ROUTE_LOCATION_LOC0
#define _CC1101_baudrate        4000000
#define _CC1101_databits        usartDatabits8
#define _CC1101_enable          usartEnable
#define _CC1101_refFreq         0
#define _CC1101_slave           true
#define _CC1101_msbf            true
#define _CC1101_SPI_MOSI_PIN    gpioPortE, 10
#define _CC1101_SPI_MISO_PIN    gpioPortE, 11
#define _CC1101_SPI_CLK_PIN     gpioPortE, 12
#define _CC1101_SPI_CS_PIN      gpioPortE, 13
#define _CC1101_clockMode       usartClockMode0
#define _CC1101_INT_PIN_ONE     gpioPortB, 0
#define _CC1101_INT_PIN_TWO     gpioPortB, 2

// Configuration Registers
#define CC1101_IOCFG2           0x00        // GDO2 output pin configuration
#define CC1101_IOCFG1           0x01        // GDO1 output pin configuration
#define CC1101_IOCFG0           0x02        // GDO0 output pin configuration
#define CC1101_FIFOTHR          0x03        // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1            0x04        // Sync word, high byte
#define CC1101_SYNC0            0x05        // Sync word, low byte
#define CC1101_PKTLEN           0x06        // Packet length
#define CC1101_PKTCTRL1         0x07        // Packet automation control
#define CC1101_PKTCTRL0         0x08        // Packet automation control
#define CC1101_ADDR             0x09        // Device address
#define CC1101_CHANNR           0x0A        // Channel number
#define CC1101_FSCTRL1          0x0B        // Frequency synthesizer control
#define CC1101_FSCTRL0          0x0C        // Frequency synthesizer control
#define CC1101_FREQ2            0x0D        // Frequency control word, high byte
#define CC1101_FREQ1            0x0E        // Frequency control word, middle byte
#define CC1101_FREQ0            0x0F        // Frequency control word, low byte
#define CC1101_MDMCFG4          0x10        // Modem configuration
#define CC1101_MDMCFG3          0x11        // Modem configuration
#define CC1101_MDMCFG2          0x12        // Modem configuration
#define CC1101_MDMCFG1          0x13        // Modem configuration
#define CC1101_MDMCFG0          0x14        // Modem configuration
#define CC1101_DEVIATN          0x15        // Modem deviation setting
#define CC1101_MCSM2            0x16        // Main Radio Cntrl State Machine config
#define CC1101_MCSM1            0x17        // Main Radio Cntrl State Machine config
#define CC1101_MCSM0            0x18        // Main Radio Cntrl State Machine config
#define CC1101_FOCCFG           0x19        // Frequency Offset Compensation config
#define CC1101_BSCFG            0x1A        // Bit Synchronization configuration
#define CC1101_AGCCTRL2         0x1B        // AGC control
#define CC1101_AGCCTRL1         0x1C        // AGC control
#define CC1101_AGCCTRL0         0x1D        // AGC control
#define CC1101_WOREVT1          0x1E        // High byte Event 0 timeout
#define CC1101_WOREVT0          0x1F        // Low byte Event 0 timeout
#define CC1101_WORCTRL          0x20        // Wake On Radio control
#define CC1101_FREND1           0x21        // Front end RX configuration
#define CC1101_FREND0           0x22        // Front end TX configuration
#define CC1101_FSCAL3           0x23        // Frequency synthesizer calibration
#define CC1101_FSCAL2           0x24        // Frequency synthesizer calibration
#define CC1101_FSCAL1           0x25        // Frequency synthesizer calibration
#define CC1101_FSCAL0           0x26        // Frequency synthesizer calibration
#define CC1101_RCCTRL1          0x27        // RC oscillator configuration
#define CC1101_RCCTRL0          0x28        // RC oscillator configuration
#define CC1101_FSTEST           0x29        // Frequency synthesizer cal control
#define CC1101_PTEST            0x2A        // Production test
#define CC1101_AGCTEST          0x2B        // AGC test
#define CC1101_TEST2            0x2C        // Various test settings
#define CC1101_TEST1            0x2D        // Various test settings
#define CC1101_TEST0            0x2E        // Various test settings

// Status registers
#define CC1101_PARTNUM          0x30        // Part number
#define CC1101_VERSION          0x31        // Current version number
#define CC1101_FREQEST          0x32        // Frequency offset estimate
#define CC1101_LQI              0x33        // Demodulator estimate for link quality
#define CC1101_RSSI             0x34        // Received signal strength indication
#define CC1101_MARCSTATE        0x35        // Control state machine state
#define CC1101_WORTIME1         0x36        // High byte of WOR timer
#define CC1101_WORTIME0         0x37        // Low byte of WOR timer
#define CC1101_PKTSTATUS        0x38        // Current GDOx status and packet status
#define CC1101_VCO_VC_DAC       0x39        // Current setting from PLL cal module
#define CC1101_TXBYTES          0x3A        // Underflow and # of bytes in TXFIFO
#define CC1101_RXBYTES          0x3B        // Overflow and # of bytes in RXFIFO

// Multi byte memory locations
#define CC1101_PATABLE          0x3E
#define CC1101_TXFIFO           0x3F
#define CC1101_RXFIFO           0x3F

// Definitions for burst/single access to registers
#define CC1101_WRITE_BURST      0x40
#define CC1101_READ_SINGLE      0x80
#define CC1101_READ_BURST       0xC0

// Strobe commands
#define CC1101_SRES             0x30        // Reset chip.
#define CC1101_SFSTXON          0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
// If in RX/TX: Go to a wait state where only the synthesizer is
// running (for quick RX / TX turnaround).
#define CC1101_SXOFF            0x32        // Turn off crystal oscillator.
#define CC1101_SCAL             0x33        // Calibrate frequency synthesizer and turn it off
// (enables quick start).
#define CC1101_SRX              0x34        // Enable RX. Perform calibration first if coming from IDLE and
// MCSM0.FS_AUTOCAL=1.
#define CC1101_STX              0x35        // In IDLE state: Enable TX. Perform calibration first if
// MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
// Only go to TX if channel is clear.
#define CC1101_SIDLE            0x36        // Exit RX / TX, turn off frequency synthesizer and exit
// Wake-On-Radio mode if applicable.
#define CC1101_SAFC             0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR             0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD             0x39        // Enter power down mode when CSn goes high.
#define CC1101_SFRX             0x3A        // Flush the RX FIFO buffer.
#define CC1101_SFTX             0x3B        // Flush the TX FIFO buffer.
#define CC1101_SWORRST          0x3C        // Reset real time clock.
#define CC1101_SNOP             0x3D        // No operation. May be used to pad strobe commands to two
// bytes for simpler software.

// Bit fields in the chip status byte
#define CC1101_STATUS_CHIP_RDYn_BM             0x80
#define CC1101_STATUS_STATE_BM                 0x70
#define CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F

// Chip states
#define CC1101_STATE_IDLE                      0x00
#define CC1101_STATE_RX                        0x10
#define CC1101_STATE_TX                        0x20
#define CC1101_STATE_FSTXON                    0x30
#define CC1101_STATE_CALIBRATE                 0x40
#define CC1101_STATE_SETTLING                  0x50
#define CC1101_STATE_RX_OVERFLOW               0x60
#define CC1101_STATE_TX_UNDERFLOW              0x70

// Other bit masks
#define CC1101_LQI_CRC_OK_BM                   0x80
#define CC1101_LQI_EST_BM                      0x7F
#define CC1101_RF_STATUS_BM                    0x70

// Other definitions
#define CC1101_SPI_FREQ                         4000000     // SPI Clock Frequency
#define CC1101_SPI_NPCS                         0           //
#define CC1101_SPI_MODE                         0           // Defines Clock Polarity (CPOL = 0), and Clock Phase (NCPAH = 1)
#define CC1101_SPI_CHAR_LENGTH                  8           // Amount of Bits which is transfered during each SPI-Write Access
#define CC1101_spck_delay                       0           //
#define CC1101_trans_delay

// CC1101 SPI interface configuration structure
typedef struct
{
	uint32_t baudrate;
	uint32_t databits;
	uint32_t refFreq;
	bool enable;
	bool master;
	bool msbf;
} cc1101_typdef;

typedef struct
{
	uint32_t ucStatusRtc;
	uint8_t ucPktLength;
} isrPktStatus_t;

union CC1101_PACKET_BUFFER
{
	struct
	{
		uint8_t length;
		uint8_t addr;
		uint8_t type;
		uint8_t payload[58];
		uint8_t rssi;
		uint8_t crc;
	} fields;

	uint8_t bytes[63];
};

// CC1101 radio configuration structure
union RF_CONFIG
{
	struct
	{
		uint8_t iocfg2;
		uint8_t iocfg1;
		uint8_t iocfg0;
		uint8_t fifothr;
		uint8_t sync1;
		uint8_t sync0;
		uint8_t pktlen;
		uint8_t pktctrl1;
		uint8_t pktctrl0;
		uint8_t addr;
		uint8_t channr;
		uint8_t fsctrl1;
		uint8_t fsctrl0;
		uint8_t freq2;
		uint8_t freq1;
		uint8_t freq0;
		uint8_t mdmcfg4;
		uint8_t mdmcfg3;
		uint8_t mdmcfg2;
		uint8_t mdmcfg1;
		uint8_t mdmcfg0;
		uint8_t deviatn;
		uint8_t mcsm2;
		uint8_t mcsm1;
		uint8_t mcsm0;
		uint8_t foccfg;
		uint8_t bscfg;
		uint8_t agcctrl2;
		uint8_t agcctrl1;
		uint8_t agcctrl0;
		uint8_t worevt1;
		uint8_t worevt0;
		uint8_t worctrl;
		uint8_t frend1;
		uint8_t frend0;
		uint8_t fscal3;
		uint8_t fscal2;
		uint8_t fscal1;
		uint8_t fscal0;
		uint8_t rcctrl1;
		uint8_t rcctrl0;
		uint8_t fstest;
		uint8_t ptest;
		uint8_t agctest;
		uint8_t test2;
		uint8_t test1;
		uint8_t test0;
	} registers;

	uint8_t bytes[47];
};

const RF_CONFIG rf_conf =
{
	{
		0x06,   // iocfg2: assert with SYNC word, deassert at end of packet
		0x06,   // iocfg1: assert with SYNC word, deassert at end of packet
		0x06,   // iocfg0: assert with SYNC word, deassert at end of packet
		0x07,   // fifothr: 33 bytes in TXFIFO, 32 bytes in RXFIFO
		0xD3,   // sync1: high byte of sync word
		0x91,   // sync0: low byte of sync word
		0xFF,   // pktlen: max. packet length (variable); packet length (fixed)
		0x04,   // pktctrl1: CRC autoflush off, append status bytes on, address check off
		0x45,   // pktctrl0: data whitening on, normal packet format, CRC enabled, variable packet length
		0x01,   // addr: device address
		0x00,   // channr: communication channel
		0x0C,   // fsctrl1:
		0x00,   // fsctrl0:
		0x10,   // freq2:
		0xB1,   // freq1:
		0x3B,   // freq0:
		0x2D,   // mdmcfg4:
		0x3B,   // mdmcfg3:
		0x13,   // mdmcfg2: filtering sensitivity optimized, GFSK modulation, manchester encoding off, 4 byte sync word
		0x22,   // mdmcfg1: FEC off, 4 byte preamble
		0xF8,   // mdmcfg0:
		0x62,   // deviatn:
		0x07,   // mcsm2:
		0x0E,   // mcsm1: CCA off, Stay in RX, Stay in TX
		0x18,   // mcsm0:
		0x1D,   // foccfg:
		0x6C,   // bscfg:
		0xC7,   // agcctrl2:
		0x00,   // agcctrl1:
		0xB0,   // agcctrl0:
		0x87,   // worevt1:
		0x6B,   // worevt0:
		0xF8,   // worctrl:
		0xB6,   // frend1:
		0x10,   // frend0:
		0xEA,   // fscal3:
		0x2A,   // fscal2:
		0x00,   // fscal1:
		0x1F,   // fscal0:
		0x41,   // rcctrl1:
		0x00,   // rcctrl0:
		0x59,   // fstest:
		0x7F,   // ptest:
		0x3F,   // agctest:
		0x88,   // test2:
		0x31,   // test1:
		0x09    // test0:
	}
};

/****************************************************************************************************************************************//**
 *
 *  Declaration of the CC1101 driver-class.
 *
 *******************************************************************************************************************************************/
class CC1101_Radio : public DebugInterface, public Statemachine
{
private:
	uint32_t                cc1101Location;
	USART_InitSync_TypeDef  interfaceInit;
	USART_TypeDef           *cc1101USART;

	CC1101_PACKET_BUFFER receive_buffer;

public:
	uint8_t register_access_SPI( uint8_t address, uint8_t access_mode, uint8_t data );
	uint8_t multi_register_access( uint8_t address, uint8_t access_mode, uint8_t *data, uint8_t length );
	uint8_t read_register_SPI( uint8_t address, uint8_t access_mode );
	uint8_t write_register_SPI( uint8_t address, uint8_t data );

	CC1101_Radio();
	~CC1101_Radio() {}

	uint8_t getStatus();
	void initializeInterface();
	void setRfConfig();
	void initializeRadioInterrupt2( bool rising, bool falling );
	void initializeRadioInterrupt0( bool rising, bool falling );

	void sendPacket( uint8_t type, uint8_t addr, uint8_t* payload, uint8_t payload_length );
	void readPacket();

	void setReceiveMode();
	void setSleepMode();

	void setOutputPower( int8_t power );
	int8_t getOutputPower();

	void setCrcCheck( bool state );
	uint8_t getCrcStatus();

	void getPacketPayload( uint8_t* array, uint8_t start, uint8_t end );
	uint8_t getPacketLength();
	uint8_t getPacketAddress();
	uint8_t getPacketType();
	int8_t getRssiValue();
	uint8_t getLqiValue();

	void setAddressCheck( bool state );
	void setAddress( uint8_t address );
	uint8_t getAddress();
	
	uint8_t strobe( uint8_t strobe );
};

#endif /* CC1101_RADIO_H_ */
