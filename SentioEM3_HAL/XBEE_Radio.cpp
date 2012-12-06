/*
 * XBee_Radio.cpp
 *
 *  Created on: Mar 16, 2011
 *      Author: Matthias Krämer
 */

#include <string.h>

#include "XBEE_Radio.h"
#include "SystemConfig.h"

#include "efm32_emu.h"
#include "efm32_cmu.h"
#include "efm32_usart.h"


/****************************************************************************************************************************************//**
 * @brief
 *  Declaration and Initialization of static members of the XBEE_Radio driver-class
 *
 *******************************************************************************************************************************************/

USART_InitAsync_TypeDef XBEE_Radio::initXBeeUart;

MAC_LEVEL_CONFIG        XBEE_Radio::defaultMAC_Configuration;
DIGIMESH_CONFIG         XBEE_Radio::defaultDigimeshConfiguration;

uint8_t *XBEE_Radio::userDataBuffer;
uint8_t *XBEE_Radio::userSourceAddress;
uint8_t *XBEE_Radio::userPayloadLength;

uint8_t XBEE_Radio::modemStatus;
uint8_t XBEE_Radio::transmitRetryCount;
uint8_t XBEE_Radio::deliveryStatus;
uint8_t XBEE_Radio::discoveryStatus;
uint8_t XBEE_Radio::receiveOptions;

uint8_t XBEE_Radio::bufferCount;
uint8_t XBEE_Radio::responseStatus;
volatile uint8_t XBEE_Radio::packetTransmitted = true;
volatile bool    XBEE_Radio::responseReceived;
uint32_t         XBEE_Radio::receivedFrameChecksum;


bool    XBEE_Radio::packetReceived ;
bool    XBEE_Radio::packetPending;


LENGTH XBEE_Radio::temp;
FRAMES XBEE_Radio::Frame;


/****************************************************************************************************************************************//**
 * @brief
 *  Basic System-Variables are set-up, which do not need to be changed during Run-Time
 *
 * @details
 * The Parameterization of the Serial-Interface and the Selection of the Serial-Interface as well as the selection of Interface is done here!
 *
 * @note
 * The User should not change the Initialization-Values for System-Variables which are not related to the Hardware-Interfaces
 *******************************************************************************************************************************************/

XBEE_Radio::XBEE_Radio()
{
	// Setup the Parameters used to configure the selected UART/USART module
    initXBeeUart.enable       = _XBEE_enable;       // Enable the UART after the Configuration is finalized
    initXBeeUart.refFreq      = _XBEE_refFreq;      // The Default HF-Clock Frequency (see CMU) is used to calculate the Baudrate-Clock-Divider Setting
    initXBeeUart.baudrate     = _XBEE_baudrate;     // Resulting Baudrate at which the UART module runs
    initXBeeUart.oversampling = _XBEE_oversampling; // RX-Pin Over-sampling-Rate
    initXBeeUart.databits     = _XBEE_databits;     // Number of Data-Bits
    initXBeeUart.parity       = _XBEE_parity;       // Parity-Bit setup
    initXBeeUart.stopbits     = _XBEE_stopbits;     // Number of Stop-Bits

	//****************************************************************************************************************************************
	//    !! NOTE !!  Do not change the Variables below
	//****************************************************************************************************************************************
	// Initialization of some Auxiliary Variables used in the XBee-driver module
	Frame.AT_Command.StartDelimiter = 0x7E;

	responseReceived  = false; // Has the Command response Frame been received?
	packetPending     = 0;     // Is the internal buffer of the Driver module empty?
	packetTransmitted = true;  // Data-Packet Transmission finalized and Transmit-Response-Frame was received.
	packetReceived    = false; // Has a Data-Packet been received on the radio channel?

	defaultMAC_Configuration.broadcastMultiTransmit = 1;
	defaultMAC_Configuration.unicastMacRetries = 1;
	defaultMAC_Configuration.powerLevel = 4;
}



/****************************************************************************************************************************************//**
 * @brief
 *  Enable/Disables the Power-Supply of the XBee-Radio and also disables the HF-Clock to the Serial-Peripheral-Module (UART/USART) connected
 *  to the XBee-Module
 *
 * @details
 *
 *
 *
 * @param[in] status
 * - True:  enable the Power-Supply/Clock
 * - False: disable the Power-Supply/Clock
 *
 * @note
 * When the HF-Clock is disabled no write-operation can be executed and the MCU code execution stops. This is a efficient way to prevent
 * accessing an disabled module, what might cause hardware damage.
 *******************************************************************************************************************************************/

void XBEE_Radio::powerXBeeRadio( bool status )
{
	if ( status ) // Enable the Power-Supply/Clock
	{
		GPIO_PinOutSet( _XBEE_PowerEnable_ );
	}

	else         // Disable the Power-Supply/Clock
	{
		GPIO_PinOutClear( _XBEE_PowerEnable_ );
	}
}


/****************************************************************************************************************************************//**
 * @brief
 *  Sets up GPIO-Pins and the Serial-Peripheral-Module (UART/USART) utilized by the Xbee-Digimesh
 *
 *
 * @return
 *  - True: Initialization Successful
 *  - False Initialization failed, chosen Interface invalid
 *
 * @note
 *  The parameterization of the Serial-Peripheral-Module and the initialization of basic system-variables
 *  has to be done in the CPP-Object constructor!
 *
 *******************************************************************************************************************************************/

void XBEE_Radio::initializeInterface()
{
	// Configure the MCU-Pin which drives the Enable-Pin of the LDO-Voltage Regulator used to power the XBee
	GPIO_PinModeSet( _XBEE_PowerEnable_, gpioModePushPull, 0 );

	GPIO_PinModeSet( _XBEE_SleepRQ_Pin_, gpioModePushPull, 0 );

	// Configuration of the GPIO-Pins which belong to the UART/USART configured. Furthermore the RX-Interrupt
	//Service Routine of the UART is enabled.
	// In a first switch-case statement the UART/USART Module is selected. Then the RX/TX Pins which have
	// to be activated are selected

// Select module UART0, valid Location on the EFM32G280 are (0,1,2,3)
#if _XBEE_USART_ < 8

	//Enable the required periphery clock
	CMU_ClockEnable( cmuClock_UART0, true );

	// Enable the RX-Interrupt Service Routine
	NVIC_EnableIRQ( UART0_RX_IRQn );

#define XBEE_USART  UART0

	// Select the IO-Pins dependent on Module-Location
#if _XBEE_USART_ == _UART0_LOC0_
#define port  gpioPortF
#define pinRX 7
#define pinTX 6
#define XBEE_LOCATION  USART_ROUTE_LOCATION_LOC0

#elif _XBEE_USART_ == _UART0_LOC1_
#define port  gpioPortE
#define pinRX 1
#define pinTX 0
#define XBEE_LOCATION  USART_ROUTE_LOCATION_LOC1

#elif _XBEE_USART_ == _UART0_LOC2_
#define port  gpioPortA
#define pinRX 4
#define pinTX 3
#define XBEE_LOCATION  USART_ROUTE_LOCATION_LOC2

#elif _XBEE_USART_ == _UART0_LOC3_
#define port  gpioPortC
#define pinRX 15
#define pinTX 14
#define XBEE_LOCATION  USART_ROUTE_LOCATION_LOC3
#endif

// Select module USART0, valid Location on the EFM32G280 are (0,1,2)
#elif ( _XBEE_USART_ > 9 ) && ( _XBEE_USART_ < 19 )

	//Enable the required periphery clock
	CMU_ClockEnable( cmuClock_USART0, true );

	// Enable the RX-Interrupt Service Routine
	NVIC_EnableIRQ( USART0_RX_IRQn );

#define XBEE_USART  USART0

	// Select the IO-Pins dependent on Module-Location
#if _XBEE_USART_ == _USART0_LOC0_
#define port  gpioPortE
#define pinRX 11
#define pinTX 10
#define XBEE_LOCATION  USART_ROUTE_LOCATION_LOC0

#elif _XBEE_USART_ == _USART0_LOC1_
#define port  gpioPortE
#define pinRX 6
#define pinTX 7
#define XBEE_LOCATION  USART_ROUTE_LOCATION_LOC1

#elif _XBEE_USART_ == _USART0_LOC2_
#define port  gpioPortC
#define pinRX 10
#define pinTX 11
#define XBEE_LOCATION  USART_ROUTE_LOCATION_LOC2
#endif


// Select module USART1, valid Location on the EFM32G280 are (0,1)
#elif( _XBEE_USART_ > 19 ) && ( _XBEE_USART_ < 29 )

	//Enable the required periphery clock
	CMU_ClockEnable( cmuClock_USART1, true );

	// Enable the RX-Interrupt Service Routine
	NVIC_EnableIRQ( USART1_RX_IRQn );

#define XBEE_USART  USART1

// Select the IO-Pins dependent on Module-Location
#if _XBEE_USART_ == _USART1_LOC0_
#define port  gpioPortC
#define pinRX 1
#define pinTX 0
#define XBEE_LOCATION  USART_ROUTE_LOCATION_LOC0

#elif _XBEE_USART_ == _USART1_LOC1_
#define port  gpioPortD
#define pinRX 1
#define pinTX 0
#define XBEE_LOCATION  USART_ROUTE_LOCATION_LOC1
#endif

	// Select module USART2, valid Location on the EFM32G280 are (0,1)
#elif( _XBEE_USART_ > 29 ) && ( _XBEE_USART_ < 39 )

	//Enable the required periphery clock
	CMU_ClockEnable( cmuClock_USART1, true );

	// Enable the RX-Interrupt Service Routine
	NVIC_EnableIRQ( USART2_RX_IRQn );

#define XBEE_USART  USART2

	// Select the IO-Pins dependent on Module-Location
#if _XBEE_USART_ == _USART2_LOC0_
#define port  gpioPortC
#define pinRX 3
#define pinTX 2
#define XBEE_LOCATION  USART_ROUTE_LOCATION_LOC0

#elif _XBEE_USART_ == _USART2_LOC1_
#define port  gpioPortB
#define pinRX 4
#define pinTX 3
#define XBEE_LOCATION  USART_ROUTE_LOCATION_LOC1
#endif
#endif


	USART_InitAsync( XBEE_USART, &initXBeeUart );

	// The Routing Register of the selected UART/USART Register is configured. The following register-access
	// enables the UART modules RX/TX shift-register and furthermore selects the one of the possible Locations of the modules IO-Pins
	//
	// NOTE!!!
	// Beside setting a modules Routing Register the functionality of the GPIO-Pins IO-Port-driver has to be configured separately

	XBEE_USART->ROUTE = USART_ROUTE_RXPEN |
						USART_ROUTE_TXPEN |
						XBEE_LOCATION;

	// Configure the IO-Port driver of the EFM32-GPIO Pins which were selected before
	GPIO_PinModeSet( port, pinRX, gpioModeInputPullFilter, 1 ); // RX needs to be a Input
	GPIO_PinModeSet( port, pinTX, gpioModePushPull, 0 );        // TX needs to be a Output


	// Configure the Interrupt
	XBEE_USART->IFC = ~0;                 // Clear pending Interrupts
	XBEE_USART->IEN = USART_IEN_RXDATAV;  // Enable the RX-Interrupt when One-Byte is in the buffer,
	// Flag is cleared when the RX buffer is read-out
}


/****************************************************************************************************************************************//**
 * @brief
 *  Provide access the Data, Address and Payload-Length Buffer/Variables allocated in the Application code
 *
 * @details
 * In order to provide a simple Interface between the XBee-Digimesh Driver and the User-Application-code three Buffers ( Data, Address and
 * Payload-Length)need to be allocated in the Application-code and the Pointer to the first element of the Buffers has to be given to the
 * XBee-Driver in order to make Read/Write-interaction between driver and application possible.
 *
 * @param[in] buffer
 * Pointer to an uint8_t Array[88] with maximal 88 elements
 *
 * @param[in] sourceAddress
 * Pointer to an uint8_t Array[8], which will contain the 64-Bit Source-Address after a Packet has been received on the
 * RF-channel.
 * sourceAdress[0] contains Address-MSB
 * sourceAdress[7] contains Address-LSB
 *
 * @param[in] payloadLength
 *  Length of the Payload in the Packet received last
 *
 * @note
 * Call this function once after the memory has been allocated. (Once after startup, once every time dynamic-memory allocation occurs)
 *
 *******************************************************************************************************************************************/

void XBEE_Radio::initializeSystemBuffer( uint8_t *buffer, uint8_t *sourceAddress, uint8_t *payloadLength )
{
	userDataBuffer    = buffer;
	userSourceAddress = sourceAddress;
	userPayloadLength = payloadLength;
}


ptISR_Handler XBEE_Radio::getISR_FunctionPointer()
{
	return &wrapper_XBeeUART_RX;
}

void XBEE_Radio::enableRadio_SM1()
{
	GPIO_PinOutClear( _XBEE_SleepRQ_Pin_ );

	for ( uint32_t volatile i = 0; i < 2000000; i++ );
}


void XBEE_Radio::disableRadio_SM1()
{
	GPIO_PinOutSet( _XBEE_SleepRQ_Pin_ );
}

/****************************************************************************************************************************************//**
 * @brief
 *  Send a Data-Packet as Unicast or Broadcast message on the RF-Channel
 *
 * @details
 * The user can send a Data-Packet with a specific payload length to the Xbee-Module which is specified by the 64-Bit MAC-Address
 * given in the in the Destination-Address Array. The Transfer is timed and performed automatically by the Digimesh Firmware. After
 * the transmission is perfomed/failed after several retries, the XBee-Module responds with a Status-Frame. The content is available
 * by calling the following driver-methods:
 * getDeliveryStatus(), getDiscoveryStatus(), getDiscoveryStatus()
 *
 * @param[in] data
 * Array Specifies the Payload of the Data-Packet that is send on the RF-channel
 *
 * @param[in] payloadLength
 * Specifies the length of the
 *
 * @param[in] destinationAddress
 * Pointer to an uint8_t Array[8], which will contain the 64-Bit Source-Address after a Packet has been received on the
 * RF-channel.
 * destinationAddress[0] contains Address-MSB
 * destinationAddress[7] contains Address-LSB
 *
 * @param[in] moduleResponse
 * - True:  XBee replies with an Transmit Status
 * - False: XBee does not respond
 *
 * @param[in] broadcastRadius
 * Transmission of Broadcast Messages over Multiple Hops
 * - If set to 0 the Maximal-Hops Parameter is applied here
 *
 * @param[in] transmitOptions
 * Disable Acknoledge(0), Do not discover route(2)
 *
 * @return
 * - True:  Packet has been transfered to the XBee-Module
 * - False: Previous Packet Transfer was not finalized, Module busy
 *
 *******************************************************************************************************************************************/

bool XBEE_Radio::sendPacket( uint8_t *data, uint8_t payloadLength, uint8_t *destinationAddress, uint8_t broadcastRadius, bool acknoledge, bool discovery )
{
	if ( packetTransmitted )
	{
		// Indicate Packet Transmission on-going
		packetTransmitted = false;

		// Calculate the Length of the API-Frame
		temp.Length = 0x0E + payloadLength;

		Frame.TransmitPacket.Length_MSB = temp.Register.MSB;
		Frame.TransmitPacket.Length_LSB = temp.Register.LSB;

		Frame.TransmitPacket.FrameType = 0x10;
		Frame.TransmitPacket.FrameID   = 0x01;

		// Copy the user-defined destination-address to the API-Frame buffer
		memcpy( Frame.TransmitPacket.DestinationAddress, destinationAddress, 8 );
		//for(volatile uint8_t i = 0; i < 8; i++)
		//Frame.TransmitPacket.DestinationAddress[i] = destinationAddress[i];

		// Set Reserve-Bytes to the required default values
		Frame.TransmitPacket.Reserve[0] = 0xFF;
		Frame.TransmitPacket.Reserve[1] = 0xFE;

		Frame.TransmitPacket.BroadcastRadius = broadcastRadius;
		Frame.TransmitPacket.TransmitOptions = ( ( uint8_t ) !discovery ) << 1 | ( ( uint8_t ) !acknoledge );

		// Copy the user-defined packet-payload to the API-Frame buffer
		for ( volatile uint8_t i = 0; i < payloadLength; i++ )
			Frame.TransmitPacket.RadioData[i] = data[i];

		// The API-Frame is send to the XBee-Module via the configured UART-interface
		for ( uint8_t i = 0; i <= ( temp.Length + 2 ); i++ )
		{
			USART_Tx( XBEE_USART, Frame.Data[i] );
		}


		// When all Data-Bytes are send out the Checksum, is calculated and will be transfered at the END of the API-Frame
		USART_Tx( XBEE_USART, processingChecksum() );

		// Wait in Energy-Mode 1 for the complete reception of the XBee-module response and the processing of the
		do
		{

			EMU_EnterEM1();

		}
		while ( !packetTransmitted );   // leave the loop when the Response-Frame is received or if a
		// invalid API-Frame or invalid Checksum is received

		return true;
	}

	else
		return false;
}


/****************************************************************************************************************************************//**
 * @brief
 *  The driver-method returns true when the a Packet has been received on the RF-channel
 *
 * @details
 * After the packet has been received it's payload is copied to the user-defined buffer, which has to be long enough
 * to fit the longest payload that is expected in the network.
 *
 * @return
 * - True:  packet is received by the XBee-Module
 * - False: no new packet is available in the buffer
 *
 *******************************************************************************************************************************************/

bool XBEE_Radio::getPacketReceived()
{
	if ( packetReceived )
	{
		// Reset the driver-status flag to allow an new packet to be transfered
		packetReceived = false;
		return true;
	}

	return false;
}


/****************************************************************************************************************************************//**
 * @brief
 *  When a packet-payload is not copied to the user--defined buffer. By calling this driver-method the user can copy the payload form the
 *  driver- to the applications-buffer "manually"
 *
 * @details
 * When a second packet is transfered to the XBee-Module before the application-code has processed the first packet, the user-defined buffer
 * is marked as not empty by an internal status flag of the driver. The payload of the new packet, is kept in the drivers internal buffer and
 * will stay there until the user initiates the transfer or until an new API-Frame is transfered by the XBee-Module
 *
 * @return
 * - True:  packet is received by the XBee-Module
 * - False: more than one packet was pending, just the last received packet is still available
 *
 *******************************************************************************************************************************************/

bool XBEE_Radio::getPendingPacketData()
{
	// Copy the Data-Payload to the user-defined-buffer
	memcpy( userDataBuffer, Frame.ReceivePacket.ReceiveData, ( temp.Length - 12 ) );

	// The MSB is not contained in RF-Packet, due to the fact that it is always expected to be 0x00,
	// therefore set the byte manually
	*userSourceAddress = 0x00 ;

	// Copy the 7-Byte remaining Bytes from their position inside the API-Frame-Buffer to the user-defined buffer
	memcpy( userSourceAddress + 1, Frame.ReceivePacket.SourceAdress64, 7 );

	// Pick the Status information from the correct position inside the frame and make
	// it available for the get-method()
	receiveOptions = Frame.ReceivePacket.ReceiveOptions;

	// Indication of a pending packet in side the buffer,
	// a packetPending value of > 1 indicates a dropped packet!
	if ( packetPending <= 1 )
		return true;

	return false;
}


/****************************************************************************************************************************************//**
 * @brief
 * This method returns the Modem(XBee)-Status that has been received with the last API-MODEM-Status Frame
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getModemStatus()
{
	return modemStatus;
}


/****************************************************************************************************************************************//**
 * @brief
 * This method returns the TransmitRetryCount-Status that has been received with the last API-Transmit Request Frame
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getTransmitRetryCount()
{
	return transmitRetryCount;
}


/****************************************************************************************************************************************//**
 * @brief
 * This method returns the Delivery-Status that has been received with the last API-Transmit Request Frame
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getDeliveryStatus()
{
	return deliveryStatus;
}


/****************************************************************************************************************************************//**
 * @brief
 * This method returns the Discovery-Status that has been received with the last API-Transmit Request Frame
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getDiscoveryStatus()
{
	return discoveryStatus;
}


/****************************************************************************************************************************************//**
 * @brief
 * This method returns the receiveOptions-Status that has been received with the last API-Transmit Request Frame //TODO
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getReceiveOptions()
{
	return receiveOptions;
}

// Special Commands
/****************************************************************************************************************************************//**
 * @brief
 * The WriteValues command of the XBee-Module stores all configuration parameters within the NON-volatile Memory
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::sendWriteValuesComand()
{
	return configRegisterAccess( WriteValues );
}


/****************************************************************************************************************************************//**
 * @brief
 * The RestoreDefaults command resets the XBee-Module to factory-default settings.
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::sendRestoreDefaultsComand()
{
	return configRegisterAccess( RestoreDefaults );
}


/****************************************************************************************************************************************//**
 * @brief
 * The Software Reset command issues and reset of the XBee-Module
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::sendSoftwareResetComand()
{
	return configRegisterAccess( SoftwareReset );
}


/****************************************************************************************************************************************//**
 * @brief
 * When a queue of registers is set using the (queueing = true) option the Apply Changes Command will trigger the new settings to be applied
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::sendApplyChangesComand()
{
	return configRegisterAccess( ApplyChanges );
}


/****************************************************************************************************************************************//**
 * @brief
 * NOT implemented yet
 *
 * @return responseStatus
 * Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::sendVersionLongComand()
{
	return 0xFF;
	//return configRegisterAccess( VersionLong );
}


// Diagnostic Commands
/****************************************************************************************************************************************//**
 * @brief
 *
 *
 * @param[in] *firmwareVersion
 * After the method has been executed the location given as (*firmwareVersion) parameter stores the current version of the
 * XBee-modules firmware revision
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getFirmwareVersion( uint32_t *firmwareVersion )
{
	return configRegisterAccess( FirmwareVersion, ( uint32_t* ) firmwareVersion, 4 );
}


/****************************************************************************************************************************************//**
 * @brief
 * After the method has been executed the location given as (*hardwareVersion) parameter stores the current version of the
 * XBee-modules hardware revision
 *
 * @param[in] *hardwareVersion
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getHardwareVersion( uint16_t *hardwareVersion )
{
	return configRegisterAccess( HardwareVersion, ( uint32_t* ) hardwareVersion, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 * After the method has been executed the location given as (*configurationCode) parameter stores the configuration code of the
 * parameter-set currently active in the XBee-Module. The configuration-code summarizes the AT commands and can be used to verify that a
 * XBee-Module has been configured as desired.
 *
 * @param[in] *configurationCode
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getConfigurationCode( uint32_t *configurationCode )
{
	return configRegisterAccess( ConfigurationCode, configurationCode, 4 );
}


/****************************************************************************************************************************************//**
 * @brief
 * After the method has been executed the location given as (*rf_Errors) parameter stores the number of Data-Packets received with
 * Data-Integrity Errors
 *
 * @param[in] *rf_Errors
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getRF_Errors( uint16_t *rf_Errors )
{
	return configRegisterAccess( RF_Errors, ( uint32_t* ) rf_Errors , 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 * After the method has been executed the location given as (*rf_Errors) parameter stores the number of Data-Packets received correctly
 *
 * @param[in] *good_Packets
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getGoodPackets( uint16_t *good_Packets )
{
	return configRegisterAccess( GoodPackets, ( uint32_t* ) good_Packets, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 * After the method has been executed the location given as (*transmissionErrors) parameter stores the number of MAC frames that exhaust
 * MAC retries without ever receiving a MAC acknowledgment message from the adjacent node. When the value reaches 0xffff, it stays there
 *
 * @param[in] *transmissionErrors
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getTransmissionErrors( uint16_t *transmissionErrors )
{
	return configRegisterAccess( TransmissionErrors, ( uint32_t* ) transmissionErrors, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 * After the method has been executed, the location given as (*temperature) parameter, stores the current XBee-Module temperature.
 *
 * @param[in] *temperature
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getTemperature( uint16_t *temperature )
{
	return configRegisterAccess( Temperature, ( uint32_t* ) temperature, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 * After the method has been executed, the location given as (*rssi) parameter, stores the received signal strength indicator
 * of the last received RF data packet. The command only indicates the signal strength of the last hop. It does not provide an accurate
 * quality measurement for a multi-hop link. The DB command value is measured in -dBm.
 *
 * @param[in] *rssi
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getReceivedSignal( uint8_t *rssi )
{
	return configRegisterAccess( ReceivedSignalStrength, ( uint32_t* ) rssi, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 * NOT implemented yet
 *
 * @return responseStatus
 * Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getSupplyVoltage( uint16_t *supplyVoltage )
{
	return 0xFF;
	//return configRegisterAccess( SupplyVoltage, (uint32_t*) supplyVoltage, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 * The (*rssiTime) parameter, stores the time that the RSSI output (indicating signal strength) will remain active after the
 * last reception. Time units are measured in tenths of seconds. This method sets the XBee-Module parameter to the specified value.
 *
 * @param[in] *rssiTime
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::setRSSI_PWM_Timer( uint8_t *rssiTime )
{
	return configRegisterAccess( SupplyVoltage, ( uint32_t* ) rssiTime, 1, false, true, true );
}


/****************************************************************************************************************************************//**
 * @brief
 * After the method has been executed, the location given as (*rssiTime) parameter, stores the time that the RSSI output (indicating
 * signal strength) will remain active after the last reception. Time units are measured in tenths of seconds.
 *
 * @param[in] *rssiTime
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getRSSI_PWM_Timer( uint8_t *rssiTime )
{
	return configRegisterAccess( RSSI_PWM_timer, ( uint32_t* ) rssiTime, 1 );
}


/****************************************************************************************************************************************//**
 * @brief
 * This method queries the MAC-Address of the locally attached XBee-Module.
 *
 * @param[in] (type: MAC_XBee) Address
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getLocalXBeeMAC( MAC_XBee Address )
{
	uint8_t i;
	uint8_t errorCount = 0;

	uint32_t temporary  = 0;
	uint32_t temporary2 = 0;

	errorCount += configRegisterAccess( SerialNumberLow, &temporary, 4 );

	for ( i = 7; i >= 4; i-- )
	{
		Address[i] = ( uint8_t )temporary;
		temporary >>= 8;
	}

	errorCount += configRegisterAccess( SerialNumberHigh, &temporary2, 4 );

	for ( i = 3; i > 0; i-- )
	{
		Address[i] = ( uint8_t )temporary2;
		temporary2 >>= 8;
	}

	return errorCount;
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method queries the destination MAC-Address of the last packet which was send on the RF channel.
 *
 * @param[in] (type: MAC_XBee) Address
 *
 * @return responseStatus
 * Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), (>3 && <=6) more than one register-access failed,
 * Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getDestinationXBeeMAC( MAC_XBee Address )
{
	uint8_t i;
	uint8_t errorCount = 0;

	uint32_t temporary  = 0;
	uint32_t temporary2 = 0;

	errorCount += configRegisterAccess( DestinationAddressLow, &temporary, 4 );

	for ( i = 7; i >= 4; i-- )
	{
		Address[i] = ( uint8_t )temporary;
		temporary >>= 8;
	}

	errorCount += configRegisterAccess( DestinationAddressHigh, &temporary2, 4 );

	for ( i = 3; i > 0; i-- )
	{
		Address[i] = ( uint8_t )temporary2;
		temporary2 >>= 8;
	}


	return errorCount;
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @param[in] *deviceTypeIdentifier
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getDeviceTypeIdentifier( uint32_t *deviceTypeIdentifier )
{
	return configRegisterAccess( DeviceTypeIdentifier, ( uint32_t* ) deviceTypeIdentifier, 4 );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @param[in] *maximumPayloadBytes
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getMaximumRF_Payload( uint16_t *maximumPayloadBytes )
{
	return configRegisterAccess( MaximumRF_PayloadBytes, ( uint32_t* ) maximumPayloadBytes, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 *
 * @param[in] *PAN_ID
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getPAN_ID( uint16_t *PAN_ID )
{
	return configRegisterAccess( ClusterIdentifier, ( uint32_t* ) PAN_ID, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @param[in] *PAN_ID
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::setPAN_ID( uint16_t *PAN_ID )
{
	return configRegisterAccess( ClusterIdentifier, ( uint32_t* ) PAN_ID, 2, false, true, true );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getChannel( uint8_t *channel )
{
	return configRegisterAccess( Channel, ( uint32_t* ) channel, 1 );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::setChannel( uint8_t *channel )
{
	//TODO: Check user Input valid?
	return configRegisterAccess( Channel, ( uint32_t* ) channel, 1, false, true, true );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getModuleType( uint8_t *type )
{
	return configRegisterAccess( Coordinator_Enddevice, ( uint32_t* ) type, 1 );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::setModuleType( uint8_t *type )
{
	if ( ( *type ) == 0 || ( *type ) == 2 )
		return configRegisterAccess( Coordinator_Enddevice, ( uint32_t* ) type, 1, false, true, true );
	else
		return 0xFF;
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), (>3 && <=9) more than one register-access failed,
 *  Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getMAC_LayerConfig( MAC_LEVEL_CONFIG *configuration )
{
	uint8_t errorCount = 0;

	errorCount += configRegisterAccess( BroadcastMultiTransmit, ( uint32_t* ) & ( configuration->broadcastMultiTransmit ), 1 );

	errorCount += configRegisterAccess( UnicastMAC_Retries, ( uint32_t* ) & ( configuration->unicastMacRetries ), 1 );

	errorCount += configRegisterAccess( PowerLevel, ( uint32_t* ) & ( configuration->powerLevel ), 1 );


	return errorCount;
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), (>3 && <=9) more than one register-access failed,
 *  Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::setMAC_LayerConfig( MAC_LEVEL_CONFIG *configuration )
{
	uint8_t errorCount = 0;

	if ( configuration->broadcastMultiTransmit <= 0x0F )
		errorCount += configRegisterAccess( BroadcastMultiTransmit, ( uint32_t* ) & ( configuration->broadcastMultiTransmit ), 1, false, true, true );
	else
		return 0xFF;

	if ( configuration->unicastMacRetries <= 0x0F )
		errorCount += configRegisterAccess( UnicastMAC_Retries, ( uint32_t* ) & ( configuration->unicastMacRetries ), 1, false, true, true );
	else
		return 0xFF;

	if ( configuration->powerLevel <= 0x04 )
		errorCount += configRegisterAccess( PowerLevel, ( uint32_t* ) & ( configuration->powerLevel ), 1, false, true, true );
	else
		return 0xFF;

	return errorCount;
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), (>3 && <=15) more than one register-access failed,
 *  Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getDigiMeshConfig( DIGIMESH_CONFIG *configuration )
{
	uint8_t errorCount = 0;

	errorCount += configRegisterAccess( NetworkHops, ( uint32_t* ) & ( configuration->networkHops ), 1 );

	errorCount += configRegisterAccess( NetworkDelaySlots, ( uint32_t* ) & ( configuration->networkDelaySlots ), 1 );

	errorCount += configRegisterAccess( MeshNetworkRetries, ( uint32_t* ) & ( configuration->meshNetworkRetries ), 1 );

	errorCount += configRegisterAccess( BroadcastRadius, ( uint32_t* ) & ( configuration->broadcastRadius ), 1 );

	errorCount += configRegisterAccess( NodeType, ( uint32_t* ) & ( configuration->nodeType ), 1 );

	return errorCount;
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), (>3 && <=15) more than one register-access failed,
 *  Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::setDigiMeshConfig( DIGIMESH_CONFIG *configuration )
{
	uint8_t errorCount = 0;

	if ( configuration->networkHops >= 1 )
		errorCount += configRegisterAccess( NetworkHops, ( uint32_t* ) & ( configuration->networkHops ), 1, false, true, true );
	else
		return 0xFF;

	if ( configuration->networkDelaySlots <= 0x0A )
		errorCount += configRegisterAccess( NetworkDelaySlots, ( uint32_t* ) & ( configuration->networkDelaySlots ), 1, false, true, true );
	else
		return 0xFF;

	if ( configuration->meshNetworkRetries <= 0x07 )
		errorCount += configRegisterAccess( MeshNetworkRetries, ( uint32_t* ) & ( configuration->meshNetworkRetries ), 1, false, true, true );
	else
		return 0xFF;

	if ( configuration->broadcastRadius <= 0x20 )
		errorCount += configRegisterAccess( BroadcastRadius, ( uint32_t* ) & ( configuration->broadcastRadius ), 1, false, true, true );
	else
		return 0xFF;

	if ( configuration->nodeType == 0x02 )
		errorCount += configRegisterAccess( NodeType, ( uint32_t* ) & ( configuration->nodeType ), 1, false, true, true );
	else
		return 0xFF;

	return errorCount;
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), (>3 && <=18) more than one register-access failed,
 *  Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getSleepConfig( SLEEP_CONFIG *configuration )
{
	uint8_t errorCount = 0;

	errorCount += configRegisterAccess( SleepMode, ( uint32_t* ) configuration->sleepMode, 1 );

	errorCount += configRegisterAccess( SleepOptions, ( uint32_t* ) configuration->sleepOptions, 1 );

	errorCount += configRegisterAccess( WakeTime, ( uint32_t* ) configuration->wakeTime, 1 );

	errorCount += configRegisterAccess( SleepPeriod, ( uint32_t* ) configuration->sleepPeriod, 1 );

	errorCount += configRegisterAccess( NumberOfSleepPeriods, ( uint32_t* ) configuration->numberOfSleepPeriods, 1 );

	errorCount += configRegisterAccess( WakeHost, ( uint32_t* ) configuration->wakeHost, 1 );

	return errorCount;
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), (>3 && <=18) more than one register-access failed,
 *  Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::setSleepConfig( SLEEP_CONFIG *configuration )
{
	uint8_t errorCount = 0;

	if ( ( configuration->sleepMode < 0x09 ) && ( configuration->sleepMode != 0x02 ) && ( configuration->sleepMode != 0x03 ) && ( configuration->sleepMode != 0x06 ) )
		errorCount += configRegisterAccess( NetworkHops, ( uint32_t* ) configuration->sleepMode, 1, false, true, true );
	else
		return 0xFF;

	if ( ( ( configuration->sleepOptions & 0x01 ) && !( configuration->sleepOptions & 0x02 ) ) || ( !( configuration->sleepOptions & 0x01 ) && ( configuration->sleepOptions & 0x02 ) ) )
		errorCount += configRegisterAccess( NetworkHops, ( uint32_t* )  configuration->sleepOptions , 1, false, true, true );
	else
		return 0xFF;

	if ( ( configuration->wakeTime > 0x45 ) && ( configuration->wakeTime < 0x0036EE80 ) )
		errorCount += configRegisterAccess( NetworkHops, ( uint32_t* )  configuration->sleepOptions , 1, false, true, true );
	else
		return 0xFF;

	//TODO: check the maximal value
	if ( configuration->sleepPeriod > 0 )
		errorCount += configRegisterAccess( SleepPeriod, ( uint32_t* ) configuration->sleepPeriod, 1, false, true, true );
	else
		return 0xFF;

	if ( configuration->numberOfSleepPeriods > 0 )
		errorCount += configRegisterAccess( NumberOfSleepPeriods, ( uint32_t* ) configuration->numberOfSleepPeriods, 1, false, true, true );
	else
		return 0xFF;

	errorCount += configRegisterAccess( WakeHost, ( uint32_t* ) configuration->wakeHost, 2, false, true, true );


	return errorCount;
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getNumberOfMissedSyncs( uint16_t *numberOfSyncCount )
{
	return configRegisterAccess( NumberOfMissedSyncs, ( uint32_t* ) numberOfSyncCount, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getMissedSyncCount( uint16_t *missedSyncCount )
{
	return configRegisterAccess( MissedSyncCount, ( uint32_t* ) missedSyncCount, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getSleepStatus( uint8_t *sleepStatus )
{
	return configRegisterAccess( SleepStatus, ( uint32_t* ) sleepStatus, 1 );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getOperationalSleepPeriod( uint16_t *operationalSleepPeriod )
{
	return configRegisterAccess( OperationalSleepPeriod, ( uint32_t* ) operationalSleepPeriod, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3), Driver related Error (0xFF)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getOperationalWakePeriod( uint16_t *operationalWakePeriod )
{
	return configRegisterAccess( OperationalWakePeriod, ( uint32_t* ) operationalWakePeriod, 2 );
}


/****************************************************************************************************************************************//**
 * @brief
 *  When a packet-payload is not copied to the user--defined buffer. By calling this driver-method the user can copy the payload form the
 *  driver- to the applications-buffer "manually"
 *
 * @details
 *  When a second packet is transfered to the XBee-Module before the application-code has processed the first packet, the user-defined buffer
 *  is marked as not empty by an internal status flag of the driver. The payload of the new packet, is kept in the drivers internal buffer and
 *  will stay there until the user initiates the transfer or until an new API-Frame is transfered by the XBee-Module
 *
 * @param[in] *command
 *  Character Array contains the ASCII representation of the command name
 *
 * @param[in] *parameter
 *  Specifies the Parameter-Value
 *
 * @param[in] size
 *  Specifies if the parameter value is a 8,16,32-Bit data-type
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::configRegisterAccess( const char *command, uint32_t *parameter, uint8_t size, bool queueing, bool moduleResponse, bool setRegister )
{
	uint8_t i;

	bufferCount = 0;
	responseStatus = 0x00;
	responseReceived = false; // signaling the reception of a Response Frame from the XBee-Module

	// Configure for set and querying the register correctly
	if ( setRegister )
		temp.Length = 0x04 + size;
	else
		temp.Length = 0x04;

	Frame.AT_Command.Length_MSB = temp.Register.MSB;
	Frame.AT_Command.Length_LSB = temp.Register.LSB;

	// The Command Frame-Type and Frame-Identification is set.
	if ( queueing )
	{
		Frame.AT_Command.FrameType = 0x09; // The XBee-Module executes the command immediately
		Frame.AT_Command.FrameID = 0x01;
	}

	else
	{
		Frame.AT_Command.FrameType = 0x08; // The Command is is queued until a the AT-Command (AC) Apply Changes is
		// issued.
		Frame.AT_Command.FrameID = 0x52;
	}

	// When the XBee-Module response is not required for secured operation of the Application-Code the User can disable this response.
	// The Frame-Identification has to be set to 0x00 regarding less which type of frame is transfered to the XBee-Module.
	if ( !moduleResponse )
		Frame.AT_Command.FrameID = 0x00;

	// Set the AT-Command Bytes
	Frame.AT_Command.AT_Command[0] = command[0];
	Frame.AT_Command.AT_Command[1] = command[1];

	// Set the Command Parameter
	// The value of the parameter is not sent in case the user queries the current status
	for ( i = 0; i < size; i++ )
	{
		Frame.AT_Command.Parameter[i] = ( uint8_t )( ( *parameter )  >> ( i * 8 ) );
	}

	// The Data-Frame is send to the XBee-Module via the configured UART-interface
	for ( i = 0; i <= ( temp.Length + 2 ); i++ )
	{
		USART_Tx( XBEE_USART, Frame.Data[i] );
	}

	// When all Data-Bytes are send out the Checksum, is calculated and will be transfered at the END of the Command-Frame

	USART_Tx( XBEE_USART, processingChecksum() );

	// If a Command-Response Frame is expected from the XBee-Module the micro-controller is waiting in Low-power mode until the entire
	// Response Frame has been transfered to the SRAM-buffer.

	if ( moduleResponse )
	{
		do
		{
			EMU_EnterEM1(); // Maintain the Peripheral-Clock in order to enable the UART-RX mode

		}
		while ( !responseReceived );
	}


	union PARAMETER
	{
		uint8_t  driverPar[4];
		uint32_t userPar;
	} para;

	// When the user queries the value which is currently set, the XBee-Module will respond with
	// a Command Response Frame that contains the current value of the register beginning from the 6th "PAYLOAD" byte in the AT-Response frame.
	if ( !setRegister )
	{
		para.userPar = 0;

		uint8_t offset = ( temp.Length - 5 );

		for ( i = 0; i < offset; i++ )
		{
			para.driverPar[i] = Frame.AT_CommandResponse.CommandData[offset - i - 1];
		}

		*parameter = para.userPar;
	}

	return responseStatus;
}


/****************************************************************************************************************************************//**
 * @brief
 *                     !!! NOTE !!!
 *  Do not call this function within the Application-Code
 *
 * @details
 *  This function is called within the Interrupt Service Routine of the UART/UASRT Interface that is utilized to drive the XBee-Module. This
 *  interrupt routine is triggered when the RXDATAV - Interrupt occurs. The first steps in the processing of the API-Response Frame is done.
 *
 * @param[in] inputChar
 *  Within the ISR of the the first byte of the UART/USART-RX Buffer readout and transfered to this function for further processsing
 *
 *******************************************************************************************************************************************/

void XBEE_Radio::wrapper_XBeeUART_RX( uint32_t inputChar )
{

	switch ( bufferCount )
	{
		// Verify the first byte, which is transfered to the USART in this communication-cycle is the expected Start-Delimiter 0x7E
		// Otherwise stop reception of the Frame.
	case 0:
		if ( ( uint8_t )inputChar != 0x7E ) // API-Response Frame wrong
		{
			responseReceived = true;
			responseStatus = 0xFF;
			bufferCount = 0;
		}
		else
			bufferCount++;
		break;

		// Process the Frame-Length Information, send by the XBee-Module
		// The Union named temp is used to assign the MSB/LSB information to an uint16_t
	case 1:
		temp.Register.MSB = ( uint8_t )inputChar;
		bufferCount++;
		break;

	case 2:
		temp.Register.LSB = ( uint8_t )inputChar;
		bufferCount++;
		break;

	default:

		// Fill the SRAM-Buffer with the Data-Bytes of the Frame currently received on the UART
		if ( bufferCount < ( temp.Length + 3 ) )
		{
			Frame.Data[bufferCount] = ( uint8_t )inputChar;
			bufferCount++;
		}

		// Reception of the Data is completed, now the XBee-Module's Response-Frame is processed.
		// Status-variables are set to default, to enable the reception of a new API-Frame from the XBee-Module
		else
		{
			bufferCount = 0;
			receivedFrameChecksum = ( uint8_t )inputChar;
			responseStatus = processingResponse();
		}
	}
}


/****************************************************************************************************************************************//**
 * @brief
 *  Calculation of the API-Frame checksum as specified in the devices documentation
 *
 * @details
 *  This method is used to calculate the the Checksum-Value which is attached at the end of an API-Frame which is send to the XBee-Module
 *  and is also used to verify the Checksum that is part of every response frame of the XBee-Module
 *
 * @return
 *  Returns the value of the checksum calculated over the current contend of the drivers SRAM-Buffer
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::processingChecksum()
{
	uint32_t tempChecksum = 0;

	// Calculation of the Checksum
	for ( uint8_t i = 3; i < ( temp.Length + 3 ); i++ )
	{
		// In the first step the Hex-Values of all "PAYLOAD-BYTES" (excluding Start-Delimiter, Length MSB/LSB,
		// and the Checksum itself are summed up
		tempChecksum += Frame.Data[i];
	}

	// Masking the Result of the Checksum Calculation with 0x000000FF. The Low-Byte of the Checksum remains unchanged, while the upper 24-bit
	// are forced to Zero. The Outcome of the bitwise AND - Operation is send to the XBee-Module at the end of the Frame.
	return ( 0xFF - ( ( uint8_t )( tempChecksum & 0x000000FF ) ) );
}


/****************************************************************************************************************************************//**
 * @brief
 *  Used in the Interrupt Service Routine !!!
 *  This method distinguishes the type of the Response Frame which was transfered by the XBee-Module to the MCU and cales the dedicated
 *  method, which is used to process the specific type of received Frame
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::processingResponse()
{

	switch ( Frame.AT_Command.FrameType )
	{
	case AT_CommandResponseFrame:
		return processingCommandResponse();

	case ModemStatusFrame:
		processingModemStatusFrame();
		break;

	case TransmitStatusFrame:
		processingTransmitStatus();
		break;

	case ReceivePacketFrame:
		processingReceivePacket();
		break;

	default:
		return 0xFF;
	}

	// return value, which is indicates, the Frame is invalid!
	return 0xFE;
}


/****************************************************************************************************************************************//**
 * @brief
 *  Used in the Interrupt Service Routine !!!
 *  This function is called to process a Command Response API-Frame, received on the UART/USART interface.
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::processingCommandResponse()
{

	// Check for a valid checksum of the received CommandResponse API-Frame
	if ( ( processingChecksum() == receivedFrameChecksum ) )
	{
		// Set driver status flag to indicate the finalization of reception and processing of the status-frame
		responseReceived = true;

		return Frame.AT_CommandResponse.CommandStatus;
	}

	else
		// Indicate an invalid checksum
		return 0xFF;
}


/****************************************************************************************************************************************//**
 * @brief
 *  Used in the Interrupt Service Routine !!!
 *  This function is called to process a Modem Status API-Frame, received on the UART/USART interface.
 *
 *******************************************************************************************************************************************/

void XBEE_Radio::processingModemStatusFrame()
{
	// Check for a valid checksum of the received ModemStatus API-Frame
	if ( XBEE_Radio::processingChecksum() == receivedFrameChecksum )
	{
		// Pick the Status information from the correct position inside the frame and make
		// it available for the get-method()
		modemStatus = Frame.Modemstatus.Status;
	}

	else
		// Indicate an invalid checksum
		modemStatus = 0xFF;
}


/****************************************************************************************************************************************//**
 * @brief
 *  Used in the Interrupt Service Routine !!!
 *  This function is called to process a Transmit Response API-Frame, received on the UART/USART interface.
 *
 *******************************************************************************************************************************************/

void XBEE_Radio::processingTransmitStatus()
{
	// Check for a valid checksum of the received TransmitStatus API-Frame
	if ( processingChecksum() == receivedFrameChecksum )
	{
		// Pick the Status information from the correct position inside the frame and make
		// it available for the get-methods()
		transmitRetryCount = Frame.TransmitStatus.TransmitRetryCount;
		deliveryStatus     = Frame.TransmitStatus.DeliveryStatus;
		discoveryStatus    = Frame.TransmitStatus.DiscoveryStatus;

		// Set driver status flag to indicate the finalization of reception and processing of the status-frame
		packetTransmitted = true;
	}

	else
	{
		// Indicate an invalid Checksum
		transmitRetryCount = deliveryStatus = discoveryStatus = 0xFF;
	}
}


/****************************************************************************************************************************************//**
 * @brief
 *  Used in the Interrupt Service Routine !!!
 *  This function is called to process a Command Response API-Frame, received on the UART/USART interface.
 *
 *******************************************************************************************************************************************/

void XBEE_Radio::processingReceivePacket()
{

	// Check for a valid checksum of the received ReceivePacket API-Frame
	if ( ( processingChecksum() == receivedFrameChecksum ) && !packetReceived )
	{
		// Calculate and store the Payload-Length of the received Packet
		*userPayloadLength = temp.Length - 12;

		// Copy the Data-Packets Payload to the user-defined buffer
		memcpy( userDataBuffer, Frame.ReceivePacket.ReceiveData, ( *userPayloadLength ) );

		// The MSB is not contained in RF-Packet, due to the fact that it is always expected to be 0x00,
		// therefore set the byte manually
		*userSourceAddress = 0x00 ;

		// Copy the 7-Byte remaining Bytes from their position inside the API-Frame-Buffer to the user-defined buffer
		memcpy( userSourceAddress + 1, Frame.ReceivePacket.SourceAdress64, 7 );

		// Pick the Status information from the correct position inside the frame and make
		// it available for the get-method()
		receiveOptions = Frame.ReceivePacket.ReceiveOptions;

		// Set driver status flag to indicate the finalization of reception and processing of the status-frame
		packetReceived = true;


	}

	else
	{
		// Indication of a pending packet in side the buffer,
		// a packetPending value of > 1 indicates a dropped packet!
		packetPending++;
	}
}


/****************************************************************************************************************************************//**
 * @brief
 * This method can be used to set any 32-Bit XBee-Configuration register
 *
 * @param[in] command[2]
 *  Character Array specifies the XBee-Register which is accessed
 *
 * @param[in] parameter
 *  Specifies if the parameter value which is written to the XBee-Module
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::setConfigRegister( char *command, uint32_t parameter, bool queueing, bool moduleResponse )
{
	return configRegisterAccess( command, &parameter, 4, queueing, moduleResponse, true );
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method can be used to query any 32-Bit XBee-Configuration register
 *
 * @param[in] command[2]
 *  Character Array specifies the XBee-Register which is accessed
 *
 * @param[in] parameter
 *  The Pointer-Location contains the queried value which has been received from the XBee-Module
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getConfigRegister( char *command, uint32_t *parameter )
{
	return configRegisterAccess( command, parameter, 4, false, true, false );
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method can be used to set any 16-Bit XBee-Configuration register
 *
 * @param[in] command[2]
 *  Character Array specifies the XBee-Register which is accessed
 *
 * @param[in] parameter
 *  Specifies if the parameter value which is written to the XBee-Module
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::setConfigRegister( char *command, uint16_t parameter, bool queueing, bool moduleResponse )
{
	return configRegisterAccess( command, ( uint32_t* ) &parameter, 2, queueing, moduleResponse, true );
}


/****************************************************************************************************************************************//**
 * @brief
 * This method can be used to query any 16-Bit XBee-Configuration register
 *
 * @param[in] command[2]
 *  Character Array specifies the XBee-Register which is accessed
 *
 * @param[in] parameter
 *  The Pointer-Location contains the queried value which has been received from the XBee-Module
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getConfigRegister( char *command, uint16_t *parameter )
{
	return configRegisterAccess( command, ( uint32_t* ) parameter, 2, false, true, false );
}


/****************************************************************************************************************************************//**
 * @brief
 *  This method can be used to set any 8-Bit XBee-Configuration register
 *
 * @param[in] command[2]
 *  Character Array specifies the XBee-Register which is accessed
 *
 * @param[in] parameter
 *  Specifies if the parameter value which is written to the XBee-Module
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::setConfigRegister( char *command, uint8_t parameter, bool queueing, bool moduleResponse )
{
	return configRegisterAccess( command, ( uint32_t* ) &parameter, 1, queueing, moduleResponse, true );
}


/****************************************************************************************************************************************//**
 * @brief
 * This method can be used to query any 8-Bit XBee-Configuration register
 *
 * @param[in] command[2]
 *  Character Array specifies the XBee-Register which is accessed
 *
 * @param[in] parameter
 *  The Pointer-Location contains the queried value which has been received from the XBee-Module
 *
 * @return responseStatus
 *  Command Status is returned OK(0), ERROR(1), Invalid Command(2), Invalid Parameter(3)
 *
 *******************************************************************************************************************************************/

uint8_t XBEE_Radio::getConfigRegister( char *command, uint8_t *parameter )
{
	return configRegisterAccess( command, ( uint32_t* ) parameter, 1, false, true, false );
}
