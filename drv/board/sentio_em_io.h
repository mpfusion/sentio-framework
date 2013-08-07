//######################################################################
// Connector Pin Description Sentio-em V3.2 
// Sensor Connector
//######################################################################

#define _SENSOR_PORT_1     gpioPortB
#define _SENSOR_PIN_1      7
#define SENSOR_PIN_1       _SENSOR_PORT_1,_SENSOR_PIN_1

#define _SENSOR_PORT_2     gpioPortA
#define _SENSOR_PIN_2      11
#define SENSOR_PIN_2       _SENSOR_PORT_2,_SENSOR_PIN_2

#define _SENSOR_PORT_3     gpioPortA
#define _SENSOR_PIN_3      12
#define SENSOR_PIN_3       _SENSOR_PORT_3,_SENSOR_PIN_3

#define _SENSOR_PORT_4     gpioPortA
#define _SENSOR_PIN_4      13
#define SENSOR_PIN_4       _SENSOR_PORT_4,_SENSOR_PIN_4

#define _SENSOR_PORT_5     gpioPortB
#define _SENSOR_PIN_5      9
#define SENSOR_PIN_5       _SENSOR_PORT_5,_SENSOR_PIN_5

#define _SENSOR_PORT_6     gpioPortC
#define _SENSOR_PIN_6      1
#define SENSOR_PIN_6       _SENSOR_PORT_6,_SENSOR_PIN_6

#define _SENSOR_PORT_7     gpioPortC
#define _SENSOR_PIN_7      0
#define SENSOR_PIN_7       _SENSOR_PORT_7,_SENSOR_PIN_7

#define _SENSOR_PORT_8     gpioPortA
#define _SENSOR_PIN_8      6
#define SENSOR_PIN_8       _SENSOR_PORT_8,_SENSOR_PIN_8

#define _SENSOR_PORT_9     gpioPortA
#define _SENSOR_PIN_9      5
#define SENSOR_PIN_9       _SENSOR_PORT_9,_SENSOR_PIN_9

#define _SENSOR_PORT_10    gpioPortA
#define _SENSOR_PIN_10     14
#define SENSOR_PIN_10      _SENSOR_PORT_10,_SENSOR_PIN_10

#define _SENSOR_PORT_11    gpioPortB
#define _SENSOR_PIN_11     11
#define SENSOR_PIN_11      _SENSOR_PORT_11,_SENSOR_PIN_11

#define _SENSOR_PORT_12    gpioPortC
#define _SENSOR_PIN_12     6
#define SENSOR_PIN_12      _SENSOR_PORT_12,_SENSOR_PIN_12

#define _SENSOR_PORT_13    gpioPortB
#define _SENSOR_PIN_13     12
#define SENSOR_PIN_13      _SENSOR_PORT_13,_SENSOR_PIN_13

#define _SENSOR_PORT_14    gpioPortC
#define _SENSOR_PIN_14     7
#define SENSOR_PIN_14      _SENSOR_PORT_14,_SENSOR_PIN_14

#define _SENSOR_PORT_17    gpioPortD
#define _SENSOR_PIN_17     11
#define SENSOR_PIN_17      _SENSOR_PORT_17,_SENSOR_PIN_17

#define _SENSOR_PORT_18    gpioPortD
#define _SENSOR_PIN_18     6
#define SENSOR_PIN_18      _SENSOR_PORT_18,_SENSOR_PIN_18

#define _SENSOR_PORT_19    gpioPortD
#define _SENSOR_PIN_19     12
#define SENSOR_PIN_19      _SENSOR_PORT_19,_SENSOR_PIN_19

#define _SENSOR_PORT_20    gpioPortD
#define _SENSOR_PIN_20     7
#define SENSOR_PIN_20      _SENSOR_PORT_20,_SENSOR_PIN_20

#define SENSOR_LEUART_RX   SENSOR_PIN_8
#define SENSOR_LEUART_TX   SENSOR_PIN_9
#define SENSOR_LEUART      LEUART1
#define SENSOR_LEUART_LOC  LEUART_ROUTE_LOCATION_LOC0

#define SENSOR_UART_RX     SENSOR_PIN_6
#define SENSOR_UART_TX     SENSOR_PIN_7
#define SENSOR_UART        USART1
#define SENSOR_UART_LOC    USART_ROUTE_LOCATION_LOC0

#define SENSOR_I2C_SDA     SENSOR_PIN_12
#define SENSOR_I2C_SCL     SENSOR_PIN_14
#define SENSOR_I2C         I2C0
#define SENSOR_I2C_LOC     I2C_ROUTE_LOCATION_LOC2

#define SENSOR_USART_RX    SENSOR_PIN_17
#define SENSOR_USART_TX    SENSOR_PIN_18
#define SENSOR_USART_CS    SENSOR_PIN_20
#define SENSOR_USART_CLK   SENSOR_PIN_19
#define SENSOR_USART       USART1
#define SENSOR_USART_LOC   USART_ROUTE_LOCATION_LOC0

#define SENSOR_ADC_0       SENSOR_PIN_17
#define SENSOR_ADC_1       SENSOR_PIN_18
#define SENSOR_ADC_2       SENSOR_PIN_19
#define SENSOR_ADC_3       SENSOR_PIN_20

//######################################################################
// Connector Pin Description Sentio-em V3.2 
// Power Connector
//######################################################################

#define _POWER_PORT_1      gpioPortD
#define _POWER_PIN_1       7
#define POWER_PIN_1        _POWER_PORT_1,_POWER_PIN_1

#define _POWER_PORT_2      gpioPortD
#define _POWER_PIN_2       5
#define POWER_PIN_2        _POWER_PORT_2,_POWER_PIN_2

#define _POWER_PORT_3      gpioPortD
#define _POWER_PIN_3       6
#define POWER_PIN_3        _POWER_PORT_3,_POWER_PIN_3

#define _POWER_PORT_4      gpioPortD
#define _POWER_PIN_4       4
#define POWER_PIN_4        _POWER_PORT_4,_POWER_PIN_4

#define _POWER_PORT_6      gpioPortD
#define _POWER_PIN_6       8
#define POWER_PIN_6        _POWER_PORT_6,_POWER_PIN_6

#define _POWER_PORT_7      gpioPortE
#define _POWER_PIN_7       1
#define POWER_PIN_7        _POWER_PORT_7,_POWER_PIN_7

#define _POWER_PORT_8      gpioPortE
#define _POWER_PIN_8       2
#define POWER_PIN_8        _POWER_PORT_8,_POWER_PIN_8

#define _POWER_PORT_9      gpioPortE
#define _POWER_PIN_9       3
#define POWER_PIN_9        _POWER_PORT_9,_POWER_PIN_9

#define _POWER_PORT_10     gpioPortE
#define _POWER_PIN_10      4
#define POWER_PIN_10       _POWER_PORT_10,_POWER_PIN_10

#define _POWER_PORT_11     gpioPortE
#define _POWER_PIN_11      5
#define POWER_PIN_11       _POWER_PORT_11,_POWER_PIN_11

#define _POWER_PORT_12     gpioPortE
#define _POWER_PIN_12      6
#define POWER_PIN_12       _POWER_PORT_12,_POWER_PIN_12

#define _POWER_PORT_13     gpioPortE
#define _POWER_PIN_13      7
#define POWER_PIN_13       _POWER_PORT_13,_POWER_PIN_13

#define POWER_LEUART_RX   POWER_PIN_2
#define POWER_LEUART_TX   POWER_PIN_4
#define POWER_LEUART      LEUART0
#define POWER_LEUART_LOC  LEUART_ROUTE_LOCATION_LOC0

#define POWER_UART_RX     POWER_PIN_12
#define POWER_UART_TX     POWER_PIN_13
#define POWER_UART        USART0
#define POWER_UART_LOC    USART_ROUTE_LOCATION_LOC1

#define POWER_I2C_SDA     POWER_PIN_7
#define POWER_I2C_SCL     POWER_PIN_8
#define POWER_I2C         I2C0
#define POWER_I2C_LOC     I2C_ROUTE_LOCATION_LOC1

#define POWER_USART_RX    POWER_PIN_12
#define POWER_USART_TX    POWER_PIN_13
#define POWER_USART_CS    POWER_PIN_10
#define POWER_USART_CLK   POWER_PIN_11
#define POWER_USART       USART0
#define POWER_USART_LOC   USART_ROUTE_LOCATION_LOC1

#define POWER_ADC_4       POWER_PIN_4
#define POWER_ADC_5       POWER_PIN_2
#define POWER_ADC_6       POWER_PIN_3
#define POWER_ADC_7       POWER_PIN_1

//######################################################################
// Connector Pin Description Sentio-em V3.2 
// Debug Connector
//######################################################################

#define _DEBUG_PORT_1     gpioPortD     
#define _DEBUG_PIN_1      10
#define DEBUG_PIN_1       _DEBUG_PORT_1,_DEBUG_PIN_1

#define _DEBUG_PORT_2     gpioPortA   
#define _DEBUG_PIN_2      15
#define DEBUG_PIN_2       _DEBUG_PORT_2,_DEBUG_PIN_2

#define _DEBUG_PORT_3     gpioPortD
#define _DEBUG_PIN_3      9
#define DEBUG_PIN_3       _DEBUG_PORT_3,_DEBUG_PIN_3

#define _DEBUG_PORT_5     gpioPortE   
#define _DEBUG_PIN_5      14
#define DEBUG_PIN_5       _DEBUG_PORT_5,_DEBUG_PIN_5

#define _DEBUG_PORT_6     gpioPortE
#define _DEBUG_PIN_6      15
#define DEBUG_PIN_6       _DEBUG_PORT_6,_DEBUG_PIN_6

#define _DEBUG_PORT_7     gpioPortE
#define _DEBUG_PIN_7      12
#define DEBUG_PIN_7       _DEBUG_PORT_7,_DEBUG_PIN_7

#define _DEBUG_PORT_8     gpioPortE
#define _DEBUG_PIN_8      13
#define DEBUG_PIN_8       _DEBUG_PORT_8,_DEBUG_PIN_8

#define _DEBUG_PORT_9     gpioPortE
#define _DEBUG_PIN_9      10
#define DEBUG_PIN_9       _DEBUG_PORT_9,_DEBUG_PIN_9

#define _DEBUG_PORT_10     gpioPortE
#define _DEBUG_PIN_10      11
#define DEBUG_PIN_10       _DEBUG_PORT_10,_DEBUG_PIN_10

#define _DEBUG_PORT_11     gpioPortB
#define _DEBUG_PIN_11      2
#define DEBUG_PIN_11       _DEBUG_PORT_11,_DEBUG_PIN_11

#define _DEBUG_PORT_12     gpioPortB
#define _DEBUG_PIN_12      0
#define DEBUG_PIN_12       _DEBUG_PORT_12,_DEBUG_PIN_12

#define _DEBUG_PORT_13     gpioPortA
#define _DEBUG_PIN_13      0
#define DEBUG_PIN_13       _DEBUG_PORT_13,_DEBUG_PIN_13

#define _DEBUG_PORT_14     gpioPortB
#define _DEBUG_PIN_14      1
#define DEBUG_PIN_14      _DEBUG_PORT_14,_DEBUG_PIN_14

#define _DEBUG_PORT_15     gpioPortA
#define _DEBUG_PIN_15      2
#define DEBUG_PIN_15       _DEBUG_PORT_15,_DEBUG_PIN_15

#define _DEBUG_PORT_16     gpioPortA
#define _DEBUG_PIN_16      1
#define DEBUG_PIN_16       _DEBUG_PORT_16,_DEBUG_PIN_16

#define _DEBUG_PORT_17     gpioPortA
#define _DEBUG_PIN_17      4
#define DEBUG_PIN_17       _DEBUG_PORT_15,_DEBUG_PIN_17

#define _DEBUG_PORT_18     gpioPortA
#define _DEBUG_PIN_18      3
#define DEBUG_PIN_18       _DEBUG_PORT_18,_DEBUG_PIN_18

#define _DEBUG_PORT_19     gpioPortF
#define _DEBUG_PIN_19      6
#define DEBUG_PIN_19       _DEBUG_PORT_19,_DEBUG_PIN_19

#define _DEBUG_PORT_20     gpioPortF
#define _DEBUG_PIN_20      7
#define DEBUG_PIN_20       _DEBUG_PORT_20,_DEBUG_PIN_20

#define _DEBUG_PORT_21     gpioPortF
#define _DEBUG_PIN_21      4
#define DEBUG_PIN_21       _DEBUG_PORT_21,_DEBUG_PIN_21

#define _DEBUG_PORT_22     gpioPortF
#define _DEBUG_PIN_22      5
#define DEBUG_PIN_22       _DEBUG_PORT_22,_DEBUG_PIN_22

#define _DEBUG_PORT_24     gpioPortF
#define _DEBUG_PIN_24      3
#define DEBUG_PIN_24       _DEBUG_PORT_24,_DEBUG_PIN_24

#define _DEBUG_PORT_28     gpioPortC
#define _DEBUG_PIN_28      15
#define DEBUG_PIN_28       _DEBUG_PORT_20,_DEBUG_PIN_20

#define _DEBUG_PORT_29     gpioPortC
#define _DEBUG_PIN_29      14
#define DEBUG_PIN_29       _DEBUG_PORT_29,_DEBUG_PIN_29

#define _DEBUG_PORT_30     gpioPortC
#define _DEBUG_PIN_30      13
#define DEBUG_PIN_30       _DEBUG_PORT_30,_DEBUG_PIN_30

#define _DEBUG_PORT_31     gpioPortC
#define _DEBUG_PIN_31      12
#define DEBUG_PIN_31       _DEBUG_PORT_31,_DEBUG_PIN_31

#define _DEBUG_PORT_32     gpioPortC
#define _DEBUG_PIN_32      11
#define DEBUG_PIN_32       _DEBUG_PORT_32,_DEBUG_PIN_32

#define DEBUG_LEUART_RX    POWER_PIN_6
#define DEBUG_LEUART_TX    POWER_PIN_5
#define DEBUG_LEUART       LEUART0
#define DEBUG_LEUART_LOC   LEUART_ROUTE_LOCATION_LOC2

#define DEBUG_UART_RX      DEBUG_PIN_20
#define DEBUG_UART_TX      DEBUG_PIN_19
#define DEBUG_UART         UART0
#define DEBUG_UART_LOC     UART_ROUTE_LOCATION_LOC0

//######################################################################
// Pin Description Sentio-em V3.2 
// XBEE compatible Radio Slot
//######################################################################

#define _RADIO_PWR_PORT    gpioPortD
#define _RADIO_PWR_PIN     12
#define RADIO_PWR_PIN      _RADIO_PWR_PORT,_RADIO_PWR_PIN

#define _RADIO_RXTX_PORT   gpioPortF
#define _RADIO_RXTX_PIN    8
#define RADIO_RXTX_PIN     _RADIO_RXTX_PORT,_RADIO_RXTX_PIN

#define _RADIO_RESET_PORT  gpioPortE
#define _RADIO_RESET_PIN   9
#define RADIO_RESET_PIN    _RADIO_RESET_PORT,_RADIO_RESET_PIN

#define RADIO_DI0          DEBUG_PIN_11
#define RADIO_DI1          DEBUG_PIN_14
#define RADIO_DI2          DEBUG_PIN_12

#define RADIO_DI5          DEBUG_PIN_1
#define RADIO_DI3          DEBUG_PIN_2
#define RADIO_DI4          DEBUG_PIN_3

#define RADIO_CLK          DEBUG_PIN_7
#define RADIO_CS           DEBUG_PIN_8
#define RADIO_TX           DEBUG_PIN_9
#define RADIO_RX           DEBUG_PIN_10
#define RADIO_USART        USART0
#define RADIO_USART_LOC    USART_ROUTE_LOCATION_LOC0

//######################################################################
// Pin Description Sentio-em V3.2
// Interface to Realtime Counter DS3234
//######################################################################

#define _RTC_32KHZ_PORT    gpioPortB
#define _RTC_32KHZ_PIN     8
#define RTC_32KHZ_PIN      _RTC_32KHZ_PORT,_RTC_32KHZ_PIN

#define _RTC_INT_PORT      gpioPortA
#define _RTC_INT_PIN       7
#define RTC_INT_PIN        _RTC_INT_PORT,_RTC_INT_PIN

#define _RTC_VDD_PORT      gpioPortA
#define _RTC_VDD_PIN       8
#define RTC_VDD_PIN        _RTC_VDD_PORT,_RTC_VDD_PIN

#define _RTC_VBAT_PORT     gpioPortA
#define _RTC_VBAT_PIN      9
#define RTC_VBAT_PIN       _RTC_VBAT_PORT,_RTC_VBAT_PIN

#define _RTC_CLK_PORT      gpioPortC
#define _RTC_CLK_PIN       4
#define RTC_CLK_PIN       _RTC_CLK_PORT,_RTC_CLK_PIN

#define _RTC_CS_PORT       gpioPortC
#define _RTC_CS_PIN        5
#define RTC_CS_PIN         _RTC_CS_PORT,_RTC_CS_PIN

#define _RTC_RX_PORT       gpioPortC
#define _RTC_RX_PIN        3
#define RTC_RX_PIN         _RTC_RX_PORT,_RTC_RX_PIN

#define _RTC_TX_PORT       gpioPortC
#define _RTC_TX_PIN        2
#define RTC_TX_PIN         _RTC_TX_PORT,_RTC_TX_PIN

#define RTC_USART          USART2
#define RTC_USART_LOC      USART_ROUTE_LOCATION_LOC2

//######################################################################
// Pin Description Sentio-em V3.2
// Interface to Micro SD Card Slot
//######################################################################

#define _SD_PWR_PORT    gpioPortA
#define _SD_PWR_PIN     10
#define SD_PWR_PIN      _SD_PWR_PORT,_SD_PWR_PIN

#define _SD_DET_PORT    gpioPortD
#define _SD_DET_PIN     8
#define SD_DET_PIN      _SD_INT_PORT,_SD_INT_PIN

#define _SD_CLK_PORT    gpioPortB
#define _SD_CLK_PIN     5
#define SD_CLK_PIN      _SD_CLK_PORT,_SD_CLK_PIN

#define _SD_CS_POT      gpioPortB
#define _SD_CS_PIN      6
#define SD_CS_PIN       _SD_CS_PORT,_SD_CS_PIN

#define _SD_RX_PORT     gpioPortB
#define _SD_RX_PIN      4
#define SD_RX_PIN       _SD_RX_PORT,_SD_RX_PIN

#define _SD_TX_PORT     gpioPortB
#define _SD_TX_PIN      3
#define SD_TX_PIN       SD_TX_PORT,_SD_TX_PIN

#define SD_USART        USART2
#define SD_USART_LOC    USART_ROUTE_LOCATION_LOC1

//######################################################################
// Pin Description Sentio-em V3.2
// On-board LEDS, Button and Reed Contact
//######################################################################

#define _RED_PORT       gpioPortC
#define _RED_PIN        8
#define RED             _RED_PORT,_RED_PIN

#define _ORANGE_PORT    gpioPortC
#define _ORANGE_PIN     9
#define ORANGE          _ORANGE_PORT,_ORANGE_PIN

#define _GREEN_PORT     gpioPortC
#define _GREEN_PIN      9
#define GREEN           _GREEN_PORT,_GREEN_PIN

#define _BUTTON_PORT    gpioPortE
#define _BUTTON_PIN     0
#define BUTTON          _BUTTON_PORT,_BUTTON_PIN 

#define _REED_PORT      gpioPortB
#define _REED_PIN       10
#define REED_PORT      _REED_PORT,_REED_PIN


#define _CMU_HF_PORT    gpioPortD
#define _CMU_HF_PIN     12
#define CMU_HF_PIN          _CMU_HF_PORT,_CMU_HF_PIN 

#define _CMU_LF_PORT    gpioPortC
#define _CMU_LF_PIN     8
#define CMU_LF_PIN     _CMU_LF_PORT,_CMU_LF_PIN