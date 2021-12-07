/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_MOTEINO_M4_
#define _VARIANT_MOTEINO_M4_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK        (F_CPU)

#define VARIANT_GCLK0_FREQ (F_CPU)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (50u)
#define NUM_DIGITAL_PINS     PINS_COUNT
#define NUM_ANALOG_INPUTS    (18u)
#define NUM_ANALOG_OUTPUTS   (2u)
#define analogInputToDigitalPin(p)  (p)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( (g_APinDescription[P].ulPinAttribute & PIN_ATTR_TIMER_PWM) == PIN_ATTR_TIMER_PWM )
//#define digitalPinToInterrupt(P)   ( g_APinDescription[P].ulExtInt )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LEDs
#define PIN_LED_37           (37u)
#define LED_BUILTIN          PIN_LED_37
#define SS_FLASHMEM          (40u)

#define EXTERNAL_FLASH_USE_SPI SPI
#define EXTERNAL_FLASH_USE_CS  SS_FLASHMEM

/*
 * Analog pins
 */
#define PIN_A0               (0ul)
#define PIN_A1               (1ul)
#define PIN_A2               (2ul)
#define PIN_A3               (3ul)
#define PIN_A4               (4ul)
#define PIN_A5               (5ul)
#define PIN_A6               (6ul)
#define PIN_A7               (7ul)
#define PIN_A8               (8ul)
#define PIN_A9               (9ul)
#define PIN_A10              (10ul)
#define PIN_A11              (11ul)
#define PIN_A34              (34ul)
#define PIN_A35              (35ul)
#define PIN_A36              (36ul)
#define PIN_A37              (37ul)
#define PIN_A38              (38ul)
#define PIN_A49              (49ul)
#define PIN_DAC0             (2ul)
#define PIN_DAC1             (5ul)

#define PA09 (38ul)

static const uint8_t A0   = PIN_A0;
static const uint8_t A1   = PIN_A1;
static const uint8_t A2   = PIN_A2;
static const uint8_t A3   = PIN_A3;
static const uint8_t A4   = PIN_A4;
static const uint8_t A5   = PIN_A5;
static const uint8_t A6   = PIN_A6;
static const uint8_t A7   = PIN_A7;
static const uint8_t A8   = PIN_A8;
static const uint8_t A9   = PIN_A9;
static const uint8_t A10  = PIN_A10;
static const uint8_t A11  = PIN_A11;
static const uint8_t A34  = PIN_A34;
static const uint8_t A35  = PIN_A35;
static const uint8_t A36  = PIN_A36;
static const uint8_t A37  = PIN_A37;
static const uint8_t A38  = PIN_A38;
static const uint8_t A49  = PIN_A49;
static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		12

/* Set default analog voltage reference */
#define VARIANT_AR_DEFAULT	AR_DEFAULT

/* Reference voltage pins (define even if not enabled with PIN_ATTR_REF in the PinDescription table) */
#define REFA_PIN    (3ul)
#define REFB_PIN    (4ul)
#if (SAMD51)
#define REFC_PIN    (35ul)
#endif

// Other pins
#define PIN_ATN              (21ul)
static const uint8_t ATN = PIN_ATN;


/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_RX       (9ul)
#define PIN_SERIAL1_TX       (8ul)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)
//#define SERCOM_INSTANCE_SERIAL1       &sercom4
  

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT     1

#define PIN_SPI_MISO         (21u)
#define PIN_SPI_MOSI         (23u)
#define PIN_SPI_SCK          (22u)
#define PIN_SPI_SS           (20u)
#define PERIPH_SPI           sercom5
#define PAD_SPI_TX           SPI_PAD_0_SCK_1  //MISO=SERCOMn.0, SCK=SERCOMn.1  //https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-spi#samd51-spi-pin-pads-3-15
#define PAD_SPI_RX           SERCOM_RX_PAD_3  //SCK=SERCOMn.3
#define SS                   SS               //ensure that any libs/sketches which call "#ifndef(SS)..." won't fail because SS is a variable and not a #define
static const uint8_t SS   = PIN_SPI_SS ;        // The SERCOM SS PAD is available on this pin but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (16u)
#define PIN_WIRE_SCL         (17u)
#define PERIPH_WIRE          sercom1
#define WIRE_IT_HANDLER      SERCOM1_Handler
#define WIRE_IT_HANDLER_0    SERCOM1_0_Handler
#define WIRE_IT_HANDLER_1    SERCOM1_1_Handler
#define WIRE_IT_HANDLER_2    SERCOM1_2_Handler
#define WIRE_IT_HANDLER_3    SERCOM1_3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE             (14ul)
#define PIN_USB_DM                      (24ul)
#define PIN_USB_DP                      (25ul)
#define PIN_USB_HOST_ENABLE_VALUE	0

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0

/*
#define I2S_DEVICE          0
#define I2S_CLOCK_GENERATOR 3

#define PIN_I2S_SDO          (11u)
#define PIN_I2S_SDI          (12u)
#define PIN_I2S_SCK          PIN_SERIAL1_TX
#define PIN_I2S_FS           (10u)
#define PIN_I2S_MCK          PIN_SERIAL1_RX
*/

//QSPI Pins
#define PIN_QSPI_SCK	(39u)
#define PIN_QSPI_CS	  (40u)
#define PIN_QSPI_IO0	(37u)
#define PIN_QSPI_IO1	(38u)
#define PIN_QSPI_IO2	(10u)
#define PIN_QSPI_IO3	(11u)

#if !defined(VARIANT_QSPI_BAUD_DEFAULT)
  // TODO: meaningful value for this
  #define VARIANT_QSPI_BAUD_DEFAULT 5000000
#endif

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

extern Uart Serial1;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_MOTEINO_M4_ */

