// TM4C129 CPU definition

#ifndef __CPU_TM4C1294_H__
#define __CPU_TM4C1294_H__

#include <stdint.h> 
#include <stdbool.h>

#include "driverlib/sysctl.h"
#include "rom_map.h"
#include "stacks.h"
#include "hw_memmap.h"
#include "platform_ints.h"
//#include "hw_ints.h"

#include "tm4c1294ncpdt.h"

// NUM_PIO has to include the ports with no pins (i.e. size of pio_port_pins)
#define NUM_PIO               17

#define NUM_SPI               4
#define NUM_UART              8	// Max is 8
// UART 0-4 have flow control support
#define NUM_TIMER             8	// Max is 12
// Hardware can do 8 32 bit, or 16 16 bit timers
#define NUM_PWM               8 // Max is 16
#define NUM_I2C			          10	// ToDo: Not written yet for lm3
// TODO: How many I2C does code support?
#define NUM_ADC               21	// Actually 20 channels, temperature sensor is last channel

#define NUM_CAN               2	// Max is 2

#define NUM_COMP			3	// ToDo: Not written yet
#define NUM_QEI			  1	// ToDo: Not written yet

// ToDo: Add support for USB
// #define NUM_USB             1


#define FORLM4F				// Common defines for LM4F/TM4C


/*
// Platform should define maximum number of item, and default number to use, let build specify a number <max
// This was draft at code for that from platform_conf.h

// UART - Max 3 unless LM4F
#if defined (FORLM4F)
  #define MAX_UART            8
#else
  #define MAX_UART 		3
#endif

// Define in configuration script if want to use different number
#ifndef NUM_UART
#if defined( FORLM3S6965 )
  #define NUM_UART            3
#elif defined( FORLM3S9B92 ) || defined( FORLM3S9D92 )
  #define NUM_UART            3
#elif defined (FORLM4F)
  #define NUM_UART            4
//  LM4F120 actually has 8, but UART number 6 uses same pins as USB
#else
  #define NUM_UART            2
#endif
#endif

#if NUM_UART > MAX_UART
#warning Too many UART, maximum is MAX_UART
#define NUM_UART MAX_UART
#endif


// NUM_TIMERS - Max 4 unless LM4F
#ifdef FORLM4F
  #define MAX_TIMER            12
#else
  #define MAX_TIMER             4
#endif

#ifndef NUM_TIMER
  #define NUM_TIMER             6
// LM4F actually has 12 (6 normal, 6 wide), but haven't added all of code to handle wide timers
// Using wide timers for PWMs
#else
  #define NUM_TIMER             4
#endif

#if NUM_TIMER > MAX_TIMER
#warning Too many TIMER, maximum is MAX_TIMER
#define NUM_TIMER MAX_TIMER
#endif
*/

// *****************************************************************************
// Support for chips that have alternative pin mappings 
// TODO: probably need for other CPU that use pinmux - suspect applies to ELUA_CPU_LM3S9B92 and ELUA_CPU_LM3S9D92

// From platform_conf.h - much of this may already be in other places, just haven't found it
/*
#if defined( FORLM4F )|| defined( ELUA_BOARD_SOLDERCORE )
#define USE_PIN_MUX
#endif

// lm3s_pio.c has specifics for alternative pin mapping in eLua


// Include LM3S GPIO functions in platform map
#define ENABLE_LM3S_GPIO

*/


#define ADC_BIT_RESOLUTION    12

// Error checking on clock frequency
#define CPU_MAX_FREQUENCY         120000000

extern unsigned long clockfreq;

// Get CPU frequency (needed by the CPU module and MMCFS code, 0 if not used)
#define CPU_FREQUENCY         clockfreq

// PIO prefix ('0' for P0, P1, ... or 'A' for PA, PB, ...)
#define PIO_PREFIX            'A'
// Pins per port configuration:
// #define PIO_PINS_PER_PORT (n) if each port has the same number of pins, or
// #define PIO_PIN_ARRAY { n1, n2, ... } to define pins per port in an array
// Use #define PIO_PINS_PER_PORT 0 if this isn't needed
//                              A  B  C  D  E  F  G  H  I  J  K  L  M  N  O  P  Q  R  S  T
//                              0  1  2  3  4  5  6  7     9 10 11 12 13    15 16 17 18 19
#define PIO_PIN_ARRAY         { 8, 6, 8, 8, 6, 5, 2, 4, 0, 2, 8, 8, 8, 2, 0, 6, 5 }

// Note: Letter assignments for ports are not contiguous, use 0 to indicate unused port letter

// FIXME: add check for NUM_PIO versus PIO_PIN_ARRAY  (Check belongs where this is used, not in header)

//#if sizeof(PIO_PIN_ARRAY) != NUM_PIO
//#error NUM PIO not match PIO PIN_ARRAY 
//#endif

#define NUM_PWM_GEN 4

// Internal Flash data
#define INTERNAL_FLASH_SIZE             ( 1024 * 1024 )

// Flash writes must be word alligned
#define INTERNAL_FLASH_WRITE_UNIT_SIZE  4
// Erase block size
#define INTERNAL_FLASH_SECTOR_SIZE      (16 * 1024)
#define INTERNAL_FLASH_START_ADDRESS    0

#define SRAM_SIZE ( 256 * 1024 )
// Allocator data: define your free memory zones here in two arrays
// (start address and end address)

// First_Free is a run time variable defined by linker, common.c defines it as type array of char
#define INTERNAL_RAM1_FIRST_FREE        end
#define INTERNAL_RAM1_LAST_FREE         ( SRAM_BASE + SRAM_SIZE - STACK_SIZE_TOTAL - 1 )

// EEPROM
#define EEPROM_SIZE				( 6 * 1024 )

// FixMe: Not sure how to tie this in to the platform code
#define PLATFORM_EEPROM_SIZE			EEPROM_SIZE

//#define PLATFORM_EEPROM_BLOCK_SIZE	4


// FIXME: Defines for LEDs, etc. should be in the board file, rather than in CPU

#define PLATFORM_PIO_CONST \
  LUA_CONST( "LED1", PLATFORM_IO_ENCODE( 13, 1, 0 ) ) \
  LUA_CONST( "LED1ON", 1) \
  LUA_CONST( "LED2", PLATFORM_IO_ENCODE( 13, 0, 0 ) ) \
  LUA_CONST( "LED2ON", 1) \
  LUA_CONST( "LED_ETH0", PLATFORM_IO_ENCODE( 5, 4, 0 ) ) \
  LUA_CONST( "LED_ETH0ON", 1) \
  LUA_CONST( "LED_ETH1", PLATFORM_IO_ENCODE( 5, 0, 0 ) ) \
  LUA_CONST( "LED_ETH1ON", 1) \
  LUA_CONST( "SW1", PLATFORM_IO_ENCODE( 9, 0, 0 ) ) \
  LUA_CONST( "SW1PULL", PLATFORM_IO_PIN_PULLUP ) \
  LUA_CONST( "SW2", PLATFORM_IO_ENCODE( 9, 1, 0 ) ) \
  LUA_CONST( "SW2PULL", PLATFORM_IO_PIN_PULLUP )

// TODO: PWM const not implemented yet
#define PLATFORM_PWM_CONST \
   LUA_CONST( "LED1", 0)      // Actually ethernet LED0

#define PLATFORM_ADC_CONST \
  LUA_CONST( "TEMP", 20)


// Made additions in order from hw_ints.h
// Could add more timers (added some, but not full number from lm4F120)

#define PLATFORM_CPU_CONSTANTS_INTS\
  _C( INT_SSI2 ),\
  _C( INT_SSI3 ),\
  _C( INT_UART3 ),\
  _C( INT_UART4 ),\
  _C( INT_UART5 ),\
  _C( INT_UART6 ),\
  _C( INT_UART7 ),\
  _C( INT_I2C2 ),\
  _C( INT_I2C3 ),\
  _C( INT_TIMER4A ),\
  _C( INT_TIMER4B ),\
  _C( INT_TIMER5A ),\
  _C( INT_TIMER5B ),\
  _C( INT_UART_RX ),\
  _C( INT_GPIO_POSEDGE ),\
  _C( INT_GPIO_NEGEDGE ),\
  _C( INT_TMR_MATCH ),


#endif // #ifndef __CPU_TM4C1294_H__


