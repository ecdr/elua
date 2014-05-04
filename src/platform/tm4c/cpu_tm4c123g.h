// LM4F230 CPU definition  (Same as TMC4123gh6pm)

#ifndef __CPU_TM4C123G_H__
#define __CPU_TM4C123G_H__

#include <stdint.h> 
#include <stdbool.h>

#include "driverlib/sysctl.h"
#include "rom_map.h"
#include "stacks.h"
#include "hw_memmap.h"
#include "platform_ints.h"

// PART_ macro MUST be defined BEFORE including hw_ints.h or cpu header file
#include "tm4c123gh6pm.h"

#define NUM_PIO               6
#define NUM_SPI               4
#define NUM_UART              4	// Max is 8
#define NUM_TIMER             6	// Max is 12
#define NUM_PWM               16	// Fixme: implement/test
#define NUM_I2C			4	// ToDo: Not written yet for lm3
#define NUM_ADC               13	// Actually 12 channels, temperature sensor is last channel
#define NUM_CAN               1	// Max is 2

#define NUM_COMP			2	// ToDo: Not written yet
#define NUM_QEI			  2	// ToDo: Not written yet
// QEI0 PHA = PD6, PHB = PD7, IDX = PD3 (or PF0, PF1, PF4 )
// QEI1 PHA = PC5, PHB = PC6, IDX = PC4

// ToDo: Add support for USB
//#define NUM_USB             1

#define FORLM4F				// Common defines for LM4F

// *****************************************************************************
// Support for chips that have alternative pin mappings 

#define USE_PIN_MUX

/*
// Platform should define maximum number of item, and default number to use, let build specify a number <max
// This was draft at code for that from platform_conf.h

// UART - Max 3 unless LM4F120
#if defined(FORLM4F230)
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
#elif defined(FORLM4F)
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




#define ADC_BIT_RESOLUTION    12

// CAN - LM4F120 can use port B, E, or F
#define CAN_PORT	B

//CPU_FREQUENCY used to be a call to MAP_SysCtlClockGet()
// But that doesn't work in Tivaware 2.1.
// Had been optimized to use a cached value, but that didn't work either for some reason
// Using a constant for the moment
//extern u32 clockfreq;

// CPU frequency (needed by the CPU module and MMCFS code, 0 if not used)
//#define CPU_FREQUENCY         clockfreq
#define CPU_FREQUENCY         80000000

// PIO prefix ('0' for P0, P1, ... or 'A' for PA, PB, ...)
#define PIO_PREFIX            'A'
// Pins per port configuration:
// #define PIO_PINS_PER_PORT (n) if each port has the same number of pins, or
// #define PIO_PIN_ARRAY { n1, n2, ... } to define pins per port in an array
// Use #define PIO_PINS_PER_PORT 0 if this isn't needed
#define PIO_PIN_ARRAY         { 8, 8, 8, 8, 6, 5 }

// FIXME: check NUM_PIO versus PIO_PIN_ARRAY  (Check belongs where this is used, not in header)

//#if sizeof(PIO_PIN_ARRAY) != NUM_PIO
//#error NUM PIO not match PIO PIN_ARRAY 
//#endif

#define NUM_PWM_GEN   8


// Internal Flash data
#define INTERNAL_FLASH_SIZE             ( 256 * 1024 )
#define INTERNAL_FLASH_WRITE_UNIT_SIZE  4
#define INTERNAL_FLASH_SECTOR_SIZE      1024
#define INTERNAL_FLASH_START_ADDRESS    0

#define SRAM_SIZE ( 0x08000 )
// Allocator data: define your free memory zones here in two arrays
// (start address and end address)

// First_Free is a run time variable defined by linker, common.c defines it as type array of char
#define INTERNAL_RAM1_FIRST_FREE        end
#define INTERNAL_RAM1_LAST_FREE         ( SRAM_BASE + SRAM_SIZE - STACK_SIZE_TOTAL - 1 )

// Made additions in order from hw_ints.h
// Could add more timers (added some, but not full number from lm4F120)

// FIXME: 
// In Tivaware, need to have PART_macro defined before including hw_ints
// However PART_macro is defined in board file, which is included after cpu file.
// But can't include the board header before the cpu header

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
  _C( INT_UART_RX ),

/*  These would handle the wide timers on LM4F120 - add if needed, before UART_RX
  _C( INT_WTIMER0A ),\
  _C( INT_WTIMER0B ),\
  _C( INT_WTIMER1A ),\
  _C( INT_WTIMER1B ),\
  _C( INT_WTIMER2A ),\
  _C( INT_WTIMER2B ),\
  _C( INT_WTIMER3A ),\
  _C( INT_WTIMER3B ),\
  _C( INT_WTIMER4A ),\
  _C( INT_WTIMER4B ),\
  _C( INT_WTIMER5A ),\
  _C( INT_WTIMER5B ),\  */




#endif // #ifndef __CPU_TM4C123G_H__
