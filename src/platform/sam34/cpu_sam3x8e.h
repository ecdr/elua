// AT91SAM3x8e CPU definition

#ifndef __CPU_AT91SAM3X8E__
#define __CPU_AT91SAM3X8E__

#include "platform_ints.h"
#include "stacks.h"
//#include <asf.h>
#include <sysclk.h>

// Number of resources (0 if not available/not implemented)
#define NUM_PIO               4

// 1 SPI device (well maybe 2, but port/pins for second are not available)
// Each device has 4 chip selects (only 3 brought out to header, but could bit bang CS on fourth)
#define NUM_SPI_DEV           1
#define NUM_SPI               3

#define NUM_UART              3
#define NUM_TIMER             9

// NUM_REAL_PWM is number of PWM devices
#define NUM_REAL_PWM          8

#ifndef NUM_TIMER_PWM
#define NUM_TIMER_PWM        0
#endif

// NUM_PWM is number of PWM channels (includes those provided by timers, or even virtual PWMs)
#define NUM_PWM               (NUM_REAL_PWM + NUM_TIMER_PWM)

#define NUM_I2C               2
#define NUM_CAN               2
// Only 1 ADC device, 15 ADC pins, plus thermometer
#define NUM_ADC               16  
// TODO: Add support for temperature from ADC

#define NUM_DAC			          2
#define NUM_I2S               1

#define ADC_BIT_RESOLUTION    12

#define DAC_BIT_RESOLUTION    DACC_RESOLUTION

// CPU frequency (needed by the CPU module and MMCFS code, 0 if not used)
#define CPU_FREQUENCY         sysclk_get_cpu_hz()
// FIXME - check if this is right call 
// 84*1000*1000
// Fixme - find CPU frequency (should be system call for it)

// PIO prefix ('0' for P0, P1, ... or 'A' for PA, PB, ...)
#define PIO_PREFIX            'A'
// Pins per port configuration:
// #define PIO_PINS_PER_PORT (n) if each port has the same number of pins, or
// #define PIO_PIN_ARRAY { n1, n2, ... } to define pins per port in an array
// Use #define PIO_PINS_PER_PORT 0 if this isn't needed
#define PIO_PIN_ARRAY         { 30, 32, 31, 11 }

// Internal Flash data
#define INTERNAL_FLASH_SIZE             IFLASH_SIZE
//#define INTERNAL_FLASH_SIZE             ( 512 * 1024 )
//#define INTERNAL_FLASH_WRITE_UNIT_SIZE  
#define INTERNAL_FLASH_SECTOR_SIZE      IFLASH0_PAGE_SIZE 
//#define INTERNAL_FLASH_START_ADDRESS    0x00080000
#define INTERNAL_FLASH_START_ADDRESS    IFLASH0_ADDR
// FIXME: Need to get write unit size and sector size before can use WOFS
// TODO: Consider reading flash size, RAM size from chipid?


// TODO: Check RAM origin - (header says something else, but this may be because of remapping)
#define SRAM_ORIGIN           0x20070000
#ifndef SRAM_SIZE
//#define SRAM_SIZE             0x18000
#define SRAM_SIZE             IRAM_SIZE
#endif

// NFC_RAM_ADDR /**< NAND Flash Controller RAM base address */

// In order for the following macro to work you need this declaration
extern char end[];

#define INTERNAL_RAM1_FIRST_FREE end
#define INTERNAL_RAM1_LAST_FREE  ( SRAM_ORIGIN + SRAM_SIZE - STACK_SIZE_TOTAL - 1 )

#define PLATFORM_CPU_CONSTANTS_INTS\
  _C( INT_UART_RX ),\
  _C( INT_GPIO_POSEDGE ),\
  _C( INT_GPIO_NEGEDGE ),\
  _C( INT_TMR_MATCH ),

// Standard pin names - seem more logical in either PIO or BD, rather than CPU
// PLATFORM_BOARD_CONST_PINS

/*
#define PLATFORM_BOARD_CONST_PINS \
  LUA_CONST( "LED1", PLATFORM_IO_ENCODE( 1, 27, 0 ) ) \
  LUA_CONST( "LED1ON", 1) \
  LUA_CONST( "LED2", PLATFORM_IO_ENCODE( 0, 21, 0 ) ) \
  LUA_CONST( "LED2ON", 0) \
  LUA_CONST( "LED3", PLATFORM_IO_ENCODE( 2, 30, 0 ) ) \
  LUA_CONST( "LED3ON", 0)
  */

#define LED1 PLATFORM_IO_ENCODE( 1, 27, 0 )
#define LED2 PLATFORM_IO_ENCODE( 0, 21, 0 )
#define LED3 PLATFORM_IO_ENCODE( 2, 30, 0 )
#define LED1ON 1
#define LED2ON  0
#define LED3ON  0

#define PLATFORM_CPU_CONSTANTS_PLATFORM \
  _C(LED1),\
  _C(LED2),\
  _C(LED3),\
  _C(LED1ON),\
  _C(LED2ON),\
  _C(LED3ON),

#endif // #ifndef __CPU_AT91SAM3X8E__

