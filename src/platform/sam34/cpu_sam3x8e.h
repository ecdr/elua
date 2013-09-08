// AT91SAM3x8e CPU definition

#ifndef __CPU_AT91SAM3X8E__
#define __CPU_AT91SAM3X8E__

#include "platform_ints.h"

// Number of resources (0 if not available/not implemented)
#define NUM_PIO               5
#define NUM_SPI               1
#define NUM_UART              3
#define NUM_TIMER             9
#define NUM_PWM               8
#define NUM_I2C               2
#define NUM_ADC               16
#define NUM_CAN               2

#define NUM_DAC			2
#define NUM_I2S               1

#define ADC_BIT_RESOLUTION    12

#define DAC_BIT_RESOLUTION    12


// CPU frequency (needed by the CPU module and MMCFS code, 0 if not used)
#define CPU_FREQUENCY         

// PIO prefix ('0' for P0, P1, ... or 'A' for PA, PB, ...)
#define PIO_PREFIX            'A'
// Pins per port configuration:
// #define PIO_PINS_PER_PORT (n) if each port has the same number of pins, or
// #define PIO_PIN_ARRAY { n1, n2, ... } to define pins per port in an array
// Use #define PIO_PINS_PER_PORT 0 if this isn't needed
#define PIO_PIN_ARRAY         { 30, 32, 31, 10, 6 }

// Internal Flash data
#define INTERNAL_FLASH_SIZE             ( 512 * 1024 )
#define INTERNAL_FLASH_WRITE_UNIT_SIZE  
#define INTERNAL_FLASH_SECTOR_SIZE      
#define INTERNAL_FLASH_START_ADDRESS    

#define SRAM_ORIGIN           0x20070000
#ifndef SRAM_SIZE
#define SRAM_SIZE             0x18000
#endif

#define INTERNAL_RAM1_FIRST_FREE end
#define INTERNAL_RAM1_LAST_FREE  ( SRAM_ORIGIN + SRAM_SIZE - STACK_SIZE_TOTAL - 1 )

#define PLATFORM_CPU_CONSTANTS_INTS\
  _C( INT_UART_RX ),
  _C( INT_GPIO_POSEDGE ),\
  _C( INT_GPIO_NEGEDGE ),\
  _C( INT_TMR_MATCH ),

#endif // #ifndef __CPU_AT91SAM3X8E__

