// eLua platform configuration
// LM3S and LM4F

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "auxmods.h"
#include "stacks.h"
#include "elua_int.h"
#include "flash_conf.h"

#if defined( FORLM4F120 ) || defined( ELUA_BOARD_SOLDERCORE )
#define USE_PIN_MUX
#endif

// *****************************************************************************
// Define here what components you want for this platform

// ToDo: ADC turned off for testing - needs work on pinmux code
#if !defined(FORLM4F120)
#define BUILD_ADC
#endif

// ToDo: Add simulated PWMs for LM4F
#if !defined(FORLM3S6918) && !defined(FORLM4F120)
#define BUILD_PWM
#endif

// ToDo: LM4F has CAN, but disabled until get other things working
#if defined( FORLM3S8962 ) || defined( FORLM3S9B92 ) || defined( FORLM3S9D92 ) // || defined(FORLM4F120)
#define BUILD_CAN
#endif

// ToDo: working on I2C
#if defined ( FORLM4F120 ) || defined( FORLM3S8962 )
//#define BUILD_I2C
#endif

// ToDo: working on comparators
// #if defined ( FORLM4F120 )
// #define BUILD_COMP
// #endif

//#if !defined( ELUA_BOARD_SOLDERCORE )
  #define BUILD_XMODEM
  #define BUILD_TERM
//#endif

#define BUILD_SHELL
#define BUILD_ROMFS

// Exclude MMCFS for EKLM3S9D92 and LM4F120 - define BUILD_MMCFS externally for SD shield
#if !defined( ELUA_BOARD_EKLM3S9D92 ) && !defined ( FORLM4F120 )
#define BUILD_MMCFS
#endif

// ToDo: Maybe build USB CDC for LM4F120?  (but probably not enough RAM)
#if defined( ELUA_BOARD_SOLDERCORE )
  #define BUILD_USB_CDC
#endif

// Network
#if !defined( FORLM3S1968 ) && !defined( ELUA_BOARD_EKLM3S9D92 ) && !defined ( FORLM4F120 )
  #define BUILD_UIP
  #define BUILD_DHCPC
  #define BUILD_DNS
#endif

#define BUILD_RPC

//#if defined( ELUA_BOARD_SOLDERCORE )
//  #define BUILD_CON_TCP
//#else
  #define BUILD_CON_GENERIC
//#endif

#define BUILD_C_INT_HANDLERS
#ifdef ELUA_BOARD_EKLM3S9D92
#define BUILD_LUA_INT_HANDLERS
#define PLATFORM_INT_QUEUE_LOG_SIZE 5
#endif

#define PLATFORM_HAS_SYSTIMER
#define PLATFORM_TMR_COUNTS_DOWN

// FIXME: BUILD_WOFS also defined below (should set in 1 place)
#ifdef INTERNAL_FLASH_CONFIGURED // this comes from flash_conf.h
#define BUILD_WOFS
#endif

#define ENABLE_LM3S_GPIO

#define BUILD_LINENOISE

#define LINENOISE_HISTORY_SIZE_LUA    30
#define LINENOISE_HISTORY_SIZE_SHELL  10

// *****************************************************************************
// UART/Timer IDs configuration data (used in main.c)

#if defined( ELUA_BOARD_SOLDERCORE )
#define CON_UART_ID         CDC_UART_ID
#else
#define CON_UART_ID           0
#endif

#define CON_UART_SPEED        115200
#define TERM_LINES            25
#define TERM_COLS             80

// *****************************************************************************
// Auxiliary libraries that will be compiled for this platform

// The name of the platform specific libs table
// FIXME: should handle partial or no inclusion of platform specific modules per conf.py
#if defined( ENABLE_DISP ) || defined( ENABLE_LM3S_GPIO )
#define PS_LIB_TABLE_NAME   "lm3s"
#endif

#ifdef BUILD_CAN
#define CANLINE  _ROM( AUXLIB_CAN, luaopen_can, can_map )
#else
#define CANLINE
#endif

#ifdef BUILD_PWM
#define PWMLINE  _ROM( AUXLIB_PWM, luaopen_pwm, pwm_map )
#else
#define PWMLINE
#endif

#ifdef BUILD_UIP
#define NETLINE  _ROM( AUXLIB_NET, luaopen_net, net_map )
#else
#define NETLINE
#endif

#ifdef BUILD_I2C
#define I2CLINE  _ROM( AUXLIB_I2C, luaopen_i2c, i2c_map )
#else
#define I2CLINE
#endif  

#ifdef BUILD_ADC
#define ADCLINE _ROM( AUXLIB_ADC, luaopen_adc, adc_map )
#else
#define ADCLINE
#endif

#ifdef BUILD_TERM
#define TERMLINE _ROM( AUXLIB_TERM, luaopen_term, term_map )
#else
#define TERMLINE
#endif

#if defined( ELUA_BOOT_RPC ) && !defined( BUILD_RPC )
#define BUILD_RPC
#endif

#if defined( BUILD_RPC ) 
#define RPCLINE _ROM( AUXLIB_RPC, luaopen_rpc, rpc_map )
#else
#define RPCLINE
#endif

#ifdef PS_LIB_TABLE_NAME
#define PLATLINE _ROM( PS_LIB_TABLE_NAME, luaopen_platform, platform_map )
#else
#define PLATLINE
#endif



#define LUA_PLATFORM_LIBS_ROM\
  _ROM( AUXLIB_PIO, luaopen_pio, pio_map )\
  _ROM( AUXLIB_SPI, luaopen_spi, spi_map )\
  _ROM( AUXLIB_TMR, luaopen_tmr, tmr_map )\
  _ROM( AUXLIB_PD, luaopen_pd, pd_map )\
  _ROM( AUXLIB_UART, luaopen_uart, uart_map )\
  I2CLINE\
  PWMLINE\
  TERMLINE\
  _ROM( AUXLIB_PACK, luaopen_pack, pack_map )\
  _ROM( AUXLIB_BIT, luaopen_bit, bit_map )\
  _ROM( AUXLIB_BITARRAY, luaopen_bitarray, bitarray_map )\
  NETLINE\
  _ROM( AUXLIB_CPU, luaopen_cpu, cpu_map )\
  _ROM( AUXLIB_ELUA, luaopen_elua, elua_map )\
  ADCLINE\
  CANLINE\
  RPCLINE\
  _ROM( LUA_MATHLIBNAME, luaopen_math, math_map )\
  PLATLINE

// *****************************************************************************
// Configuration data

// Static TCP/IP configuration
#define ELUA_CONF_IPADDR0     192
#define ELUA_CONF_IPADDR1     168
#define ELUA_CONF_IPADDR2     100
#define ELUA_CONF_IPADDR3     90

#define ELUA_CONF_NETMASK0    255
#define ELUA_CONF_NETMASK1    255
#define ELUA_CONF_NETMASK2    255
#define ELUA_CONF_NETMASK3    0

#define ELUA_CONF_DEFGW0      192
#define ELUA_CONF_DEFGW1      168
#define ELUA_CONF_DEFGW2      100
#define ELUA_CONF_DEFGW3      20

#define ELUA_CONF_DNS0        192
#define ELUA_CONF_DNS1        168
#define ELUA_CONF_DNS2        100
#define ELUA_CONF_DNS3        20

// *****************************************************************************
// Configuration data

// Virtual timers (0 if not used)
#define VTMR_NUM_TIMERS       4
#define VTMR_FREQ_HZ          5

// Number of resources (0 if not available/not implemented)
// NUM_PIO Maximum 8
#if defined(FORLM3S1968)
  #define NUM_PIO             8
#elif defined(FORLM3S9B92) || defined( FORLM3S9D92 )
  #define NUM_PIO             9
#elif defined(FORLM4F120)
  #define NUM_PIO             6
#else
  #define NUM_PIO             7
#endif

// NUM_SPI - Max 2, unless LM4F120
#ifndef NUM_SPI
#ifdef FORLM4F120
  #define NUM_SPI             4
#elif defined( FORLM3S9B92 ) || defined( FORLM3S9D92 )
  #define NUM_SPI            2
#else
  #define NUM_SPI            1
#endif
#endif


// UART - Max 3 unless LM4F120
#if defined (FORLM4F120)
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
#elif defined (FORLM4F120)
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


// NUM_TIMERS - Max 4 unless LM4F120
#ifdef FORLM4F120
  #define MAX_TIMER            12
#else
  #define MAX_TIMER             4
#endif

#ifndef NUM_TIMER
  #define NUM_TIMER             6
// LM4F120 actually has 12 (6 normal, 6 wide), but haven't added all of code to handle wide timers
#else
  #define NUM_TIMER             4
#endif

#if NUM_TIMER > MAX_TIMER
#warning Too many TIMER, maximum is MAX_TIMER
#define NUM_TIMER MAX_TIMER
#endif


//TODO: Use timers for PWM on LM4F
#if defined( FORLM3S6918 ) || defined (FORLM4F120)
  #define NUM_PWM             0
#elif defined( FORLM3S9B92 ) || defined( FORLM3S9D92 )
  #define NUM_PWM             8
#else
  #define NUM_PWM             6
#endif  


#if defined( BUILD_I2C )
#warning I2C support not written for LM3/4 
#ifdef FORLM4F120
  #define NUM_I2C             4
#elif defined( FORLM6918 ) || defined( FORLM6965 ) || defined( FORLM3S9B92 ) || defined( FORLM3S9D92 )
  #define NUM_I2C             2
#elif defined( FORLM8962 )
  #define NUM_I2C             1
#else
  #define NUM_I2C             0		
#endif
#endif // ifdef Build_I2C


#ifdef BUILD_COMP
#warning Comparators not written yet for eLua or for platform
#ifdef FORLM4F120
  #define NUM_COMP            2
#elif defined( FORLM8962 )
  #define NUM_COMP            1
#else
  #define NUM_COMP            0
// FIXME: Fill in number comparators for other lm3s...
#endif
#endif

#ifdef BUILD_ADC
#if defined( FORLM3S9B92 ) || defined( FORLM3S9D92 )
  #define NUM_ADC             16
#elif defined( FORLM4F120 )
  #define NUM_ADC             2
#else
  #define NUM_ADC             4
#endif
#else
  #define NUM_ADC 			  0
#endif // BUILD_ADC

/* 
// ToDo: Add support for USB
#ifdef FORLM4F120
  #define NUM_USB             1
#else
  #define NUM_USB             0
// FIXME: Need to fill in number USB for other lm3s...
#endif
*/

// CAN - Maximum is 1
#ifdef BUILD_CAN
#define NUM_CAN               1
#else
#define NUM_CAN               0
#endif


// Enable RX buffering on UART
#define BUF_ENABLE_UART
#define CON_BUF_SIZE          BUF_SIZE_128

// ADC Configuration Parameters
#ifdef FORLM4F120
  #define ADC_BIT_RESOLUTION    12
#else
  #define ADC_BIT_RESOLUTION    10
#endif  
#define BUF_ENABLE_ADC
#define ADC_BUF_SIZE          BUF_SIZE_2

// TODO: These should be adjusted to support multiple ADC devices
#define ADC_TIMER_FIRST_ID    0
#define ADC_NUM_TIMERS        NUM_TIMER  

// RPC boot options
#define RPC_UART_ID           CON_UART_ID
#define RPC_UART_SPEED        CON_UART_SPEED

#if defined( ELUA_BOARD_EKLM3S6965 )
  // EK-LM3S6965
  #define MMCFS_CS_PORT                3
  #define MMCFS_CS_PIN                 0
  #define MMCFS_SPI_NUM                0
#elif defined( ELUA_BOARD_EKLM3S8962 )
  // EK-LM3S8962
  #define MMCFS_CS_PORT                6
  #define MMCFS_CS_PIN                 0
  #define MMCFS_SPI_NUM                0
#elif defined( ELUA_BOARD_EAGLE100 )
  // Eagle-100
  #define MMCFS_CS_PORT                6
  #define MMCFS_CS_PIN                 1
  #define MMCFS_SPI_NUM                0
#elif defined( ELUA_BOARD_SOLDERCORE )
  // Soldercore
  #define MMCFS_CS_PORT                6
  #define MMCFS_CS_PIN                 7
  #define MMCFS_SPI_NUM                1
#elif defined( BUILD_MMCFS ) && !defined( MMCFS_SPI_NUM )
  #warning "MMCFS was enabled, but required SPI & CS data are undefined, disabling MMCFS"
  #undef BUILD_MMCFS
#endif


// CPU frequency (needed by the CPU module and MMCFS code, 0 if not used)
#define CPU_FREQUENCY         MAP_SysCtlClockGet()

// PIO prefix ('0' for P0, P1, ... or 'A' for PA, PB, ...)
#define PIO_PREFIX            'A'
// Pins per port configuration:
// #define PIO_PINS_PER_PORT (n) if each port has the same number of pins, or
// #define PIO_PIN_ARRAY { n1, n2, ... } to define pins per port in an array
// Use #define PIO_PINS_PER_PORT 0 if this isn't needed
#if defined(FORLM3S1968)
  #define PIO_PIN_ARRAY         { 8, 8, 8, 4, 4, 8, 8, 4}
#elif defined(FORLM3S9B92) || defined( FORLM3S9D92 )
  #define PIO_PIN_ARRAY         { 8, 8, 8, 8, 8, 6, 8, 8, 8 }
#elif defined( FORLM4F120 )
  #define PIO_PIN_ARRAY         { 8, 8, 8, 8, 6, 5 }
#else
  #define PIO_PIN_ARRAY         { 8, 8, 8, 8, 4, 4, 2 }
#endif
//                                A, B, C, D, E, F, G, H, J

// FIXME: check NUM_PIO versus PIO_PIN_ARRAY


// *****************************************************************************
// Memory

#if defined( FORLM3S9B92 ) || defined( FORLM3S9D92 )
  #define SRAM_SIZE ( 0x18000 )
#elif defined( FORLM4F120 )
  #define SRAM_SIZE ( 0x08000 )
#else
  #define SRAM_SIZE ( 0x10000 )
#endif

// Flash data (only for LM3S8962 for now)
#ifdef ELUA_CPU_LM3S8962

//FIXME: BUILD_WOFS also set above - should only set in one place
#define BUILD_WOFS
#endif // #ifdef ELUA_CPU_LM3S8962

// Allocator data: define your free memory zones here in two arrays
// (start address and end address)
#define MEM_START_ADDRESS     { ( void* )end }
#define MEM_END_ADDRESS       { ( void* )( SRAM_BASE + SRAM_SIZE - STACK_SIZE_TOTAL - 1 ) }


// Interrupt list
// Must have a 1-to-1 correspondence with the interrupt table in platform_int.c!
// (See doc/en/arch_ints.txt)
#define INT_UART_RX           ELUA_INT_FIRST_ID
#define INT_GPIO_POSEDGE      ( ELUA_INT_FIRST_ID + 1 )
#define INT_GPIO_NEGEDGE      ( ELUA_INT_FIRST_ID + 2 )
#define INT_TMR_MATCH         ( ELUA_INT_FIRST_ID + 3 )
#define INT_ELUA_LAST         INT_TMR_MATCH

// *****************************************************************************
// CPU constants that should be exposed to the eLua "cpu" module



// Does the order matter here? (look almost in order, except not sure what INT_UART_RX is

#if defined( FORLM4F120 )
// Made additions in order from hw_ints.h
// Could add more timers (added some, but not full number from lm4F120)

#define EXTRA_CPU_CONSTANTS\
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
  _C( INT_TIMER5B ),

/*  These would handle the wide timers on LM4F120 - add if needed
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

#else
#define EXTRA_CPU_CONSTANTS
#endif


#define PLATFORM_CPU_CONSTANTS\
  _C( INT_GPIOA ),\
  _C( INT_GPIOB ),\
  _C( INT_GPIOC ),\
  _C( INT_GPIOD ),\
  _C( INT_GPIOE ),\
  _C( INT_UART0 ),\
  _C( INT_UART1 ),\
  _C( INT_SSI0 ),\
  _C( INT_I2C0 ),\
  _C( INT_PWM_FAULT ),\
  _C( INT_PWM0 ),\
  _C( INT_PWM1 ),\
  _C( INT_PWM2 ),\
  _C( INT_QEI0 ),\
  _C( INT_ADC0 ),\
  _C( INT_ADC1 ),\
  _C( INT_ADC2 ),\
  _C( INT_ADC3 ),\
  _C( INT_WATCHDOG ),\
  _C( INT_TIMER0A ),\
  _C( INT_TIMER0B ),\
  _C( INT_TIMER1A ),\
  _C( INT_TIMER1B ),\
  _C( INT_TIMER2A ),\
  _C( INT_TIMER2B ),\
  _C( INT_COMP0 ),\
  _C( INT_COMP1 ),\
  _C( INT_COMP2 ),\
  _C( INT_SYSCTL ),\
  _C( INT_FLASH ),\
  _C( INT_GPIOF ),\
  _C( INT_GPIOG ),\
  _C( INT_GPIOH ),\
  _C( INT_UART2 ),\
  _C( INT_SSI1 ),\
  _C( INT_TIMER3A ),\
  _C( INT_TIMER3B ),\
  _C( INT_I2C1 ),\
  _C( INT_QEI1 ),\
  _C( INT_CAN0 ),\
  _C( INT_CAN1 ),\
  _C( INT_CAN2 ),\
  _C( INT_ETH ),\
  _C( INT_HIBERNATE ),\
  _C( INT_USB0 ),\
  _C( INT_PWM3 ),\
  _C( INT_UDMA ),\
  _C( INT_UDMAERR ),\
  EXTRA_CPU_CONSTANTS \
  _C( INT_UART_RX ),\
  _C( INT_GPIO_POSEDGE ),\
  _C( INT_GPIO_NEGEDGE ),\
  _C( INT_TMR_MATCH )



#endif // #ifndef __PLATFORM_CONF_H__
