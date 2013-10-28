// sam interrupt support


#include "platform_conf.h"
// #include "platform_int.h"

#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )

// Generic headers
#include "platform.h"
#include "elua_int.h"
#include "common.h"


#define GPIO_INT_POSEDGE_ENABLED        1
#define GPIO_INT_NEGEDGE_ENABLED        2
#define GPIO_INT_BOTH_ENABLED           ( GPIO_INT_POSEDGE_ENABLED | GPIO_INT_NEGEDGE_ENABLED )

// ****************************************************************************
// Interrupt handlers


// ----------------------------------------------------------------------------
// UART_RX interrupt

static void uart_common_rx_handler( elua_int_resnum resnum )
{
//  MAP_UARTIntClear( uart_base[ resnum ], uart_int_mask );
//  while( MAP_UARTCharsAvail( uart_base[ resnum ] ) )  
//    cmn_int_handler( INT_UART_RX, resnum );  
}

// __attribute__((__interrupt__))  - this was in avr platform ??
void UART_Handler(void)
{
  uart_common_rx_handler( 0 );
}

void USART0_Handler(void)
{
  uart_common_rx_handler( 1 );
}

void USART1_Handler(void)
{
  uart_common_rx_handler( 2 );
}

void USART2_Handler(void)
{
  uart_common_rx_handler( 3 );
}

// ----------------------------------------------------------------------------
// GPIO interrupts (POSEDGE/NEGEDGE)

static void gpio_common_handler( u8 port )
{

//  for( pin = 0, pinmask = 1; pin < 8; pin ++, pinmask <<= 1 )
//  cmn_int_handler( INT_GPIO_POSEDGE, resnum );
//  cmn_int_handler( INT_GPIO_POSEDGE, PLATFORM_IO_ENCODE( port, pin, 0 ) );
//  cmn_int_handler( INT_GPIO_NEGEDGE, PLATFORM_IO_ENCODE( port, pin, 0 ) );
}

void PIOA_Handler(void)
{
  gpio_common_handler( 0 );
}

void PIOB_Handler(void)
{
  gpio_common_handler( 1 );
}

void PIOC_Handler(void)
{
  gpio_common_handler( 2 );
}

void PIOD_Handler(void)
{
  gpio_common_handler( 3 );
}

void PIOE_Handler(void)
{
  gpio_common_handler( 4 );
}

void gpiof_handler(void)
{
  gpio_common_handler( 5 );
}


// ----------------------------------------------------------------------------
// Timer interrupts

static void tmr_common_handler( elua_int_resnum id )
{
  cmn_int_handler( INT_TMR_MATCH, id );
}

void TC0_Handler()
{
  tmr_common_handler( 0 );
}

void TC1_Handler()
{
  tmr_common_handler( 1 );
}

void TC2_Handler()
{
  tmr_common_handler( 2 );
}

void TC3_Handler()
{
  tmr_common_handler( 3 );
}

void TC4_Handler()
{
  tmr_common_handler( 4 );
}

void TC5_Handler()
{
  tmr_common_handler( 5 );
}

#if NUM_TIMERS > 6

void TC6_Handler()
{
  tmr_common_handler( 6 );
}

void TC7_Handler()
{
  tmr_common_handler( 7 );
}

void TC8_Handler()
{
  tmr_common_handler( 8 );
}

#endif


// ****************************************************************************
// Helpers

// Get GPIO interrupt status as a mask
static int inth_gpio_get_int_status( elua_int_resnum resnum )
{
}

// ****************************************************************************
// Interrupt: INT_UART_RX

static int int_uart_rx_get_status( elua_int_resnum resnum )
{  
}

static int int_uart_rx_set_status( elua_int_resnum resnum, int status )
{
}

static int int_uart_rx_get_flag( elua_int_resnum resnum, int clear )
{
}

// ****************************************************************************
// Interrupt: INT_GPIO_POSEDGE

static int int_gpio_posedge_get_status( elua_int_resnum resnum )
{
}

static int int_gpio_posedge_set_status( elua_int_resnum resnum, int status )
{
}

static int int_gpio_posedge_get_flag( elua_int_resnum resnum, int clear )
{
}

// ****************************************************************************
// Interrupt: INT_GPIO_NEGEDGE

static int int_gpio_negedge_get_status( elua_int_resnum resnum )
{
}

static int int_gpio_negedge_set_status( elua_int_resnum resnum, int status )
{
}

static int int_gpio_negedge_get_flag( elua_int_resnum resnum, int clear )
{
}

// ****************************************************************************
// Interrupt: INT_TMR_MATCH

static int int_tmr_match_get_status( elua_int_resnum resnum )
{
}

static int int_tmr_match_set_status( elua_int_resnum resnum, int status )
{
}

static int int_tmr_match_get_flag( elua_int_resnum resnum, int clear )
{
}

// ****************************************************************************
// Interrupt initialization

void platform_int_init(void)
{
}

// ****************************************************************************
// Interrupt table
// Must have a 1-to-1 correspondence with the interrupt enum in platform_conf.h!

const elua_int_descriptor elua_int_table[ INT_ELUA_LAST ] = 
{
  { int_uart_rx_set_status, int_uart_rx_get_status, int_uart_rx_get_flag },
  { int_gpio_posedge_set_status, int_gpio_posedge_get_status, int_gpio_posedge_get_flag },
  { int_gpio_negedge_set_status, int_gpio_negedge_get_status, int_gpio_negedge_get_flag },
  { int_tmr_match_set_status, int_tmr_match_get_status, int_tmr_match_get_flag }
};

#else // #if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )

/* 
// ASF provides default implementations, so don't need to provide dummies here
void PIOA_Handler(void)
{
}

void PIOB_Handler(void)
{
}

void PIOC_Handler(void)
{
}

void PIOD_Handler(void)
{
}

void PIOE_Handler(void)
{
}

/*
void gpiof_handler(void)
{
} */

/*
void TC0_Handler(void)
{
}

void TC1_Handler(void)
{
}

void TC2_Handler(void)
{
}

void TC3_Handler(void)
{
}

void TC4_Handler(void)
{
}

void TC5_Handler(void)
{
}

void TC6_Handler(void)
{
}

void TC7_Handler(void)
{
}

void TC8_Handler(void)
{
}

*/


#endif // #if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )

