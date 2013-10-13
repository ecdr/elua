// sam interrupt support


#include "platform_conf.h"
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

static void uart_common_rx_handler( int resnum )
{
  MAP_UARTIntClear( uart_base[ resnum ], uart_int_mask );
  while( MAP_UARTCharsAvail( uart_base[ resnum ] ) )  
    cmn_int_handler( INT_UART_RX, resnum );  
}

void uart0_handler()
{
  uart_common_rx_handler( 0 );
}

void uart1_handler()
{
  uart_common_rx_handler( 1 );
}

void uart2_handler()
{
  uart_common_rx_handler( 2 );
}

// ----------------------------------------------------------------------------
// GPIO interrupts (POSEDGE/NEGEDGE)

static void gpio_common_handler( int port )
{
}

void gpioa_handler()
{
  gpio_common_handler( 0 );
}

void gpiob_handler()
{
  gpio_common_handler( 1 );
}

void gpioc_handler()
{
  gpio_common_handler( 2 );
}

void gpiod_handler()
{
  gpio_common_handler( 3 );
}

void gpioe_handler()
{
  gpio_common_handler( 4 );
}

void gpiof_handler()
{
  gpio_common_handler( 5 );
}


// ----------------------------------------------------------------------------
// Timer interrupts

static void tmr_common_handler( elua_int_resnum id )
{
  cmn_int_handler( INT_TMR_MATCH, id );
}

void tmr0_handler()
{
  tmr_common_handler( 0 );
}

void tmr1_handler()
{
  tmr_common_handler( 1 );
}

void tmr2_handler()
{
  tmr_common_handler( 2 );
}

void tmr3_handler()
{
  tmr_common_handler( 3 );
}

void tmr4_handler()
{
  tmr_common_handler( 4 );
}

void tmr5_handler()
{
  tmr_common_handler( 5 );
}

void tmr6_handler()
{
  tmr_common_handler( 6 );
}

void tmr7_handler()
{
  tmr_common_handler( 7 );
}

void tmr8_handler()
{
  tmr_common_handler( 8 );
}



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

void platform_int_init()
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

void gpioa_handler()
{
}

void gpiob_handler()
{
}

void gpioc_handler()
{
}

void gpiod_handler()
{
}

void gpioe_handler()
{
}

void gpiof_handler()
{
}


void tmr0_handler()
{
}

void tmr1_handler()
{
}

void tmr2_handler()
{
}

void tmr3_handler()
{
}

void tmr4_handler()
{
}

void tmr5_handler()
{
}

void tmr6_handler()
{
}

void tmr7_handler()
{
}

void tmr8_handler()
{
}



#endif // #if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )

