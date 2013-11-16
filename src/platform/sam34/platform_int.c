// eLua interrupt support
// AT91SAM3/4

#include "platform_conf.h"
// #include "platform_int.h"

#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )


// Generic headers
#include "platform.h"
#include "elua_int.h"
#include "common.h"

#include <asf.h>

// header

// From platform.c
extern Tc * tc(unsigned id);
extern u32 tchanel(unsigned id);
extern u8 platform_timer_int_periodic_flag[ NUM_TIMER ];

extern Pio * const pio_base[];
extern const u32 pio_id[];

extern Usart* const uart_base[];

// TODO: See if library files provide a name for this
#define PLATFORM_TIMER_COUNT_MAX ( 0xFFFFFFFFUL )


// Function prototypes for platform_int.c

void negedge_handler(uint32_t id, uint32_t mask);
void posedge_handler(uint32_t id, uint32_t mask);


// FIXME: - need to find actual functions or macros for these
extern u8 getport(u32 id);
extern u8 MASK_TO_PIN(u32 mask);



// End of header

const IRQn_Type pio_irq[] =   { PIOA_IRQn, PIOB_IRQn, PIOC_IRQn, PIOD_IRQn };
const IRQn_Type uart_irq[] =  { UART_IRQn, USART0_IRQn, USART1_IRQn, USART2_IRQn, USART3_IRQn };
const IRQn_Type timer_irq[] = { TC0_IRQn, TC1_IRQn, TC2_IRQn, TC3_IRQn, 
  TC4_IRQn, TC5_IRQn, TC6_IRQn, TC7_IRQn, TC8_IRQn};
  // Not sure why sam3x8e defines TC0 .. TC8 IRQ


#define GPIO_INT_POSEDGE_ENABLED        1
#define GPIO_INT_NEGEDGE_ENABLED        2
#define GPIO_INT_BOTH_ENABLED           ( GPIO_INT_POSEDGE_ENABLED | GPIO_INT_NEGEDGE_ENABLED )

// ****************************************************************************
// Interrupt handlers


// ----------------------------------------------------------------------------
// UART_RX interrupt

static void uart_common_rx_handler( elua_int_resnum resnum )
{
  u32 status;

	status = usart_get_status(uart_base[resnum]);
	if (status & US_CSR_RXRDY)
    while( usart_is_rx_ready(uart_base[resnum]) )
      cmn_int_handler( INT_UART_RX, resnum );
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

// TODO: Instead consider just using ASF pio int handlers (and access functions)

/*
static void gpio_common_handler( u8 port )
{
// example - from pio_handler.c
	uint32_t status;
	uint32_t i;

//	 Read PIO controller status 

	status = pio_get_interrupt_status(p_pio[port]);
	status &= pio_get_interrupt_mask(p_pio[port]);

//  Check pending events
//	if (status != 0) {
//		Find triggering source 
//		i = 0;
//		while (status != 0) {
			// Source is configured on the same controller 
//			if (gs_interrupt_sources[i].id == ul_id[port]) {
				// Source has PIOs whose statuses have changed 
//				if ((status & gs_interrupt_sources[i].mask) != 0) {
//					gs_interrupt_sources[i].handler(gs_interrupt_sources[i].id,
//							gs_interrupt_sources[i].mask);
//					status &= ~(gs_interrupt_sources[i].mask);
//				}
//			}
//			i++;
//		}
//	}
  
//  for( pin = 0, pinmask = 1; pin < platform_pio_get_num_pins( port ); pin ++, pinmask <<= 1 )
//  {
//  cmn_int_handler( INT_GPIO_POSEDGE, resnum );
//  cmn_int_handler( INT_GPIO_POSEDGE, PLATFORM_IO_ENCODE( port, pin, 0 ) );
//  cmn_int_handler( INT_GPIO_NEGEDGE, PLATFORM_IO_ENCODE( port, pin, 0 ) );
//  }
//}

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
*/

void negedge_handler(uint32_t id, uint32_t mask)
{
  u8 port = getport(id);          // FIXME
  u8 pin = MASK_TO_PIN(mask);     // FIXME


  cmn_int_handler( INT_GPIO_NEGEDGE, PLATFORM_IO_ENCODE( port, pin, 0 ) );
}
  
void posedge_handler(uint32_t id, uint32_t mask)
{
  u8 port = getport(id);          // FIXME
  u8 pin = MASK_TO_PIN(mask);     // FIXME
  
  cmn_int_handler( INT_GPIO_POSEDGE, PLATFORM_IO_ENCODE( port, pin, 0 ) );
}


// ----------------------------------------------------------------------------
// Timer interrupts

static void tmr_common_handler( elua_int_resnum id)
{
  Tc * tmr = tc(id);
  u32 channel = tchanel(id);

  if ((tc_get_status(tmr, channel) & TC_SR_CPCS) == TC_SR_CPCS) {
    if ( platform_timer_int_periodic_flag[ id ] != PLATFORM_TIMER_INT_CYCLIC )
    {
      tc_disable_interrupt(tmr, channel, TC_IDR_CPCS);
      tc_write_rc(tmr, channel, PLATFORM_TIMER_COUNT_MAX);
    }
  cmn_int_handler( INT_TMR_MATCH, id );
  }
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


// ****************************************************************************
// Helpers

// Get GPIO interrupt status as a mask
/*
static int inth_gpio_get_int_status( elua_int_resnum resnum )
{
}
*/

// ****************************************************************************
// Interrupt: INT_UART_RX

static int int_uart_rx_get_status( elua_int_resnum resnum )
{
  return (usart_get_interrupt_mask(uart_base[ resnum ]) & US_IMR_RXRDY) ? 1 : 0;
}

static int int_uart_rx_set_status( elua_int_resnum resnum, int status )
{
  int prev = int_uart_rx_get_status( resnum );

  if( status == PLATFORM_CPU_ENABLE )
    usart_enable_interrupt(uart_base[ resnum ], US_IER_RXRDY);
  else
    usart_disable_interrupt(uart_base[ resnum ], US_IDR_RXRDY);
  return prev;
}


// FIXME: Is there any way to read status without clearing it?  Maybe need to make own status cache?
static int int_uart_rx_get_flag( elua_int_resnum resnum, int clear )
{
  int flag = ( usart_get_status( uart_base[ resnum ] ) & US_CSR_RXRDY ) ? 1 : 0;

  UNUSED( clear );
/*  if( clear )
    MAP_UARTIntClear( uart_base[ resnum ], uart_int_mask ); */
  return flag;
}

// ****************************************************************************
// Interrupt: INT_GPIO_POSEDGE

#warning GPIO Posedge not written
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

// Is the interrupt configured for this port/pin?
static int int_gpio_negedge_get_status( elua_int_resnum resnum )
{
  Pio * port = pio_base[ PLATFORM_IO_GET_PORT( resnum ) ];
  u32 pinmask = 1 << PLATFORM_IO_GET_PIN( resnum );

  return (pio_get_interrupt_mask(port) & pinmask);  // Not right - have to find if NEGEDGE int set
}

// 
static int int_gpio_negedge_set_status( elua_int_resnum resnum, int status )
{
  u8 port_num = PLATFORM_IO_GET_PORT( resnum );
  Pio * port = pio_base[ port_num ];
  u32 port_id = pio_id[ port_num ];
  u32 pinmask = 1 << PLATFORM_IO_GET_PIN( resnum );
  
  if (status)
  {
    pio_handler_set(port, port_id, pinmask, PIO_IT_FALL_EDGE, negedge_handler);
    NVIC_EnableIRQ(pio_irq[port_num]);
    pio_enable_interrupt(port, pinmask);
    // Record that int handler set
  }
  else
  {
    pio_disable_interrupt(port, pinmask);
  }
  return ;  //FIXME: What
}

// Is there an interrupt from this port/pin?
static int int_gpio_negedge_get_flag( elua_int_resnum resnum, int clear )
{
  Pio * port = pio_base[ PLATFORM_IO_GET_PORT( resnum ) ];
  u32 pinmask = 1 << PLATFORM_IO_GET_PIN( resnum );

  if( pio_get(port, PIO_TYPE_PIO_INPUT, pinmask) != 0 )
    return 0;   // Might there be cases where want interrupt on pin with alternate function?
  if( pio_get_interrupt_status(port) & pinmask )
  {
    if( clear )
//      MAP_GPIOPinIntClear( portbase, pinmask );   // Not sure how to clear an interrupt
      return 0;
    return 1;
  }
  return 0;
}

// ****************************************************************************
// Interrupt: INT_TMR_MATCH

static int int_tmr_match_get_status( elua_int_resnum resnum )
{
  return (tc_get_interrupt_mask(tc(resnum), tchanel(resnum)) & TC_IMR_CPCS) ? 1 : 0;
}

static int int_tmr_match_set_status( elua_int_resnum resnum, int status )
{
  int prev = int_tmr_match_get_status( resnum );
  Tc * tmr = tc(resnum);
  u32 channel = tchanel(resnum);
  
  if( status == PLATFORM_CPU_ENABLE )
  {
    tc_start(tmr, channel);
    tc_enable_interrupt(tmr, channel, TC_IER_CPCS);
  }
  else
  {
    tc_disable_interrupt(tmr, channel, TC_IDR_CPCS);
    tc_stop(tmr, channel);    
  }
  return prev;
}

static int int_tmr_match_get_flag( elua_int_resnum resnum, int clear )
{
  Tc * tmr = tc(resnum);
  u32 channel = tchanel(resnum);
  
  u32 status = (tc_get_status(tmr, channel) & TC_SR_CPCS);

// FIXME: handle clear (i.e. can I read status without clearing it)?
//  if( clear )
//    MAP_TimerIntClear( base, TIMER_TIMA_TIMEOUT );
  return status && tmr_is_enabled( resnum ) ? 1 : 0;
}

// FIXME: tmr_is_enabled - not written

// ****************************************************************************
// Interrupt initialization


#warning Interrupt init not tested

// Set priority levels for various interrupts
//#define USART_INT_PRI 
//#define TIMER_INT_PRI 
//#define PIO_INT_PRI   

void platform_int_init(void)
{
  unsigned id;

// PIO
  for (id = 0; id < NUM_PIO; id++) {
#ifdef   PIO_INT_PRI
    NVIC_SetPriority(pio_irq[id], PIO_INT_PRI);
#endif
    NVIC_EnableIRQ(pio_irq[id]);
  }

// Timer
  for (id = 0; id < NUM_TIMER; id++) {
#ifdef   TIMER_INT_PRI
    NVIC_SetPriority(timer_irq[id], TIMER_INT_PRI);
#endif
    NVIC_EnableIRQ(timer_irq[id]);
  }
    
// USART
  for (id = 0; id < NUM_UART; id++) {
#ifdef   USART_INT_PRI
  NVIC_SetPriority(uart_irq[id], USART_INT_PRI);
#endif
    NVIC_EnableIRQ(uart_irq[id]);
  };
}


// ****************************************************************************
// Interrupt table
// Must have a 1-to-1 correspondence with the interrupt enum in platform_ints.h!

const elua_int_descriptor elua_int_table[ INT_ELUA_LAST ] = 
{
  { int_uart_rx_set_status, int_uart_rx_get_status, int_uart_rx_get_flag },
  { int_gpio_posedge_set_status, int_gpio_posedge_get_status, int_gpio_posedge_get_flag },
  { int_gpio_negedge_set_status, int_gpio_negedge_get_status, int_gpio_negedge_get_flag },
  { int_tmr_match_set_status, int_tmr_match_get_status, int_tmr_match_get_flag }
};

#else // #if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )

// ASF provides default handlers, so don't need dummies here


#endif // #if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )

