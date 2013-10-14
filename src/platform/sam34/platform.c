// eLua Platform-dependent functions
// AT91SAM3/4

#include "platform.h"
#include "type.h"
#include "devman.h"
#include "genstd.h"
#include <reent.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>

// FIXME: These ifdefs may not work, since those symbols may not be defined until later
#ifdef BUILD_UIP
#include "uip_arp.h"
#include "elua_uip.h"
#include "uip-conf.h"
#endif

#ifdef BUILD_ADC
#include "elua_adc.h"
#endif

#include "platform_conf.h"
#include "common.h"
#include "math.h"
#include "diskio.h"
#include "lua.h"
#include "lauxlib.h"
#include "lrotable.h"
#include "elua_int.h" 

// Platform specific includes

// Not needed - handled by platform_conf.h
//#if defined( FORSAM3X8E)
//
//#include "cpu_sam3x8e.h"
//#else
//#warning SAM - Unknown CPU
//#endif   // CPU

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>

// ****************************************************************************
// Platform initialization

// forward
static void timers_init();
static void uarts_init();
static void spis_init();
static void pios_init();

#ifdef BUILD_PWM
static void pwms_init();
#endif

#ifdef BUILD_ADC
static void adcs_init();
#endif

#ifdef BUILD_CAN
static void cans_init();
#endif

#ifdef BUILD_USB_CDC
static void usb_init();
#endif

#ifdef BUILD_I2C
static void i2cs_init();
#endif

int platform_init()
{
  // Set the clocking to run from PLL
#warning FIXME - check clock setup
  sysclk_init();

  board_init();		// ASF
  
  // Setup PIO
  pios_init();

  // Setup SSIs
  spis_init();

  // Setup UARTs
  uarts_init();

  // Setup timers
  timers_init();

#ifdef BUILD_I2C
  // Setup I2Cs
  i2cs_init();
#endif // ifdef BUILD_I2C

#ifdef BUILD_PWM
  // Setup PWMs
  pwms_init();
#endif

#ifdef BUILD_ADC
  // Setup ADCs
  adcs_init();
#endif

#ifdef BUILD_CAN
  // Setup CANs
  cans_init();
#endif

#ifdef BUILD_USB_CDC
  // Setup USB
  usb_init();
#endif

  // Setup system timer
#warning FIXME - setup system timer

  cmn_platform_init();

  // All done
  return PLATFORM_OK;
}

// ****************************************************************************
// PIO

// PIOE net defined, but thought there were 5 ports - ??
const u32 pio_id[] =        { ID_PIOA, ID_PIOB, ID_PIOC, ID_PIOD };
Pio * const pio_base[] = { PIOA,    PIOB,    PIOC,    PIOD };

#define PIO_MASK_ALL  0xFFFFFFFF


static void pios_init()
{
  unsigned i;
  
  for( i = 0; i < NUM_PIO; i ++)
    pmc_enable_periph_clk(pio_id[i]);
}

pio_type platform_pio_op( unsigned port, pio_type pinmask, int op )
{
  pio_type retval = 1; 
  Pio * base = pio_base[ port ];

  switch( op )
  {
    case PLATFORM_IO_PORT_SET_VALUE:
      pio_set_output( base, PIO_MASK_ALL, pinmask, DISABLE, DISABLE );
#warning Need to figure settings for default level, open drain, pull up      
      break;

    case PLATFORM_IO_PIN_SET:
      pio_set( base, pinmask );
      break;

    case PLATFORM_IO_PIN_CLEAR:
      pio_clear( base, pinmask );
      break;

    case PLATFORM_IO_PORT_DIR_INPUT:
      pinmask = PIO_MASK_ALL;
    case PLATFORM_IO_PIN_DIR_INPUT:
      pio_set_input( base, pinmask, DISABLE );
#warning How handle pullup?
      break;

    case PLATFORM_IO_PORT_DIR_OUTPUT:
      pinmask = PIO_MASK_ALL;
    case PLATFORM_IO_PIN_DIR_OUTPUT:
      pio_set_output( base, pinmask, LOW, DISABLE, DISABLE );
#warning Need to figure settings for default level, open drain, pull up
      break;

    case PLATFORM_IO_PORT_GET_VALUE:
      retval = pio_get( base, PIO_TYPE_PIO_INPUT, PIO_MASK_ALL );
      break;

    case PLATFORM_IO_PIN_GET:
      retval = pio_get( base, PIO_TYPE_PIO_INPUT, pinmask ) ? 1 : 0;
      break;

    case PLATFORM_IO_PIN_PULLUP:
      pio_pull_up( base, pinmask, ENABLE);
      break;
      
    case PLATFORM_IO_PIN_NOPULL:
      pio_pull_up( base, pinmask, DISABLE);
      break;

    case PLATFORM_IO_PIN_PULLDOWN:
#warning Check to see if has pulldown
    default:
      retval = 0;
      break;
  }
  return retval;
}


// ****************************************************************************
// CAN

#if defined( BUILD_CAN )

Can * const can_base[] = { CAN0, CAN1 };

// Speed used in INIT
#ifndef CAN_INIT_SPEED
#define CAN_INIT_SPEED	500
#endif

#warning Check CAN controller frequency - just a guess that might be CPU_FREQ
#define CANCLK CPU_FREQUENCY

can_mb_conf_t can_rx_mailbox;


void CANIntHandler(void)
{
}

void cans_init( void )
{
  pmc_enable_periph_clk(ID_CAN0);
  can_init(CAN0, CANCLK, CAN_BPS_1000K);
#warning Need to check speed - use default from above
  can_reset_all_mailbox(CAN0);
//  can_enable_interrupt(CAN0, uint32_t dw_mask)
#warning - What is dw_mask?

  #warning Need to check mailbox particular
  can_rx_mailbox.ul_mb_idx = 0;
  can_rx_mailbox.uc_obj_type = CAN_MB_RX_MODE;
  can_rx_mailbox.ul_id_msk = CAN_MAM_MIDvA_Msk | CAN_MAM_MIDvB_Msk;
  can_rx_mailbox.ul_id = CAN_MID_MIDvA(0x00);
  can_mailbox_init(CAN01, &can_rx_mailbox);
}

u32 platform_can_setup( unsigned id, u32 clock )
{
  can_disable(can_base[id]);
  can_init(can_base[id], CANCLK, clock);
  can_enable(can_base[id]);
}

void platform_can_send( unsigned id, u32 canid, u8 idtype, u8 len, const u8 *data )
{
}

int platform_can_recv( unsigned id, u32 *canid, u8 *idtype, u8 *len, u8 *data )
{
}

#endif

// ****************************************************************************
// SPI

static void spis_init()
{
}

// cpol - clock polarity (0 or 1), cpha - clock phase (0 or 1)
u32 platform_spi_setup( unsigned id, int mode, u32 clock, unsigned cpol, unsigned cpha, unsigned databits )
{
}

spi_data_type platform_spi_send_recv( unsigned id, spi_data_type data )
{
}

void platform_spi_select( unsigned id, int is_select )
{
}

// ****************************************************************************
// I2C

#ifdef BUILD_I2C

u32 platform_i2c_setup( unsigned id, u32 speed )
{
}
// Address is 0..127, direction - from enum, return is boolean

int platform_i2c_send_address( unsigned id, u16 address, int direction )
{
}

int platform_i2c_send_byte( unsigned id, u8 data )
{
}

void platform_i2c_send_start( unsigned id )
{
}

void platform_i2c_send_stop( unsigned id )
{
}

int platform_i2c_recv_byte( unsigned id, int ack )
{
}

#endif	// BUILD_I2C

// ****************************************************************************
// UART

static void uarts_init()
{
}

u32 platform_uart_setup( unsigned id, u32 baud, int databits, int parity, int stopbits )
{
}

void platform_s_uart_send( unsigned id, u8 data )
{
}

int platform_s_uart_recv( unsigned id, timer_data_type timeout )
{
}

int platform_s_uart_set_flow_control( unsigned id, int type )
{
}


// ****************************************************************************
// Timers

static void timers_init()
{
}

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{
}

timer_data_type platform_s_timer_op( unsigned id, int op, timer_data_type data )
{
}

u64 platform_timer_sys_raw_read()
{
}

void platform_timer_sys_disable_int()
{
}

void platform_timer_sys_enable_int()
{
}

timer_data_type platform_timer_read_sys()
{
}

int platform_s_timer_set_match_int( unsigned id, timer_data_type period_us, int type )
{
}


// ****************************************************************************
// PWMs

#ifdef BUILD_PWM

static void pwms_init()
{
}

u32 platform_pwm_get_clock( unsigned id )
{
}

// Set the PWM clock
u32 platform_pwm_set_clock( unsigned id, u32 clock )
{
}

//duty - PWM channel duty cycle, specified as integer from 0 to 100 (percent).
u32 platform_pwm_setup( unsigned id, u32 frequency, unsigned duty )
{
}

void platform_pwm_start( unsigned id )
{
}

void platform_pwm_stop( unsigned id )
{
}

#endif // BUILD_PWM

// *****************************************************************************
// ADC specific functions and variables

#ifdef BUILD_ADC

int platform_adc_check_timer_id( unsigned id, unsigned timer_id )
{
}

void platform_adc_stop( unsigned id )
{
}

// Handle ADC interrupts
void ADCIntHandler( void )
{
}

static void adcs_init()
{
}

u32 platform_adc_set_clock( unsigned id, u32 frequency )
{
}

int platform_adc_update_sequence( )
{
}

int platform_adc_start_sequence()
{
}


#endif // ifdef BUILD_ADC

// ****************************************************************************
// Support for specific onboard devices 

// ****************************************************************************
// Ethernet functions
#ifdef BUILD_UIP
#endif // BUILD_UIP


// ****************************************************************************
// USB functions

// ****************************************************************************
// Flash access functions

#ifdef BUILD_WOFS

u32 platform_s_flash_write( const void *from, u32 toaddr, u32 size )
{
}

int platform_flash_erase_sector( u32 sector_id )
{
}

#endif // #ifdef BUILD_WOFS

// ****************************************************************************
// Platform specific modules go here

// ****************************************************************************
// Random number generator

static void rand_init()
{
  trng_enable(TRNG);
}

u32 rand_read()
{
  return trng_read_output_data(TRNG);
}
