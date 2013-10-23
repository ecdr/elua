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


/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <assert.h>

// NOTE: when using virtual timers, SYSTICKHZ and VTMR_FREQ_HZ should have the
// same value, as they're served by the same timer (the systick)
#define SYSTICKHZ 5


// ****************************************************************************
// Platform initialization

// forward
static void pios_init();
static void spis_init();
static void uarts_init();
static void timers_init();


#if NUM_PWM > 0
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

//	LED_Off(LED0_GPIO);
//	LED_Off(LED1_GPIO);
  
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

#if NUM_PWM > 0
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
  	if (SysTick_Config(platform_cpu_get_frequency() / SYSTICKHZ))
      return PLATFORM_ERR;    // SysTick error

  cmn_platform_init();

  // All done
  return PLATFORM_OK;
}


// ****************************************************************************
// PIO

// FIXME: PIOE net defined, but thought there were 5 ports - ??
const u32 pio_id[] =     { ID_PIOA, ID_PIOB, ID_PIOC, ID_PIOD };
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

const u32 can_id[]     = { ID_CAN0, ID_CAN1 };
Can * const can_base[] = { CAN0,    CAN1 };

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
  can_mailbox_init(CAN0, &can_rx_mailbox);
}

u32 platform_can_setup( unsigned id, u32 clock )
{
  can_disable(can_base[id]);
  can_init(can_base[id], CANCLK, clock);
  can_enable(can_base[id]);
  return clock;    // FIXME: Should return clock actually set
}

void platform_can_send( unsigned id, u32 canid, u8 idtype, u8 len, const u8 *data )
{
}

int platform_can_recv( unsigned id, u32 *canid, u8 *idtype, u8 *len, u8 *data )
{
#warning Not finished - return bogus result
  return PLATFORM_ERR;
}

#endif


// ****************************************************************************
// SPI

const u32 spi_id[]     = { ID_SPI0 };
Spi * const spi_base[] = { SPI0 };

static void spis_init()
{
  unsigned i;
  
  for (i = 0; i < NUM_SPI; i++)
  {
  pmc_enable_periph_clk(spi_id[i]); // FIXME: Is this needed? Not listed in example
  
  spi_enable_clock(spi_base[i]);
  spi_reset(spi_base[i]);
  spi_set_master_mode(spi_base[i]);
  spi_disable_mode_fault_detect(spi_base[i]);
  spi_disable_loopback(spi_base[i]);
//  spi_set_peripheral_chip_select_value(spi_base[i], DEFAULT_CHIP_ID);
  spi_set_fixed_peripheral_select(spi_base[i]);         // ??
  spi_disable_peripheral_select_decode(spi_base[i]);    // ??
  spi_set_delay_between_chip_select(spi_base[i], CONFIG_SPI_MASTER_DELAY_BCS);
  }
}

// cpol - clock polarity (0 or 1), cpha - clock phase (0 or 1)
// mode - PLATFORM_SPI_MASTER/SLAVE

u32 platform_spi_setup( unsigned id, int mode, u32 clock, unsigned cpol, unsigned cpha, unsigned databits )
{
  if (mode = PLATFORM_SPI_MASTER) 
  {
    struct spi_device *device;
    
// FIXME: Copied from quickstart code - ??? not sure what the device or ID is.
//void spi_master_setup_device(struct spi_device *device) - this was calling signature
    spi_set_transfer_delay(spi_base[id], device->id, CONFIG_SPI_MASTER_DELAY_BS,  CONFIG_SPI_MASTER_DELAY_BCT);

    spi_set_bits_per_transfer(spi_base[id], device->id, databits);
    spi_set_baudrate_div(spi_base[id], device->id, spi_calc_baudrate_div(clock, platform_cpu_get_frequency()));

    spi_configure_cs_behavior(spi_base[id], device->id, SPI_CS_KEEP_LOW);
    spi_set_clock_polarity(spi_base[id], device->id, cpol);
    spi_set_clock_phase(spi_base[id], device->id, cpha);

// FIXME: So far just enables configures SPI pins 
    gpio_configure_pin(SPI0_MISO_GPIO, SPI0_MISO_FLAGS);
    gpio_configure_pin(SPI0_MOSI_GPIO, SPI0_MOSI_FLAGS);
    gpio_configure_pin(SPI0_SPCK_GPIO, SPI0_SPCK_FLAGS);
// Alternate pins for NPCS on SPI0 (pick 1)
// FIXME: Needs uniform way to handle pin mapping (or at least add a regular control variable)
#ifndef SPI0_NPCS_PIN1
    gpio_configure_pin(SPI0_NPCS0_GPIO, SPI0_NPCS0_FLAGS);
#else
    gpio_configure_pin(SPI0_NPCS1_PA29_GPIO, SPI0_NPCS1_PA29_FLAGS);
#endif
    return 0; // clock; // FIXME: Return clock set
  }
  else
  { // FIXME: Need to implement slave
    return 0;
  }
}
//  spi_enable(spi_base[id])

spi_data_type platform_spi_send_recv( unsigned id, spi_data_type data )
{

//spi_status_t 	spi_read (spi_base[id], uint16_t *us_data, uint8_t *p_pcs)
//spi_status_t 	spi_write (spi_base[id], uint16_t data, uint8_t uc_pcs, uint8_t uc_last)
// result - SPI_OK or SPI_ERROR_TIMEOUT

// FIXME: Fill in body
  return data;
}

// FIXME: This is just a guess at what this might mean
// is_select - flag (PLATFORM_SPI_SELECT_ON or OFF)
void platform_spi_select( unsigned id, int is_select )
{
  spi_set_peripheral_chip_select_value(spi_base[id], is_select);
}


// ****************************************************************************
// I2C / TWI

#if NUM_I2C > 0

const u32 i2c_id[]     = { ID_TWI0, ID_TWI1 };
Twi * const i2c_base[] = { TWI0,    TWI1 };

// speed - PLATFORM_I2C_SPEED_FAST, PLATFORM_I2C_SPEED_SLOW
u32 platform_i2c_setup( unsigned id, u32 speed )
{
	twi_options_t opt;
  
  pmc_enable_periph_clk(i2c_id[id]);

// Configure pins
// TODO: Make this data driven (table)
  if (id == 0)
  {
    gpio_configure_pin(TWI0_DATA_GPIO, TWI0_DATA_FLAGS);
    gpio_configure_pin(TWI0_CLK_GPIO, TWI0_CLK_FLAGS);
  }
  else
  {
    gpio_configure_pin(TWI1_DATA_GPIO, TWI1_DATA_FLAGS);
    gpio_configure_pin(TWI1_CLK_GPIO, TWI1_CLK_FLAGS);
  }
  
  opt.master_clk = platform_cpu_get_frequency();
	opt.speed      = speed;

  if (twi_master_init(i2c_base[id], &opt) != TWI_SUCCESS) 
    return PLATFORM_ERR;
  else
    return ;// FIXME: What?
}

// Address is 0..127, direction - from enum (PLATFORM_I2C_DIRECTION_TRANSMITTER, PLATFORM_I2C_DIRECTION_RECEIVER), 
// return is boolean
int platform_i2c_send_address( unsigned id, u16 address, int direction )
{
/*
	twi_packet_t packet_tx;

	/* Configure the data packet to be transmitted */
/*	packet_tx.chip        = AT24C_ADDRESS;
	packet_tx.addr[0]     = EEPROM_MEM_ADDR >> 8;
	packet_tx.addr[1]     = EEPROM_MEM_ADDR;
	packet_tx.addr_length = EEPROM_MEM_ADDR_LENGTH;
	packet_tx.buffer      = (uint8_t *) test_data_tx;
	packet_tx.length      = TEST_DATA_LENGTH;

	if (twi_master_write(BOARD_BASE_TWI_EEPROM, &packet_tx) != TWI_SUCCESS) 
  
  */
}

int platform_i2c_send_byte( unsigned id, u8 data )
{
  twi_write_byte(i2c_base[id], data);
}

// FIXME: What are send_start and send_stop for?
#warning: i2c_send_start and send_stop not implemented
void platform_i2c_send_start( unsigned id )
{
}

void platform_i2c_send_stop( unsigned id )
{
}

// FIXME: What is ack for?
int platform_i2c_recv_byte( unsigned id, int ack )
{
  return twi_read_byte(i2c_base[id]);
}

#endif	// NUM_I2C > 0


// ****************************************************************************
// UART

// FIXME: Think there are only 3 UART/USART on Due
const u32 uart_id[] = { ID_UART, ID_USART0, ID_USART1, ID_USART3};
Usart* const uart_base[] = { (Usart *) UART, USART0, USART1, USART3};
//#define USART_SERIAL_PIO PINS_USART_PIO
//#define USART_SERIAL_TYPE PINS_USART_TYPE
//#define USART_SERIAL_PINS PINS_USART_PINS
//#define USART_SERIAL_MASK PINS_USART_MASK

static void uarts_init()
{
//  unsigned i;

//  for( i = 0; i < NUM_UART; i ++ )
//    sysclk_enable_peripheral_clock(uart_id[i]);
  sysclk_enable_peripheral_clock(ID_UART);
//  stdio_serial_init(CONF_UART, &uart_serial_options);

}

u32 platform_uart_setup( unsigned id, u32 baud, int databits, int parity, int stopbits )
{
  sam_usart_opt_t usart_settings;

  if( id < NUM_UART )
  {
    usart_settings.baudrate = baud;
    switch( databits )
    {
      case 5:
        usart_settings.char_length = US_MR_CHRL_5_BIT;
        break;
      case 6:
        usart_settings.char_length = US_MR_CHRL_6_BIT;
        break;
      case 7:
        usart_settings.char_length = US_MR_CHRL_7_BIT;
        break;
      case 9:
        usart_settings.char_length = US_MR_MODE9;
        break;
      default:
        usart_settings.char_length = US_MR_CHRL_8_BIT;
        break;
    };
    switch( parity )
    {
      case PLATFORM_UART_PARITY_EVEN:
        usart_settings.parity_type = US_MR_PAR_EVEN;
        break;
      case PLATFORM_UART_PARITY_ODD:
        usart_settings.parity_type = US_MR_PAR_ODD;
        break;
      case PLATFORM_UART_PARITY_MARK:
        usart_settings.parity_type = US_MR_PAR_MARK;
        break;
      case PLATFORM_UART_PARITY_SPACE:
        usart_settings.parity_type = US_MR_PAR_SPACE;
        break;
      default:
        usart_settings.parity_type = US_MR_PAR_NO;
        break;
    };
    switch( stopbits )
    {
      case PLATFORM_UART_STOPBITS_2:
        usart_settings.stop_bits = US_MR_NBSTOP_2_BIT;
        break;
      case PLATFORM_UART_STOPBITS_1_5:
        usart_settings.stop_bits = US_MR_NBSTOP_1_5_BIT;
        break;
      default:
        usart_settings.stop_bits = US_MR_NBSTOP_1_BIT;
        break;
    };
    usart_settings.channel_mode = US_MR_CHMODE_NORMAL;
    
    if (usart_init_rs232(uart_base[id], &usart_settings, platform_cpu_get_frequency()))
      return 0;     // Fixme: Failure
      
    usart_enable_tx(uart_base[id]);
    usart_enable_rx(uart_base[id]);
  // Need to configure pins for ports other than UART0 (assuming UART0 handled by ASF)

/*    PIO_Configure( g_APinDescription[PINS_UART].pPort, g_APinDescription[PINS_UART].ulPinType,
    g_APinDescription[PINS_UART].ulPin, g_APinDescription[PINS_UART].ulPinConfiguration); */
    
// FIXME: Make something data driven (table) here
  switch(id)
  {
    case 0:
    break;
    case 1:
    gpio_configure_pin(PIN_USART0_RXD_IDX, PIN_USART0_RXD_FLAGS);
    gpio_configure_pin(PIN_USART0_TXD_IDX, PIN_USART0_TXD_FLAGS);
    break;
    case 2:
    gpio_configure_pin(PIN_USART1_RXD_IDX, PIN_USART1_RXD_FLAGS);
    gpio_configure_pin(PIN_USART1_TXD_IDX, PIN_USART1_TXD_FLAGS);
    break;
    case 3:
    gpio_configure_pin(PIN_USART3_RXD_IDX, PIN_USART3_RXD_FLAGS);
    gpio_configure_pin(PIN_USART3_TXD_IDX, PIN_USART3_TXD_FLAGS);
    default:
    
    break;
    }
  }

  return baud;
}

void platform_s_uart_send( unsigned id, u8 data )
{
  usart_putchar(uart_base[id], data);
}

int platform_s_uart_recv( unsigned id, timer_data_type timeout )
{
  uint32_t c;

  if( timeout == 0 )
//    return MAP_UARTCharGetNonBlocking( base );
//  Fixme: Need non-blocking read (check if available, if available, then get, else return what)
      return PLATFORM_ERR;
  else
    if (!usart_getchar(uart_base[id], & c)) //TODO: Make it use timout value
      return c;
    else
      return PLATFORM_ERR;   // Fixme
}

int platform_s_uart_set_flow_control( unsigned id, int type )
{
  return PLATFORM_ERR;
}


// ****************************************************************************
// Timers
const u32 timer_id[] = {ID_TC0, ID_TC1, ID_TC2};
Tc * const timer_base[] = {TC0, TC1, TC2};

#define NUM_TC (sizeof timer_id/sizeof u32)

// FIXME: Something said there were 9 counter timers, but only TC0 through 2??
#define PLATFORM_TIMER_COUNT_MAX ((u32) 0xFFFFFFFF )

static void timers_init()
{
//  unsigned i;

//  for( i = 0; i < NUM_TC; i ++ )
//    pmc_enable_periph_clk(timer_id[i]);
  pmc_enable_periph_clk(ID_TC0);
}

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{
}

// Return timer number for given timer ID
static u32 timer(unsigned id)
{
  return id/3;
}

static Tc * tc(unsigned id)
{
  return timer_base[timer(id)];
}

// Timer id -> channel # 
static u32 tchanel(unsigned id) 
{
  return (u32) id - timer(id);
}

//	uint32_t ul_div;
//	uint32_t ul_tcclks;
//	static uint32_t ul_sysclk;

	/* Get system clock. */
//	ul_sysclk = sysclk_get_cpu_hz();

	/* Configure TC for a 50Hz frequency and trigger on RC compare. */
//	tc_find_mck_divisor(TC_FREQ, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
//	tc_init(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);
//	tc_write_rc(TC0, 0, (ul_sysclk / ul_div) / TC_FREQ);

#warning FIXME: Need to figure out what timer mode to use
#define TC_MODE_COUNTER 0

/*
component_tc.h
Capture mode - TIOA,B are inputs (trigger/measure signals)
Wave mode - free running (wave gen, etc.) TIOA output, TIOB out if not trigger

?? It several places says 32 bit counter, but it says that counter resets on 0xFFFF 
(i.e. 16 bits) which one is correct?

Trigger - software, 
  Compare to register C

Counter value
Registers A, B, C
Status
Timer channel mode:
TC_CMR_TCCLKS_TIMER_CLOCK1-5 (MCK/2, MCK/8, MCK/32, MCK/128, SLCK), TC_CMR_TCCLKS_XC0-2 
TC_CMR_CLKI - invert clock
Burst - gate clock with external signal (XC0-2)
 TC_CMR_BURST_NONE, TC_CMR_BURST_XC0-2
TC_CMR_LDBSTOP, TC_CMR_LDBDIS
Edge trigger - 
TC_CMR_ETRGEDG_NONE, TC_CMR_ETRGEDG_RISING, TC_CMR_ETRGEDG_FALLING, TC_CMR_ETRGEDG_EDGE (both)

TC_CMR_ABETRG
TC_CMR_CPCTRG
TC_CMR_WAVE - waveform mode
TC_CMR_LDRA_NONE, RISING, FALLING, EDGE - load reg. A on given edge
TC_CMR_LDRB_NONE, RISING, FALLING, EDGE - load reg. B on given edge
TC_CMR_CPCSTOP, TC_CMR_CPCDIS - Stopped/disabled with RC compare
TC_CMR_EEVTEDG - external event edge
*/
  
timer_data_type platform_s_timer_op( unsigned id, int op, timer_data_type data )
{
  u32 res = 0;

  data = data;
  switch( op )
  {
    case PLATFORM_TIMER_OP_START:
        res = 0xFFFFFFFF;      // FIXME
//      MAP_TimerControlTrigger(base, TIMER_A, false);
//      MAP_TimerLoadSet( base, TIMER_A, 0xFFFFFFFF );
        tc_init(TC0, id, TC_CMR_WAVE);   // FIXME - figure out mode
        tc_start(TC0, id);    // FIXME
      break;

    case PLATFORM_TIMER_OP_READ:
      res = tc_read_cv(TC0, id);
      break;

    case PLATFORM_TIMER_OP_SET_CLOCK:
    case PLATFORM_TIMER_OP_GET_CLOCK:
//      res = MAP_SysCtlClockGet();
// FIXME
      break;

    case PLATFORM_TIMER_OP_GET_MAX_CNT:
      res = PLATFORM_TIMER_COUNT_MAX;
      break;
  }
  return res;
}

int platform_s_timer_set_match_int( unsigned id, timer_data_type period_us, int type )
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


// ****************************************************************************
// PWMs

#if NUM_PWM > 0

const uint32_t pwm_chan[] = { PWM_CHANNEL_0, PWM_CHANNEL_1, PWM_CHANNEL_2, PWM_CHANNEL_3, 
                              PWM_CHANNEL_4, PWM_CHANNEL_5, PWM_CHANNEL_6, PWM_CHANNEL_7 };

static void pwms_init()
{
  pmc_enable_periph_clk(ID_PWM);
}

// PWM clock - master, /1,2,4,8,16,32,64,128,256,512,1024
// clocka, clockb - dividers, each can divide one of above by (1 to 255)

#define PWM_CLOCK_PRE_MAX  11

#define PWM_MAX_CLOCK 84*1000*1000
#warning PWM_MAX_CLOCK needs fixing
// FIXME: Should read from system clock

// PWM channel prescalers
const static u32 pwm_div_ctl[] = 
{ PWM_CMR_CPRE_MCK,         PWM_CMR_CPRE_MCK_DIV_2,   PWM_CMR_CPRE_MCK_DIV_4,  PWM_CMR_CPRE_MCK_DIV_8, 
  PWM_CMR_CPRE_MCK_DIV_16,  PWM_CMR_CPRE_MCK_DIV_32,  PWM_CMR_CPRE_MCK_DIV_64, PWM_CMR_CPRE_MCK_DIV_128, 
  PWM_CMR_CPRE_MCK_DIV_256, PWM_CMR_CPRE_MCK_DIV_512, PWM_CMR_CPRE_MCK_DIV_1024, 
  PWM_CMR_CPRE_CLKA, PWM_CMR_CPRE_CLKB };

// const static u8 pwm_div_data[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 };
// prescalers[11] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 };
#define pwm_prescalers( n ) ( 1 << (n) )
const unsigned nprescalers = PWM_CLOCK_PRE_MAX;
#define PWM_PRE_CLOCKA (nprescalers)
#define PWM_PRE_CLOCKB (nprescalers+1)
#define PWM_PRE_UNUSED (PWM_CLOCK_PRE_MAX+2)

// Which clock is each channel using
u32 pwm_chan_clock[NUM_PWM] = { 0 };
u8 pwm_chan_pre[NUM_PWM] = { PWM_PRE_UNUSED };

// Frequency for custom channel clocks
pwm_clock_t pwm_clock = { .ul_clka = 0, .ul_clkb = 0 };
// Use counters - how many channels using each custom clock
 u8 pwm_clka_users = 0, pwm_clkb_users = 0;

 
/*
 * Helper function:
 * Find a prescaler to generate the closest available clock frequency.
 * If the configuration cannot be met (because freq is too high), set the
 * maximum frequency possible.
 */
static u32 pwm_find_clock_prescaler( u32 frequency )
{
  unsigned prescaler;	// Which of the prescalers are we considering?

  if ( frequency > PWM_MAX_CLOCK )
    return 0; // Select master clock frequency

  for (prescaler = 0; prescaler < nprescalers; prescaler++)
    if ( pwm_prescalers( prescaler ) * frequency > PWM_MAX_CLOCK)
      break;
//  Assert( max/prescalers( prescaler-1) > frequency > max/prescalers(prescaler) or prescaler = nprescalers )
//  if ( prescaler < nprescalers )    
// Taking the absolute values of the differences in following comparison handles case where prescaler==nprescalers
    if ( abs(PWM_MAX_CLOCK - frequency * pwm_prescalers( prescaler-1 )) < abs(frequency * pwm_prescalers( prescaler ) - PWM_MAX_CLOCK))
//    ABSDIFF(max, frequency * pwm_prescalers( prescaler-1 )) < ABSDIFF(max, frequency * pwm_prescalers( prescaler ))
      return prescaler - 1;
    else
      return prescaler;
}

// Return the frequency of master clock divided by given prescaler
static u32 freq_from_prescaler( unsigned prescaler )
{
  return PWM_MAX_CLOCK / pwm_prescalers( prescaler );
}


#define FREQ_EPSILON  100
// Scaler for relative difference (100 would be % difference)
#define PWM_PARTS_PER 1000

// TODO: Use relative difference, rather than absolute difference
static int rel_diff(u32 f1, u32 f2)
{
  return (PWM_PARTS_PER * 2 * (f1 - f2)) / (f1 + f2);
}

u32 platform_pwm_get_clock( unsigned id )
{
  return pwm_chan_clock[id];
}

/* 
 TODO: PWM Clock allocation -
 If ask for a clock already available (one of prescaller dividers, or matches clocka or b) - then give them that
 Otherwise either allocate custom clock or give them a prescaller clock
   Greedy - Give custom clock if one available
  Keep track of what channels using what clock
    If no channels using a custom clock, then consider it available for (re)use
    Use count
 */

// Set the PWM clock
u32 platform_pwm_set_clock( unsigned id, u32 clock )
{
/*
  if clock == 0
    unuse previous clock;
  
  if clock == clocka
    Add another user of clocka
  if clock == clockb
    Add another user of clockb
  prescale = pwm_find_clock_prescaler(clock);
  if (freq_from_pre(prescaler) == clock )
    add user of appropriate division
  if clocka available
    use it
  if clockb available
    use it
  Make do with closest match
    how close is prescaler vs clocka vs clockb
*/
  if (clock)
  {
    // FIXME: Should disable any active PWM channels before setting clock
    // FIXME: Allow use clockb and fixed divisors
    //  e.g. keep track of clock value settings (and accept 2 different values, plus using divisions of mck)
    pwm_clock.ul_clka = clock;
    pwm_clock.ul_mck = CPU_FREQUENCY;
    pwm_init(PWM, &pwm_clock);
    if (pwm_chan_clock[id] == 0) pwm_clka_users++;        // Increment use count if wasn't already using
    pwm_chan_clock[id] = clock;
    pwm_chan_pre[id] = PWM_PRE_CLOCKA;
  }
  else
    if (pwm_chan_clock[id])           // Clearing clock on channel that was in use
    {
    pwm_chan_clock[id] = 0;
    pwm_clka_users--;
    }
  Assert(pwm_clka_users < NUM_PWM);
  return clock; // FIXME - what is it supposed to return?
}


#define PWM_MAX_PERIOD  ((1<<16) - 1)


/*
 * Configure a PWM channel to run at "frequency" Hz with a duty cycle of
 * "duty" (0-100).  0 means low all the time, 100 high all the time.
 * Return actual frequency set.
 */
//duty - PWM channel duty cycle, specified as integer from 0 to 100 (percent).
u32 platform_pwm_setup( unsigned id, u32 frequency, unsigned duty )
{
// FIXME: Probably need to fill in default values for other fields in pwm_inst
  pwm_channel_t pwm_inst;
  u32 pwmclk;        // base clock frequency for PWM counter
  u32 period;        // number of base clocks per cycle
  u32 duty_clocks;    // number of base clocks to be high (low?) for
  
  if (id >= NUM_PWM || duty > 100 || pwm_chan_clock[id] == 0) return 0;

  pwmclk = pwm_chan_clock[id];  

  // Compute period and duty period in clock cycles.
  //
  // PWM output wave frequency is requested in Hz but programmed as a
  // number of cycles of the master PWM clock frequency.
  //
  // Here, we use rounding to select the numerically closest available
  // frequency and return the closest integer in Hz to that.

  period = (pwmclk + frequency/2) / frequency;
  if (period == 0) period = 1;  
  if (period > PWM_MAX_PERIOD) period = PWM_MAX_PERIOD;
  duty_clocks = (period * duty + 50) / 100;
  
  pwm_inst.ul_prescaler = PWM_CMR_CPRE_CLKA;
  pwm_inst.channel = pwm_chan[id];
  pwm_inst.ul_period = period;
  pwm_inst.ul_duty = duty_clocks;
  pwm_channel_init(PWM, &pwm_inst);
  //FIXME: Need to configure associated pin appropriately

  return (pwmclk + period/2) / period;
}

//  pwm_inst.alignment = PWM_ALIGN_LEFT;
//  pwm_inst.polarity = PWM_LOW;
// PWM_CMR_CPRE_CLKB


void platform_pwm_start( unsigned id )
{
  if ( pwm_chan_clock[id] )
    pwm_channel_enable(PWM, pwm_chan[id]);
}

void platform_pwm_stop( unsigned id )
{
  pwm_channel_disable(PWM, pwm_chan[id]);
}
#undef pwm_prescalers

#endif // NUM_PWM > 0


// *****************************************************************************
// ADC specific functions and variables

#ifdef BUILD_ADC

#warning fix ADC_TIMER_IDs (they are arbitrarily set for now)
#define ADC_TIMER_FIRST_ID 8
#define ADC_NUM_TIMERS 1

int platform_adc_check_timer_id( unsigned id, unsigned timer_id )
{
  return ( ( timer_id >= ADC_TIMER_FIRST_ID ) && ( timer_id < ( ADC_TIMER_FIRST_ID + ADC_NUM_TIMERS ) ) );
}

void platform_adc_stop( unsigned id )
{
}

// Handle ADC interrupts
void ADCIntHandler( void )
{
}

static void adcs_init( void )
{
}

u32 platform_adc_set_clock( unsigned id, u32 frequency )
{
//  return frequency;
  return 0;   // FIXME: Signal an error - should return frequency set
}

int platform_adc_update_sequence( void )
{
//  return PLATFORM_OK;
  return PLATFORM_ERR;
}

int platform_adc_start_sequence( void )
{
//  return PLATFORM_OK;
  return PLATFORM_ERR;
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


#if defined( BUILD_USB_CDC )

static void usb_init()
{
// From stdio_usb example
  	// Initialize interrupt vector table support.
//	irq_initialize_vectors();

	// Enable interrupts
//	cpu_irq_enable();

  stdio_usb_init();
}

/*
unsigned long TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData)
{
}

unsigned long RxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData)
{
}

unsigned long
ControlHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData)
{
}
*/

#endif // BUILD_USB_CDC


// ****************************************************************************
// Flash access functions

#ifdef BUILD_WOFS

// INTERNAL_FLASH_START_ADDRESS, INTERNAL_FLASH_SIZE
#ifdef IFLASH0_SIZE
#define IFLASH0_END (IFLASH0_ADDR + IFLASH0_SIZE)
#endif

#ifdef IFLASH1_SIZE
#define IFLASH1_END (IFLASH1_ADDR + IFLASH1_SIZE)
#endif

// Wait cycles copied from example
#define FLASH_WAIT_CYCLES 6

int platform_flash_init()
{
 // Set access mode, wait cycles
  return (flash_init(FLASH_ACCESS_MODE_128, FLASH_WAIT_CYCLES) == FLASH_RC_OK) 
    ? PLATFORM_OK : PLATFORM_ERR;
}

static u8 flash_bank(u32 addr)
{
  if ((IFLASH0_ADDR <= addr) && (addr <= IFLASH0_END))
    return IFLASH0;
  if ((IFLASH1_ADDR <= addr) && (addr <= IFLASH1_END))
    return IFLASH1;   // FIXME: Need to define return values
  return PLATFORM_ERR;
}

// TODO: Check for crossing bank boundary (and handle appropriately)
u32 platform_s_flash_write( const void *from, u32 toaddr, u32 size )
{
  u32 result;

 if ( flash_bank(toaddr) != flash_bank(toaddr + size)
    return PLATFORM_ERR;  // FIXME: Really should do in 2 parts
	result = flash_write(toaddr, from, size, 1);  // Last arg - 1 if erase first
  return (result == FLASH_RC_OK) ? PLATFORM_OK : PLATFORM_ERR;
}

int platform_flash_erase_sector( u32 sector_id )
{
  return (flash_erase_sector(sector_id) == FLASH_RC_OK) ? PLATFORM_OK : PLATFORM_ERR;
}

#endif // #ifdef BUILD_WOFS


// ****************************************************************************
// Platform specific modules go here


// ****************************************************************************
// Random sequence generator

#ifdef BUILD_RAND

// TODO: Add buffer (so can generate ahead of need)
// TODO: Add interface to check if number available

// Need to arrange to call rand_init - maybe should be in system init(?)
static u8 f_rand_init = false;

/*
static u8 buf_space = true;
static u32 rand_buffer;

void TRNG_Handler(void)
{
	u32 status;

  status = trng_get_interrupt_status(TRNG);
  
  if (buf_space)
  {
    if ((status & TRNG_ISR_DATRDY) == TRNG_ISR_DATRDY) {
      buf_space = false;
      rand_buffer = trng_read_output_data(TRNG);
    }
  }
}
*/

u8 platform_rand_init()
{
  if ( !f_rand_init ) {
    f_rand_init = true;
    pmc_enable_periph_clk(ID_TRNG);
  
    trng_enable(TRNG);

	/* Enable TRNG interrupt */
/*
	NVIC_DisableIRQ(TRNG_IRQn);
	NVIC_ClearPendingIRQ(TRNG_IRQn);
	NVIC_SetPriority(TRNG_IRQn, 0);
	NVIC_EnableIRQ(TRNG_IRQn);
	trng_enable_interrupt(TRNG);
 */
  }
  return TRUE;
}

u32 platform_rand_next()
{
  if ( !f_rand_init )
    platform_rand_init();

  return trng_read_output_data(TRNG);
}

#endif // BUILD_RAND
