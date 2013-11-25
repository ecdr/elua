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

#include "platform_conf.h"
#include "common.h"
#include "math.h"
#include "diskio.h"
#include "lua.h"
#include "lauxlib.h"
#include "lrotable.h"
#include "elua_int.h" 

#ifdef BUILD_UIP
#include "uip_arp.h"
#include "elua_uip.h"
#include "uip-conf.h"
#endif

#ifdef BUILD_ADC
#include "elua_adc.h"
#endif

// Platform specific includes


/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <assert.h>

// NOTE: when using virtual timers, SYSTICKHZ and VTMR_FREQ_HZ should have the
// same value, as they're served by the same timer (the systick)
#define SYSTICKHZ 10

#if (SYSTICKHZ < 6)
// SAM3X8E - SYSTICKHZ must be greater than system clock/16777215 (SysTick_LOAD_RELOAD_Msk)
// (i.e. >5 for system clock 84MHz)
#warning SYSTICKHZ too small
#endif 

#if (VTMR_NUM_TIMERS > 0)
#if (SYSTICKHZ != VTMR_FREQ_HZ)
#warning SYSTICKHZ and VTMR_FREQ_HZ should have the same value
#endif
#endif // VTMR_NUM_TIMERS > 0


typedef struct _sam_pin_config {
  uint32_t pin;
  const uint32_t flags;
  } sam_pin_config;

// Interrupt priorities (0 is highest)
//#define RAND_INT_PRI  8
//#define CAN_INT_PRI   
//#define ADC_INT_PRI   

// ****************************************************************************
// Platform initialization

// forward
static void pios_init( void );
static void spis_init( void );
static void uarts_init( void );
static void timers_init( void );


#if NUM_PWM > 0
static void pwms_init( void );
#endif

#ifdef BUILD_ADC
void ADC_Handler( void );
static void adcs_init( void );
#endif

#ifdef BUILD_CAN
void can_common_handler(u8 id);

static void cans_init( void );
#endif

#ifdef BUILD_USB_CDC
static void usb_init( void );

void platform_usb_cdc_send( u8 data );
int platform_usb_cdc_recv( timer_data_type timeout );

#endif

#ifdef BUILD_I2C
static void i2cs_init( void );
#endif

int platform_init()
{
  // Set the clocking to run from PLL
  sysclk_init();

  board_init();		// Atmel Software Framework

  // Setup PIO
  pios_init();

//	LED_Off(LED0_GPIO);
  platform_pio_op( 1, PIN_LED_0_MASK, PLATFORM_IO_PIN_CLEAR );  
  // Bootloader leaves LED on, so turn it off signal eLua running

  // Setup SPIs
#if NUM_SPI > 0
#warning SPI not tested
  spis_init();
#endif

  // Setup UARTs
  uarts_init();

  // Setup timers
#warning TMR not done  
  timers_init();

#ifdef BUILD_I2C
  // Setup I2Cs
#warning I2C/TWI not written  
  i2cs_init();
#endif // ifdef BUILD_I2C

#if NUM_PWM > 0
  // Setup PWMs
  pwms_init();
#endif

#ifdef BUILD_ADC
  // Setup ADCs
#warning ADC not written  
  adcs_init();
#endif

#ifdef BUILD_CAN
  // Setup CANs
#warning CAN not done
  cans_init();
#endif

#ifdef BUILD_USB_CDC
  // Setup USB
  usb_init();
#endif


#if defined(PLATFORM_HAS_SYSTIMER) && defined(SYSTICKHZ)
  // Setup system timer
  	if (SysTick_Config(platform_cpu_get_frequency() / ((u32) SYSTICKHZ)))
      return PLATFORM_ERR;    // SysTick error
#endif //SYSTIMER

  cmn_platform_init();

  // All done
  return PLATFORM_OK;
}


// ****************************************************************************
// PIO

const u32 pio_id[] =     { ID_PIOA, ID_PIOB, ID_PIOC, ID_PIOD };
Pio * const pio_base[] = { PIOA,    PIOB,    PIOC,    PIOD };

#ifdef PROTECT_PINS
// PIN_C0 - erase button

// Don't absolutely have to protect these, since startup with special function
// C0 = 1 on startup will erase, and JTAG active on startup
// But for the novice/experimenter, might be well to protect from shooting in foot by reprogram console (e.g.)

// UARTRX, TX - console (unless BUILD_USBCDC) FIXME: Find macro definition for pinmask
// Could extend protection to other devices used (CAN// CANRX, TX
// JTAG/SWD lines - PB28-31

// Note: C29 connected to A28, C26 to A29

#define PROTECT_BASE  ((u32) 0)
#define PROTECT_A (PROTECT_BASE | PIN8 | PIN9 )
#define PROTECT_B (PROTECT_BASE | PROTECT_JTAG )
#define PROTECT_C (PROTECT_BASE | PROTECT_ERASE )
#define PROTECT_D PROTECT_BASE

u32 pio_protect[] = { PROTECT_A, PROTECT_B, PROTECT_C, PROTECT_D };
#endif

#define PIO_MASK_ALL  0xFFFFFFFFUL


static void pios_init(void)
{
  unsigned i;
  
  for( i = 0; i < NUM_PIO; i ++)
    pmc_enable_periph_clk(pio_id[i]);
}



// TODO: I think platform modules generally return 0 for error, though seems to vary some?

pio_type platform_pio_op( unsigned port, pio_type pinmask, int op )
{
  pio_type retval = 1; 
  Pio * base = pio_base[ port ];

    
  switch( op )
  {
    case PLATFORM_IO_PORT_SET_VALUE:
    // Is there way to write all pins (low and high) at same time?
      pio_clear(base, ~(pinmask));
      pio_set(base, pinmask);
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
// TODO: Arduino source code says that can save power by disabling clock on a port 
//  if all pins on port are output.
// Could wait to enable clock on port until pins set as input, 
//  and disable clock after all pins set as output
/* if all pins are output, disable PIO Controller clocking, reduce power consumption */
//            if ( g_APinDescription[ulPin].pPort->PIO_OSR == PIO_MASK_ALL )
//            {
//                pmc_disable_periph_clk( pio_id[port] ) ;
//            }
//
//      pio_set_output(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_default_level,
//		const uint32_t ul_multidrive_enable,
//		const uint32_t ul_pull_up_enable)

    case PLATFORM_IO_PIN_DIR_OUTPUT:
#ifdef PROTECT_PINS
      if ((2 == port) && (pinmask & 1))
        pinmask &= (~ (u32) 1);       // PIN_C0 is erase button - do not reprogram it
#endif // PROTECT_PINS
        
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

    case PLATFORM_IO_PIN_PULLDOWN:    // No pulldown on this platform, so return error
    default:
      retval = 0;
      break;
  }
  return retval;
}

//void pio_set_multi_driver(Pio *p_pio, const uint32_t ul_mask,
//		const uint32_t ul_multi_driver_enable);


// ****************************************************************************
// CAN

#if defined( BUILD_CAN )

const u32 can_id[]     = { ID_CAN0, ID_CAN1 };
Can * const can_base[] = { CAN0,    CAN1 };

// Pins

#define PIN_CFG_CANRX0  {PIO_PA1_IDX, (PIO_PERIPH_A | PIO_DEFAULT)}
#define PIN_CFG_CANTX0  {PIO_PA0_IDX, (PIO_PERIPH_A | PIO_DEFAULT)}
#define PIN_CFG_CANRX1  {PIO_PB15_IDX, (PIO_PERIPH_A | PIO_DEFAULT)}
#define PIN_CFG_CANTX1  {PIO_PB14_IDX, (PIO_PERIPH_A | PIO_DEFAULT)}

const struct { sam_pin_config rx, tx; } can_pins[] = {
  {PIN_CFG_CANRX0, PIN_CFG_CANTX0}, // CAN0
  {PIN_CFG_CANRX1, PIN_CFG_CANTX1}  // CAN1
  };


// Speed used in INIT
#ifndef CAN_INIT_SPEED
#define CAN_INIT_SPEED	500
#endif

// FIXME: Check CAN controller frequency - just a guess that might be CPU_FREQ
#define CANCLK CPU_FREQUENCY

can_mb_conf_t can_rx_mailbox;

// Interrupt handlers
void can_common_handler(u8 id)
{
}

void CAN0_Handler(void)
{
  can_common_handler(0);
}
  
void CAN1_Handler(void)
{
  can_common_handler(1);
}

// Setup

// FIXME: Generalize to cover both CANs
void cans_init( void )
{
  pmc_enable_periph_clk(ID_CAN0);
  can_init(CAN0, CANCLK, CAN_BPS_1000K);
// FIXME: Need to check speed - use default from above
  can_reset_all_mailbox(CAN0);
//  can_enable_interrupt(CAN0, uint32_t dw_mask)
// FIXME: What is dw_mask?

// FIXME: Need to check mailbox particulars
  can_rx_mailbox.ul_mb_idx = 0;
  can_rx_mailbox.uc_obj_type = CAN_MB_RX_MODE;
  can_rx_mailbox.ul_id_msk = CAN_MAM_MIDvA_Msk | CAN_MAM_MIDvB_Msk;
  can_rx_mailbox.ul_id = CAN_MID_MIDvA(0x00);
  can_mailbox_init(CAN0, &can_rx_mailbox);
}

// Platform 

u32 platform_can_setup( unsigned id, u32 clock )
{
  gpio_configure_pin(can_pins[id].rx.pin, can_pins[id].rx.flags);
  gpio_configure_pin(can_pins[id].tx.pin, can_pins[id].tx.flags);

  can_disable(can_base[id]);
  can_init(can_base[id], CANCLK, clock);
  can_enable(can_base[id]);
  return clock;    // FIXME: Should return clock actually set
}

// FIXME: To be written
void platform_can_send( unsigned id, u32 canid, u8 idtype, u8 len, const u8 *data )
{
  lua_assert(false);
}

// FIXME: To be written
int platform_can_recv( unsigned id, u32 *canid, u8 *idtype, u8 *len, u8 *data )
{
  lua_assert(false);
  return PLATFORM_ERR;
}

#endif


// ****************************************************************************
// SPI

// TODO: USART can also do SPI

// One SPI device can handle 4 peripherals (well actually 16, but not messing with the chip select decode)
// However ship select 3 is not brought out to header on Due
const u32 spi_id[]     = { ID_SPI0
#ifdef ID_SPI1
, ID_SPI1 
#endif
};
Spi * const spi_base[] = { SPI0
#ifdef ID_SPI1
, SPI1 
#endif
};


// Pins
#define SPI_NPCS1 'A'
#define SPI_NPCS2 'A'
#define SPI_NPCS3 'A'

// FIXME: Needs uniform way to handle pin mapping (or at least add a regular control variable)
const struct { sam_pin_config miso, mosi, spck, npcs0, npcs1, npcs2, npcs3; } spi_pins[NUM_SPI] = {
  {{SPI0_MISO_GPIO, SPI0_MISO_FLAGS}, {SPI0_MOSI_GPIO, SPI0_MOSI_FLAGS}, {SPI0_SPCK_GPIO, SPI0_SPCK_FLAGS}, 
  {SPI0_NPCS0_GPIO, SPI0_NPCS0_FLAGS},
// Alternate pins for NPCS on SPI0 (pick 1)
#if SPI0_NPCS1 == 'A'
  {SPI0_NPCS1_PA29_GPIO, SPI0_NPCS1_PA29_FLAGS},  // Cross-connect to C.26
#else
  {SPI0_NPCS1_PB20_GPIO, SPI0_NPCS1_PB20_FLAGS},  // pin A11
#endif
#if SPI0_NPCS2 == 'A'
  {SPI0_NPCS2_PA30_GPIO, SPI0_NPCS2_PA30_FLAGS},  // ?? no pin
#else
  {SPI0_NPCS2_PB21_GPIO, SPI0_NPCS2_PB21_FLAGS},  // pin 52
#endif
#if SPI0_NPCS3 == 'A'
  {SPI0_NPCS3_PA31_GPIO, SPI0_NPCS3_PA31_FLAGS}   // ?? no pin
#else
  {SPI0_NPCS3_PB23_GPIO, SPI0_NPCS3_PB23_FLAGS}   // ?? no pin
#endif
  }
#ifdef ID_SPI1
  ,  // SPI0
  {
    {SPI1_MISO_GPIO, SPI1_MISO_FLAGS},   {SPI1_MOSI_GPIO, SPI1_MOSI_FLAGS},   {SPI1_SPCK_GPIO, SPI1_SPCK_FLAGS}, 
    {SPI1_NPCS0_GPIO, SPI1_NPCS0_FLAGS}, {SPI1_NPCS1_GPIO, SPI1_NPCS1_FLAGS}, {SPI1_NPCS2_GPIO, SPI1_NPCS2_FLAGS}, 
    {SPI1_NPCS3_GPIO, SPI1_NPCS3_FLAGS}
  }   // SPI1 Port E (so not on Due)
#endif
};


static void spis_init( void )
{
  unsigned i;
  
  for (i = 0; i < NUM_SPI_DEV; i++)
  {
  pmc_enable_periph_clk(spi_id[i]); // FIXME: Is this needed? Not listed in example
  
  spi_master_init(spi_base[i]);
  }
}


// Convert number of bits to field value (range 8 to 16; 8 -> 0, 9->1, etc.;
// value is shifted left to bit field position)  See component_spi.h, e.g. SPI_CSR_BITS_8_BIT
static u32 spi_trans_bits_encode(unsigned databits)
{
  if (databits < 8)
    databits = 8;
  if (databits > 16)
    databits = 16;
  return (databits - 8) << SPI_CSR_BITS_Pos;
}


// cpol - clock polarity (0 or 1), cpha - clock phase (0 or 1)
// mode - PLATFORM_SPI_MASTER/SLAVE
u32 platform_spi_setup( unsigned id, int mode, u32 clock, unsigned cpol, unsigned cpha, unsigned databits )
{
  unsigned spi_id = id/4;
  Spi * spi_dev = spi_base[spi_id];
  unsigned spi_chan = id & 0x3;   // 3 channels per SPI device
  
#if NUM_SPI > 0
  if (mode == PLATFORM_SPI_MASTER) 
  {
    int16_t baud_div = spi_calc_baudrate_div(clock, platform_cpu_get_frequency());

    if (-1 == baud_div) {
      return 0;// Assert(0 == "Failed to find baudrate divider");
    }
    spi_set_transfer_delay(spi_dev, spi_chan, CONFIG_SPI_MASTER_DELAY_BS,  CONFIG_SPI_MASTER_DELAY_BCT);

    spi_set_bits_per_transfer(spi_dev, spi_chan, spi_trans_bits_encode(databits));
    spi_set_baudrate_div(spi_dev, spi_chan, baud_div);

    spi_configure_cs_behavior(spi_dev, spi_chan, SPI_CS_KEEP_LOW);
    spi_set_clock_polarity(spi_dev, spi_chan, cpol);   // TODO: Check polarity/phase whether values need massage
    spi_set_clock_phase(spi_dev, spi_chan, cpha);

    gpio_configure_pin(spi_pins[spi_id].miso.pin, spi_pins[spi_id].miso.flags);
    gpio_configure_pin(spi_pins[spi_id].mosi.pin, spi_pins[spi_id].mosi.flags);
    gpio_configure_pin(spi_pins[spi_id].spck.pin, spi_pins[spi_id].spck.flags);
    switch(spi_chan)  // Set up chip select pin for this channel
    {
      case 0:
        gpio_configure_pin(spi_pins[spi_id].npcs0.pin, spi_pins[spi_id].npcs0.flags);
        break;
      case 1:
        gpio_configure_pin(spi_pins[spi_id].npcs1.pin, spi_pins[spi_id].npcs1.flags);
        break;
      case 2:
        gpio_configure_pin(spi_pins[spi_id].npcs2.pin, spi_pins[spi_id].npcs2.flags);
        break;
      case 3:
        gpio_configure_pin(spi_pins[spi_id].npcs3.pin, spi_pins[spi_id].npcs3.flags);
        break;
    }
    return platform_cpu_get_frequency()/baud_div; // Return clock set
  }
  else
  { // FIXME: Need to implement slave
    return 0;
  }
#else
  return 0;
#endif
}

#define NONE_CHIP_SELECT_ID 0x0f

spi_data_type platform_spi_send_recv( unsigned id, spi_data_type data )
{
  unsigned spi_id = id/4;
  uint16_t read_data;
  uint8_t dummy = NONE_CHIP_SELECT_ID;  // Dummy argument, not used in fixed peripheral select mode
  
// result - SPI_OK or SPI_ERROR_TIMEOUT
//  if (spi_write(spi_base[id], data, dummy, false) == SPI_ERROR_TIMEOUT)
//    return ; // rest of code does not have facility to handle error returns

//  if (spi_read(spi_base[id], &read_data, &dummy) == SPI_ERROR_TIMEOUT)
//    return ;  // what?
  spi_write(spi_base[spi_id], data, dummy, false);
  spi_read(spi_base[spi_id], &read_data, &dummy);
  
  return read_data;
}


// is_select - flag (PLATFORM_SPI_SELECT_ON or OFF)
void platform_spi_select( unsigned id, int is_select )
{
  unsigned spi_id = id/4;
  unsigned spi_chan = id & 0x3;
  
  spi_set_peripheral_chip_select_value(spi_base[spi_id],
    ((PLATFORM_SPI_SELECT_ON == is_select) ? ~((0x1) << (spi_chan) ) : NONE_CHIP_SELECT_ID));
  // chip select is a 4 bit wide bit map, select the lowest line with a 0 in it
}

// TODO: If this is supposed to deslect when SPI_SELECT_OFF, then see 
// spi_deselect_device - basically adds wait until TX done before select and spi_set_lastxfer(p_spi) after;


// ****************************************************************************
// I2C / TWI

#if NUM_I2C > 0

const u32 i2c_id[]     = { ID_TWI0, ID_TWI1 };
Twi * const i2c_base[] = { TWI0,    TWI1 };


const struct { sam_pin_config data, clk; } twi_pins[] = {
  {{TWI0_DATA_GPIO, TWI0_DATA_FLAGS}, {TWI0_CLK_GPIO, TWI0_CLK_FLAGS}}, // I2C0
  {{TWI1_DATA_GPIO, TWI1_DATA_FLAGS}, {TWI1_CLK_GPIO, TWI1_CLK_FLAGS}}  // I2C1
  };


// speed - PLATFORM_I2C_SPEED_FAST, PLATFORM_I2C_SPEED_SLOW
u32 platform_i2c_setup( unsigned id, u32 speed )
{
	twi_options_t opt;
  
  pmc_enable_periph_clk(i2c_id[id]);

// Configure pins
  gpio_configure_pin(twi_pins[id].data.pin, twi_pins[id].data.flags);
  gpio_configure_pin(twi_pins[id].clk.pin,  twi_pins[id].clk.flags);
/*
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
*/
  
  opt.master_clk = platform_cpu_get_frequency();
	opt.speed      = speed;

  if (twi_master_init(i2c_base[id], &opt) != TWI_SUCCESS) 
    return 0;
  else
    return speed;// FIXME: should return actual speed set
}

// Address is 0..127, direction - from enum (PLATFORM_I2C_DIRECTION_TRANSMITTER, PLATFORM_I2C_DIRECTION_RECEIVER), 
// return is boolean
int platform_i2c_send_address( unsigned id, u16 address, int direction )
{
/*
	twi_packet_t packet_tx;

//   Configure the data packet to be transmitted 
	packet_tx.chip        = AT24C_ADDRESS;
	packet_tx.addr[0]     = EEPROM_MEM_ADDR >> 8;
	packet_tx.addr[1]     = EEPROM_MEM_ADDR;
	packet_tx.addr_length = EEPROM_MEM_ADDR_LENGTH;
	packet_tx.buffer      = (uint8_t *) test_data_tx;
	packet_tx.length      = TEST_DATA_LENGTH;

	if (twi_master_write(BOARD_BASE_TWI_EEPROM, &packet_tx) != TWI_SUCCESS) 
  
  */
  lua_assert(false);
  return PLATFORM_ERR;  //? what is return
}

int platform_i2c_send_byte( unsigned id, u8 data )
{
  twi_write_byte(i2c_base[id], data);
  lua_assert(false);
  return ; //? what is return
}

// FIXME: What are send_start and send_stop for?
void platform_i2c_send_start( unsigned id )
{
  lua_assert(false);
}

void platform_i2c_send_stop( unsigned id )
{
  lua_assert(false);
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

// Only some U(S)ARTs have hardware flow control
const bool no_hw_flow[] = { true, false, false, true };

// Pins
#define PIN_CFG_CTS0  {PIO_PB26_IDX, (PIO_PERIPH_A | PIO_DEFAULT)}
#define PIN_CFG_RTS0  {PIO_PB25_IDX, (PIO_PERIPH_A | PIO_DEFAULT)}
#define PIN_CFG_SCK0  {PIO_PA17_IDX, (PIO_PERIPH_B | PIO_DEFAULT)}

#define PIN_CFG_CTS1  {PIO_PA15_IDX, (PIO_PERIPH_A | PIO_DEFAULT)}
#define PIN_CFG_RTS1  {PIO_PA14_IDX, (PIO_PERIPH_A | PIO_DEFAULT)}
#define PIN_CFG_SCK1  {PIO_PA16_IDX, (PIO_PERIPH_A | PIO_DEFAULT)}

#define PIO_INVALID_IDX 0xFFFF
#define PIN_CFG_NOPIN  {PIO_INVALID_IDX, 0}

// DANGER Will Robinson - Note that this array starts with USART0, not UART so need to use ID-1 as index
// Might be safer to put in dummy record for UART at the beginning

const struct { sam_pin_config rx, tx, cts, rts, sck; } usart_pins[] = {
        {{PIN_USART0_RXD_IDX, PIN_USART0_RXD_FLAGS}, {PIN_USART0_TXD_IDX, PIN_USART0_TXD_FLAGS}, 
          PIN_CFG_CTS0, PIN_CFG_RTS0, PIN_CFG_SCK0},  // USART0
        {{PIN_USART1_RXD_IDX, PIN_USART1_RXD_FLAGS}, {PIN_USART1_TXD_IDX, PIN_USART1_TXD_FLAGS},
          PIN_CFG_CTS1, PIN_CFG_RTS1, PIN_CFG_SCK1},  // USART1
        {{PIN_USART3_RXD_IDX, PIN_USART3_RXD_FLAGS}, {PIN_USART3_TXD_IDX, PIN_USART3_TXD_FLAGS},
          PIN_CFG_NOPIN, PIN_CFG_NOPIN, PIN_CFG_NOPIN}};  // USART3 (note no RTS/CTS)


//PIO_PA8A_URXD
//PIO_PA9A_UTXD 


static void uarts_init()
{
  unsigned i;

  for( i = 0; i < NUM_UART; i ++ )
    sysclk_enable_peripheral_clock(uart_id[i]);
//  sysclk_enable_peripheral_clock(ID_UART);

//  stdio_serial_init(CONF_UART, &uart_serial_options);
}


u32 platform_uart_setup( unsigned id, u32 baud, int databits, int parity, int stopbits )
{
  sam_usart_opt_t usart_settings;

#ifdef BUILD_USB_CDC
  if( id == CDC_UART_ID )
    return 0; //FIXME
#endif

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

    /* Disable all uart interrupts. */
//    usart_disable_interrupt(uart_base[id], ALL_INTERRUPT_MASK);
  
    usart_enable_tx(uart_base[id]);
    usart_enable_rx(uart_base[id]);

    // Configure pins for ports other than UART0 (assuming UART0 handled by ASF)
    if (id) {
      gpio_configure_pin(usart_pins[id-1].rx.pin, usart_pins[id-1].rx.flags);
      gpio_configure_pin(usart_pins[id-1].tx.pin, usart_pins[id-1].tx.flags);
    };
  }

  return baud;
}

        
void platform_s_uart_send( unsigned id, u8 data )
{
#ifdef BUILD_USB_CDC
  if( id == CDC_UART_ID )
    platform_usb_cdc_send( data );
  else
#endif
  {
    WAIT_WHILE(!usart_is_tx_ready(uart_base[id]));  // FIXME: Should add timeout on wait
    usart_putchar(uart_base[id], data);
  }
}


// platform_s_uart_recv - return character, or -1 for error
int platform_s_uart_recv( unsigned id, timer_data_type timeout )
{
  uint32_t c;

#ifdef BUILD_USB_CDC
  if( id == CDC_UART_ID )
    return platform_usb_cdc_recv( timeout );
#endif

// If character already waiting, don't check timeout, just get it
  if( timeout == 0 || usart_is_rx_ready(uart_base[id]))
    if (!usart_read(uart_base[id], & c))  // Nonblocking read
      return c;
    else
      return -1;
  else
  {
    usart_set_rx_timeout(uart_base[id], timeout);
    usart_start_rx_timeout(uart_base[id]);
    if (!usart_getchar(uart_base[id], & c)) // Wait for character or timeout
      return c;
    else
      return -1;
  }
}


// TODO: Test flow control
int platform_s_uart_set_flow_control( unsigned id, int type )
{
#ifdef BUILD_USB_CDC
  if( id == CDC_UART_ID)
  {
    if (type == PLATFORM_UART_FLOW_NONE)  // FIXME: For now assume that USB CDC doesn't have flow control
      return PLATFORM_OK;
    else
      return PLATFORM_ERR;
  };
#endif
  
  if (no_hw_flow[id])  // UART, USART3 - do not have RTS/CTS
  {
    if (type == PLATFORM_UART_FLOW_NONE)
      return PLATFORM_OK;
    else
      return PLATFORM_ERR;
  };

  if( type != PLATFORM_UART_FLOW_NONE && type != ( PLATFORM_UART_FLOW_RTS | PLATFORM_UART_FLOW_CTS ) )
    return PLATFORM_ERR;

  if( type != PLATFORM_UART_FLOW_NONE ){ // enable pins for UART functionality
      gpio_configure_pin(usart_pins[id-1].rts.pin, usart_pins[id-1].rts.flags);
      gpio_configure_pin(usart_pins[id-1].cts.pin, usart_pins[id-1].cts.flags);
        // From usart.c - usart_init_hw_handshaking()
      uart_base[id]->US_MR = (uart_base[id]->US_MR & ~US_MR_USART_MODE_Msk) | US_MR_USART_MODE_HW_HANDSHAKING;
    }
  else // release pins to GPIO module
    {
      uart_base[id]->US_MR = uart_base[id]->US_MR & ~US_MR_USART_MODE_HW_HANDSHAKING;
      gpio_configure_pin(usart_pins[id-1].rts.pin, PIO_DEFAULT);
      gpio_configure_pin(usart_pins[id-1].cts.pin, PIO_DEFAULT);
    }

  return PLATFORM_OK;
}


// ****************************************************************************
// Timers

const u32 timer_id[] = {ID_TC0, ID_TC1, ID_TC2};
Tc * const timer_tc[] = {TC0, TC1, TC2};

static u32 tc_num(unsigned id);
Tc * tc(unsigned id);
u32 tchannel(unsigned id);

// TODO: See if library files provide a name for this
#define PLATFORM_TIMER_COUNT_MAX ( 0xFFFFFFFFUL )


// 3 timer devices, each with 3 channels

// FIXME: NUM_TC or num_tc - Pick one (if it will work, the const might be more type safe)
#define NUM_TC (sizeof(timer_id)/sizeof (u32))
const unsigned num_tc = (sizeof(timer_id)/sizeof(u32));

// Each TC has 3 timer channels
unsigned const channels_per_tc = 3;

// Indicate which module is using each timer
// TODO: Look at other modules that use timers, as timers - ADC, UART (should they also have indicator)?
typedef enum {
  TMODE_UNINIT = 0,
  TMODE_TIMER,
  TMODE_COUNTER,
  TMODE_PWM } timer_mode_t;


timer_mode_t tmode[ NUM_TIMER ] = { TMODE_UNINIT };  // TODO: Or should this be handled up in timer module?

u8 platform_timer_int_periodic_flag[ NUM_TIMER ] = {0};

// timer clock
// TODO: Is there an interface to read the clock so don't have to cache/compute it?
static uint32_t tmr_div[ NUM_TIMER ] = { 0 };

// TODO: Way to check if a timer is running

// Helper functions

// Return timer device number for given eLua timer ID
static inline u32 tc_num(unsigned id)
{
  return id/channels_per_tc;
}

// Return pointer to Tc for given eLua timer id
Tc * tc(unsigned id)
{
  return timer_tc[tc_num(id)];
}

// channel number from eLua timer id
u32 tchannel(unsigned id) 
{
  return (u32) id - (tc_num(id) * channels_per_tc);
}


static void timers_init()
{
  unsigned i;

  for( i = 0; i < NUM_TC; i ++ )
    pmc_enable_periph_clk(timer_id[i]);
}


static u32 plat_timer_get_clock( unsigned id )
{
  if (tmr_div[id])
    return CPU_FREQUENCY/tmr_div[id];
  else
    return 0;
}


static u32 plat_timer_set_clock(unsigned id, timer_data_type clock, u32 mode)
{
	static uint32_t ul_sysclk;    // FIXME: What static for?
	uint32_t tmr_tcclks;
  ul_sysclk = CPU_FREQUENCY;  /* Get system clock. */
  
  if ( tc_find_mck_divisor(clock, ul_sysclk, &tmr_div[id], &tmr_tcclks, ul_sysclk) )
  {
    tc_init(tc(id), tchannel(id), tmr_tcclks | mode);
    return plat_timer_get_clock( id );
  }
  else  // Could not find suitable divisor
  {
    tmr_div[id] = 0;
    return 0;
  }
}

// Platform functions

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{
  timer_data_type final;  // u64?

  final = ( ( u64 )delay_us * plat_timer_get_clock( id ) ) / 1000000;

  if( final == 0) return;
  if( final > PLATFORM_TIMER_COUNT_MAX )
    final = PLATFORM_TIMER_COUNT_MAX;     // FIXME: this isn't right (copied from another platform) - should do rollover instead (on the other hand, unless use u64 for final, this test makes no sense
// TODO: stop timer, set count to 0, start timer
  tc_init( tc(id), tchannel(id), TC_CMR_WAVE);
  tc_start( tc(id), tchannel(id) );
  WAIT_WHILE( ( tc_read_cv(tc(id), tchannel(id)) < final ) );
// TODO: should probably stop the timer (?)
}

// Example code using timer
//	uint32_t ul_div;
//	uint32_t ul_tcclks;
//	static uint32_t ul_sysclk;

	/* Get system clock. */
//	ul_sysclk = CPU_FREQUENCY;

	/* Configure TC for a 50Hz frequency and trigger on RC compare. */
//	tc_find_mck_divisor(TC_FREQ, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
//	tc_init(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);
//	tc_write_rc(TC0, 0, (ul_sysclk / ul_div) / TC_FREQ);

// FIXME: Need to figure out what timer mode to use
//#define TC_MODE_TIMER   
//#define TC_MODE_COUNTER 
//#define TC_MODE_PWM     
  
timer_data_type platform_s_timer_op( unsigned id, int op, timer_data_type data )
{
  u32 res = 0;

  data = data;  // TODO: What is this for?
  switch( op )
  {
    case PLATFORM_TIMER_OP_START:
      tc_start( tc(id), tchannel(id) );
      break;

    case PLATFORM_TIMER_OP_READ:
      res = tc_read_cv( tc(id), tchannel(id) );
      break;

    case PLATFORM_TIMER_OP_SET_CLOCK:
      // Timer mode: Wave, Clear when count reaches register C, Count up to register C, then reset
      //          | TC_IER_CPCS // Interrupt when count reaches register C
      res = plat_timer_set_clock(id, data, TC_CMR_WAVE | TC_CMR_CPCTRG | TC_CMR_WAVSEL_UP_RC );
      if (res)
        tmode[id] = TMODE_TIMER;
      break;
      
    case PLATFORM_TIMER_OP_GET_CLOCK:
      res = plat_timer_get_clock( id );
      break;

    case PLATFORM_TIMER_OP_GET_MAX_CNT:
      res = PLATFORM_TIMER_COUNT_MAX;
      break;
  }
  return res;
}


// TODO: Needs testing

// period_us - period in microseconds, if period = 0, then use maximum possible period (I think that what means)??
// type: PLATFORM_TIMER_INT_CYCLIC or PLATFORM_TIMER_INT_ONESHOT
int platform_s_timer_set_match_int( unsigned id, timer_data_type period_us, int type )
{
  u64 final;

  if( period_us == 0 )  // Might be handled by setting period_us to largest possible period and continue
  {
    tc_stop(tc(id), tchannel(id));
//    tc_disable_interrupt(tc(id), tchannel(id), TC_IDR_CPCS);   // Disable reg C interrupt (would only do if set an interrupt on counter overflow)
    tc_get_status(tc(id), tchannel(id));
    platform_timer_int_periodic_flag[ id ] = type;              // This was ignored in one example - ??
    tc_write_rc(tc(id), tchannel(id), PLATFORM_TIMER_COUNT_MAX); // Or could just drop the register C mode and go to full wave (changing interrupt type).
    // Should set CMR to CPCstop 
    tc_start( tc(id), tchannel(id) );
    return PLATFORM_TIMER_INT_OK;
  } 
  
  final = ( ( u64 )period_us * plat_timer_get_clock( id ) ) / 1000000;
  if ( final == 0 )
    return PLATFORM_TIMER_INT_TOO_SHORT;  
  if ( final > PLATFORM_TIMER_COUNT_MAX )
    return PLATFORM_TIMER_INT_TOO_LONG;
  tc_stop( tc(id), tchannel(id) );
  tc_get_status( tc(id), tchannel(id) );    // Guessing that this clears pending interrupt stats (?) - may not need
  platform_timer_int_periodic_flag[ id ] = type;
  // FIXME: Need to handle one-shot vs. continuous (stop when reaches register C, vs reset)
  // FIXME: Should be more elegant way to handle oneshot - set CMR to reset and keep going when reach CPC
  tc_write_rc( tc(id), tchannel(id), ( u32 )final );  // TODO: Check if that should be final or final - 1
  tc_start( tc(id), tchannel(id) );
  return PLATFORM_TIMER_INT_OK;
}


// *****************
// SysTick timer - interrupt SYSTICKHZ times/second

//static volatile u64 g_ms_ticks = 0;   // Workaround in case system timer doesn't work

// interrupt handler
void SysTick_Handler(void)
{
//	g_ms_ticks++;
  cmn_virtual_timer_cb();

  // System timer handling
  cmn_systimer_periodic();
}

u64 platform_timer_sys_raw_read(void)
{
//  return g_ms_ticks;
  return SysTick->LOAD - SysTick->VAL;
}

void platform_timer_sys_disable_int(void)
{
  SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);   // based on core_cm3.h and stm32 platform.h
//	NVIC_DisableIRQ(SysTick_IRQn);
}

void platform_timer_sys_enable_int(void)
{
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
//  NVIC_EnableIRQ(SysTick_IRQn);
}

timer_data_type platform_timer_read_sys(void)
{
  return cmn_systimer_get();
}


// ****************************************************************************
// PWMs

#if NUM_PWM > 0


const uint32_t pwm_chan[] = { PWM_CHANNEL_0, PWM_CHANNEL_1, PWM_CHANNEL_2, PWM_CHANNEL_3, 
                              PWM_CHANNEL_4, PWM_CHANNEL_5, PWM_CHANNEL_6, PWM_CHANNEL_7 };

#define PWM_MAX_CLOCK CPU_FREQUENCY

// TODO: Check PWM library see if it exports a constant to replace this
#define PWM_MAX_PERIOD  ((1<<16) - 1)


// PWM clock - master, /1,2,4,8,16,32,64,128,256,512,1024
// clocka, clockb - dividers, each can divide one of above by (1 to 255)

// PWM channel prescalers
const static u32 pwm_div_ctl[] = { PWM_CMR_CPRE_MCK,  PWM_CMR_CPRE_MCK_DIV_2,
  PWM_CMR_CPRE_MCK_DIV_4,   PWM_CMR_CPRE_MCK_DIV_8,   PWM_CMR_CPRE_MCK_DIV_16,
  PWM_CMR_CPRE_MCK_DIV_32,  PWM_CMR_CPRE_MCK_DIV_64,  PWM_CMR_CPRE_MCK_DIV_128, 
  PWM_CMR_CPRE_MCK_DIV_256, PWM_CMR_CPRE_MCK_DIV_512, PWM_CMR_CPRE_MCK_DIV_1024, 
  PWM_CMR_CPRE_CLKA,        PWM_CMR_CPRE_CLKB };

// const static u8 pwm_div_data[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 };
// prescalers[11] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 };
#define pwm_prescalers( n ) ( 1 << (n) )

#define PWM_CLOCK_PRE_MAX  11
const unsigned nprescalers = PWM_CLOCK_PRE_MAX;

#define PWM_PRE_CLOCKA (nprescalers)
#define PWM_PRE_CLOCKB (PWM_PRE_CLOCKA+1)
#define PWM_PRE_UNUSED (PWM_CLOCK_PRE_MAX+2)

// FIXME: pwm_chan_clock and pwm_chan_pre may be redundant? (only need one or other)
// Clock each channel is using
u32 pwm_chan_clock[NUM_PWM] = { 0 };

// Prescaler used by each channel
u8 pwm_chan_pre[NUM_PWM] = { PWM_PRE_UNUSED };

// Use counters - how many channels using each custom clock (if no channels using, can change clock)
u8 pwm_clka_users = 0, pwm_clkb_users = 0;

// Frequency for custom channel clocks
pwm_clock_t pwm_clock = { .ul_clka = 0, .ul_clkb = 0 };


// void PWM_Handler(void)


/* Helper functions */

#define FREQ_EPSILON  100

// Scaler for relative difference (100 would be % difference)
#define PWM_PARTS_PER 1000

// TODO: Use relative difference, rather than absolute difference in clock selection
static int rel_diff(u32 f1, u32 f2)
{
  return (PWM_PARTS_PER * 2 * (f1 - f2)) / (f1 + f2);
}

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

  for (prescaler = 1; prescaler < nprescalers; prescaler++)
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

// Return the frequency of master clock divided by 2 ^ given prescaler
static u32 freq_from_prescaler( unsigned prescaler )
{
  return PWM_MAX_CLOCK >> prescaler;
}


static int pwm_enable_pin(unsigned id);

// For now - just provide a way to initialize fixed pins for PWM (where can fill in which pin to use)

u8 pwm_pin_enabled[NUM_PWM] = {false};


// set mapping PWM channel -> pin (or pin -> PWM channel)

// 8 PWMs (0-7), each have 2 outputs (H, L) [appear to be at least approximately complementary]
//   also some have one input FI (0-2)
// Output signals can be mapped to various pins
// PWMH0 - PA8(B), PB12(B), PC3(B)
// PIN_ATTR_PWM
/*  
PIO_PA5B_PWMFI0
PIO_PA3B_PWMFI1
PIO_PD6B_PWMFI2 
*/

// Alternate pin selection (FIXME: should use a general pin map at runtime)

// Define these constants to port letter of pin to be used.  
// Define to something else (e.g. nil) if don't want to use.

// TODO: Could make these selectable from the build system
//  e.g., pwm = {L0 = A, L1 = C, ...}

// Allow outside pin definition (shortcut for now - assume if define 1, then defined all)
#ifndef PWML0

// PWML0 A -> onboard LED "Tx" (low = on)
#define PWML0 'A'
#define PWML1 'C'
#define PWML2 'C'
#define PWML3 'C'
// PWML4 C -> arduino pin "PWM9"
#define PWML4 'C'
// PWML5 C -> arduino pin "PWM8"
#define PWML5 'C'
// PWML6 C -> arduino pin "PWM7"
#define PWML6 'C'
// PWML7 C -> arduino pin "PWM6"
#define PWML7 'C'

#endif

// TODO: Remove #warnings below - were just there for testing

static const sam_pin_config pwm_pins_l[] = {
#if ('A' == PWML0)
  { PIO_PA21_IDX, (PIO_PERIPH_B| PIO_DEFAULT)}, //PWML0 - onboard LED Tx (low = on?)
#elif ('B' == PWML0)
  { PIO_PB16_IDX, (PIO_PERIPH_B| PIO_DEFAULT)}, //PWML0  PIO_PB16B_PWML0
#elif ('C' == PWML0)
  { PIO_PC2_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},  //PWML0
#else
  PIN_CFG_NOPIN, 
//#warning PWML0 not used
#endif
#if ('A' == PWML1)
  { PIO_PA12_IDX, (PIO_PERIPH_B| PIO_DEFAULT)}, //PWML1
#elif ('B' == PWML1)
  { PIO_PB17_IDX, (PIO_PERIPH_B| PIO_DEFAULT)}, //PWML1
#elif ('C' == PWML1)
  { PIO_PC4_IDX,  (PIO_PERIPH_B| PIO_DEFAULT)}, //PWML1
#else
  PIN_CFG_NOPIN, 
//#warning PWML1 not used
#endif

#if ('A' == PWML2)
  { PIO_PA20_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('B' == PWML2)
  { PIO_PB18_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('C' == PWML2)
  { PIO_PC6_IDX,  (PIO_PERIPH_B| PIO_DEFAULT)},
#else
  PIN_CFG_NOPIN, 
//#warning PWML2 not used
#endif // PWML2

#if ('A' == PWML3)
  { PIO_PA0_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('B' == PWML3)
  { PIO_PB19_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('C' == PWML3)
  { PIO_PC8_IDX,  (PIO_PERIPH_B| PIO_DEFAULT)}, // PWML3
#else
  PIN_CFG_NOPIN, 
//#warning PWML3 not used
#endif // PWML3

#if ('B' == PWML4)
  { PIO_PB6_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('C' == PWML4)
  { PIO_PC21_IDX, (PIO_PERIPH_B| PIO_DEFAULT)}, // PWML4
#else
  PIN_CFG_NOPIN, 
//#warning PWML4 not used
#endif // PWML4

#if ('B' == PWML5)
  { PIO_PB7_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('C' == PWML5)
  { PIO_PC22_IDX, (PIO_PERIPH_B| PIO_DEFAULT)}, // PWML5
#else
  PIN_CFG_NOPIN, 
//#warning PWML5 not used
#endif // PWML5

#if ('B' == PWML6)
  { PIO_PB8_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('C' == PWML6)
  { PIO_PC23_IDX, (PIO_PERIPH_B| PIO_DEFAULT)}, // PWML6
#else
  PIN_CFG_NOPIN, 
//#warning PWML6 not used
#endif // PWML6

#if ('B' == PWML7)
  { PIO_PB9_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('C' == PWML7)
  { PIO_PC24_IDX, (PIO_PERIPH_B| PIO_DEFAULT)}  // PWML7
#else
  PIN_CFG_NOPIN 
//#warning PWML7 not used
#endif // PWML7
  };


#ifndef PWMH0

#define PWMH no
//#define PWMH 'C'

#define PWMH0 PWMH
#define PWMH1 PWMH
#define PWMH2 PWMH
#define PWMH3 PWMH
#define PWMH4 no
#define PWMH5 PWMH
#define PWMH6 PWMH

#endif

static const sam_pin_config pwm_pins_h[] = {
#if ('A' == PWMH0)
  { PIO_PA8_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('B' == PWMH0)
  { PIO_PB12_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('C' == PWMH0)
  { PIO_PC3_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#else
  PIN_CFG_NOPIN, 
//#warning PWMH0 not used
#endif

#if ('A' == PWMH1)
  { PIO_PA19_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('B' == PWMH1)
  { PIO_PB13_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('C' == PWMH1)
  { PIO_PC5_IDX,  (PIO_PERIPH_B| PIO_DEFAULT)},
#else
  PIN_CFG_NOPIN, 
//#warning PWMH1 not used
#endif

#if ('A' == PWMH2)
  { PIO_PA13_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('B' == PWMH2)
  { PIO_PB14_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('C' == PWMH2)
  { PIO_PC7_IDX,  (PIO_PERIPH_B| PIO_DEFAULT)},
#else
  PIN_CFG_NOPIN, 
//#warning PWMH2 not used
#endif // PWMH2

#if ('A' == PWMH3)
  { PIO_PA9_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('B' == PWMH3)
  { PIO_PB15_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#elif ('C' == PWMH3)
  { PIO_PC9_IDX,  (PIO_PERIPH_B| PIO_DEFAULT)},
#else
  PIN_CFG_NOPIN, 
//#warning PWMH3 not used
#endif // PWMH3

#if ('C' == PWMH4)
  { PIO_PC20_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#else
  PIN_CFG_NOPIN, 
//#warning PWMH4 not used
#endif // PWMH4

#if ('C' == PWMH5)
  { PIO_PC19_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#else
  PIN_CFG_NOPIN, 
//#warning PWMH5 not used
#endif // PWMH5

#if ('C' == PWMH6)
  { PIO_PC18_IDX, (PIO_PERIPH_B| PIO_DEFAULT)},
#else
  PIN_CFG_NOPIN, 
//#warning PWMH6 not used
#endif // PWMH6

  PIN_CFG_NOPIN 
  };


static int pwm_enable_pin(unsigned id)
{
  if (!pwm_pin_enabled[id]) {
		// Setup PWM for pins on this PWM
    if ( pwm_pins_l[id].pin != PIO_INVALID_IDX)
      gpio_configure_pin(pwm_pins_l[id].pin, pwm_pins_l[id].flags);
    if ( pwm_pins_h[id].pin != PIO_INVALID_IDX)
      gpio_configure_pin(pwm_pins_h[id].pin, pwm_pins_h[id].flags);
    pwm_pin_enabled[id] = true;
  }
  return 0; // Return 0 for success
}


// ********************
// PWMs provided by timers
// NUM_TIMER_PWM - number of PWMs done by timers

//#define TIMER_PWM

#ifdef TIMER_PWM

#define MAX_DUTY  100

#define INVALID_PWM NUM_PWM


// TIO pins
// FIXME: Fill in pin id and flags for TIO (NOPIN are just garbage place fillers)
// Should be in order TIO0A, TIO0B, TIO1A, TIO1B, ...
static const sam_pin_config pwm_tc_pins[]  = {
  PIN_CFG_NOPIN,
  PIN_CFG_NOPIN,
  PIN_CFG_NOPIN,
  PIN_CFG_NOPIN 
  };

// FIXME: Need pin mapping code also
  
// TODO: Flags -> bitarray?
static u8 tc_pinEnabled[ NUM_TIMER_PWM ] = { 0 };


// TODO: merge with tmode?
// Has period been set for this timer device?
static u8 tmr_per[ NUM_TIMER ] = {0};

// Is this subchannel running? FIXME: needs work on logic
static u8 tmr_running[ NUM_TIMER_PWM ] = { 0 };


/*
static u32 tc_pwm_get_clock( unsigned id );
static u32 tc_pwm_set_clock( unsigned id, u32 clock );
static u32 tc_pwm_setup( unsigned id, u32 frequency, unsigned duty );
static void tc_pwm_start( unsigned id );
static void tc_pwm_stop( unsigned id );
*/

// Timer PWM Helper functions

// Differentiate PWMs handled by pwm vs. by timer (or other mechanism)
static bool pwm_id_is_pwm( unsigned id )
{
  return id < NUM_REAL_PWM;
}

// Map PWM id to id of timer that controls that PWM
// pwm_id -> tc (0-3), tc channel (0-3), A or B
//0: TC0, Ch0, A; 1: TC0, Ch0, B; 2: TC0, Ch1, A; TC0, Ch1, B

// FIXME: Need to clarify terms - make standard term for each of these and fix code appropriately
//  timer device number (TC0, TC1, TC2) 
//  channel number (0-3 - channel handeled by the timer device)
//  pwm_id
//  subchanel (A or B) since each timer can handle 2 PWM pins

static unsigned pwm_id_to_pwm_tc( unsigned id )
{
  if (pwm_id_is_pwm(id))
    return INVALID_PWM;
  return id - NUM_REAL_PWM;
}

static inline unsigned pwm_tc_id_to_timer_id( unsigned id )
{
  return (id >> 1);
}

//  timer_id - id number for the timer (same as in timer module) (e.g. [0-9]
static unsigned pwm_id_to_timer_id( unsigned id )
{
  return pwm_tc_id_to_timer_id(pwm_id_to_pwm_tc(id));
}

// TODO: Could do this with table lookups, which faster/better space?
static unsigned pwm_tc_id_to_timer( unsigned id )
{
  return pwm_tc_id_to_timer_id(id) / channels_per_tc;
}

static unsigned pwm_tc_id_to_channel( unsigned id )
{
  return pwm_tc_id_to_timer_id(id) - (pwm_tc_id_to_timer( id ) * channels_per_tc);
}

static bool pwm_tc_id_isA( unsigned id)
{
  return !(id & 1);
}



// TODO: See how many places used, may be easy to get rid of this
static u32 tc_pwm_get_clock( unsigned id )
{
  return plat_timer_get_clock( pwm_id_to_timer_id( id ) );
}


// TODO: Add error checking on setup (so don't change timer in use for one thing to something else)
static u32 tc_pwm_set_clock( unsigned id, u32 clock )
{
  unsigned timer_id = pwm_id_to_timer_id( id );
  const u32 res = plat_timer_set_clock(timer_id, clock, 
      TC_CMR_TCCLKS_TIMER_CLOCK1 |  // FIXME: Not sure what this does
      TC_CMR_WAVE |         // Waveform mode
      TC_CMR_WAVSEL_UP_RC | // Counter running up and reset when equals to RC
      TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
      TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR | // clear TIOA on RA and RC
      TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR); // clear TIOB on RB and RC;
  
  if (res)
    tmode[timer_id] = TMODE_PWM;
  return res;
}


static void tc_SetCMR_ChannelA(Tc *tc, uint32_t chan, uint32_t v)
{
	tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xFFF0FFFF) | v;
}
// ~ (TC_CMR_ACPA_Msk | TC_CMR_ACPC_Msk)

static void tc_SetCMR_ChannelB(Tc *tc, uint32_t chan, uint32_t v)
{
	tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xF0FFFFFF) | v;
}
// ~ (TC_CMR_BCPB_Msk | TC_CMR_BCPC_Msk)


static u32 tc_pwm_setup( unsigned id, u32 frequency, unsigned duty )
{
  const u32 tcid = pwm_id_to_pwm_tc(id);  
  Tc *chTC = tc(pwm_tc_id_to_timer(tcid));
  const uint32_t chNo = pwm_tc_id_to_channel(tcid);
  u32 period;
  u32 duty_clocks;
  const unsigned timer_id = pwm_id_to_timer_id(id);
  u32 pwmclk = tc_pwm_get_clock( id );
  
  // Check that timer set up for PWM
  if (tmode[timer_id] != TMODE_PWM)
    return 0;   // FIXME: Or whatever indicates failure

  period = (pwmclk + frequency/2) / frequency;
  if (period > PLATFORM_TIMER_COUNT_MAX) period = PLATFORM_TIMER_COUNT_MAX;
  duty_clocks = (period * duty + (MAX_DUTY/2) ) / MAX_DUTY; // duty::100 as duty_clocks::period Calculate from duty cycle
  
	if (!tmr_per[timer_id]) { 
    // Init handled by pwm_set_clock
/*    tc_init(chTC, chNo,
      TC_CMR_TCCLKS_TIMER_CLOCK1 |
      TC_CMR_WAVE |         // Waveform mode
      TC_CMR_WAVSEL_UP_RC | // Counter running up and reset when equals to RC
      TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
      TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR | // clear TIOA on RA and RC
      TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR); // clear TIOB on RB and RC */
    tc_write_rc(chTC, chNo, period);

		tmr_per[timer_id] = true;   // FIXME: Not right yet - have we programmed it, then have we started it (so need more than 2 states, or maybe 2 separate flags)  Programmed, started
    // FIXME: also need way to clear this (e.g. clear it when change to using as timer)
	}
  else 
    // Check period against frequency currently using, if set to different freq - have a problem
    // Actually this isn't right - need to check that the other channel is in use
    // Or maybe just let them run one PWM per TC (and pick which pin they want A or B)
    if (period != tc_read_rc(chTC, chNo))
      return 0;   // FIXME: Or whatever indicates failure

  if (duty_clocks == 0) {
    if (pwm_tc_id_isA(tcid))
      tc_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR);  // RA -> clear TIOA, RC -> clear TIOA
    else
      tc_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
  } else {
    if (pwm_tc_id_isA(tcid)) {
      tc_write_ra(chTC, chNo, duty_clocks);
      tc_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);  // RA -> clear TIOA, RC -> set TIOA
    } else {
      tc_write_rb(chTC, chNo, duty_clocks);
      tc_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET);
    }
  }
  return ( pwmclk + period/2) / period;
}


static void tc_pwm_start( unsigned id )
{
  u32 tcid = pwm_id_to_pwm_tc(id);
  const unsigned timer_id = pwm_id_to_timer_id(id);
  
  if (!tc_pinEnabled[tcid]) {
    if ( pwm_tc_pins[tcid].pin != PIO_INVALID_IDX)
      gpio_configure_pin(pwm_tc_pins[tcid].pin, pwm_tc_pins[tcid].flags);
    tc_pinEnabled[tcid] = 1;
  };

  if (!tmr_running[timer_id]) {
    tc_start(tc(timer_id), pwm_tc_id_to_channel(tcid));
    tmr_running[timer_id] = 1;  // FIXME: Running logic not right - need timer running, and maybe channel running
  };
}


static void tc_pwm_stop( unsigned id )
{
  u32 tcid = pwm_id_to_pwm_tc(id);
  const unsigned timer_id = pwm_id_to_timer_id(id);

  // FIXME: This doesn't quite work - since the other pin in the channel could still be in use
  // Really what need to do is check if other pin in use, 
  //    if so, then disable this pin (disconnect output from timer, or reprogram register, or ??)
  //    if not, then stop the channel - and release it back to being a timer channel again
  if (tmr_running[timer_id]) {
    tc_stop(tc(timer_id), pwm_tc_id_to_channel(tcid));
    tmr_running[timer_id] = 0;
  };
}

#else

static bool pwm_id_is_pwm( unsigned id )
{
  return true;
}

#endif // TIMER_PWM


// Platform functions

// Assumes that timers enabled by timer section
static void pwms_init( void )
{
  pmc_enable_periph_clk(ID_PWM);
}


#define GET_FIELD(value, mask, pos) ((mask & (value)) >> pos)

//#define PWM_CLK_DIVA_GET(value) ((PWM_CLK_DIVA_Msk & (value)) >> PWM_CLK_DIVA_Pos)))
#define PWM_CLK_DIVA_GET(value) GET_FIELD(value, PWM_CLK_DIVA_Msk, PWM_CLK_DIVA_Pos)
#define PWM_CLK_DIVB_GET(value) GET_FIELD(value, PWM_CLK_DIVB_Msk, PWM_CLK_DIVB_Pos)
#define PWM_CLK_PREA_GET(value) GET_FIELD(value, PWM_CLK_PREA_Msk, PWM_CLK_PREA_Pos)
#define PWM_CLK_PREB_GET(value) GET_FIELD(value, PWM_CLK_PREB_Msk, PWM_CLK_PREB_Pos)


// pwm.c does not provide interface to fetch clock, so use low level component_pwm macros
u32 platform_pwm_get_clock( unsigned id )
{
	u32 pre;
  u32 div;
  u32 res = 0;

  if (pwm_id_is_pwm(id)) {
    pre = PWM->PWM_CH_NUM[id].PWM_CMR & PWM_CMR_CPRE_Msk;  // Channel clock prescaler
  
//  printf("PWM get clock: channel_pre = %lu", pre);
  
    switch (pre)
    {
      case PWM_CMR_CPRE_CLKA:
        div = PWM_CLK_DIVA_GET(PWM->PWM_CLK);
//       printf(" clocka div = %lu", div);
        if (div)  // if div=0 then clock not set, so return 0
          {
          pre = PWM_CLK_PREA_GET(PWM->PWM_CLK);
//          printf(" clocka pre = %lu\n", pre);
          if (pre < PWM_CLOCK_PRE_MAX)
            res = freq_from_prescaler(pre)/ div;
//        res = (PWM_MAX_CLOCK / pwm_prescalers( pre ))/ div;
          Assert(freq_from_prescaler(pre) == (PWM_MAX_CLOCK / pwm_prescalers( pre )));
          // FIXME: Just for testing to check math
          }
        break;
      case PWM_CMR_CPRE_CLKB:
        div = PWM_CLK_DIVB_GET(PWM->PWM_CLK);
        if (div)  // if div=0 then clock not set, so return 0
          {
          pre = PWM_CLK_PREB_GET(PWM->PWM_CLK);
          if (pre < PWM_CLOCK_PRE_MAX)
            res = freq_from_prescaler(pre)/ div;
          }
        break;
      default:
        if (pre < PWM_CLOCK_PRE_MAX)
          res = freq_from_prescaler(pre) ;  // divisor of MCK
    }
  } else {
#ifdef TIMER_PWM
    return tc_pwm_get_clock(id);
#endif  
  }
  return res;
}


/* 
 TODO: PWM Clock allocation -
 If ask for a clock already available (one of prescaller dividers, or matches clocka or b) - then give them that
 Otherwise either allocate custom clock or give them a prescaller clock
   Greedy - Give custom clock if one available
  Keep track of what channels using what clock
    If no channels using a custom clock, then consider it available for (re)use
    Use count

  Problem - eLua won't let me feed it a 0 frequency
  
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

// Set the PWM clock
u32 platform_pwm_set_clock( unsigned id, u32 clock )
{
  platform_pwm_stop(id);
  if (pwm_id_is_pwm(id)) {
    if (clock)
    {
      // FIXME: Should disable any active PWM channels before setting clock
      // FIXME: Allow use clockb and fixed divisors
      //  e.g. keep track of clock value settings (and accept 2 different values, plus using divisions of mck)
      pwm_clock.ul_clka = clock;
      pwm_clock.ul_clkb = 0;          // FIXME: Should already be 0, but just to be sure
      pwm_clock.ul_mck = CPU_FREQUENCY;
      if(pwm_init(PWM, &pwm_clock))
        return 0;
      // FIXME: Need to find out what clock actually set

      if (pwm_chan_clock[id] == 0) pwm_clka_users++;        // Increment use count if wasn't already using
      
      // FIXME: Returns wrong value 
      // *** Problem is it doesn't assign a clock to channel until init channel, 
      // so although clocka is set, we are reading giberish.
      // (So why is it returning system clock value?)
      
      pwm_chan_clock[id] = platform_pwm_get_clock(id);
      pwm_chan_pre[id] = PWM_PRE_CLOCKA;
    }
    else
      if (pwm_chan_clock[id])           // Clearing clock on channel that was in use
      {
      pwm_chan_clock[id] = 0;
      pwm_clka_users--;         // FIXME: Should reduce count for a or b depending which was in use
      }
    Assert(pwm_clka_users < NUM_PWM);
  }
#ifdef TIMER_PWM
  else
    return tc_pwm_set_clock(id, clock);
#endif
  return pwm_chan_clock[id];
//  return clock; // FIXME - what is it supposed to return?
}



static pwm_channel_t pwm_inst;

// TODO: Check how other platforms work, should there be way to change duty cycle without having to restart?


/*
 * Configure a PWM channel to run at "frequency" Hz with a duty cycle of
 * "duty" (0-100).  0 means low all the time, 100 high all the time.
 * Return actual frequency set.
 */
//duty - PWM channel duty cycle, specified as integer from 0 to 100 (percent).
u32 platform_pwm_setup( unsigned id, u32 frequency, unsigned duty )
{
//  pwm_channel_t pwm_inst;
  u32 pwmclk;        // base clock frequency for PWM counter
  u32 period;        // number of base clocks per cycle
  u32 duty_clocks;    // number of base clocks to be high (low?) for
  
  if (id >= NUM_PWM || duty > 100 || pwm_chan_clock[id] == 0) return 0;

  if (pwm_id_is_pwm(id)) {
    //Had tried enabling pin after channel init, but some examples showed it before, so trying it here
    if (pwm_enable_pin(id))
      return 0; // Error

    pwmclk = platform_pwm_get_clock( id );
    
    // Compute period and duty period in clock cycles.
    //
    // PWM output wave frequency is requested in Hz, but programmed as a
    // number of cycles of the master PWM clock frequency.
    //
    // Here, we use rounding to select the numerically closest available
    // frequency and return the closest integer in Hz to that.

    // FIXME: from a different platform, check whether for center or left alligned
    period = (pwmclk + frequency/2) / frequency;
    if (period == 0) period = 1;  
    if (period > PWM_MAX_PERIOD) period = PWM_MAX_PERIOD;
    duty_clocks = (period * duty + 50) / 100;

    // For debugging
    //printf("PWM Setup: period = %u, duty clocks = %u\n", period, duty_clocks);

    // Put known value in all the other fields that example did not cover.
    // Or tell it to start whole structure as 0
    // FIXME: Find out right values for all of these
    pwm_inst.b_deadtime_generator = 0;
    pwm_inst.b_pwmh_output_inverted = 0;
    pwm_inst.b_pwml_output_inverted = 0;
    pwm_inst.us_deadtime_pwml = 0;
    pwm_inst.us_deadtime_pwmh = 0;
    pwm_inst.output_selection.b_override_pwmh = 0;
    pwm_inst.output_selection.b_override_pwml = 0;
    pwm_inst.output_selection.override_level_pwmh = 0;
    pwm_inst.output_selection.override_level_pwml = 0;
    pwm_inst.b_sync_ch  = 0;
    pwm_inst.ul_fault_output_pwmh = 0;
    pwm_inst.ul_fault_output_pwml = 0;
    pwm_inst.fault_id = 0;

    // Fields actually covered by examples
    pwm_channel_disable(PWM, pwm_chan[id]);  
    pwm_inst.ul_prescaler = PWM_CMR_CPRE_CLKA;    // FIXME: Should be whichever clock/prescaler selected
    pwm_inst.channel = pwm_chan[id];
    pwm_inst.ul_period = period;
    pwm_inst.ul_duty = duty_clocks;
    pwm_inst.alignment = PWM_ALIGN_LEFT;
    pwm_inst.polarity = PWM_LOW;                // Start low
    if(pwm_channel_init(PWM, &pwm_inst))
      return 0; // Error
    return (pwmclk + period/2) / period;
  }
#ifdef TIMER_PWM
  else
    return tc_pwm_setup(id, frequency, duty);
#endif
  return 0; // Error
}
//  TODO: Other bits from example code - consider
// PWM_CMR_CPRE_CLKB


// TODO: Should there be way to test channel status?
//uint32_t bitmask = pwm_channel_get_status(PWM)


void platform_pwm_start( unsigned id )
{
//  if ( pwm_chan_clock[id] )
  if (pwm_id_is_pwm(id)) 
    pwm_channel_enable(PWM, pwm_chan[id]);
#ifdef TIMER_PWM
  else
    tc_pwm_start(id);
#endif
}


void platform_pwm_stop( unsigned id )
{
  if (pwm_id_is_pwm(id))
    pwm_channel_disable(PWM, pwm_chan[id]);
#ifdef TIMER_PWM
  else
    tc_pwm_stop( id );
#endif
}

#undef pwm_prescalers

#endif // NUM_PWM > 0


// *****************************************************************************
// ADC specific functions and variables

#ifdef BUILD_ADC

// 16 ADC channels, plus Temperature


// Pins

//#define PIO_PA3X1_WKUP1      (1u << 3)  /**< \brief Adc signal: AD1/WKUP1 */
//#define PIO_PB21X1_WKUP13    (1u << 21) /**< \brief Adc signal: AD14/WKUP13 */
//#define PIO_PA11B_ADTRG      (1u << 11) /**< \brief Adc signal: ADTRG */

/*  PIOs mapping handled automatically
const struct sam_pin_config adc_pins[] = {
  {PIO_PA2_IDX, (PIO_DEFAULT)}   // AD0
  {PIO_PA3_IDX, (PIO_DEFAULT)}   // AD1
  {PIO_PA4_IDX, (PIO_DEFAULT)}   // AD2
  {PIO_PA6_IDX, (PIO_DEFAULT)}   // AD3
  {PIO_PA22_IDX, (PIO_DEFAULT)}   // AD4
  {PIO_PA23_IDX, (PIO_DEFAULT)}   // AD5
  {PIO_PA24_IDX, (PIO_DEFAULT)}   // AD6
  {PIO_PA16_IDX, (PIO_DEFAULT)}   // AD7
  {PIO_PB12_IDX, (PIO_DEFAULT)}   // AD8
  {PIO_PB13_IDX, (PIO_DEFAULT)}   // AD9
  {PIO_PB17_IDX, (PIO_DEFAULT)}   // AD10
  {PIO_PB18_IDX, (PIO_DEFAULT)}   // AD11
  {PIO_PB19_IDX, (PIO_DEFAULT)}   // AD12
  {PIO_PB20_IDX, (PIO_DEFAULT)}   // AD13
  {PIO_PB21_IDX, (PIO_DEFAULT)}   // AD14
  };
*/

// FIXME: fix ADC_TIMER_IDs (they are arbitrarily set for now)
#define ADC_TIMER_FIRST_ID 8
#define ADC_NUM_TIMERS 1

/** Reference voltage for ADC, in mv. */
#define VOLT_REF        (3300)

// Could be useful const (either this or # bits)
#define MAX_DIGITAL     (4095)


// TODO: See what did with LM4F on temperature sensor

// Temperature sensor - channel 15, 0 to 3.3 v, hence ADVREF must be set to 3300mv
// TSON bit in ADC_ACR
//		f_temp = (float)(l_vol - 800) * 0.37736 + 27.0;

// Constant: THERMOMETER
#define ADC_THERM_ID  ADC_TEMPERATURE_SENSOR


/*
static enum adc_channel_num_t adc_channel(unsigned id)
{
  return id;
}
*/

int platform_adc_check_timer_id( unsigned id, unsigned adc_timer_id )
{
  return ( ( adc_timer_id >= ADC_TIMER_FIRST_ID ) && ( adc_timer_id < ( ADC_TIMER_FIRST_ID + ADC_NUM_TIMERS ) ) );
}


// Given ADC reading taged with channel number - get just the channel number
static inline u8 chan_num(u32 adc_val)
{
  return ((adc_val & ADC_LCDR_CHNB_Msk) >> ADC_LCDR_CHNB_Pos);
}


// FIXME: Incomplete

// Handle ADC interrupts
void ADC_Handler( void )
{
  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  elua_adc_ch_state *s = d->ch_state[ d->seq_ctr ]; // FIXME: This is almost shurely wrong

	uint32_t i;
	uint32_t adctemp;
	uint8_t ch_num;

  if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY) {
		adctemp = adc_get_latest_value(ADC);
    ch_num = chan_num(adctemp);
  
//  for (d->seq_ctr = 0; d->seq_ctr < d->seq_len ; d->seq_ctr++ )
//   s = d->ch_state[ d->seq_ctr ];
//    get fresh data for s, or if data available for s

    // Need to fetch data from any channels 
UNUSED(i);
UNUSED(ch_num);
  
/*
		for (i = 0; i < NUM_CHANNELS; i++) {
      if (g_adc_sample_data.uc_ch_num[i] == ch_num) {
				g_adc_sample_data.us_value[i] = (temp & ADC_LCDR_LDATA_Msk);
				g_adc_sample_data.us_done |= 1 << i;
			}
		}
*/

    s->value_fresh = 1;

    if ( s->logsmoothlen > 0 && s->smooth_ready == 0)
      adc_smooth_data( s->id );
#if defined( BUF_ENABLE_ADC )
    else if ( s->reqsamples > 1 )
    {
      buf_write( BUF_ID_ADC, s->id, ( t_buf_data* )s->value_ptr );
      s->value_fresh = 0;
    }
#endif

    // If we have the number of requested samples, stop sampling
    if ( adc_samples_available( s->id ) >= s->reqsamples && s->freerunning == 0 )
      platform_adc_stop( s->id );
      
  };

  // Only attempt to refresh sequence order if still running
  // This allows us to "cache" an old sequence if all channels
  // finish at the same time
  if ( d->running == 1 )
    adc_update_dev_sequence( 0 );

  if ( d->clocked == 0 && d->running == 1 )
  {
    // Need to manually fire off sample request in single sample mode
//    MAP_ADCProcessorTrigger( ADC_BASE, d->seq_id );
  }    
}
//ADC_ISR_DRDY
//ADC_ISR_RXBUFF

//			while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY);
//			ulValue = adc_get_latest_value(ADC);
 
 
// 		adc_read_buffer(ADC, gs_s_adc_values, BUFFER_SIZE)
//  	adc_enable_interrupt(ADC, ADC_ISR_RXBUFF);



static void adcs_init( void )
{
  unsigned id;
//  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  
	pmc_enable_periph_clk(ID_ADC);

#ifdef ADC_TEMP  
  // TODO: constant for ADC channel #
  
	/* Enable channel for temperature sensor. */
	adc_enable_channel(ADC, ADC_TEMPERATURE_SENSOR);

	/* Enable the temperature sensor. */
	adc_enable_ts(ADC);
#endif //ADC_TEMP

//  adc_enable_channel( ADC, ulChannel );

  for( id = 0; id < NUM_ADC; id ++ )
    adc_init_ch_state( id );

// Should set ADC int priority also

  /* Enable Data ready interrupt. */
  adc_enable_interrupt(ADC, ADC_IER_DRDY);

  /* Enable ADC interrupt. */
  NVIC_EnableIRQ(ADC_IRQn);

  // Perform sequencer setup
  platform_adc_set_clock( 0, 0 );
}


	/*
	 * Formula: ADCClock = MCK / ( (PRESCAL+1) * 2 )  [PRESCAL 0 to 255 ]
	 * For example, MCK = 64MHZ, PRESCAL = 4, then:
	 *     ADCClock = 64 / ((4+1) * 2) = 6.4MHz;
 	 */

   /*
    * MCK = 80MHZ, prescal = 4 -> ADC clock = 8 MHz
    * Settling time = 3 / 8 MHz = 375 ns (Min is 200 ns) 
    */
   /*
    *   [settling 0 = 3/  ADC clock (15 MHz or less), 
    *   [settling 1 = 5/  ADC clock (25 MHz or less),
    *   [settling 2 = 9/  ADC clock (45 MHz or less), // So this should work with any prescaler on 80Mhz part
    *   [settling 3 = 17/ ADC clock (85 Mhz or less)
    *   ADC clock limit = n/200 ns -> ADC clock * 200 ns = n, then look for next larger n
    *  settling[] = {3, 5, 9, 17}
    *  setl = ADC clock / 5000000; // (same as * 200 ns)
    *  look up setl in settling, return the index in settling (0, 1, 2, 3)
    */
   // Sampling freq 0.05 to 1 MHz
   // t Startup (Off to normal 30 us typ, 40 us max; Standby to normal 8 us typ, 12 us max)
   //   startup value = ADCClock * 40 us (for off, or 12 us for standby)
   // track time - min 160 ns
   //  TRACKTIM = ( 160 ns * ADCClock ) - 1
   // t convert typ 20 T CP_ADC
   // t settle - min 200 ns
// What limit on transfer time?

// Minimum tracking time - 160 ns
#define MIN_TRACK  (0.000000160)
#define MIN_SETTLE (0.000000200)

// FIXME: 40 microseconds (may actually only need 12 us)
#define MAX_STARTUP (0.000040)


	/* Set ADC clock. */
	/* Formula:
	 *     Startup  Time = startup value / ADCClock
	 *     Transfer Time = (TRANSFER * 2 + 3) / ADCClock
	 *     Tracking Time = (TRACKTIM + 1) / ADCClock
	 *     Settling Time = settling value / ADCClock
	 * For example, ADC clock = 6MHz (166.7 ns)
	 *     Startup time = 512 / 6MHz = 85.3 us
	 *     Transfer Time = (1 * 2 + 3) / 6MHz = 833.3 ns
	 *     Tracking Time = (0 + 1) / 6MHz = 166.7 ns
	 *     Settling Time = 3 / 6MHz = 500 ns
	 */
	/* Initialize ADC. */
	/*  startup = 8:    512 periods of ADCClock
	 * for prescale = 4
	 *     prescale: ADCClock = MCK / ( (PRESCAL+1) * 2 ) => 64MHz / ((4+1)*2) = 6.4MHz
	 *     ADC clock = 6.4 MHz
	 */

u32 platform_adc_set_clock( unsigned id, u32 frequency )
{
  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  u32 tracktime;
  
  if ( frequency > 0 )
  {
    d->clocked = 1;
    u8 pre = (CPU_FREQUENCY/frequency / 2) - 1;
    u32 adcClock = CPU_FREQUENCY / ((pre + 1) * 2);
    uint8_t startup = MAX_STARTUP * adcClock;

    //  adc_init(ADC, CPU_FREQUENCY, 6400000, 8 );
    adc_init(ADC, CPU_FREQUENCY, frequency, startup );

    tracktime = ( MIN_TRACK * adcClock ) - 1;
    
    adc_configure_timing(ADC, tracktime, ADC_SETTLING_TIME_2, 1); // track time, settle time, transfer time
    // Examp had settling time 3 (not sure why)
    // Why were tracking time 0, transfer time 1 in example?

//	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);  // ADC_TRIG_TIO_CH_0-2

  // Checks ADC settings, prints error messages if problems
    adc_check(ADC, CPU_FREQUENCY);
    
    adc_configure_trigger(ADC, ADC_TRIG_SW, 0);	/* Disable hardware trigger. */ 

    frequency = adc_get_actual_adc_clock(ADC, CPU_FREQUENCY);
  }
  else
  {
    d->clocked = 0;
    // Conversion will run back-to-back until required samples are acquired
    adc_configure_trigger(ADC, ADC_TRIG_SW, 1);
  }
  
	/* Enable channel number tag. */
	adc_enable_tag(ADC);

  return frequency;
}


enum adc_channel_num_t ch_list[NUM_ADC] = { 0 };


int platform_adc_update_sequence( void )
{
  elua_adc_dev_state *d = adc_get_dev_state( 0 );

  // For each channel in sequence
  for (d->seq_ctr = 0; d->seq_ctr < d->seq_len-1 ; d->seq_ctr++)
  {
    adc_enable_channel( ADC, d->ch_state[ d->seq_ctr ]->id );
    // Pin config happens automatically
    ch_list[d->seq_ctr] = d->ch_state[ d->seq_ctr ]->id;
  }

  /* Set user defined channel sequence. */
  adc_configure_sequence(ADC, ch_list,  d->seq_len);
  
  d->seq_ctr = 0;
  return PLATFORM_OK;
}


int platform_adc_start_sequence( void )
{
  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  
  if( d->running != 1 ) // If already running, changes will be picked up at interrupt
  {
    adc_update_dev_sequence( 0 );

    d->running = 1;
    if( d->clocked == 1 )
    {
//      MAP_TimerControlTrigger(timer_base[d->timer_id], TIMER_A, true);
//      MAP_TimerEnable(timer_base[d->timer_id], TIMER_A);
    }
    else
    {
//      MAP_ADCProcessorTrigger( ADC_BASE, d->seq_id );
        adc_configure_trigger(ADC, ADC_TRIG_SW, 1);  // Free running
        adc_start(ADC); 	/* Start conversion. */
    }

// FIXME: Not sure where rest of this should go
    /* Enable sequencer. */
    adc_start_sequencer(ADC);
  }
  
  return PLATFORM_OK;
}

void platform_adc_stop( unsigned id )
{
  elua_adc_ch_state *s = adc_get_ch_state( id );
  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  
  s->op_pending = 0;
  INACTIVATE_CHANNEL(d, id);
  
  // If there are no more active channels, stop the sequencer
  if( d->ch_active == 0 )
  {
    adc_stop_sequencer(ADC);
    d->running = 0;
  }

//	adc_disable_channel(ADC, adc_channel(id));
//  adc_stop(ADC);
}

#endif // ifdef BUILD_ADC


// ****************************************************************************
// Support for specific onboard devices 

// ****************************************************************************
// Ethernet functions
#ifdef BUILD_UIP

#warning Ethernet - platform driver not written.

#endif // BUILD_UIP


// ****************************************************************************
// USB functions

// TODO: Clean up (remove extra functions)

// TODO: see if can get CDC_STDIO working (works with USB CDC)
//   CDC_STDIO - get unrecognized device, with VID and PID 0

#if defined( BUILD_USB_CDC )

static void usb_init( void )
{
// From stdio_usb example
  	// Initialize interrupt vector table support.
	irq_initialize_vectors();

	// Enable interrupts
	cpu_irq_enable();

#ifdef USB_CDC_STDIO  
  stdio_usb_init();
#else  
	// Initialize the sleep manager
//	sleepmgr_init();
  
  udc_start();
// * Add to the main IDLE loop:
// *     sleepmgr_enter_sleep(); // Optional

#endif  
}

#ifdef USB_CDC_STDIO

#warning building USB_CDC_STDIO - this does not work yet

void platform_usb_cdc_send( u8 data )
{
//  putchar(data);
  printf("%c", data);   // FIXME: Probably simpler function to use - copied this from example
}

int platform_usb_cdc_recv( timer_data_type timeout )
{
  char ch;
  
//  return getchar();
  scanf("%c",&ch);    // FIXME: Probably simpler function to use - copied this from example
  return ch;
}

#else


static volatile bool platform_cdc_enable_flag = false;

  
void platform_usb_cdc_send( u8 data )
{
  if (platform_cdc_enable_flag)
  {
    if (!udi_cdc_is_tx_ready()) {
			// Fifo full
			udi_cdc_signal_overrun();
		} else {
      udi_cdc_putc(data);
    };
  };
}


int platform_usb_cdc_recv( timer_data_type timeout )
{
  if (platform_cdc_enable_flag){
    if (0 == timeout)
      if (udi_cdc_is_rx_ready())  // No waiting - so get character if available
        return udi_cdc_getc();   
      else
        return -1;      // No character available, so return immediately (FIXME: Check return value)
    else
      return udi_cdc_getc();    // FIXME: Need to implement a timeout on this wait
  } else 
    return -1;
}


// TODO: Most of these callbacks are not needed - copied from example, but do nothing, could get rid of
void platform_suspend_action(void)
{
//	ui_powerdown();
}

void platform_resume_action(void)
{
//	ui_wakeup();
}

void platform_sof_action(void)
{
	if (!platform_cdc_enable_flag)
		return;
//	ui_process(udd_get_frame_number());
}

bool platform_cdc_enable(void)
{
	platform_cdc_enable_flag = true;
//	platform_port_open = 0;
	return true;
}

void platform_cdc_disable(void)
{
	platform_cdc_enable_flag = false;
}

void platform_cdc_set_dtr(bool enable)
{
	if ( enable ) {
		// Host terminal has open COM
//		platform_port_open |= 1 << port;
//		ui_com_open(port);
	}else{
		// Host terminal has close COM
//		platform_port_open &= ~(1 << port);
//		ui_com_close(port);
	}
}


void platform_cdc_rx_notify(void)
{
// TODO: Could do some buffering, or something
}


#endif // USB_CDC_STDIO

#else

// Callbacks needed even when don't use USB - just to make the library happy
// TODO: Could remove if build system would not build the USB library when not asked for

bool platform_cdc_enable(void)
{
  return true;
}

void platform_cdc_disable(void)
{
}

void platform_cdc_rx_notify(void)
{
}

#endif // BUILD_USB_CDC


// ****************************************************************************
// Flash access functions

#ifdef BUILD_WOFS

#warning WOFS not tested

// INTERNAL_FLASH_START_ADDRESS, INTERNAL_FLASH_SIZE
#ifdef IFLASH0_SIZE
#define IFLASH0_END (IFLASH0_ADDR + IFLASH0_SIZE)
#endif

#ifdef IFLASH1_SIZE
#define IFLASH1_END (IFLASH1_ADDR + IFLASH1_SIZE)
#endif

// Wait cycles copied from example
#define FLASH_WAIT_CYCLES 6

// FIXME: Should use an enum, or some other way to designate multiple flash banks
#define IFLASH0 1
#define IFLASH1 2

/* Unlock entire flash
	ul_error = flash_unlock(IFLASH_ADDR, (IFLASH_ADDR + IFLASH_SIZE - 1), 0, 0);
	if (FLASH_RC_OK != ul_error) {
		puts("Unlock internal flash failed.\r\n");
		return 0;
	}
  */
  
int platform_flash_init( void )
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

 if ( flash_bank(toaddr) != flash_bank(toaddr + size))
    return PLATFORM_ERR;  // FIXME: Really should do in 2 parts
	result = flash_write(toaddr, from, size, 1);  // Last arg - 1 if erase first
  return (result == FLASH_RC_OK) ? PLATFORM_OK : PLATFORM_ERR;
}

int platform_flash_erase_sector( u32 sector_id )
{
//  return (flash_erase_sector(sector_id) == FLASH_RC_OK) ? PLATFORM_OK : PLATFORM_ERR;
// FIXME: ?? Thought from example that above would be code, but not defined for SAM3X??
  return PLATFORM_ERR;
}

#endif // #ifdef BUILD_WOFS


// ****************************************************************************
// Platform specific shell command(s)
#if defined(BUILD_SHELL_REFLASH)

#include "shell.h"


static void restart_bootloader(void)
{
  flash_clear_gpnvm(1);   // Next time boot from ROM

  // reset processor and periphirals (register address from cmsis)
  asm ("dsb"); // Ensure completion of memory access
  // reboot - set key, reset perrst (peripherals) and procrst (processor)
  REG_RSTC_CR = RSTC_CR_KEY(0xa5) | RSTC_CR_PERRST | RSTC_CR_PROCRST;
  asm ("dsb"); // Ensure completion of memory access
  while (1);  // Wait for reset

  //  NVIC_SystemReset();     // This way of doing reset did not work (to make it boot from ROM)
}


const char shell_help_reflash[] = "\n"
  "Restart to the bootloader, ready to flash new program.\n";
const char shell_help_summary_reflash[] = "back to bootloader";

void platform_shell_reflash( int argc, char **argv )
{
  if( argc != 1 )
  {
    SHELL_SHOW_HELP( reflash );
    return;
  }
  printf( "Reflash - restarting from ROM bootloader\n" );

  restart_bootloader();
}


#endif


// ****************************************************************************
// Platform specific modules go here


// ****************************************************************************
// Random sequence generator

#ifdef BUILD_RAND

#include "module_rand.h"

// TODO: Add buffer (so can generate ahead of need)
// TODO: Add interface to check if number available

// Indicate if need to arrange to call rand_init - maybe should just call in system init(?)
static u8 f_rand_init = false;

#ifdef RAND_BUF

#warning Rand buffer not finished

//static u8 buf_space = true;
//static u32 rand_buffer;

static fifo_desc_t rand_fifo;

#define RAND_BUF_SIZE 16

static u8 rand_buf[RAND_BUF_SIZE];

void TRNG_Handler(void)
{
	u32 status;
  u32 rand; // make this a union (u32 and array of u8)

  status = trng_get_interrupt_status(TRNG);
  
  if ((status & TRNG_ISR_DATRDY) == TRNG_ISR_DATRDY) {
    if (fifo_is_full(&rand_fifo))
      disable TRNG interrupt
    else {
      rand = trng_read_output_data(TRNG)
      for (i = 0; i < 4; i++)
        if (fifo_push_uint8(&rand_fifo, rand[i]) == FIFO_ERROR_OVERFLOW){
          disable TRNG int
          break;
        }
    }
  }
  
/*  if (buf_space)
  {
    if ((status & TRNG_ISR_DATRDY) == TRNG_ISR_DATRDY) {
      buf_space = false;
      rand_buffer = trng_read_output_data(TRNG);
    }
  }
 */
  
}
#endif


u8 platform_rand_init( void )
{
  if ( !f_rand_init ) {
    pmc_enable_periph_clk(ID_TRNG);
    trng_enable(TRNG);

	/* Enable TRNG interrupt */
  /*
	NVIC_DisableIRQ(TRNG_IRQn);
	NVIC_ClearPendingIRQ(TRNG_IRQn);
	NVIC_SetPriority(TRNG_IRQn, RAND_INT_PRI);
	NVIC_EnableIRQ(TRNG_IRQn);
	trng_enable_interrupt(TRNG);
 */
#ifdef RAND_BUF
    if (fifo_init(&rand_fifo, rand_buf, RAND_BUF_SIZE) == FIFO_ERROR)
      return FALSE;
#endif
    f_rand_init = true;    
  }
  return TRUE;
}

// TODO: Fetch given number bytes from random sequence next(n) 
u32 platform_rand_next( void )
{
  if ( !f_rand_init )
    platform_rand_init();

  WAIT_WHILE(!trng_get_interrupt_status(TRNG));
  return trng_read_output_data(TRNG);
}


#endif // BUILD_RAND
