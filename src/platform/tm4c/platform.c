// eLua - Platform-dependent functions
// TM4C

#include "platform.h"
#include "type.h"
#include "devman.h"
#include "genstd.h"
#include <reent.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "uip_arp.h"
#include "elua_uip.h"
#include "elua_adc.h"
#include "uip-conf.h"
#include "platform_conf.h"
#include "common.h"
#include "math.h"
#include "diskio.h"
#include "lua.h"
#include "lauxlib.h"
#include "lrotable.h"
#include "elua_int.h" 

// Platform specific includes

// PART_ macro MUST be defined BEFORE including pin_map.

#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"

#ifdef BUILD_UIP
#include "driverlib/emac.h"
#include "inc/hw_emac.h"

#include "inc/hw_types.h"
#include "elua_net.h"
#include "dhcpc.h"
#endif

#include "driverlib/systick.h"
#include "driverlib/flash.h"
#include "buf.h"

#include "utils.h"

// FIXME: These may be redundant since putting includes in cpu files
#if defined( FORTM4C1233 )
  #include "tm4c1233h6pm.h"
#elif defined( FORTM4C123G )
  #include "tm4c123gh6pm.h"
#elif defined( FORTM4C1294 )
  #include "tm4c1294ncpdt.h"
#else
#warning Unknown CPU  
#endif


#include "driverlib/rom.h"
#include "driverlib/rom_map.h"


// USB CDC
#if defined( BUILD_USB_CDC )
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"
#endif

// UIP sys tick data
// NOTE: when using virtual timers, SYSTICKHZ and VTMR_FREQ_HZ should have the
// same value, as they're served by the same timer (the systick)
#define SYSTICKHZ               5
#define SYSTICKMS               (1000 / SYSTICKHZ)


// Arrays of structs might be clearer and less error prone (keep all information about a port or peripheral together)
// typedef struct { u32 base, sysctl; }  peripheral; // consider naming - is this clear name?
// typedef struct { u32 base; u8 pins; } gpio_pins;


// ****************************************************************************
// Platform initialization

// forward
static void timers_init();
static void uarts_init();

#if NUM_SPI > 0
static void spis_init();
#endif

static void pios_init();

#if NUM_PWM > 0
// #ifdef BUILD_PWM
static void pwms_init();
#endif

#ifdef BUILD_UIP
static void eth_init();
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

#ifdef BUILD_COMP
static void comps_init();
#endif

#ifdef BUILD_I2C
static void i2cs_init();
#endif


unsigned long clockfreq;

#ifdef DEBUG  
BOOL consoleinited = false;
#endif

int platform_init()
{
  // Set the clocking to run from PLL

// ToDo: Maybe define crystal/divider in CPU header files - make easier to run at lower clock or use diff crystal
// #define CPU_CLOCK	SYSCTL_SYSDIV_2_5 | SYSCTL_XTAL_16MHZ
//   or
// #define CPU_CLOCK	SYSCTL_SYSDIV_4 | SYSCTL_XTAL_8MHZ
//

#if defined ( FORTM4C1294 )

#ifdef CPU_MAX_FREQUENCY
#if ( ELUA_BOARD_CPU_CLOCK_HZ > CPU_MAX_FREQUENCY )
#warning CPU clock too fast
#endif

#ifndef ELUA_BOARD_CPU_CLOCK_HZ
#define ELUA_BOARD_CPU_CLOCK_HZ CPU_MAX_FREQUENCY
#endif
#endif // CPU_MAX_FREQUENCY

#ifdef BUILD_UIP
  // Enable main oscillator - required for ethernet PHY
  MAP_SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
#endif

  clockfreq = MAP_SysCtlClockFreqSet(( SYSCTL_CFG_VCO_480 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_25MHZ), 
    ELUA_BOARD_CPU_CLOCK_HZ);
    
  if (clockfreq == 0)
    return PLATFORM_ERR;
// Fixme: This check may belong at end (to get uart set up so can give error message).
// TODO: Could add more checking on clock return value

#elif defined ( FORLM4F )
//  MAP_SysCtlClockSet(SYSCTL_SYSDIV_3 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // 66MHz clock
//  MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // 50MHz clock

// !!!! FIXME: So how do I set the system clock to 80 MHz?
// The following works in Stellarisware (sets the clock to 80MHz)
// But in Tivaware I get problems with Uart speed (serial output on uart0/debug port comes up gibberish).
// Note there are some changes in sysctl.c - SysCtlClockSet (e.g. one delay is missing in new version)
// In Tivaware seem to get clock 66,666,666 hz
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // 80MHz clock

  clockfreq = MAP_SysCtlClockGet();
#else
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);

  clockfreq = MAP_SysCtlClockGet();
#endif

  // Setup PIO
  pios_init();

#if NUM_SPI > 0
  // Setup SPIs
  spis_init();
#endif

  // Setup UARTs
  uarts_init();

  // Setup timers
  timers_init();

#ifdef BUILD_I2C
  // Setup I2Cs
  i2cs_init();
#endif // ifdef BUILD_I2C

#if NUM_PWM > 0
//#ifdef BUILD_PWM
  // Setup PWMs
  pwms_init();
#endif

#ifdef BUILD_COMP
  // Setup comparators
  comps_init();
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
  cmn_systimer_set_base_freq( clockfreq );
  cmn_systimer_set_interrupt_freq( SYSTICKHZ );

#ifdef BUILD_UIP
  // Setup ethernet (TCP/IP)
  eth_init();
#endif

  
  // Common platform initialization code
  cmn_platform_init();
#ifdef DEBUG
// Console has been set up now, so can print error messages for assertion failures, etc.
  consoleinited = true;
#endif

  // Virtual timers
  // If the ethernet controller is used the timer is already initialized, so skip this sequence
#if VTMR_NUM_TIMERS > 0 
#if !defined( BUILD_UIP )
  // Configure SysTick for a periodic interrupt.
  MAP_SysTickPeriodSet( clockfreq / SYSTICKHZ );
  MAP_SysTickEnable();
//    MAP_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);  
  MAP_SysTickIntEnable();

  MAP_IntMasterEnable();
#endif  
#endif

#if ! defined ( FORLM4F )
  MAP_FlashUsecSet( SysCtlClockGet() );   // not required on Tiva devices
#endif

  // All done
  return PLATFORM_OK;
}

// ****************************************************************************
// PIO
// Same configuration on LM3S8962, LM3S6965, LM3S6918 (8 ports)
// 9B92 has 9 ports (Port J in addition to A-H)
// LM4F120, LM4F230 have 6 ports
// TM4C1294 has 13 ports

#if defined( FORLM3S9B92 ) || defined( FORLM3S9D92 )
  const u32 pio_base[] = { GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTC_BASE, GPIO_PORTD_BASE,
                                  GPIO_PORTE_BASE, GPIO_PORTF_BASE, GPIO_PORTG_BASE, GPIO_PORTH_BASE, 
                                  GPIO_PORTJ_BASE };
                                  
  const u32 pio_sysctl[] = { SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOC, SYSCTL_PERIPH_GPIOD,
                                    SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOF, SYSCTL_PERIPH_GPIOG, SYSCTL_PERIPH_GPIOH,
                                    SYSCTL_PERIPH_GPIOJ };

#elif defined ( FORTM4C1294 )

// Dummy entries for ports with no pins I, O
  const u32 pio_base[] = { GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTC_BASE,
                           GPIO_PORTD_BASE, GPIO_PORTE_BASE, GPIO_PORTF_BASE,
                           GPIO_PORTG_BASE, GPIO_PORTH_BASE, 0, GPIO_PORTJ_BASE,
                           GPIO_PORTK_BASE, GPIO_PORTL_BASE, GPIO_PORTM_BASE,
                           GPIO_PORTN_BASE, 0, GPIO_PORTP_BASE, GPIO_PORTQ_BASE };

  const u32 pio_sysctl[] = { SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOC, 
                             SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOF,
                             SYSCTL_PERIPH_GPIOG, SYSCTL_PERIPH_GPIOH, 0, SYSCTL_PERIPH_GPIOJ, 
                             SYSCTL_PERIPH_GPIOK, SYSCTL_PERIPH_GPIOL, SYSCTL_PERIPH_GPIOM,
                             SYSCTL_PERIPH_GPION, 0, SYSCTL_PERIPH_GPIOP, SYSCTL_PERIPH_GPIOQ };

#elif defined( FORLM4F )
  const u32 pio_base[] = { GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTC_BASE, 
                           GPIO_PORTD_BASE, GPIO_PORTE_BASE, GPIO_PORTF_BASE };
                                  
  const u32 pio_sysctl[] = { SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOC, 
                             SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOF };


#else
  const u32 pio_base[] = { GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTC_BASE, GPIO_PORTD_BASE,
                                  GPIO_PORTE_BASE, GPIO_PORTF_BASE, GPIO_PORTG_BASE, GPIO_PORTH_BASE };
  
  const u32 pio_sysctl[] = { SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOC, SYSCTL_PERIPH_GPIOD,
                                    SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOF, SYSCTL_PERIPH_GPIOG, SYSCTL_PERIPH_GPIOH };
#endif

static void pios_init()
{
  unsigned i;

  ASSERT(NUM_PIO <= sizeof(pio_base)/sizeof(u32));
  ASSERT(NUM_PIO <= sizeof(pio_sysctl)/sizeof(u32));
  
  for( i = 0; i < NUM_PIO; i ++ )
    if (pio_sysctl[ i ])          // Skip dummy entries
      MAP_SysCtlPeripheralEnable(pio_sysctl[ i ]);
#ifdef PIO_UNLOCK_NMI
// LM4F pin F0 is MUXed with NMI, also connected to button on Stellaris Launchpad
//  In order to use as GPIO have to unlock it to reprogram
//  Added option to unlock in driver.
//  PD7 can also be NMI, also needs unlock

// PC0 - PC3 are JTAG pins - also need special handling to reprogram 
//  Reprogramming could make chip hard to reflash, so not unlocked in driver.

//  Fixme: Locked pins should return an error when asked to do things can not (like set PF0 as input)

// Unlock PD7 and PF0 (NMI) so can reprogram
    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTD_CR_R = 0xFF;
    GPIO_PORTD_LOCK_R = 0;
#if !defined( FORTM4C1294 )
// Unlock PF0 (NMI) for TM4C123x
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R = 0xFF;
    GPIO_PORTF_LOCK_R = 0;
#endif // ! FORTM4C1294
    
#endif //UNLOCK_NMI
}

pio_type platform_pio_op( unsigned port, pio_type pinmask, int op )
{
  pio_type retval = 1, base = pio_base[ port ];

  switch( op )
  {
    case PLATFORM_IO_PORT_SET_VALUE:
      MAP_GPIOPinWrite( base, 0xFF, pinmask );
      break;

    case PLATFORM_IO_PIN_SET:
      MAP_GPIOPinWrite( base, pinmask, pinmask );
      break;

    case PLATFORM_IO_PIN_CLEAR:
      MAP_GPIOPinWrite( base, pinmask, 0 );
      break;

    case PLATFORM_IO_PORT_DIR_INPUT:
      pinmask = 0xFF;
    case PLATFORM_IO_PIN_DIR_INPUT:
      MAP_GPIOPinTypeGPIOInput( base, pinmask );
      break;

    case PLATFORM_IO_PORT_DIR_OUTPUT:
      pinmask = 0xFF;
    case PLATFORM_IO_PIN_DIR_OUTPUT:
      MAP_GPIOPinTypeGPIOOutput( base, pinmask );
      break;

    case PLATFORM_IO_PORT_GET_VALUE:
      retval = MAP_GPIOPinRead( base, 0xFF );
      break;

    case PLATFORM_IO_PIN_GET:
      retval = MAP_GPIOPinRead( base, pinmask ) ? 1 : 0;
      break;

    case PLATFORM_IO_PIN_PULLUP:
    case PLATFORM_IO_PIN_PULLDOWN:
      MAP_GPIOPadConfigSet( base, pinmask, GPIO_STRENGTH_8MA, op == PLATFORM_IO_PIN_PULLUP ? GPIO_PIN_TYPE_STD_WPU : GPIO_PIN_TYPE_STD_WPD );
      break;

    case PLATFORM_IO_PIN_NOPULL:
      MAP_GPIOPadConfigSet( base, pinmask, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD );
      break;

    default:
      retval = 0;
      break;
  }
  return retval;
}


// ****************************************************************************
// CAN

#if defined( BUILD_CAN )

// Added handle different port mapping (So far this assumed fixed ports PORTD)

#if defined (FORTM4C1294)

// Caution: On TM4C1294 CAN0 conflicts with Uart0, change console to UART4 if using CAN0

#define CAN_PORT_BASE	GPIO_PORTA_BASE
#define CAN_PORT_PINS	( GPIO_PIN_0 | GPIO_PIN_1 )

#define PIN_CAN0RX	GPIO_PA0_CAN0RX
#define PIN_CAN0TX	GPIO_PA1_CAN0TX

#define CAN1_PINS		( GPIO_PIN_0 | GPIO_PIN_1 )
#define PIN_CAN1RX  GPIO_PB0_CAN1RX
#define PIN_CAN1TX  GPIO_PB1_CAN1TX

#elif defined( FORLM4F )

// PIN information from LM4F120H5QR Datasheet

// LM4F120 CAN can use various ports/pins, port selection made at compile time
//
// TODO: Test selection mechanism
// TODO: Look at pin overlap, is there preference for which port to use by default?
//
// TODO: give mechanism to select which port to use for CAN at run time
// BASE			PINS					PIN_CAN0RX			PIN_CAN0TX
// GPIO_PORTB_BASE 	GPIO_PIN_4 | GPIO_PIN_5 	GPIO_PB4_CAN0RX		GPIO_PB5_CAN0TX
// GPIO_PORTE_BASE	GPIO_PIN_4 | GPIO_PIN_5		GPIO_PE4_CAN0RX         GPIO_PE5_CAN0TX         
// GPIO_PORTF_BASE	GPIO_PIN_0 | GPIO_PIN_3		GPIO_PF0_CAN0RX         GPIO_PF3_CAN0TX         

// FIXME: CAN_PORT should be selected in board file

#if CAN_PORT==B
#define CAN_PORT_BASE	GPIO_PORTB_BASE
#define CAN_PORT_PINS	( GPIO_PIN_4 | GPIO_PIN_5 )

#define PIN_CAN0RX	GPIO_PB4_CAN0RX
#define PIN_CAN0TX	GPIO_PB5_CAN0TX

#elif CAN_PORT==E

#define CAN_PORT_BASE	GPIO_PORTE_BASE
#define CAN_PORT_PINS	( GPIO_PIN_4 | GPIO_PIN_5 )

#define PIN_CAN0RX	GPIO_PE4_CAN0RX
#define PIN_CAN0TX	GPIO_PE5_CAN0TX

#elif CAN_PORT==F

// Note that F0 is connected to pushbutton, and F3 is connected to RGB LED (so would have to modify LP to use)

#if PIO_UNLOCK_NMI
#warning CAN using portF, but PF0 and PF3 connected to peripherals on launchpad.
#else
#error Need to define PIO_UNLOCK_NMI, or fix code to allow reprogramming of pin PF_0
#endif


#define CAN_PORT_BASE	GPIO_PORTF_BASE
#define CAN_PORT_PINS	( GPIO_PIN_0 | GPIO_PIN_3 )

#define PIN_CAN0RX	GPIO_PF0_CAN0RX
#define PIN_CAN0TX	GPIO_PF3_CAN0TX

#else
#error Unrecognized CAN port: CAN_PORT
#endif // CAN_PORT

#if defined( FORLM4F230 ) || defined( FORTM4C123G )

// CAN1_BASE already defined in hw_memmap.h
//#define CAN1_BASE		GPIO_PORTA_BASE
//#define CAN1_PINS		( GPIO_PIN_0 | GPIO_PIN_1 )

//#elif
// FIXME: What was this test supposed to be/do

#endif


#else

#define CAN_PORT_BASE	GPIO_PORTD_BASE
#define CAN_PORT_PINS	( GPIO_PIN_0 | GPIO_PIN_1 )

#if defined( FORLM3S8962 )
// Figure out what these should be on 3s8962 - not sure why below not defined
#define PIN_CAN0RX	CAN0RX_PIN
#define PIN_CAN0TX	CAN0TX_PIN
#warning Do not use CAN - pins not set up right
#else
// Check this - does it work on 8962?  (Not sure that GPIO_PD0_CAN0RX, TX are defined on that platform)
// FixMe: Doesn't work on 8962
#define PIN_CAN0RX	GPIO_PD0_CAN0RX
#define PIN_CAN0TX	GPIO_PD1_CAN0TX
#endif

#endif

/* Code assumes that RX and TX pins will be on same port.  
Stellarisware defines the following for some CPUs (e.g. 8962)
#define CAN0RX_PERIPH           SYSCTL_PERIPH_GPIOD
#define CAN0RX_PORT             GPIO_PORTD_BASE
#define CAN0RX_PIN              GPIO_PIN_0

#define CAN0TX_PERIPH           SYSCTL_PERIPH_GPIOD
#define CAN0TX_PORT             GPIO_PORTD_BASE
#define CAN0TX_PIN              GPIO_PIN_1
*/


#if (NUM_CAN > 1)
#define CAN1B , CAN1_BASE
#define CAN1SYS , SYSCTL_PERIPH_CAN1
#define CAN1INT , INT_CAN1
#define CAN1GPB , CAN1_BASE
#define CAN1P   , CAN1_PINS
#else
#define CAN1B
#define CAN1SYS
#define CAN1INT
#define CAN1GPB
#define CAN1P
#endif

static const u32 can_base[] =   { CAN0_BASE           CAN1B };
static const u32 can_sysctl[] = { SYSCTL_PERIPH_CAN0  CAN1SYS };
static const u32 can_int[] =    { INT_CAN0            CAN1INT };

static const u32 can_gpio_base[] =  { CAN_PORT_BASE   CAN1GPB };
static const u8 can_gpio_pins[] =   { CAN_PORT_PINS   CAN1P };

// FIXME: Flags could be BOOL or bit
typedef struct {
  volatile u8 rx_flag, tx_flag, err_flag; 
  char tx_buf[PLATFORM_CAN_MAXLEN]; 
  tCANMsgObject msg_rx;
} can_status_t;

can_status_t can_status[ NUM_CAN-1 ];

// Speed used in INIT
#ifndef CAN_INIT_SPEED
#define CAN_INIT_SPEED	500000
#endif

// Message object for receiving
#define CAN_MSG_OBJ_RX	1

// Message object for transmitting
#define CAN_MSG_OBJ_TX	2


// LM3S9Bxx and LM4F120 MCU CAN seems to run off of system clock, LM3S8962 has 8 MHz clock

#if defined( FORLM3S8962 )
#define LM3S_CAN_CLOCK  8000000
#else
#define LM3S_CAN_CLOCK  SysCtlClockGet()
#endif

// CAN Interrupt Handler
void CANIntHandler(unsigned id);

void can0_handler(void)
{
	CANIntHandler(0);
}

#if (NUM_CAN > 1)
void can1_handler(void)
{
	CANIntHandler(1);
}

#if (NUM_CAN > 2)
void can2_handler(void)
{
	CANIntHandler(2);
}

#if (NUM_CAN > 3)

#error At most 3 CANs

#endif
#endif
#endif

void CANIntHandler(unsigned id)
{
// Fixme: Needs to handle multiple CANs (flags)

  u32 status = MAP_CANIntStatus(can_base[id], CAN_INT_STS_CAUSE);

// ToDO: Why not use a switch statement?
  if(status == CAN_INT_INTID_STATUS)
  {
    status = MAP_CANStatusGet(can_base[id], CAN_STS_CONTROL);

// Why is the status fetched but not used?
    can_status[id].err_flag = 1;
    can_status[id].tx_flag = 0;
  }
  else if( status == CAN_MSG_OBJ_RX ) // Message receive
  {
    MAP_CANIntClear(can_base[id], CAN_MSG_OBJ_RX);
    can_status[id].rx_flag = 1;
    can_status[id].err_flag = 0;
  }
  else if( status == CAN_MSG_OBJ_TX ) // Message send
  {
    MAP_CANIntClear(can_base[id], CAN_MSG_OBJ_TX);
    can_status[id].tx_flag = 0;
    can_status[id].err_flag = 0;
  }
  else
    MAP_CANIntClear(can_base[id], status);
}


static void cans_init( void )
{
  unsigned id;

  ASSERT(NUM_CAN <= sizeof(can_base)/sizeof(u32));
  ASSERT(NUM_CAN <= sizeof(can_sysctl)/sizeof(u32));
  ASSERT(NUM_CAN <= sizeof(can_int)/sizeof(u32));
  ASSERT(NUM_CAN <= sizeof(can_gpio_base)/sizeof(u32));
  
  for( id = 0; id < NUM_CAN; id++){
    MAP_SysCtlPeripheralEnable( can_sysctl[id] );
    MAP_CANInit( can_base[id] );
    MAP_CANBitRateSet(can_base[id], LM3S_CAN_CLOCK, CAN_INIT_SPEED);
    MAP_CANIntEnable(can_base[id], CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS );
    MAP_IntEnable(can_int[id]);
    MAP_CANEnable(can_base[id]);

    can_status[id].rx_flag = 0;
    can_status[id].tx_flag = 0;
    can_status[id].err_flag = 0;
      // Configure default catch-all message object
    can_status[id].msg_rx.ui32MsgID = 0;
    can_status[id].msg_rx.ui32MsgIDMask = 0;
    can_status[id].msg_rx.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
      can_status[id].msg_rx.ui32MsgLen = PLATFORM_CAN_MAXLEN;
    MAP_CANMessageSet(can_base[id], CAN_MSG_OBJ_RX, &can_status[id].msg_rx, MSG_OBJ_TYPE_RX);
  }
}


// Platform functions

u32 platform_can_setup( unsigned id, u32 clock )
{
  //TODO: generalize to more than 2 CANs
  if (id == 0) 
  {
    MAP_GPIOPinConfigure(PIN_CAN0RX);
    MAP_GPIOPinConfigure(PIN_CAN0TX);
  } else {
#if (NUM_CAN == 2)  
    MAP_GPIOPinConfigure(PIN_CAN1RX);
    MAP_GPIOPinConfigure(PIN_CAN1TX);
#elif (NUM_CAN > 2)
  #error Need to extend setup to handle more than 2 CAN
#endif // NUM_CAN > 2
  }
  MAP_GPIOPinTypeCAN(can_gpio_base[id], can_gpio_pins[id]);

  MAP_CANDisable(can_base[id]);
  MAP_CANBitRateSet(can_base[id], LM3S_CAN_CLOCK, clock );
  MAP_CANEnable(can_base[id]);
  return clock;
}

int platform_can_send( unsigned id, u32 canid, u8 idtype, u8 len, const u8 *data )
{
  tCANMsgObject msg_tx;
  const char *s = ( char * )data;
  char *d;

  // Wait for outgoing messages to clear
  WAIT_WHILE( can_status[id].tx_flag == 1 );

  msg_tx.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
  
  if( idtype == ELUA_CAN_ID_EXT )
    msg_tx.ui32Flags |= MSG_OBJ_EXTENDED_ID;
  
  msg_tx.ui32MsgIDMask = 0;
  msg_tx.ui32MsgID = canid;
  msg_tx.ui32MsgLen = len;
  msg_tx.pui8MsgData = ( u8 * )can_status[id].tx_buf;

  d = can_status[id].tx_buf;
  DUFF_DEVICE_8( len,  *d++ = *s++ );

  can_status[id].tx_flag = 1;
  MAP_CANMessageSet(can_base[id], CAN_MSG_OBJ_TX, &msg_tx, MSG_OBJ_TYPE_TX);

  return PLATFORM_OK;
}

int platform_can_recv( unsigned id, u32 *canid, u8 *idtype, u8 *len, u8 *data )
{
  // wait for a message
  if( can_status[id].rx_flag != 0 )
  {
    can_status[id].msg_rx.pui8MsgData = data;
    MAP_CANMessageGet(can_base[id], CAN_MSG_OBJ_RX, &can_status[id].msg_rx, 0);
    can_status[id].rx_flag = 0;

    *canid = ( u32 )can_status[id].msg_rx.ui32MsgID;
    *idtype = ( can_status[id].msg_rx.ui32Flags & MSG_OBJ_EXTENDED_ID )? ELUA_CAN_ID_EXT : ELUA_CAN_ID_STD;
    *len = can_status[id].msg_rx.ui32MsgLen;
    return PLATFORM_OK;
  }
  else
    return PLATFORM_UNDERFLOW;
}

#endif // BUILD_CAN



// ****************************************************************************
// SPI

#if NUM_SPI > 0

// TODO: LM4F120 has 4 SPIs - need way to specify how many to use

// TODO: Fixed the defines, but haven't looked at the code to see if anything needs adapting for LM4F/TM4C.

#if defined( FORLM4F )

static const u32 spi_base[] = { SSI0_BASE, SSI1_BASE, SSI2_BASE, SSI3_BASE };
static const u32 spi_sysctl[] = { SYSCTL_PERIPH_SSI0, SYSCTL_PERIPH_SSI1, 
                                  SYSCTL_PERIPH_SSI2, SYSCTL_PERIPH_SSI3};

#else // defined( FORLM4F )

// Same configuration on LM3S8962, LM3S6965, LM3S6918 and LM3S9B92 (2 SPI ports)
// All possible LM3S SPIs defs

//  PIN info extracted from LM3S6950 and 5769 datasheets
static const u32 spi_base[] = { SSI0_BASE, SSI1_BASE };
static const u32 spi_sysctl[] = { SYSCTL_PERIPH_SSI0, SYSCTL_PERIPH_SSI1 };

#endif // defined( FORLM4F )


// Pin mapping

#if defined( FORTM4C1294 )

// Note: SSI1 is split, some pins on port B, some on port E PB5/PB4 PE4/PE5

// TODO: Pin Map - SSI3 chose between portQ and portF
// SSI3 alternate - clock: PQ0, FSS: PQ1, TX: PQ2, RX: PQ3

static const u32 spi_gpio_base[] = { GPIO_PORTA_BASE, GPIO_PORTE_BASE,
                                     GPIO_PORTD_BASE, GPIO_PORTQ_BASE };
static const u8  spi_gpio_pins[] = { GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_4,
                                                  GPIO_PIN_5 | GPIO_PIN_4,
                                     GPIO_PIN_3 | GPIO_PIN_0 | GPIO_PIN_1,
                                     GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_2};
//                                    SSIxClk       SSIxRx       SSIxTx

static const u32 spi_gpio_clk_base[] = 
  { GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTD_BASE, GPIO_PORTQ_BASE };
static const u8  spi_gpio_clk_pin[] = 
  { GPIO_PIN_2,      GPIO_PIN_5,      GPIO_PIN_3,      GPIO_PIN_0 };

// Note: While SSI2 is SPI for BoosterPack 1 connector, PD2 is on BoosterPack2 connector
// SSI3 is SPI for BoosterPack 2 connector, FSS (PQ1) is "CS Other" on that connector
static const u32 spi_gpio_fss_base[] = 
  { GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTD_BASE, GPIO_PORTQ_BASE };
static const u8  spi_gpio_fss_pin[] = 
  { GPIO_PIN_3,      GPIO_PIN_4,      GPIO_PIN_2,      GPIO_PIN_1 };

#elif defined( FORLM4F )

// TODO: LM4F120 can map SSI 1 to either port D or port F (check code to see implications)
//  Port D, uses same pins as SSI3
//  Port F, uses pins connected to buttons and LED

// PIN information from LM4F120H5QR Datasheet
static const u32 spi_gpio_base[] = { GPIO_PORTA_BASE, GPIO_PORTF_BASE, 
                                     GPIO_PORTB_BASE, GPIO_PORTD_BASE };
static const u8 spi_gpio_pins[] = { GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5,
                                    GPIO_PIN_2 | GPIO_PIN_0 | GPIO_PIN_1,
                                    GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7,
                                    GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3,
  };
//                                  SSIxClk       SSIxRx       SSIxTx

static const u32 spi_gpio_clk_base[] = 
  { GPIO_PORTA_BASE, GPIO_PORTF_BASE, GPIO_PORTB_BASE, GPIO_PORTD_BASE };
static const u8 spi_gpio_clk_pin[] = 
  { GPIO_PIN_2,      GPIO_PIN_2,      GPIO_PIN_4,      GPIO_PIN_0};

static const u32 spi_gpio_fss_base[] = 
  { GPIO_PORTA_BASE, GPIO_PORTF_BASE, GPIO_PORTB_BASE, GPIO_PORTD_BASE };
static const u8  spi_gpio_fss_pin[] = 
  { GPIO_PIN_3,      GPIO_PIN_3,      GPIO_PIN_5,      GPIO_PIN_1 };
  
#elif defined( ELUA_BOARD_SOLDERCORE )

static const u32 spi_gpio_base[] = { GPIO_PORTA_BASE, GPIO_PORTF_BASE };
static const u8 spi_gpio_pins[] = {  GPIO_PIN_4 | GPIO_PIN_5,
                                     GPIO_PIN_4 | GPIO_PIN_5 };
//                                   SSIxRx       SSIxTx

static const u32 spi_gpio_clk_base[] = { GPIO_PORTA_BASE, GPIO_PORTH_BASE };
static const u8 spi_gpio_clk_pin[] = { GPIO_PIN_2, GPIO_PIN_4 };
#else

static const u32 spi_gpio_base[] = { GPIO_PORTA_BASE, GPIO_PORTE_BASE };
static const u8 spi_gpio_pins[] = { GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,
                                    GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 };
//                                  SSIxClk      SSIxFss      SSIxRx       SSIxTx

static const u32 spi_gpio_clk_base[] = { GPIO_PORTA_BASE, GPIO_PORTE_BASE };
static const u8 spi_gpio_clk_pin[] = { GPIO_PIN_2, GPIO_PIN_0 };

// TODO: Add fss (if it exists for these processors)?

#endif // defined( ELUA_BOARD_SOLDERCORE )



// Pin mapping

#if defined( ELUA_BOARD_SOLDERCORE )

static const u32 ssi_rx_pin[] =  { GPIO_PF4_SSI1RX };
static const u32 ssi_tx_pin[] =  { GPIO_PF5_SSI1TX };
static const u32 ssi_clk_pin[] = { GPIO_PH4_SSI1CLK };
// static const u32 ssi_pin_fss[] = {  };

// ToDo: Need general pin mux handling
#elif defined( FORTM4C1294 )

static const u32 ssi_rx_pin[] =  
  {GPIO_PA4_SSI0XDAT0,  GPIO_PE4_SSI1XDAT0,  GPIO_PD1_SSI2XDAT0,  GPIO_PQ2_SSI3XDAT0 };
static const u32 ssi_tx_pin[] =  
  {GPIO_PA5_SSI0XDAT1,  GPIO_PE5_SSI1XDAT1,  GPIO_PD0_SSI2XDAT1,  GPIO_PQ3_SSI3XDAT1 };
static const u32 ssi_clk_pin[] = 
  {GPIO_PA2_SSI0CLK, GPIO_PB5_SSI1CLK, GPIO_PD3_SSI2CLK, GPIO_PQ0_SSI3CLK};
static const u32 ssi_fss_pin[] = 
  {GPIO_PA3_SSI0FSS, GPIO_PB4_SSI1FSS, GPIO_PD2_SSI2FSS, GPIO_PQ1_SSI3FSS};

#elif defined( FORLM4F )

static const u32 ssi_rx_pin[] =  
  {GPIO_PA4_SSI0RX,  GPIO_PF0_SSI1RX,  GPIO_PB6_SSI2RX,  GPIO_PD2_SSI3RX };
static const u32 ssi_tx_pin[] =  
  {GPIO_PA5_SSI0TX,  GPIO_PF1_SSI1TX,  GPIO_PB7_SSI2TX,  GPIO_PD3_SSI3TX };
static const u32 ssi_clk_pin[] = 
  {GPIO_PA2_SSI0CLK, GPIO_PF2_SSI1CLK, GPIO_PB4_SSI2CLK, GPIO_PD0_SSI3CLK};
static const u32 ssi_fss_pin[] = 
  {GPIO_PA3_SSI0FSS, GPIO_PF3_SSI1FSS, GPIO_PB5_SSI2FSS, GPIO_PD1_SSI3FSS};

/*
GPIO_PB4_SSI2CLK  (also CAN0RX)
GPIO_PB5_SSI2FSS  (also CAN0TX)

GPIO_PF0_SSI1RX   (also CAN0RX)
GPIO_PF1_SSI1TX 
GPIO_PF2_SSI1CLK
GPIO_PF3_SSI1FSS  (also CAN0TX)
*/
#elif defined( USE_PIN_MUX )

#error USE_PIN_MUX defined, but no pin mappings for this CPU

#endif


static void spis_init()
{
  unsigned i;

  ASSERT(NUM_SPI <= sizeof(spi_base)/sizeof(u32));
  ASSERT(NUM_SPI <= sizeof(spi_sysctl)/sizeof(u32));
  ASSERT(NUM_SPI <= sizeof(spi_gpio_base)/sizeof(u32));
  ASSERT(NUM_SPI <= sizeof(spi_gpio_clk_base)/sizeof(u32));

#if defined( ELUA_BOARD_SOLDERCORE )
  GPIOPinConfigure( GPIO_PH4_SSI1CLK );
  GPIOPinConfigure( GPIO_PF4_SSI1RX );
  GPIOPinConfigure( GPIO_PF5_SSI1TX );
#else
// TODO: fix Pin Mux
#ifdef USE_PIN_MUX

  ASSERT(NUM_SPI <= sizeof(ssi_rx_pin)/sizeof(u32));
  ASSERT(NUM_SPI <= sizeof(ssi_tx_pin)/sizeof(u32));
  ASSERT(NUM_SPI <= sizeof(ssi_clk_pin)/sizeof(u32));

#error Debug: USE_PIN_MUX is on
// TODO: Instead should probably do pin configure first time actually setup a SPI port (so don't configure unused ports)

  for( i = 0; i < NUM_SPI; i ++ ){
    MAP_GPIOPinConfigure(ssi_rx_pin[i]);
    MAP_GPIOPinConfigure(ssi_tx_pin[i]);
    MAP_GPIOPinConfigure(ssi_clk_pin[i]);
#ifdef SSI_USE_FSS
    MAP_GPIOPinConfigure(ssi_fss_pin[i]);
#endif
  };
#endif // USE_PIN_MUX

#endif // ELUA_BOARD_SOLDERCORE

  for( i = 0; i < NUM_SPI; i ++ )
    MAP_SysCtlPeripheralEnable( spi_sysctl[ i ] );
}

// cpol - clock polarity (0 or 1), cpha - clock phase (0 or 1)
u32 platform_spi_setup( unsigned id, int mode, u32 clock, unsigned cpol, unsigned cpha, unsigned databits )
{
  unsigned protocol;

  if( cpol == 0 )
    protocol = cpha ? SSI_FRF_MOTO_MODE_1 : SSI_FRF_MOTO_MODE_0;
  else
    protocol = cpha ? SSI_FRF_MOTO_MODE_3 : SSI_FRF_MOTO_MODE_2;
  mode = mode == PLATFORM_SPI_MASTER ? SSI_MODE_MASTER : SSI_MODE_SLAVE;
  MAP_SSIDisable( spi_base[ id ] );

  MAP_GPIOPinTypeSSI( spi_gpio_base[ id ], spi_gpio_pins[ id ] );
  MAP_GPIOPinTypeSSI( spi_gpio_clk_base[ id ], spi_gpio_clk_pin[ id ] );
#ifdef SSI_USE_FSS
  MAP_GPIOPinTypeSSI( spi_gpio_fss_base[ id ], spi_gpio_fss_pin[ id ] );
#endif

  // FIXME: not sure this is always "right"
  MAP_GPIOPadConfigSet( spi_gpio_base[ id ], spi_gpio_pins[ id ], 
    GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU );
  MAP_GPIOPadConfigSet( spi_gpio_clk_base[ id ], spi_gpio_clk_pin[ id ],
    GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU );
#ifdef SSI_USE_FSS
  MAP_GPIOPadConfigSet( spi_gpio_fss_base[ id ], spi_gpio_fss_pin[ id ],
    GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU );
#endif

  MAP_SSIConfigSetExpClk( spi_base[ id ], clockfreq, protocol, mode, clock, databits );
  MAP_SSIEnable( spi_base[ id ] );
  return clock;
}

spi_data_type platform_spi_send_recv( unsigned id, spi_data_type data )
{
  MAP_SSIDataPut( spi_base[ id ], data );
  MAP_SSIDataGet( spi_base[ id ], &data );
  return data;
}

void platform_spi_select( unsigned id, int is_select )
{
#ifdef SSI_USE_FSS

  // FIXME: Not sure if there is anything to do here to manage FSS

#else
  // This platform doesn't have a hardware SS pin, so there's nothing to do here
  id = id;
  is_select = is_select;
#endif
}

#endif // NUM_SPI > 0


// ****************************************************************************
// I2C

// Testing - code exists up in elua, there is platform code in AVR32, STR9
// TODO: Code here is incomplete (doesn't handle ACK, doesn't handle burst transfers)

// TODO: Add support for other speeds(?) - fast mode plus (1 Mb/s), High speed mode (3.33 Mb/s)

#ifdef BUILD_I2C

#if defined (FORTM4C1294)
// 10 I2C
// TODO: support alternate pin mapping for I2C: 2, 5, 7
// Caution: I2C9 uses PortA pins 0/1 - conflict with UART0 (Serial console)

const u32 i2c_base[] = { I2C0_MASTER_BASE, I2C1_MASTER_BASE, I2C2_MASTER_BASE, I2C3_MASTER_BASE,
                          I2C4_MASTER_BASE, I2C5_MASTER_BASE, I2C6_MASTER_BASE, I2C7_MASTER_BASE,
                          I2C8_MASTER_BASE, I2C9_MASTER_BASE };
static const u32 i2c_sysctl[] = { SYSCTL_PERIPH_I2C0, SYSCTL_PERIPH_I2C1, SYSCTL_PERIPH_I2C2, SYSCTL_PERIPH_I2C3,
                          SYSCTL_PERIPH_I2C4, SYSCTL_PERIPH_I2C5, SYSCTL_PERIPH_I2C6, SYSCTL_PERIPH_I2C7,
                          SYSCTL_PERIPH_I2C8, SYSCTL_PERIPH_I2C9 };

static const u32 i2c_gpio_base[] = { GPIO_PORTB_BASE, GPIO_PORTG_BASE, GPIO_PORTL_BASE, GPIO_PORTK_BASE, 
                                      GPIO_PORTK_BASE, GPIO_PORTB_BASE, GPIO_PORTA_BASE, GPIO_PORTD_BASE, 
                                      GPIO_PORTD_BASE, GPIO_PORTA_BASE };
static const u8 i2c_gpio_pins[] = { GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_1 | GPIO_PIN_0, 
                                    GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_0 | GPIO_PIN_1,
                                    GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_2 | GPIO_PIN_3, 
                                    GPIO_PIN_0 | GPIO_PIN_1 };

static const i2c_scl_pin[] = { GPIO_PB2_I2C0SCL, GPIO_PG0_I2C1SCL, GPIO_PL1_I2C2SCL, GPIO_PK4_I2C3SCL,
                               GPIO_PK6_I2C4SCL, GPIO_PB0_I2C5SCL, GPIO_PA6_I2C6SCL, GPIO_PD0_I2C7SCL,
                               GPIO_PD2_I2C8SCL, GPIO_PA0_I2C9SCL };
static const i2c_sda_pin[] = { GPIO_PB3_I2C0SDA, GPIO_PG1_I2C1SDA, GPIO_PL0_I2C2SDA, GPIO_PK5_I2C3SDA,
                               GPIO_PK7_I2C4SDA, GPIO_PB1_I2C5SDA, GPIO_PA7_I2C6SDA, GPIO_PD1_I2C7SDA,
                               GPIO_PD3_I2C8SDA, GPIO_PA1_I2C9SDA };

#elif defined( FORLM4F )

static const u32 i2c_gpio_base[] = { GPIO_PORTB_BASE, GPIO_PORTA_BASE, GPIO_PORTE_BASE, GPIO_PORTD_BASE };
static const u8 i2c_gpio_pins[] = { GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_0 | GPIO_PIN_1 };

static const i2c_scl_pin[] = { GPIO_PB2_I2C0SCL, GPIO_PA6_I2C1SCL, GPIO_PE4_I2C2SCL, GPIO_PD0_I2C3SCL };
static const i2c_sda_pin[] = { GPIO_PB3_I2C0SDA, GPIO_PA7_I2C1SDA, GPIO_PE5_I2C2SDA, GPIO_PD1_I2C3SDA };


#elif defined( FORLM6918 ) || defined( FORLM6965 ) || defined( FORLM3S9B92 ) || defined( FORLM3S9D92 )
//FIXME: Fill in base, pins, etc - have 2 I2C (though may not all use same ports)

#elif defined( FORLM8962 )
//FIXME: Fill in base, pins, etc. (1 I2C)


const u32 i2c_base[] = { I2C_MASTER_BASE };
static const u32 i2c_sysctl[] = { SYSCTL_PERIPH_I2C0 };
//FIXME: needs pin mux data (or suppress that behavior)
static const u32 i2c_gpio_base[] = {};
static const u8 i2c_gpio_pins[] = {};

static const i2c_scl_pin[] = {};
static const i2c_sda_pin[] = {};
#endif


// FIXME: just copied from others, need to check documents
static void I2Cs_init()
{
  ASSERT(NUM_I2C <= sizeof(i2c_base)/sizeof(u32));
  ASSERT(NUM_I2C <= sizeof(i2c_sysctl)/sizeof(u32));
  ASSERT(NUM_I2C <= sizeof(i2c_gpio_base)/sizeof(u32));

  unsigned i;
  for( i = 0; i < NUM_I2C; i ++ )
    MAP_SysCtlPeripheralEnable(i2c_sysctl[ i ]);
}


// Platform specific
// Not finished - needs to enable I2C, needs to figure out speed, clock should be from cache
u32 platform_i2c_setup( unsigned id, u32 speed )
{
  //FIXME: Pin mux - may not need for all parts

#ifdef USE_PIN_MUX
  MAP_GPIOPinConfigure(i2c_scl_pin[ id ]);
  MAP_GPIOPinConfigure(i2c_sda_pin[ id ]);
#endif

  MAP_GPIOPinTypeI2C( i2c_gpio_base[ id ], i2c_gpio_pins[ id ] );

  MAP_I2CMasterInitExpClk( i2c_base[id], clockfreq,  
                    speed == PLATFORM_I2C_SPEED_SLOW);
  //FIXME: Needs to return speed actually set
};


// Address is 0..127, direction - from enum, return is boolean
int platform_i2c_send_address( unsigned id, u16 address, int direction )
{
	bool bReceive = (direction == PLATFORM_I2C_DIRECTION_RECEIVER);

  MAP_I2CMasterSlaveAddrSet( i2c_base[id], address, receive	);

  if (bReceive) MAP_I2CMasterControl(i2c_base[id], I2C_MASTER_CMD_SINGLE_RECEIVE);
  // Dummy receive so don't get junk on first read

  // FIXME: Not finished - needs to return boolean
}; // Returns boolean  - need to figure out semantics for return "1 for success, 0 for error."

// FIXME: Arbitrarily defined - just number of loops, should be based on time
#define I2C_MAX_WAIT	100000

//TODO: Current code waits until done, not sure if that is desired semantics
int platform_i2c_send_byte( unsigned id, u8 data )
{
  u32 i;
  bool busy;

  //FIXME: Timeout should probably be based on time, not just loop cycles
  for(i=0; i<I2C_MAX_WAIT && busy = MAP_I2CMasterBusBusy( i2c_base[id] ); i++) {}; // Wait until bus free
  if(busy)
	  return 0;	// Still busy - return error
  MAP_I2CMasterDataPut( i2c_base[id], data);
  MAP_I2CMasterControl( i2c_base[id], I2C_MASTER_CMD_SINGLE_SEND);

  //FIXME: Timeout should probably be based on time, not just loop cycles
  for(i=0; i<I2C_MAX_WAIT && MAP_I2CMasterBusBusy( i2c_base[id] ); i++) {};		// Wait until sent

  // Return 0 for problem, 1 for success
  return (MAP_I2CMasterError( i2c_base[id] ) == I2C_MASTER_ERR_NONE) ? 1 : 0;
};

//FIXME: what should this do? "Send an I2C START condition on the specified interface."
void platform_i2c_send_start( unsigned id )
{
};

//FIXME: Just a guess at what this might do  "Send an I2C STOP condition on the specified interface."
void platform_i2c_send_stop( unsigned id )
{
	MAP_I2CMasterControl( i2c_base[id], I2C_MASTER_CMD_BURST_SEND_FINISH );
};


// return byte read, or -1, ack is true except for last item read
// (ack is acknowledge bit to send 1 to acknowledge, 0 to reject/stop)

int platform_i2c_recv_byte( unsigned id, int ack )
{
    MAP_I2CMasterControl( i2c_base[id], I2C_MASTER_CMD_SINGLE_RECEIVE);

	return MAP_I2CMasterDataGet( i2c_base[id] );
	// FIXME: this doesn't check for errors, doesn't use ack
};



/*
From AVR:

int platform_i2c_send_address( unsigned id, u16 address, int direction )
{
  // Convert enum codes to R/w bit value.
  // If TX == 0 and RX == 1, this test will be removed by the compiler
  if ( ! ( PLATFORM_I2C_DIRECTION_TRANSMITTER == 0 &&
           PLATFORM_I2C_DIRECTION_RECEIVER == 1 ) ) {
    direction = ( direction == PLATFORM_I2C_DIRECTION_TRANSMITTER ) ? 0 : 1;
  }

  // Low-level returns nack (0=acked); we return ack (1=acked).
  return ! i2c_write_byte( (address << 1) | direction );
}

*/

#endif	// BUILD_I2C


// ****************************************************************************
// UART
// Different configurations for LM3S8962, LM3S6918 (2 UARTs) and LM3S6965, LM3S9B92 (3 UARTs)

// FIXME FORLM4F120 has 8 UART - need way to specify how many want to use
// If implement USB, then remove UART6 - since it uses PD4,5 (so put UART6 last)

// TODO: add flow control support, at least some ports have it on the TM4C129x

#ifdef FORLM4F

#define UART_PIN_CONFIGURE

#if !defined( FORTM4C1294 )

const u32 uart_base[] = { 	UART0_BASE, UART1_BASE, UART2_BASE, UART3_BASE, 
      				UART4_BASE, UART5_BASE, UART7_BASE, UART6_BASE };
static const u32 uart_sysctl[] = { 
	SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_UART1, SYSCTL_PERIPH_UART2, SYSCTL_PERIPH_UART3, 
	SYSCTL_PERIPH_UART4, SYSCTL_PERIPH_UART5, SYSCTL_PERIPH_UART7, SYSCTL_PERIPH_UART6 };

static const u32 uart_gpio_base[] = { GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTD_BASE, 
      GPIO_PORTC_BASE, GPIO_PORTC_BASE, GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTD_BASE };
static const u8 uart_gpio_pins[] = { GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1, 
	GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_6 | GPIO_PIN_7, 
	GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_4 | GPIO_PIN_5, 
	GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_4 | GPIO_PIN_5 };

// This is really 2 fields per UUART (should be in a struct) with GPIO_PORTs and PINs
static const u32 uart_gpiofunc[] = {
	GPIO_PA0_U0RX, GPIO_PA1_U0TX, GPIO_PB0_U1RX, GPIO_PB1_U1TX,
	GPIO_PD6_U2RX, GPIO_PD7_U2TX, GPIO_PC6_U3RX, GPIO_PC7_U3TX, 
	GPIO_PC4_U4RX, GPIO_PC5_U4TX, GPIO_PE4_U5RX, GPIO_PE5_U5TX,
	GPIO_PE0_U7RX, GPIO_PE1_U7TX, GPIO_PD4_U6RX, GPIO_PD5_U6TX };

#else

const u32 uart_base[] = { UART0_BASE, UART1_BASE, UART2_BASE, UART3_BASE, 
      				            UART4_BASE, UART5_BASE, UART6_BASE, UART7_BASE };
static const u32 uart_sysctl[] = { 
	SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_UART1, SYSCTL_PERIPH_UART2, SYSCTL_PERIPH_UART3, 
	SYSCTL_PERIPH_UART4, SYSCTL_PERIPH_UART5, SYSCTL_PERIPH_UART6, SYSCTL_PERIPH_UART7 };

static const u32 uart_gpio_base[] = { GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTD_BASE, 
      GPIO_PORTJ_BASE, GPIO_PORTK_BASE, GPIO_PORTC_BASE, GPIO_PORTP_BASE, GPIO_PORTC_BASE };
static const u8 uart_gpio_pins[] = { GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1, 
	GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_0 | GPIO_PIN_1, 
	GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_6 | GPIO_PIN_7, 
	GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_4 | GPIO_PIN_5 };

// Looks like this is really 2 fields per UUART (should be in a struct) with GPIO_PORTs and PINs
static const u32 uart_gpiofunc[] = {
	GPIO_PA0_U0RX, GPIO_PA1_U0TX, GPIO_PB0_U1RX, GPIO_PB1_U1TX,
	GPIO_PD4_U2RX, GPIO_PD5_U2TX, GPIO_PJ0_U3RX, GPIO_PJ1_U3TX, 
	GPIO_PK0_U4RX, GPIO_PK1_U4TX, GPIO_PC6_U5RX, GPIO_PC7_U5TX,
	GPIO_PP0_U6RX, GPIO_PP1_U6TX, GPIO_PC4_U7RX, GPIO_PC5_U7TX };

// U1: Rx: PB0/Q4
// U2 PD4/5 or PA 6/7, CTS: PN3/PD7, RTS: PN2/PD6
// U3 PA4/5 or PJ0/1, RTS PP4/PN4 CTS: PP5/PN5
//  U4 PK0/1 or PA2/3

#endif

 
#else // LM3S

// All possible LM3S uart defs
const u32 uart_base[] = { UART0_BASE, UART1_BASE, UART2_BASE };
static const u32 uart_sysctl[] = { SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_UART1, SYSCTL_PERIPH_UART2 };
static const u32 uart_gpio_base[] = { GPIO_PORTA_BASE, GPIO_PORTD_BASE, GPIO_PORTG_BASE };
static const u8 uart_gpio_pins[] = { GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_0 | GPIO_PIN_1 };

#ifdef GPIO_PA0_U0RX

// FIXME: These symbols used to be defined in gpio.h but were removed in Stellarisware 9453
// TODO: Figure out neater test/fix

static const u32 uart_gpiofunc[] = { GPIO_PA0_U0RX, GPIO_PA1_U0TX, GPIO_PD2_U1RX, GPIO_PD3_U1TX, GPIO_PG0_U2RX, GPIO_PG1_U2TX };
#define UART_PIN_CONFIGURE

#endif // GPIO_PA0_U0RX

#endif // FORLM4F


static void uarts_init()
{
  unsigned i;

  ASSERT(NUM_UART <= sizeof(uart_sysctl)/sizeof(u32));
  ASSERT(NUM_UART <= sizeof(uart_base)/sizeof(u32));
  ASSERT(NUM_UART <= sizeof(uart_gpio_base)/sizeof(u32));
  
  for( i = 0; i < NUM_UART; i ++ )
    MAP_SysCtlPeripheralEnable(uart_sysctl[ i ]);
}

u32 platform_uart_setup( unsigned id, u32 baud, int databits, int parity, int stopbits )
{
  u32 config;
// FIXME: This may need to disable interrupts and then restore them to previous state
// Or at least throw an error if UART interrupt enabled when this called
// TODO: What about setting FIFO Int size (check int code).

  if( id < NUM_UART )
  {
#ifdef UART_PIN_CONFIGURE
    MAP_GPIOPinConfigure( uart_gpiofunc[ id << 1 ] );
    MAP_GPIOPinConfigure( uart_gpiofunc[ ( id << 1 ) + 1 ] );
#endif
    MAP_GPIOPinTypeUART( uart_gpio_base[ id ], uart_gpio_pins[ id ] );

// Clock source - try PIOSC rather than system clock
// Tivaware seems to have a problem setting either the CPU speed or the UART clock when request 80MHz on Stellaris LP
// Using alternate clock might fix baud rate problems with 1294 (but don't know what real problem is)
  unsigned long uart_clock;
#if defined( UART_ALT_CLOCK )
    MAP_UARTClockSourceSet(uart_base[ id ], UART_CLOCK_PIOSC);
    uart_clock = 16000000;
#else
    uart_clock = clockfreq;
#endif 
    
    switch( databits )
    {
      case 5:
        config = UART_CONFIG_WLEN_5;
        break;
      case 6:
        config = UART_CONFIG_WLEN_6;
        break;
      case 7:
        config = UART_CONFIG_WLEN_7;
        break;
      default:
        config = UART_CONFIG_WLEN_8;
        break;
    }
    config |= ( stopbits == PLATFORM_UART_STOPBITS_1 ) ? UART_CONFIG_STOP_ONE : UART_CONFIG_STOP_TWO;
    if( parity == PLATFORM_UART_PARITY_EVEN )
      config |= UART_CONFIG_PAR_EVEN;
    else if( parity == PLATFORM_UART_PARITY_ODD )
      config |= UART_CONFIG_PAR_ODD;
    else if( parity == PLATFORM_UART_PARITY_MARK )
      config |= UART_CONFIG_PAR_ONE;
    else if( parity == PLATFORM_UART_PARITY_SPACE )
      config |= UART_CONFIG_PAR_ZERO;
    else
      config |= UART_CONFIG_PAR_NONE;

    MAP_UARTConfigSetExpClk( uart_base[ id ], uart_clock, baud, config );
    MAP_UARTConfigGetExpClk( uart_base[ id ], uart_clock, &baud, &config );

    MAP_UARTEnable( uart_base[ id ] );
  }
  return baud;
}

void platform_s_uart_send( unsigned id, u8 data )
{
  MAP_UARTCharPut( uart_base[ id ], data );
}

int platform_s_uart_recv( unsigned id, timer_data_type timeout )
{
  u32 base = uart_base[ id ];

  if( timeout == 0 )
    return MAP_UARTCharGetNonBlocking( base );

  return MAP_UARTCharGet( base );
}

int platform_s_uart_set_flow_control( unsigned id, int type )
{
  return PLATFORM_ERR;
}


// ****************************************************************************
// Timers

#define TIMER_MAX_COUNT	0xFFFFFFFF


#if defined( FORTM4C1294 )

const u32 timer_base[] = { TIMER0_BASE, TIMER1_BASE, TIMER2_BASE, TIMER3_BASE, 
                           TIMER4_BASE, TIMER5_BASE, TIMER6_BASE, TIMER7_BASE };

static const u32 timer_sysctl[] = { 
	SYSCTL_PERIPH_TIMER0, SYSCTL_PERIPH_TIMER1, SYSCTL_PERIPH_TIMER2, 
	SYSCTL_PERIPH_TIMER3, SYSCTL_PERIPH_TIMER4, SYSCTL_PERIPH_TIMER5,
  SYSCTL_PERIPH_TIMER6, SYSCTL_PERIPH_TIMER7 };

#elif defined( FORLM4F )

// FIXME LM4F has 12 timers (6 32 and 6 64 bit)

const u32 timer_base[] = { 	TIMER0_BASE, TIMER1_BASE, TIMER2_BASE, 
					TIMER3_BASE, TIMER4_BASE, TIMER5_BASE, 
					WTIMER0_BASE, WTIMER1_BASE, WTIMER2_BASE,
					WTIMER3_BASE, WTIMER4_BASE, WTIMER5_BASE };
// FIXME: Remove timers if using them for PWMs
// TODO: TIMER0 B and TIMER1 A, B output pins are PWMs for RGB LED - maybe should make them PWMs

//#define WTIMER_MAX_COUNT	((u64) 0xFFFFFFFFFFFFFFFF)

static const u32 timer_sysctl[] = { 
	SYSCTL_PERIPH_TIMER0, SYSCTL_PERIPH_TIMER1, SYSCTL_PERIPH_TIMER2, 
	SYSCTL_PERIPH_TIMER3, SYSCTL_PERIPH_TIMER4, SYSCTL_PERIPH_TIMER5, 
	SYSCTL_PERIPH_WTIMER0, SYSCTL_PERIPH_WTIMER1, SYSCTL_PERIPH_WTIMER2,
	SYSCTL_PERIPH_WTIMER3, SYSCTL_PERIPH_WTIMER4, SYSCTL_PERIPH_WTIMER5 };

#else
// Same on LM3S8962, LM3S6965, LM3S6918 and LM3S9B92 (4 timers)

// All possible LM3S timers defs
const u32 timer_base[] = { TIMER0_BASE, TIMER1_BASE, TIMER2_BASE, TIMER3_BASE };
static const u32 timer_sysctl[] = { SYSCTL_PERIPH_TIMER0, SYSCTL_PERIPH_TIMER1, SYSCTL_PERIPH_TIMER2, SYSCTL_PERIPH_TIMER3 };

#endif

/*
// FIXME: Attempt at error checking - not working yet
const unsigned num_timer_base = sizeof(timer_base)/sizeof(u32);
const unsigned num_timer_sysctl = sizeof(timer_sysctl)/sizeof(u32);

#if (NUM_TIMER > num_timer_base) || (NUM_TIMER > num_timer_sysctl)
#error Not all your timer base are belong to us.
#endif
*/

static void timers_init()
{
  unsigned i;

  ASSERT(NUM_TIMER <= sizeof(timer_base)/sizeof(u32));
  ASSERT(NUM_TIMER <= sizeof(timer_sysctl)/sizeof(u32));

  for( i = 0; i < NUM_TIMER; i ++ )
  {
    MAP_SysCtlPeripheralEnable(timer_sysctl[ i ]);
    MAP_TimerConfigure(timer_base[ i ], TIMER_CFG_PERIODIC);
    MAP_TimerEnable(timer_base[ i ], TIMER_A);
  }
}

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{
  timer_data_type final;
  u32 base = timer_base[ id ];

  final = TIMER_MAX_COUNT - ( ( ( u64 )delay_us * clockfreq ) / 1000000 );
  MAP_TimerLoadSet( base, TIMER_A, TIMER_MAX_COUNT );
  while( MAP_TimerValueGet( base, TIMER_A ) > final );
	// Fixme: Looks like it does busy waiting for timer, consider sleep? - WAIT_WHILE
}

timer_data_type platform_s_timer_op( unsigned id, int op, timer_data_type data )
{
  u32 res = 0;
  u32 base = timer_base[ id ];

  data = data;
  switch( op )
  {
    case PLATFORM_TIMER_OP_START:
      res = TIMER_MAX_COUNT;
      MAP_TimerControlTrigger(base, TIMER_A, false);
      MAP_TimerLoadSet( base, TIMER_A, TIMER_MAX_COUNT );
      break;

    case PLATFORM_TIMER_OP_READ:
      res = MAP_TimerValueGet( base, TIMER_A );
      break;

    case PLATFORM_TIMER_OP_SET_CLOCK:
    case PLATFORM_TIMER_OP_GET_CLOCK:
      res = clockfreq;
      break;

    case PLATFORM_TIMER_OP_GET_MAX_CNT:
      res = TIMER_MAX_COUNT;
      break;

  }
  return res;
}

u64 platform_timer_sys_raw_read()
{
  return MAP_SysTickPeriodGet() - 1 - MAP_SysTickValueGet();
}

void platform_timer_sys_disable_int()
{
  MAP_SysTickIntDisable();
}

void platform_timer_sys_enable_int()
{
  MAP_SysTickIntEnable();
}

timer_data_type platform_timer_read_sys()
{
  return cmn_systimer_get();
}

// Todo: Could be optimized for memory - Can only have 2 (or maybe 3) values (Cyclic, 1 Time, invalid)
u8 lm3s_timer_int_periodic_flag[ NUM_TIMER ];

int platform_s_timer_set_match_int( unsigned id, timer_data_type period_us, int type )
{
  u32 base = timer_base[ id ];
  u64 final;

  if( period_us == 0 )
  {
    MAP_TimerDisable( base, TIMER_A );
    MAP_TimerIntDisable( base, TIMER_TIMA_TIMEOUT );
    MAP_TimerIntClear( base, TIMER_TIMA_TIMEOUT );
    MAP_TimerLoadSet( base, TIMER_A, TIMER_MAX_COUNT );
    MAP_TimerEnable( base, TIMER_A );
    return PLATFORM_TIMER_INT_OK;
  }
  final = ( ( u64 )period_us * clockfreq ) / 1000000;
  if( final == 0 )
    return PLATFORM_TIMER_INT_TOO_SHORT;
  if( final > 0xFFFFFFFFULL )				// todo: check
    return PLATFORM_TIMER_INT_TOO_LONG;
  lm3s_timer_int_periodic_flag[ id ] = type;
  MAP_TimerDisable( base, TIMER_A );
  MAP_TimerIntClear( base, TIMER_TIMA_TIMEOUT );
  MAP_TimerLoadSet( base, TIMER_A, ( u32 )final - 1 );
  return PLATFORM_TIMER_INT_OK;
}

// ****************************************************************************
// PWMs
// Similar on LM3S8962 and LM3S6965
// LM3S6918 has no PWM
// LM4F230 has 2 PWM modules, each with 4 generators, 8 pins, 8 PWM_OUTs


#if ( NUM_PWM > 0 )
// #ifdef BUILD_PWM


// PWM Clock
#ifdef EMULATE_PWM
// TODO: Implement clock divisors (at moment just uses system clock)

#else // EMULATE_PWM

// SYSCTL div data and actual div factors
#ifdef FORTM4C1294

const static u32 pwm_div_ctl[] = { PWM_SYSCLK_DIV_1, PWM_SYSCLK_DIV_2, PWM_SYSCLK_DIV_4, PWM_SYSCLK_DIV_8,
  PWM_SYSCLK_DIV_16, PWM_SYSCLK_DIV_32, PWM_SYSCLK_DIV_64 };

#else

// Only for TM4C123x
const static u32 pwm_div_ctl[] = { SYSCTL_PWMDIV_1, SYSCTL_PWMDIV_2, SYSCTL_PWMDIV_4, SYSCTL_PWMDIV_8, 
	SYSCTL_PWMDIV_16, SYSCTL_PWMDIV_32, SYSCTL_PWMDIV_64 };

#endif // FORTM4C1294

const static u8 pwm_div_data[] = { 1, 2, 4, 8, 16, 32, 64 };

// Tivaware uses a different function for PWM clock set/get on TM4C123x vs other processors
static uint32_t AUX_GetPWMClock( uint32_t base )
{
#ifdef FORTM4C1294
return MAP_PWMClockGet( base );
#else
return MAP_SysCtlPWMClockGet( );
#endif
}

static void AUX_SetPWMClock( uint32_t base, uint32_t div )
{
#ifdef FORTM4C1294
  MAP_PWMClockSet( base, div );
#else
  MAP_SysCtlPWMClockSet( div );
#endif
}

#endif // EMULATE_PWM


// Emulated timers
#if defined( EMULATE_PWM )

static BOOL pwm_enabled[NUM_PWM] = { FALSE };

#if defined( FORLM4F120 ) 
const static u32 pwm_timer_base[] = { WTIMER0_BASE, WTIMER1_BASE, WTIMER2_BASE,
					WTIMER3_BASE, WTIMER4_BASE, WTIMER5_BASE };
// FIXME: Remove wide timers from timer area if using them for PWMs

const static u32 pwm_timer_sysctl[] = { 
	SYSCTL_PERIPH_WTIMER0, SYSCTL_PERIPH_WTIMER1, SYSCTL_PERIPH_WTIMER2,
	SYSCTL_PERIPH_WTIMER3, SYSCTL_PERIPH_WTIMER4, SYSCTL_PERIPH_WTIMER5 };
#endif

#endif //EMULATE_PWM



// Port/pin information for all channels
#if defined(FORLM3S1968)

  const static u32 pwm_ports[] =  { GPIO_PORTG_BASE, GPIO_PORTD_BASE, GPIO_PORTH_BASE, 
						GPIO_PORTH_BASE, GPIO_PORTF_BASE, GPIO_PORTF_BASE };
  const static u8 pwm_pins[] = { GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3 };

#elif defined(FORLM3S6965)

  const static u32 pwm_ports[] =  { GPIO_PORTF_BASE, GPIO_PORTD_BASE, GPIO_PORTB_BASE, 
						GPIO_PORTB_BASE, GPIO_PORTE_BASE, GPIO_PORTE_BASE };
  const static u8 pwm_pins[] = { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_1 };

#elif defined( ELUA_BOARD_SOLDERCORE ) && defined( FORLM3S9D92 )

  const static u32 pwm_ports[] =  { GPIO_PORTG_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE, 
						GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTC_BASE, GPIO_PORTC_BASE };
  const static u8 pwm_pins[] = { 	GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, 
						GPIO_PIN_6, GPIO_PIN_7,  GPIO_PIN_4,  GPIO_PIN_6 };
  const static u32 pwm_configs[] = { GPIO_PG0_PWM0, GPIO_PD1_PWM1, GPIO_PD2_PWM2, GPIO_PD3_PWM3, 
						GPIO_PE6_PWM4, GPIO_PE7_PWM5, GPIO_PC4_PWM6, GPIO_PC6_PWM7 };

#elif defined( FORLM3S9B92 ) || ( defined(FORLM3S9D92) && !defined( ELUA_BOARD_SOLDERCORE ) )

  const static u32 pwm_ports[] =  { GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE, 
						GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTC_BASE, GPIO_PORTC_BASE };
  const static u8 pwm_pins[] = { 	GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, 
						GPIO_PIN_6, GPIO_PIN_7,  GPIO_PIN_4,  GPIO_PIN_6 };
  const static u32 pwm_configs[] = { GPIO_PD0_PWM0, GPIO_PD1_PWM1, GPIO_PD2_PWM2, GPIO_PD3_PWM3, 
						GPIO_PE6_PWM4, GPIO_PE7_PWM5, GPIO_PC4_PWM6, GPIO_PC6_PWM7 };

#elif defined( FORLM4F120 ) 
// FIXME FORLM4F120 has no PWM, simulate PWM using timers
// TODO: At the moment just using wide timers, could look at possible using arbitrary timers (based on which pin want PWMd)

  const static u32 pwm_ports[] = {  GPIO_PORTC_BASE, GPIO_PORTC_BASE, GPIO_PORTD_BASE,
						GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE};
  const static u8 pwm_pins[] = {    GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_0, 
						GPIO_PIN_2, GPIO_PIN_4,  GPIO_PIN_6 };

  const static u32 pwm_configs[] = { GPIO_PC4_WT0CCP0, GPIO_PC6_WT1CCP0, GPIO_PD0_WT2CCP0, 
						GPIO_PD2_WT3CCP0, GPIO_PD4_WT4CCP0, GPIO_PD6_WT5CCP0};

#elif defined( FORTM4C1294 ) 

  const static u32 pwm_ports[] = { GPIO_PORTF_BASE, GPIO_PORTF_BASE, GPIO_PORTF_BASE, GPIO_PORTF_BASE, 
            GPIO_PORTG_BASE, GPIO_PORTG_BASE, GPIO_PORTK_BASE, GPIO_PORTK_BASE };
  const static u8 pwm_pins[] = { 	GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, 
						GPIO_PIN_0, GPIO_PIN_1,  GPIO_PIN_4,  GPIO_PIN_5 };
  const static u32 pwm_configs[] = { GPIO_PF0_M0PWM0, GPIO_PF1_M0PWM1, GPIO_PF2_M0PWM2, GPIO_PF3_M0PWM3,
            GPIO_PG0_M0PWM4, GPIO_PG1_M0PWM5, GPIO_PK4_M0PWM6, GPIO_PK5_M0PWM7 };

#elif defined( FORLM4F230 ) || defined( FORTM4C123G)

// Has two PWM modules - PWM_BASE0, PWM_BASE1 each with 4 generators, 8 pins, 8 PWM_OUTs
// TODO: Parameterize better (number of modules, number pins/module, etc.)
//   so get better error checking on various mappings,
//   so fewer cases for different CPUs 
//   make easier to add a new CPU

// TODO: Maybe way to chose alternative pins for PWMS (PWM6 C4 vs D0, PWM7 C5 vs D1, PWM10 A6 vs E4, PWM11 A7 vs E5)
// However - the alternate pins are also handled by other PWMs D0 is PWM8, etc., so may not need to handle map

// Module0, 8 PWMS PORTB:4-7, E:4-5, C:4-5 [or] D:0-1, fault on D:2 or D:6 or F:2
// Module1, 8 PWMS PORTD:0-1, A:6-7 [or] E:4-5, F:0-3, fault on F:4

  const static u32 pwm_ports[] = {  GPIO_PORTB_BASE, GPIO_PORTB_BASE, GPIO_PORTB_BASE, GPIO_PORTB_BASE, 
						GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTC_BASE, GPIO_PORTC_BASE, 
						GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTA_BASE, GPIO_PORTA_BASE,
						GPIO_PORTF_BASE, GPIO_PORTF_BASE, GPIO_PORTF_BASE, GPIO_PORTF_BASE  };
  const static u8 pwm_pins[] = {    GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_4, GPIO_PIN_5,
						GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_5,
						GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_6, GPIO_PIN_7,
						GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3 };

  const static u32 pwm_configs[] = { GPIO_PB6_M0PWM0, GPIO_PB7_M0PWM1, GPIO_PB4_M0PWM2, GPIO_PB5_M0PWM3, 
						GPIO_PE4_M0PWM4, GPIO_PE5_M0PWM5, GPIO_PC4_M0PWM6, GPIO_PC5_M0PWM7, 
						GPIO_PD0_M1PWM0, GPIO_PD1_M1PWM1, GPIO_PA6_M1PWM2, GPIO_PA7_M1PWM3, 
						GPIO_PF0_M1PWM4, GPIO_PF1_M1PWM5, GPIO_PF2_M1PWM6, GPIO_PF3_M1PWM7 };

#elif defined( FORLM3S6918 )

// TODO: So why is this here, why doesn't the 6918 just not build the PWM code?
  const static u32 pwm_ports[] = {};
  const static u8 pwm_pins[] = {};

#else

  const static u32 pwm_ports[] =  { GPIO_PORTF_BASE, GPIO_PORTG_BASE, GPIO_PORTB_BASE, 
						GPIO_PORTB_BASE, GPIO_PORTE_BASE, GPIO_PORTE_BASE };
  const static u8 pwm_pins[] = { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_1 };

#endif


// PWM modules

// Module has at most 8 PWM channels
#define PWM_PER_MOD 8
// TODO: Use this parameter instead of >> 3

#if (PWM_PER_MOD * 2 < NUM_PWM)
#error Need to extend support for more than 2 PWM modules
#elif (PWM_PER_MOD < NUM_PWM)
#define NUM_PWM_MOD 2
#else
#define NUM_PWM_MOD 1
#endif

#if (NUM_PWM_MOD == 2)
  const static u32 pwm_base[] = { PWM0_BASE, PWM1_BASE };
#else

#if defined(PWM_BASE)
  const static u32 pwm_base[] = { PWM_BASE };
#else
  const static u32 pwm_base[] = { PWM0_BASE };
#endif // PWM_BASE

#endif // NUM_PWM_MOD

// FIXME: Range checks to be sure arrays filled in - need to fix syntax
/*
#if ((sizeof(pwm_base)/sizeof(u32)) < (NUM_PWM >> 3))
#error Not enough pwm_base
#endif
*/


// PWM generators
// Up to 4 generators per module

#if (4 == NUM_PWM_GEN)
  const static u16 pwm_gens[] = { PWM_GEN_0, PWM_GEN_1, PWM_GEN_2, PWM_GEN_3 };

#elif (8 == NUM_PWM_GEN)
  const static u16 pwm_gens[] = { PWM_GEN_0, PWM_GEN_1, PWM_GEN_2, PWM_GEN_3,
    PWM_GEN_0, PWM_GEN_1, PWM_GEN_2, PWM_GEN_3 };
    //MODULE1 PWM_GEN_2 Covers M1PWM4 and M1PWM5
    //MODULE1 PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc

  // TODO: Since 5 through 8 are repeats, could use modulo on array index instead

#elif (3 == NUM_PWM_GEN)
  const static u16 pwm_gens[] = { PWM_GEN_0, PWM_GEN_1, PWM_GEN_2 };

#elif defined(EMULATE_PWM)
// Don't need NUM_PWM_GEN if emulating PWM

#else

#error NUM_PWM_GEN not defined

#endif


/*
#if (sizeof(pwm_gens)/sizeof(u16) < (NUM_PWM/2))
#error Not enough pwm_gens
#endif
//  assert( id >> 1 < sizeof(pwm_gens)/ sizeof( u16 ));
*/


// PWM outputs
#ifndef EMULATE_PWM
#if (NUM_PWM > 6)
const static u16 pwm_outs[] = { PWM_OUT_0, PWM_OUT_1, PWM_OUT_2, PWM_OUT_3, 
					PWM_OUT_4, PWM_OUT_5, PWM_OUT_6, PWM_OUT_7};
/*
#elif defined( FORLM4F230 ) 
const static u16 pwm_outs[] = { PWM_OUT_0, PWM_OUT_1, PWM_OUT_2, PWM_OUT_3, 
					PWM_OUT_4, PWM_OUT_5, PWM_OUT_6, PWM_OUT_7,
          PWM_OUT_0, PWM_OUT_1, PWM_OUT_2, PWM_OUT_3, 
					PWM_OUT_4, PWM_OUT_5, PWM_OUT_6, PWM_OUT_7
          };

// since second module same PWM_OUTs as first, 
// Use array covering one module, and modulo in access code
*/

#else
const static u16 pwm_outs[] = { PWM_OUT_0, PWM_OUT_1, PWM_OUT_2, PWM_OUT_3, PWM_OUT_4, PWM_OUT_5 };
#endif

/*
#if (sizeof(pwm_outs)/sizeof(u16) < NUM_PWM)
#error Not enough pwm_outs
#endif
*/

#endif // EMULATE_PWM


// TODO: What do these do on a system with no PWMs?
static void pwms_init()
{
#if defined( EMULATE_PWM )

#warning PWM Emulation not written

//TODO: Write me!!
//  for(id = 0; id < NUM_PWM; id++)
//	pwm_enabled[id] = FALSE;

#else

  unsigned i;

#if (NUM_PWM_MOD == 2)
// TODO: Generalize to more than 2 PWM modules
  MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM0 );
  MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM1 );

#else
#ifdef SYSCTL_PERIPH_PWM
  MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM );
#else
  MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM0 );
#endif  
#endif // NUM_PWM_MOD
  
  MAP_SysCtlPWMClockSet( SYSCTL_PWMDIV_1 );

//  for ( i = 0; i < NUM_PWM; i++)
//    MAP_PWMGenConfigure( pwm_base[i >> 3], pwm_gens[ i >> 1 ], PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC );
   
// PWMGenConfigure was in platform_pwm_setup, and it seemed to work on EK-LM3S8962
// However the Stellarisware documentation says that it leaves the generator disabled.
// The Stellarisware code does not appear to disable the generator (version 9453)
// However processor may be using ROM code.
// Having problems running on the Tiva launchpad.
// TODO: Check Tiva LP to see if running a start after configure fixes the problem (or moving configure out of platform_pwm_setup).

// FIXME: eLua documentation does not specify what (if any) effect platform_pwm_setup should have on the start/stoped status of the PWM.
// The example code seems to assume that platform_pwm_setup will not stop the generator.

// FIXME: Documentation says that this leaves the generator disabled.  So would have to call start after each time change period.
//   That is not how the example program works.  
// However, I thought this code and the example worked on the EK-LM3S8962
//
// Although the documentation says that this leaves the PWM disabled, I do not see in the library code where that is the case.
//   However - since may be using the library in ROM, the code it is running may not be the code in the library source.
//     (i.e. they may have fixed the Tivaware code to match documentation)
//
// Check eLua documentation (what is it supposed to do) - unspecified.  
// Check how functions work on other platforms - on AVR32 and LPC17xx it looks like platform_pwm_setup does not stop the PWM.
  
#endif // EMULATE_PWM
}

// Return the PWM clock
u32 platform_pwm_get_clock( unsigned id )
{
#ifdef EMULATE_PWM
  return clockfreq;

#else
  unsigned i;
  u32 clk;

  clk = AUX_GetPWMClock( pwm_base[id >> 3] );
  for( i = 0; i < sizeof( pwm_div_ctl ) / sizeof( u32 ); i ++ )
    if( clk == pwm_div_ctl[ i ] )
      break;
  return clockfreq / pwm_div_data[ i ];
#endif // EMULATE_PWM
}

// Set the PWM clock
u32 platform_pwm_set_clock( unsigned id, u32 clock )
{
#ifdef EMULATE_PWM
  return 0;

#else
  unsigned i, min_i;
  
  ASSERT(sizeof(pwm_div_ctl)/sizeof(u32) == sizeof(pwm_div_data)/sizeof(u8));
  
  for( i = min_i = 0; i < sizeof( pwm_div_data ) / sizeof( u8 ); i ++ )
    if( ABSDIFF( clock, clockfreq / pwm_div_data[ i ] ) < ABSDIFF( clock, clockfreq / pwm_div_data[ min_i ] ) )
      min_i = i;
  AUX_SetPWMClock( pwm_base[id >> 3], pwm_div_ctl[ min_i ] );

// FIXME: Doesn't return actual clock - might be better to read clock back  
  return clockfreq / pwm_div_data[ min_i ];
#endif // EMULATE_PWM
}


//duty - PWM channel duty cycle, specified as integer from 0 to 100 (percent).
u32 platform_pwm_setup( unsigned id, u32 frequency, unsigned duty )
{
  u32 pwmclk = platform_pwm_get_clock( id );
  u32 period;

#if defined( FORLM3S9B92 ) || defined( FORLM3S9D92 ) || defined( FORLM4F )
  MAP_GPIOPinConfigure( pwm_configs[ id ] );
#endif

#ifdef EMULATE_PWM

// Assumes that GPIO for pins (sysctrl) have already been enabled by PIO module

//TODO: Be sure timer disabled before making changes
//  if allow timer to be used as PWM or timer, then need to keep track of use
//TODO: Maybe belong in general init 
  if (!pwm_enabled[id]){	// Initialize timer if not already set up
    MAP_GPIOPinTypeTimer( pwm_ports[id], pwm_pins[id] ); 
    MAP_SysCtlPeripheralEnable( pwm_timer_base[id] );	
    MAP_TimerConfigure( pwm_timer_base[id], TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM );
    pwm_enabled[id] = TRUE;
  }

  // Compute period
  period = pwmclk / frequency;
  MAP_TimerLoadSet( pwm_timer_base[id], TIMER_A, period);
  MAP_TimerMatchSet( pwm_timer_base[id], TIMER_A, ( period * duty ) / 100);
#else

  // Set pin as PWM
  MAP_GPIOPinTypePWM( pwm_ports[ id ], pwm_pins[ id ] );

  MAP_PWMGenConfigure( pwm_base[id >> 3], pwm_gens[ id >> 1 ], PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC );

  // Compute period
  period = pwmclk / frequency;
  // Set the period
    
  MAP_PWMGenPeriodSet( pwm_base[id >> 3], pwm_gens[ id >> 1 ], period );
  // Set duty cycle
  MAP_PWMPulseWidthSet( pwm_base[id >> 3], pwm_outs[ id % PWM_PER_MOD ], ( period * duty ) / 100 );
//  MAP_PWMPulseWidthSet( pwm_base[id >> 3], pwm_outs[ id ], ( period * duty ) / 100 );
  // Return actual frequency
#endif // EMULATE_PWM

  return pwmclk / period;
}

void platform_pwm_start( unsigned id )
{
#ifdef EMULATE_PWM
//FIXME: what does PWMOutputState do?  - // Turn on the Output pins
    MAP_TimerEnable(pwm_timer_base[id], TIMER_A);
#else
  MAP_PWMGenEnable( pwm_base[id >> 3], pwm_gens[ id >> 1 ] );  // Enable the PWM generator
  MAP_PWMOutputState( pwm_base[id >> 3], pwm_outs[ id % PWM_PER_MOD ], true );      // Turn on the Output pin
//  MAP_PWMOutputState( pwm_base[id >> 3], pwm_outs[ id ], true );      // Turn on the Output pin
#endif // EMULATE_PWM
}

void platform_pwm_stop( unsigned id )
{
#ifdef EMULATE_PWM
//FIXME: what does PWMOutputState do?
    MAP_TimerDisable(pwm_timer_base[id], TIMER_A);
#else
  MAP_PWMOutputState( pwm_base[id >> 3], pwm_outs[ id % PWM_PER_MOD ], false );      // Turn off the Output pin
//  MAP_PWMOutputState( pwm_base[id >> 3], pwm_outs[ id ], false );      // Turn off the Output pin
  MAP_PWMGenDisable( pwm_base[id >> 3], pwm_gens[ id >> 1 ] );
#endif // EMULATE_PWM
}

//#endif // BUILD_PWM
#endif // NUM_PWM > 0


// *****************************************************************************
// ADC specific functions and variables

#ifdef BUILD_ADC

// Pin configuration if necessary
#if defined( FORLM3S9B92 ) || defined( FORLM3S9D92 )
  const static u32 adc_ports[] =  { GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTE_BASE,
                                    GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE,
                                    GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTB_BASE, GPIO_PORTB_BASE,
                                    GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE };
                                    
  const static u8 adc_pins[] =    { GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_4,
                                    GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_4,
                                    GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_5,
                                    GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0 };

  const static u32 adc_ctls[] = { ADC_CTL_CH0, ADC_CTL_CH1, ADC_CTL_CH2, ADC_CTL_CH3,
                                  ADC_CTL_CH4, ADC_CTL_CH5, ADC_CTL_CH6, ADC_CTL_CH7,
                                  ADC_CTL_CH8, ADC_CTL_CH9, ADC_CTL_CH10, ADC_CTL_CH11,
                                  ADC_CTL_CH12, ADC_CTL_CH13, ADC_CTL_CH14, ADC_CTL_CH15 };

#define ADC_PIN_CONFIG

#elif defined( FORTM4C1294 )
// 20 Inputs, plust therm channel

  const static u32 adc_ports[] = { GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTE_BASE, 
                                   GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE,
                                   GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTB_BASE, GPIO_PORTB_BASE,
                                   GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE,
                                   GPIO_PORTK_BASE, GPIO_PORTK_BASE, GPIO_PORTK_BASE, GPIO_PORTK_BASE };

  const static u8 adc_pins[] =  {  GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0,
                                   GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_4,
                                   GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_4, GPIO_PIN_5,
                                   GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0,
                                   GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3 };

  const static u32 adc_ctls[] = { ADC_CTL_CH0, ADC_CTL_CH1, ADC_CTL_CH2, ADC_CTL_CH3,
                                  ADC_CTL_CH4, ADC_CTL_CH5, ADC_CTL_CH6, ADC_CTL_CH7,
                                  ADC_CTL_CH8, ADC_CTL_CH9, ADC_CTL_CH10, ADC_CTL_CH11,
                                  ADC_CTL_CH12, ADC_CTL_CH13, ADC_CTL_CH14, ADC_CTL_CH15,
                                  ADC_CTL_CH16, ADC_CTL_CH17, ADC_CTL_CH18, ADC_CTL_CH19, 
                                  ADC_CTL_TS };

#define ADC_PIN_CONFIG

#elif defined( FORLM4F )

// 11 Pins - AIN0 .. AIN 11

  const static u32 adc_ports[] =  { GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTE_BASE,
                                    GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE,
                                    GPIO_PORTE_BASE, GPIO_PORTE_BASE, GPIO_PORTB_BASE, GPIO_PORTB_BASE };
                                    
  const static u8 adc_pins[] =    { GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0,
                                    GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0,
                                    GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_4, GPIO_PIN_5 };

// Do pin maping

  const static u32 adc_ctls[] = { ADC_CTL_CH0, ADC_CTL_CH1, ADC_CTL_CH2, ADC_CTL_CH3,
                                  ADC_CTL_CH4, ADC_CTL_CH5, ADC_CTL_CH6, ADC_CTL_CH7,
                                  ADC_CTL_CH8, ADC_CTL_CH9, ADC_CTL_CH10, ADC_CTL_CH11,
                                  ADC_CTL_TS };

#define ADC_PIN_CONFIG

#else

const static u32 adc_ctls[] = { ADC_CTL_CH0, ADC_CTL_CH1, ADC_CTL_CH2, ADC_CTL_CH3 };
#endif

// ToDo: Have 2 ADC, could figure how to split the work
// ToDo: Not sure how many ADC ints should have.  (Is it one per ADC or ?)

const static u32 adc_ints[] = { INT_ADC0SS0, INT_ADC0SS1, INT_ADC0SS2, INT_ADC0SS3 };


//ToDo: Add error checking for NUM_ADC
//#if (NUM_ADC > sizeof(adc_ctls)/sizeof(u32))
//#error More ADC specified than hardware has.
//#endif


int platform_adc_check_timer_id( unsigned id, unsigned timer_id )
{
  return ( ( timer_id >= ADC_TIMER_FIRST_ID ) && ( timer_id < ( ADC_TIMER_FIRST_ID + ADC_NUM_TIMERS ) ) );
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
    MAP_ADCSequenceDisable( ADC0_BASE, d->seq_id );
    d->running = 0;
  }
}

// Handle ADC interrupts
void ADCIntHandler( void )
{
  u32 tmpbuff[ NUM_ADC ];
  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  elua_adc_ch_state *s;

  MAP_ADCIntClear( ADC0_BASE, d->seq_id );
  MAP_ADCSequenceDataGet( ADC0_BASE, d->seq_id, tmpbuff );
  
  d->seq_ctr = 0;
  
  // Update smoothing and/or write to buffer if needed
  while( d->seq_ctr < d->seq_len )
  {
    s = d->ch_state[ d->seq_ctr ];
    d->sample_buf[ d->seq_ctr ] = ( u16 )tmpbuff[ d->seq_ctr ];
    s->value_fresh = 1; // Mark sample as fresh
    
    // Fill in smoothing buffer until warmed up
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
    
    d->seq_ctr++;
  }
  d->seq_ctr = 0;
  
  // Only attempt to refresh sequence order if still running
  // This allows us to "cache" an old sequence if all channels
  // finish at the same time
  if ( d->running == 1 )
    adc_update_dev_sequence( 0 );
  
  if ( d->clocked == 0 && d->running == 1 )
  {
    // Need to manually fire off sample request in single sample mode
    MAP_ADCProcessorTrigger( ADC0_BASE, d->seq_id );
  }
}

static void adcs_init()
{
  unsigned id;
  elua_adc_dev_state *d = adc_get_dev_state( 0 );

// FIXME: Adjust assertions for temperature channel, which has no port/pin
//  ASSERT(NUM_ADC <= sizeof(adc_ports)/sizeof(u32));
//  ASSERT(NUM_ADC <= sizeof(adc_pins)/sizeof(adc_pins[0]));

  ASSERT(NUM_ADC <= sizeof(adc_ctls)/sizeof(u32));

  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

  // Try ramping up max sampling rate
// FIXME: The following function and its arguments are not defined in Tivaware 
//   (however not mentioned in the conversion docs), not sure what replaces their function
#warning MAP_SysCtlADCSpeedSet - missing from Tivaware see what should replace
//  MAP_SysCtlADCSpeedSet(SYSCTL_ADCSPEED_500KSPS);
//  MAP_SysCtlADCSpeedSet(SYSCTL_ADCSPEED_1MSPS);
  
  for( id = 0; id < NUM_ADC; id ++ )
    adc_init_ch_state( id );

  // Perform sequencer setup
  platform_adc_set_clock( 0, 0 );
  MAP_ADCIntEnable( ADC0_BASE, d->seq_id );
  MAP_IntEnable( adc_ints[ 0 ] ); // Enable sequencer 0 int
}

// Note: If use ADCClockConfigSet - 16MHz final freq on 123, between 16 and 32 MHz on 1294

u32 platform_adc_set_clock( unsigned id, u32 frequency )
{
  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  
  // Make sure sequencer is disabled before making changes
  MAP_ADCSequenceDisable( ADC0_BASE, d->seq_id );
  
  if ( frequency > 0 )
  {
    d->clocked = 1;
    // Set sequence id to be triggered repeatedly, with priority id
    MAP_ADCSequenceConfigure( ADC0_BASE, d->seq_id, ADC_TRIGGER_TIMER, d->seq_id );

    // Set up timer trigger
    MAP_TimerLoadSet( timer_base[ d->timer_id ], TIMER_A, clockfreq / frequency );
    frequency = clockfreq / MAP_TimerLoadGet( timer_base[ d->timer_id ], TIMER_A );
  }
  else
  {
    d->clocked = 0;
    // Conversion will run back-to-back until required samples are acquired
    MAP_ADCSequenceConfigure( ADC0_BASE, d->seq_id, ADC_TRIGGER_PROCESSOR, d->seq_id ) ;
  }
    
  return frequency;
}


int platform_adc_update_sequence( )
{  
  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  
  MAP_ADCSequenceDisable( ADC0_BASE, d->seq_id );
  
  // NOTE: seq ctr should have an incrementer that will wrap appropriately..
  d->seq_ctr = 0; 
  while( d->seq_ctr < d->seq_len-1 )
  {
    MAP_ADCSequenceStepConfigure( ADC0_BASE, d->seq_id, d->seq_ctr, adc_ctls[ d->ch_state[ d->seq_ctr ]->id ] );
#ifdef ADC_PIN_CONFIG
    if (adc_ctls[ d->ch_state[ d->seq_ctr ]->id ] != ADC_CTL_TS)	// Temperature sensor channel does not have pin
        MAP_GPIOPinTypeADC( adc_ports[ d->ch_state[ d->seq_ctr ]->id ], adc_pins[ d->ch_state[ d->seq_ctr ]->id ] );
#endif
    d->seq_ctr++;
  }
  MAP_ADCSequenceStepConfigure( ADC0_BASE, d->seq_id, d->seq_ctr, ADC_CTL_IE | ADC_CTL_END | adc_ctls[ d->ch_state[ d->seq_ctr ]->id ] );
#ifdef ADC_PIN_CONFIG
  if (adc_ctls[ d->ch_state[ d->seq_ctr ]->id ] != ADC_CTL_TS)	// Temperature sensor channel does not have pin
    MAP_GPIOPinTypeADC( adc_ports[ d->ch_state[ d->seq_ctr ]->id ], adc_pins[ d->ch_state[ d->seq_ctr ]->id ] );
#endif
  d->seq_ctr = 0;
  
  MAP_ADCSequenceEnable( ADC0_BASE, d->seq_id );
      
  return PLATFORM_OK;
}


int platform_adc_start_sequence()
{ 
  elua_adc_dev_state *d = adc_get_dev_state( 0 );
  
  if( d->running != 1 )
  {
    adc_update_dev_sequence( 0 );

    MAP_ADCSequenceEnable( ADC0_BASE, d->seq_id );
    d->running = 1;

    if( d->clocked == 1 )
    {
      MAP_TimerControlTrigger(timer_base[d->timer_id], TIMER_A, true);
      MAP_TimerEnable(timer_base[d->timer_id], TIMER_A);
    }
    else
    {
      MAP_ADCProcessorTrigger( ADC0_BASE, d->seq_id );
    }
  }
  
  return PLATFORM_OK;
}

#endif // ifdef BUILD_ADC


// ****************************************************************************
// Analog comparator
//
// TODO: Add code for comparators

#ifdef BUILD_COMP

#warning Building comparators

#if defined( FORTM4C1294 )

// TM4C1294 - 3 comparators
// C0, positive input PC6, negative input, PC7, output PD0 or PL2
// C1, positive input PC5, negative input, PC4, output PD1 or PL3
// C2, positive input PP0, negative input, PP1, output PD2


const static u32 comp_in_ports[] =  { GPIO_PORTC_BASE, GPIO_PORTC_BASE, GPIO_PORTP_BASE };
const static u8 comp_in_pins[] =    { (GPIO_PIN_6| GPIO_PIN_7), (GPIO_PIN_5 | GPIO_PIN_4), (GPIO_PIN_0 | GPIO_PIN_1) };

// TODO: Handle alternative pin mappings
const static u32 comp_out_ports[] =  { GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE };
const static u8 comp_out_pins[] =    { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2 };

static const u32 comp_pin_out[] = { GPIO_PD0_C0O, GPIO_PD1_C1O, GPIO_PD2_C2O };

// FIXME: Think comp_ctls is redundant/unneeded
const static u32 comp_ctls[] = { 0, 1, 2 };
// all comparators use same base - COMP_BASE

const static u32 comp_ints[] = { INT_COMP0, INT_COMP1, INT_COMP2 };

#elif defined( FORLM4F )

// LM4F120 - 2 comparators
// C0, positive input PC6, negative input, PC7, output PF0
// C1, positive input PC5, negative input, PC4, output PF1

const static u32 comp_in_ports[] =  { GPIO_PORTC_BASE, GPIO_PORTC_BASE };
const static u8 comp_in_pins[] =    { GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_5 | GPIO_PIN_4 };

// FIXME: Need special handling to reprogram Port F0, also note F1 is chanel of LED
const static u32 comp_out_ports[] = { GPIO_PORTF_BASE, GPIO_PORTF_BASE };
const static u8 comp_out_pins[] =   { GPIO_PIN_0, GPIO_PIN_1 };

static const u32 comp_pin_out[] =   { GPIO_PF0_C0O, GPIO_PF1_C1O };

// FIXME: Think comp_ctls is redundant/unneeded
const static u32 comp_ctls[] = { 0, 1 };
// all comparators use same base - COMP_BASE

const static u32 comp_ints[] = { INT_COMP0, INT_COMP1 };

#else

#error Comparator data not set up for this CPU

#endif // FORLM4F

// Predefined reference values, in order
#warning Comparator - need to fill in voltage levels and codes

const static u32 comp_refs[] = {};	// Voltage bins
const static u32 comp_ref_codes[] = {};	// Codes used by library

#define NUM_COMP_REFS	(sizeof(comp_refs)/sizeof(u32))


// TODO: What can you do with a comparator 
// Generate an output
// Generate an interrupt
// Trigger ADC

// Compare to pin or to internal reference

// Helper functions

static void comps_init()
{
  ASSERT(NUM_CMP <= sizeof(comp_in_ports)/sizeof(u32));
  ASSERT(NUM_CMP <= sizeof(comp_out_ports)/sizeof(u32));

  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_COMP0);
}

// Translate whatever voltage reference scheme eLua uses to platform specific
// FIXME: for now vref is reference voltage * 1000 (maybe should make it * 1024)
static unsigned long comp_ref_encode(unsigned vref)
{
#warning Comparator - need to write comp_ref_encode
// Assuming that bins are of fairly uniform width - use an index search
/* FIXME: Pseudocode
n bins between VMIN and VMAX
	if vref < VMIN -> 0
	if vref > VMAX -> NUM_COMP_REFS-1
	i = ((vref - VMIN) * NUM_COMP_REFS )/(VMAX-VMIN);	// Want integer division
	ASSERT (comp_refs[i] <= vref <= comp_refs[i+1]);
	return i;			// Actually, return whichever is nearer to vref
*/
}

// Translate back from platform voltage ref to eLua 
static unsigned comp_ref_decode(unsigned long refcode)
{
	return comp_refs[i];
}

static unsigned comp_ref_index = 0;


// This platform only allows one vref - should interface allow setting vref on a per comparator basis?

// Set reference voltage, returns voltage set
unsigned platform_comp_ref_set(unsigned id, unsigned vref)
{
	unsigned long refindex;

	comp_ref_index = comp_vref_encode( vref);
	MAP_ComparatorRefSet(COMP_BASE, comp_ref_codes[comp_ref_index]);
	return comp_vref_decode(comp_ref_index);
}

// Options - invert or normal, reference Comp+, Comp0+ or Internal
// Int trigger: High, Low, Rise, Fall, Both
// output on pin, or not
int platform_comp_setup(unsigned id)
{
  // Todo: Map pins
  MAP_GPIOPinTypeComparator( comp_in_ports[ id ], comp_in_pins[ id ] );
// ?? does it need pin configures? - have not found the macros in pin_map

  // Only do these if output on pin
  MAP_GPIOPinTypeComparator( comp_out_ports[ id ], comp_out_pins[ id ] );
  MAP_GPIOPinConfigure( comp_pin_out[ id ] );

  MAP_ComparatorConfigure(COMP_BASE, id, (COMP_TRIG_NONE | COMP_ASRCP_REF | COMP_OUTPUT_NORMAL) );
    // NO ADC trigger, use internal reference, non-inverted output
  return PLATFORM_OK;
}

int platform_comp_stop(unsigned id)
{
#warning platform_comp_stop not written
  return PLATFORM_ERR;
}

// Read reference voltage
unsigned platform_comp_ref_get(unsigned id)
{
	return comp_vref_decode(comp_ref_index);
}

// Read comparator result (boolean)
unsigned platform_comp_value_get(unsigned id)
{
	return MAP_ComparatorValueGet(COMP_BASE, id);
}


#endif // ifdef BUILD_COMP


// ****************************************************************************
// Support for specific onboard devices on 
// Texas Instruments / Luminary Micro kits.
//


// ****************************************************************************
// Ethernet functions

#ifdef BUILD_UIP
#ifdef FORLM4F


//*****************************************************************************
//
// Ethernet DMA descriptors.
//
// The MAC hardware needs a minimum of 3 receive descriptors to operate.
//
//*****************************************************************************
#define NUM_TX_DESCRIPTORS 3
#define NUM_RX_DESCRIPTORS 3

tEMACDMADescriptor g_psRxDescriptor[NUM_TX_DESCRIPTORS];
tEMACDMADescriptor g_psTxDescriptor[NUM_RX_DESCRIPTORS];

uint32_t g_ui32RxDescIndex;
uint32_t g_ui32TxDescIndex;

//*****************************************************************************
//
// Transmit and receive buffers. 
// These will typically be allocated within your network stack somewhere.
//
//*****************************************************************************

// TODO: Is there way to combine buffer use?
// Sort of counterproductive to have these extra buffers, and be using uip

// FIXME: Tune these in relation to UIP_CONF_BUFFER_SIZE
// Receive buffer should be same size as UIP buffer
// Transmit buffer should be larger than UIP buffer
// In example code they used 
// RX, TX buffer 1536, and UIP_CONF_BUFFER_SIZE        1600
#define RX_BUFFER_SIZE 1536
#define TX_BUFFER_SIZE 1536
uint8_t g_pui8RxBuffer[RX_BUFFER_SIZE];
uint8_t g_pui8TxBuffer[TX_BUFFER_SIZE];

//*****************************************************************************
//
// A set of flags.  The flag bits are defined as follows:
//
//     0 -> An indicator that a SysTick interrupt has occurred.
//     1 -> An RX Packet has been received.
//     2 -> A TX packet DMA transfer is pending.
//     3 -> A RX packet DMA transfer is pending.
//
//*****************************************************************************
/*
#define FLAG_SYSTICK            0
#define FLAG_RXPKT              1
#define FLAG_TXPKT              2
#define FLAG_RXPKTPEND          3
static volatile uint32_t g_ui32Flags;
*/

// Helper functions


//*****************************************************************************
//
// Transmit a packet from the supplied buffer. 
// pui8Buf points to the Ethernet frame to send and 
// i32BufLen contains the number of bytes in the frame.
//
//*****************************************************************************
static int32_t PacketTransmit(uint32_t ui32Base, uint8_t *pui8Buf, int32_t i32BufLen)
{
  int_fast32_t i32Loop;

  ASSERT(ui32Base == EMAC0_BASE);
  ASSERT(pui8Buf != 0);
  ASSERT(i32BufLen >= 0);

  //
  // Indicate that a packet is being sent.
  //
//  HWREGBITW(&g_ui32Flags, FLAG_TXPKT) = 1;

  //
  // Wait for the previous packet to be transmitted.
  //
  // TODO: busywait macro - WAIT_WHILE
  while(g_psTxDescriptor[g_ui32TxDescIndex].ui32CtrlStatus &
        DES0_TX_CTRL_OWN)
  {
  }

  //
  // Check that we're not going to overflow the transmit buffer.  
  // uIP buffer should be smaller than the DMA transmit buffer.
  //
  if(i32BufLen > TX_BUFFER_SIZE)
  {
    i32BufLen = TX_BUFFER_SIZE;
  }

  //
  // Copy the packet data into the transmit buffer.
  //
  for(i32Loop = 0; i32Loop < i32BufLen; i32Loop++)
  {
    g_pui8TxBuffer[i32Loop] = pui8Buf[i32Loop];
  }

  //
  // Move to the next descriptor.
  //
  g_ui32TxDescIndex++;
  if(g_ui32TxDescIndex == NUM_TX_DESCRIPTORS)
  {
    g_ui32TxDescIndex = 0;
  }

  //
  // Fill in the packet size and tell the transmitter to start work.
  //
  g_psTxDescriptor[g_ui32TxDescIndex].ui32Count = (uint32_t)i32BufLen;
  g_psTxDescriptor[g_ui32TxDescIndex].ui32CtrlStatus =
      (DES0_TX_CTRL_LAST_SEG | DES0_TX_CTRL_FIRST_SEG |
       DES0_TX_CTRL_INTERRUPT | DES0_TX_CTRL_IP_ALL_CKHSUMS |
       DES0_TX_CTRL_CHAINED | DES0_TX_CTRL_OWN);

  //
  // Tell the DMA to reacquire the descriptor now that we've filled it in.
  //
  MAP_EMACTxDMAPollDemand(EMAC0_BASE);

  //
  // Return the number of bytes sent.
  //
  return(i32BufLen);
}

//*****************************************************************************
//
// Read a packet from the DMA receive buffer into the supplied packet buffer.
//
//*****************************************************************************
int32_t
PacketReceive(uint32_t ui32Base, uint8_t *pui8Buf, int32_t i32BufLen)
{
  int_fast32_t i32FrameLen, i32Loop;

  //
  // Check the arguments.
  //
  ASSERT(ui32Base == EMAC0_BASE);
  ASSERT(pui8Buf != 0);
  ASSERT(i32BufLen > 0);

  //
  // By default, we assume we got a bad frame.
  //
  i32FrameLen = 0;

  //
  // Make sure that we own the receive descriptor.
  //
  if(!(g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus & DES0_RX_CTRL_OWN))
  {
    //
    // We own the receive descriptor so check to see if it contains a valid
    // frame.  Look for a descriptor error, indicating that the incoming
    // packet was truncated or, if this is the last frame in a packet,
    // the receive error bit.
    //
    if(!(g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus &
         DES0_RX_STAT_ERR))
    {
        //
        // We have a valid frame so copy the content to the supplied
        // buffer. First check that the "last descriptor" flag is set.  We
        // sized the receive buffer such that it can always hold a valid
        // frame so this flag should never be clear at this point but...
        //
        if(g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus &
           DES0_RX_STAT_LAST_DESC)
        {
          i32FrameLen =
            ((g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus &
              DES0_RX_STAT_FRAME_LENGTH_M) >> DES0_RX_STAT_FRAME_LENGTH_S);

          //
          // Sanity check.  This shouldn't be required since we sized the
          // uIP buffer such that it's the same size as the DMA receive
          // buffer but, just in case...
          //
          if(i32FrameLen > i32BufLen)
          {
            i32FrameLen = i32BufLen;
          }

          //
          // Copy the data from the DMA receive buffer into the provided
          // frame buffer.
          //
          for(i32Loop = 0; i32Loop < i32FrameLen; i32Loop++)
          {
            pui8Buf[i32Loop] = g_pui8RxBuffer[i32Loop];
          }
        }
    }

    //
    // Move on to the next descriptor in the chain.
    //
    g_ui32RxDescIndex++;
    if(g_ui32RxDescIndex == NUM_RX_DESCRIPTORS)
    {
        g_ui32RxDescIndex = 0;
    }

    //
    // Mark the next descriptor in the ring as available for the receiver
    // to write into.
    //
    g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus = DES0_RX_CTRL_OWN;
  }

  //
  // Return the Frame Length
  //
  return(i32FrameLen);
}

//*****************************************************************************
//
// Initialize the transmit and receive DMA descriptors.
//
// Appearently need minimum of 3 descriptors per chain.
// uIP uses a single, common transmit and receive buffer.
// All descriptors use one Rx and one Tx buffer, make sure to only hand to DMA one at a time.
//
//*****************************************************************************
static void InitDescriptors(uint32_t ui32Base)
{
  uint32_t ui32Loop;
  //
  // Initialize each of the transmit descriptors. 
  // Leave the OWN bit clear since we have not set up any transmissions yet.
  //
  for(ui32Loop = 0; ui32Loop < NUM_TX_DESCRIPTORS; ui32Loop++)
    {
      g_psTxDescriptor[ui32Loop].ui32Count =
        (DES1_TX_CTRL_SADDR_INSERT | (TX_BUFFER_SIZE << DES1_TX_CTRL_BUFF1_SIZE_S));
      g_psTxDescriptor[ui32Loop].pvBuffer1 = g_pui8TxBuffer;
      g_psTxDescriptor[ui32Loop].DES3.pLink =
        (ui32Loop == (NUM_TX_DESCRIPTORS - 1)) ? 
          g_psTxDescriptor : &g_psTxDescriptor[ui32Loop + 1];
      g_psTxDescriptor[ui32Loop].ui32CtrlStatus = 
        (DES0_TX_CTRL_LAST_SEG | DES0_TX_CTRL_FIRST_SEG |
         DES0_TX_CTRL_INTERRUPT | DES0_TX_CTRL_CHAINED | DES0_TX_CTRL_IP_ALL_CKHSUMS);
    }
    
  //
  // Initialize each of the receive descriptors. 
  // Clear the OWN bit here to make sure that the receiver doesn’t start writing 
  // anything immediately.
  //
  for(ui32Loop = 0; ui32Loop < NUM_RX_DESCRIPTORS; ui32Loop++)
    {
    g_psRxDescriptor[ui32Loop].ui32CtrlStatus = 0;
    g_psRxDescriptor[ui32Loop].ui32Count =
      (DES1_RX_CTRL_CHAINED | (RX_BUFFER_SIZE << DES1_RX_CTRL_BUFF1_SIZE_S));
    g_psRxDescriptor[ui32Loop].pvBuffer1 = g_pui8RxBuffer;
    g_psRxDescriptor[ui32Loop].DES3.pLink = (ui32Loop == (NUM_RX_DESCRIPTORS - 1)) ?
      g_psRxDescriptor : &g_psRxDescriptor[ui32Loop + 1];
    }
  //
  // Set the descriptor pointers in the hardware.
  //
  MAP_EMACRxDMADescriptorListSet(ui32Base, g_psRxDescriptor);
  MAP_EMACTxDMADescriptorListSet(ui32Base, g_psTxDescriptor);
  //
  // Start from the beginning of both descriptor chains. We actually set
  // the transmit descriptor index to the last descriptor in the chain
  // since it will be incremented before use and this means the first
  // transmission we perform will use the correct descriptor.
  //
  g_ui32RxDescIndex = 0;
  g_ui32TxDescIndex = NUM_TX_DESCRIPTORS - 1;
}

#endif // FORLM4F
#endif // BUILD_UIP


static void eth_init()
{
#ifdef BUILD_UIP
#ifdef FORLM4F
  uint32_t user0, user1;
  uint8_t ui8PHYAddr;
#else
  u32 user0, user1, intStatus;
#endif //FORLM4F
  static struct uip_eth_addr sTempAddr;

  // Enable Ethernet LEDs

#ifdef FORLM4F
  ROM_GPIOPinConfigure(GPIO_PF0_EN0LED0);
  ROM_GPIOPinConfigure(GPIO_PF4_EN0LED1);

  GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

#else
#if defined( FORLM3S9B92 ) || defined(FORLM3S9D92)
  MAP_GPIOPinConfigure(GPIO_PF2_LED1);
  MAP_GPIOPinConfigure(GPIO_PF3_LED0);
#endif

  MAP_GPIODirModeSet( GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_DIR_MODE_HW );
  MAP_GPIOPadConfigSet( GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

#endif // FORLM4F

  // Configure the hardware MAC address for Ethernet Controller filtering of
  // incoming packets.
  //
  // For the Ethernet Eval Kits, the MAC address will be stored in the
  // non-volatile USER0 and USER1 registers.  These registers can be read
  // using the FlashUserGet function, as illustrated below.
  
  //
  // Read the MAC address from the user registers.
  //
#if defined( ELUA_BOARD_SOLDERCORE )
  user0 = 0x00b61a00;
  user1 = 0x006d0a00;
#else
  MAP_FlashUserGet(&user0, &user1);
#endif

  if((user0 == 0xffffffff) || (user1 == 0xffffffff))
  {
    //
    // We should never get here.  This is an error if the MAC address has
    // not been programmed into the device.  Exit the program.
    //
// FIXME: Perhaps offer MAC address set at compile, or ??
// FIXME: Better error feedback
//  UpdateStatus("MAC Address Not Programmed!");
    while(1)
    {
    }
  }

  // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
  // address needed to program the hardware registers, then program the MAC
  // address into the Ethernet Controller registers.
  sTempAddr.addr[0] = ((user0 >>  0) & 0xff);
  sTempAddr.addr[1] = ((user0 >>  8) & 0xff);
  sTempAddr.addr[2] = ((user0 >> 16) & 0xff);
  sTempAddr.addr[3] = ((user1 >>  0) & 0xff);
  sTempAddr.addr[4] = ((user1 >>  8) & 0xff);
  sTempAddr.addr[5] = ((user1 >> 16) & 0xff);

  //
  // Configure SysTick for a periodic interrupt.
  //
  ROM_SysTickPeriodSet(clockfreq / SYSTICKHZ);
  ROM_SysTickEnable();
  ROM_SysTickIntEnable();

#ifdef FORLM4F  
  //
  // Enable and reset the Ethernet modules.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EMAC0);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_EMAC0);
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_EPHY0);
  //
  // Wait for the MAC to be ready.
  //
// TODO: Use busywait macro WAIT_WHILE
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_EMAC0))
  {
  }

  //
  // Configure for use with the internal PHY.
  //
  ui8PHYAddr = 0;
  MAP_EMACPHYConfigSet(EMAC0_BASE, (EMAC_PHY_TYPE_INTERNAL | EMAC_PHY_INT_MDIX_EN | EMAC_PHY_AN_100B_T_FULL_DUPLEX));
  //
  // Reset the MAC to latch the PHY configuration.
  //
  MAP_EMACReset(EMAC0_BASE);
  //
  // Initialize the MAC and set the DMA mode.
  //
  MAP_EMACInit(EMAC0_BASE, clockfreq, EMAC_BCONFIG_MIXED_BURST | EMAC_BCONFIG_PRIORITY_FIXED, 4, 4, 0);

  //
  // Set MAC configuration options.
  //
  MAP_EMACConfigSet(EMAC0_BASE, (EMAC_CONFIG_FULL_DUPLEX | EMAC_CONFIG_CHECKSUM_OFFLOAD | EMAC_CONFIG_7BYTE_PREAMBLE |
    EMAC_CONFIG_IF_GAP_96BITS | EMAC_CONFIG_USE_MACADDR0 | EMAC_CONFIG_SA_FROM_DESCRIPTOR | EMAC_CONFIG_BO_LIMIT_1024),
    (EMAC_MODE_RX_STORE_FORWARD | EMAC_MODE_TX_STORE_FORWARD | EMAC_MODE_TX_THRESHOLD_64_BYTES |
    EMAC_MODE_RX_THRESHOLD_64_BYTES), 0);

  //
  // Initialize the Ethernet DMA descriptors.
  //
  InitDescriptors(EMAC0_BASE);

  //
  // Program the hardware with its MAC address (for filtering).
  //
  MAP_EMACAddrSet(EMAC0_BASE, 0, (unsigned char *)&sTempAddr);

  //
  // Wait for the link to become active.
  //
  // TODO: Add busywait macro WAIT_WHILE
  while((MAP_EMACPHYRead(EMAC0_BASE, ui8PHYAddr, EPHY_BMSR) & EPHY_BMSR_LINKSTAT) == 0)
    {
    }

  //
  // Set MAC filtering options. We receive all broadcast and multicast
  // packets along with those addressed specifically for us.
  //
  MAP_EMACFrameFilterSet(EMAC0_BASE, (EMAC_FRMFILTER_SADDR | EMAC_FRMFILTER_PASS_MULTICAST |
    EMAC_FRMFILTER_PASS_NO_CTRL));
  //
  // Clear any pending interrupts.
  //
  MAP_EMACIntClear(EMAC0_BASE, MAP_EMACIntStatus(EMAC0_BASE, false));

  //
  // Initialize the uIP TCP/IP stack.
  //
  elua_uip_init( &sTempAddr );

  //
  // Enable the Ethernet MAC transmitter and receiver.
  //
  MAP_EMACTxEnable(EMAC0_BASE);
  MAP_EMACRxEnable(EMAC0_BASE);

  //
  // Enable the Ethernet interrupt.
  //
  MAP_IntEnable(INT_EMAC0);

  //
  // Enable the Ethernet RX Packet interrupt source.
  //
  MAP_EMACIntEnable(EMAC0_BASE, EMAC_INT_RECEIVE);

  //
  // Mark the first receive descriptor as available to the DMA to start
  // the receive processing.
  //
  g_psRxDescriptor[g_ui32RxDescIndex].ui32CtrlStatus |= DES0_RX_CTRL_OWN;
  
#else

  // Enable and reset the controller
  MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_ETH );
  MAP_SysCtlPeripheralReset( SYSCTL_PERIPH_ETH );

  // Initialize the Ethernet Controller and disable all Ethernet Controller interrupt sources.
  MAP_EthernetIntDisable(EMAC0_BASE , (ETH_INT_PHY | ETH_INT_MDIO | ETH_INT_RXER |
                     ETH_INT_RXOF | ETH_INT_TX | ETH_INT_TXER | ETH_INT_RX));
  intStatus = MAP_EthernetIntStatus(EMAC0_BASE , false);
  MAP_EthernetIntClear(EMAC0_BASE , intStatus);

  // Initialize the Ethernet Controller for operation.
  MAP_EthernetInitExpClk(EMAC0_BASE , clockfreq);

  // Configure the Ethernet Controller for normal operation.
  // - Full Duplex
  // - TX CRC Auto Generation
  // - TX Padding Enabled
  MAP_EthernetConfigSet(EMAC0_BASE , (ETH_CFG_TX_DPLXEN | ETH_CFG_TX_CRCEN | ETH_CFG_TX_PADEN));

  // Enable the Ethernet Controller.
  MAP_EthernetEnable(EMAC0_BASE );

  // Enable the Ethernet interrupt.
  MAP_IntEnable(INT_ETH);

  // Enable the Ethernet RX Packet interrupt source.
  MAP_EthernetIntEnable(EMAC0_BASE , ETH_INT_RX);

  // Enable all processor interrupts.
  MAP_IntMasterEnable();

  // Program the hardware with it's MAC address (for filtering).
  MAP_EthernetMACAddrSet(ETH_BASE, (unsigned char *)&sTempAddr);

  // Initialize the eLua uIP layer
  elua_uip_init( &sTempAddr );
#endif // FORLM4F

#endif // BUILD_UIP
}

#ifdef BUILD_UIP
//#if !defined( FORLM4F )
static int eth_timer_fired;
//#endif

void platform_eth_send_packet( const void* src, u32 size )
{
#ifdef FORLM4F
  PacketTransmit( EMAC0_BASE, (uint8_t *) src, size);
#else
  MAP_EthernetPacketPut( EMAC0_BASE, src, size );
#endif // FORLM4F
}

u32 platform_eth_get_packet_nb( void* buf, u32 maxlen )
{
#ifdef FORLM4F
  return PacketReceive( EMAC0_BASE, buf, maxlen );
#else
  return MAP_EthernetPacketGetNonBlocking( EMAC0_BASE, buf, maxlen );
#endif // FORLM4F
}

void platform_eth_force_interrupt()
{
#ifdef FORLM4F
// FIXME: What should it do?
#warning platform_eth_force_interrupt not done for TM4C1294
#else
  NVIC_SW_TRIG_R |= INT_ETH - 16;
#endif // FORLM4F
}

u32 platform_eth_get_elapsed_time()
{
  if( eth_timer_fired )
  {
    eth_timer_fired = 0;
    return SYSTICKMS;
  }
  else
    return 0;
}

void SysTickIntHandler()
{
  // Handle virtual timers
  cmn_virtual_timer_cb();

  // Indicate that a SysTick interrupt has occurred.
//#ifdef FORLM4F
//  HWREGBITW(&g_ui32Flags, FLAG_SYSTICK) = 1;
//#else  
  eth_timer_fired = 1;
//#endif
    
  // Generate a fake Ethernet interrupt.  This will perform the actual work
  // of incrementing the timers and taking the appropriate actions.
  platform_eth_force_interrupt();

  // System timer handling
  cmn_systimer_periodic();
}

void EthernetIntHandler(void)
{
#ifdef FORLM4F
  uint32_t ui32Temp;
  //
  // Read and Clear the interrupt.
  //
  ui32Temp = MAP_EMACIntStatus(EMAC0_BASE, true);
  MAP_EMACIntClear(EMAC0_BASE, ui32Temp);
  //
  // Check to see if an RX Interrupt has occurred.
  //
  if(ui32Temp & EMAC_INT_RECEIVE)
    {
    //
    // Indicate that a packet has been received.
    //
//      HWREGBITW(&g_ui32Flags, FLAG_RXPKT) = 1;

    elua_uip_mainloop();
    }
  //
  // Has the DMA finished transferring a packet to the transmitter?
  //
  if(ui32Temp & EMAC_INT_TRANSMIT)
  {
    //
    // Indicate that a packet has been sent.
    //
//    HWREGBITW(&g_ui32Flags, FLAG_TXPKT) = 0;
  }

#else
  u32 temp;

  // Read and Clear the interrupt.
  temp = MAP_EthernetIntStatus( EMAC0_BASE , false );
  MAP_EthernetIntClear( EMAC0_BASE , temp );

  // Call the UIP main loop
  elua_uip_mainloop();
#endif // FORLM4F
}

#else  // #ifdef BUILD_UIP

void SysTickIntHandler()
{
  cmn_virtual_timer_cb();

  // System timer handling
  cmn_systimer_periodic();
}

void EthernetIntHandler()
{
}
#endif // #ifdef BUILD_UIP


// ****************************************************************************
// USB functions

#if defined( BUILD_USB_CDC )

// Appears that USB functions are not included in ROM

static void usb_init()
{
  USBBufferInit( &g_sTxBuffer );
  USBBufferInit( &g_sRxBuffer );

  // Pass the device information to the USB library and place the device
  // on the bus.
  USBDCDCInit( 0, &g_sCDCDevice );
}

void platform_usb_cdc_send( u8 data )
{
  USBBufferWrite( &g_sTxBuffer, &data, 1 );
}

int platform_usb_cdc_recv( s32 timeout )
{
  unsigned char data;
  unsigned long read;

  // Try to read one byte from buffer, if none available return -1 or
  // retry if timeout
  // FIXME: Respect requested timeout
  do {
    read = USBBufferRead(&g_sRxBuffer, &data, 1);
  } while( read == 0 && timeout != 0 );

  if( read == 0 )
    return -1;
  else
    return data;
}

unsigned long TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData)
{
  // Which event was sent?
  switch(ulEvent)
  {
    case USB_EVENT_TX_COMPLETE:
    {
        // Nothing to do, already handled by USBBuffer
        break;
    }
      
    default:
        break;
  }
  
  return(0);
}

unsigned long RxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData)
{
// TODO: Not sure why these local variables were defined, since not used - is there code missing?
//  unsigned long ulCount;
//  unsigned char ucChar;
//  unsigned long ulRead;


  // Which event was sent?
  switch(ulEvent)
  {
    // A new packet has been received.
    case USB_EVENT_RX_AVAILABLE:
    {
      break;
    }

    //
    // This is a request for how much unprocessed data is still waiting to
    // be processed.  Return 0 if the UART is currently idle or 1 if it is
    // in the process of transmitting something.  The actual number of
    // bytes in the UART FIFO is not important here, merely whether or
    // not everything previously sent to us has been transmitted.
    //
    case USB_EVENT_DATA_REMAINING:
    {
      //
      // Get the number of bytes in the buffer and add 1 if some data
      // still has to clear the transmitter.
      //
      return(0);
    }

    //
    // This is a request for a buffer into which the next packet can be
    // read.  This mode of receiving data is not supported so let the
    // driver know by returning 0.  The CDC driver should not be sending
    // this message but this is included just for illustration and
    // completeness.
    //
    case USB_EVENT_REQUEST_BUFFER:
    {
      return(0);
    }

    // Other events can be safely ignored.
    default:
    {
      break;
    }
  }

  return(0);
}

unsigned long
ControlHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
               void *pvMsgData)
{
  switch(ulEvent) // Check event
  {
    // The host has connected.
    case USB_EVENT_CONNECTED:
    {
      USBBufferFlush(&g_sTxBuffer);
      USBBufferFlush(&g_sRxBuffer);
      break;
    }

    
    // The host has disconnected.
    
    case USB_EVENT_DISCONNECTED:
    {
      break;
    }
    
    // Return the current serial communication parameters.
    case USBD_CDC_EVENT_GET_LINE_CODING:
    {
      break;
    }

    // Set the current serial communication parameters.
    case USBD_CDC_EVENT_SET_LINE_CODING:
    {
      break;
    }

    
    // Set the current serial communication parameters.
    case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
    {
      break;
    }

    //
    // Send a break condition on the serial line.
    //
    case USBD_CDC_EVENT_SEND_BREAK:
    {
      break;
    }

    //
    // Clear the break condition on the serial line.
    //
    case USBD_CDC_EVENT_CLEAR_BREAK:
    {
      break;
    }

    //
    // Ignore SUSPEND and RESUME for now.
    //
    case USB_EVENT_SUSPEND:
    case USB_EVENT_RESUME:
    {
      break;
    }

    //
    // Other events can be safely ignored.
    //
    default:
    {
      break;
    }
  }

  return(0);
}

#endif // BUILD_USB_CDC


// ****************************************************************************
// Flash access functions

#ifdef BUILD_WOFS
u32 platform_s_flash_write( const void *from, u32 toaddr, u32 size )
{
  return MAP_FlashProgram( ( unsigned long * )from, toaddr, size );
}

// TM4C123x - 1KB Block, TM4C129x - 16KB blocks
int platform_flash_erase_sector( u32 sector_id )
{
  return MAP_FlashErase( sector_id * INTERNAL_FLASH_SECTOR_SIZE ) == 0 ? PLATFORM_OK : PLATFORM_ERR;
}
#endif // #ifdef BUILD_WOFS


// ****************************************************************************
// Platform specific modules go here

/* This all was removed from master - don't know what it did

#if defined( ENABLE_DISP ) || defined( ENABLE_LM3S_GPIO )

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"

extern const LUA_REG_TYPE disp_map[];
extern const LUA_REG_TYPE lm3s_pio_map[];

const LUA_REG_TYPE platform_map[] =
{
#if LUA_OPTIMIZE_MEMORY > 0
#ifdef ENABLE_DISP
  { LSTRKEY( "disp" ), LROVAL( disp_map ) },
#endif
#ifdef ENABLE_LM3S_GPIO
  { LSTRKEY( "pio" ), LROVAL( lm3s_pio_map ) },
#endif
#endif
  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_platform( lua_State *L )
{
#if LUA_OPTIMIZE_MEMORY > 0
  return 0;
#else // #if LUA_OPTIMIZE_MEMORY > 0
  luaL_register( L, PS_LIB_TABLE_NAME, platform_map );

  // Setup the new tables inside platform table
  lua_newtable( L );
  luaL_register( L, NULL, disp_map );
  lua_setfield( L, -2, "disp" );
  lua_newtable( L );
  luaL_register( L, NULL, lm3s_pio_map );
  lua_setfield( L, -2, "pio" );

  return 1;
#endif // #if LUA_OPTIMIZE_MEMORY > 0
}

#else // #if defined( ENABLE_DISP ) || defined( ENABLE_LM3S_GPIO )

LUALIB_API int luaopen_platform( lua_State *L )
{
  return 0;
}

#endif // #if defined( ENABLE_DISP ) || defined( ENABLE_LM3S_GPIO )

*/


// Assertion failure error handler
// give feedback on console (if it has been setup), and place for breakpoint for debugging

#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
  if (consoleinited)
    printf("Error in file %s, line %lu\n", pcFilename, ulLine);
}
#endif

