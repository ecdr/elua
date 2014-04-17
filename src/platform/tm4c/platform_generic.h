// Generic platform customization

#ifndef __PLATFORM_GENERIC_H__
#define __PLATFORM_GENERIC_H__

#define PLATFORM_HAS_SYSTIMER
#define PLATFORM_TMR_COUNTS_DOWN

#if NUM_CAN > 0
#define BUILD_CAN
#endif


// Macro for waiting - loop at the moment, but make it easier to slow down, use low power, etc.
#define WAIT_WHILE( cond ) while( cond );
// FIXME: Busy waiting (should do low power/sleep and wake on interrupt, or do something useful)


// The following construct created by the linker, common.c declares it as array of char, so did same here compatibility
//extern unsigned long end;
extern char end[];

// TODO: Removed several interrupts that no longer exist in Tivaware - see if should add any to replace
#if defined( FORTM4C1294)
#define CPU_CONST_ETHERNET_INT  _C( INT_EMAC0 ),
#else
#define CPU_CONST_ETHERNET_INT
#endif  

#if defined( INT_PWM0_0 )
#define PLATFORM_CPU_CONSTANTS_PWM    _C( INT_PWM0_FAULT ),\
  _C( INT_PWM0_0 ),\
  _C( INT_PWM0_1 ),\
  _C( INT_PWM0_2 ),
#else
#define PLATFORM_CPU_CONSTANTS_PWM
#endif

#if defined( INT_PWM0_3 )
#define PLATFORM_CPU_CONSTANTS_PWM03    _C( INT_PWM0_3 ),
#else
#define PLATFORM_CPU_CONSTANTS_PWM03
#endif

#if defined( INT_QEI0 )
#define PLATFORM_CPU_CONSTANTS_QEI0   _C( INT_QEI0 ),
#else
#define PLATFORM_CPU_CONSTANTS_QEI0
#endif

#if defined( INT_QEI1 )
#define PLATFORM_CPU_CONSTANTS_QEI1   _C( INT_QEI1 ),
#else
#define PLATFORM_CPU_CONSTANTS_QEI1
#endif

#if defined( INT_CAN1 )
#define PLATFORM_CPU_CONSTANTS_CAN1  _C( INT_CAN1 ),
#else
#define PLATFORM_CPU_CONSTANTS_CAN1
#endif

#if defined( INT_PWM0_0 )
#define PLATFORM_CPU_CONSTANTS_PWM    _C( INT_PWM0_FAULT ),\
  _C( INT_PWM0_0 ),\
  _C( INT_PWM0_1 ),\
  _C( INT_PWM0_2 ),
#else
#define PLATFORM_CPU_CONSTANTS_PWM
#endif

#if defined( INT_PWM0_3 )
#define PLATFORM_CPU_CONSTANTS_PWM03    _C( INT_PWM0_3 ),
#else
#define PLATFORM_CPU_CONSTANTS_PWM03
#endif

#if defined( INT_QEI0 )
#define PLATFORM_CPU_CONSTANTS_QEI0   _C( INT_QEI0 ),
#else
#define PLATFORM_CPU_CONSTANTS_QEI0
#endif

#if defined( INT_QEI1 )
#define PLATFORM_CPU_CONSTANTS_QEI1   _C( INT_QEI1 ),
#else
#define PLATFORM_CPU_CONSTANTS_QEI1
#endif

#if defined( INT_CAN1 )
#define PLATFORM_CPU_CONSTANTS_CAN1  _C( INT_CAN1 ),
#else
#define PLATFORM_CPU_CONSTANTS_CAN1
#endif

#define PLATFORM_CPU_CONSTANTS_PLATFORM\
  _C( INT_GPIOA ),\
  _C( INT_GPIOB ),\
  _C( INT_GPIOC ),\
  _C( INT_GPIOD ),\
  _C( INT_GPIOE ),\
  _C( INT_UART0 ),\
  _C( INT_UART1 ),\
  _C( INT_SSI0 ),\
  _C( INT_I2C0 ),\
  PLATFORM_CPU_CONSTANTS_PWM\
  PLATFORM_CPU_CONSTANTS_QEI0\
  _C( INT_ADC0SS0 ),\
  _C( INT_ADC0SS1 ),\
  _C( INT_ADC0SS2 ),\
  _C( INT_ADC0SS3 ),\
  _C( INT_WATCHDOG ),\
  _C( INT_TIMER0A ),\
  _C( INT_TIMER0B ),\
  _C( INT_TIMER1A ),\
  _C( INT_TIMER1B ),\
  _C( INT_TIMER2A ),\
  _C( INT_TIMER2B ),\
  _C( INT_COMP0 ),\
  _C( INT_COMP1 ),\
  _C( INT_SYSCTL ),\
  _C( INT_FLASH ),\
  _C( INT_GPIOF ),\
  _C( INT_UART2 ),\
  _C( INT_SSI1 ),\
  _C( INT_TIMER3A ),\
  _C( INT_TIMER3B ),\
  _C( INT_I2C1 ),\
  PLATFORM_CPU_CONSTANTS_QEI1\
  _C( INT_CAN0 ),\
  PLATFORM_CPU_CONSTANTS_CAN1\
  CPU_CONST_ETHERNET_INT\
  _C( INT_HIBERNATE ),\
  _C( INT_USB0 ),\
  PLATFORM_CPU_CONSTANTS_PWM03\
  _C( INT_UDMA ),\
  _C( INT_UDMAERR ),


//  _C( INT_COMP2 ),
//  _C( INT_GPIOG ),
//  _C( INT_GPIOH ),


#endif // #ifndef __PLATFORM_GENERIC_H__

