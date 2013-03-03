//*****************************************************************************
//
// startup_gcc.c - Startup code for use with GNU tools.
//
// Copyright (c) 2007-2008 Luminary Micro, Inc.  All rights reserved.
//
// Software License Agreement
//
// Luminary Micro, Inc. (LMI) is supplying this software for use solely and
// exclusively on LMI's microcontroller products.
//
// The software is owned by LMI and/or its suppliers, and is protected under
// applicable copyright laws.  All rights are reserved.  You may not combine
// this software with "viral" open-source software in order to form a larger
// program.  Any use in violation of the foregoing restrictions may subject
// the user to criminal sanctions under applicable laws, as well as to civil
// liability for the breach of the terms and conditions of this license.
//
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2752 of the Stellaris Peripheral Driver Library.
//
//*****************************************************************************

//*****************************************************************************
//
// startup_gcc.c - Startup code for use with GNU tools.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 9453 of the EK-LM4F120XL Firmware Package.
//
//*****************************************************************************

#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"


//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);

// eLua

// External interrupt handlers
//extern void UARTIntHandler();	// TODO remove, doesn't appear to exist


#include "platform_conf.h"

// Handlers in platform.c
extern void EthernetIntHandler(void);
extern void SysTickIntHandler(void);
#if defined( BUILD_ADC )
extern void ADCIntHandler(void);
#endif

#if defined( BUILD_CAN )
extern void CANIntHandler(void);
#endif

// Handlers from usblib (usbdevice.h)
#if defined( BUILD_USB_CDC )
extern void USB0DeviceIntHandler(void);
#endif

// Handlers in platform_int.c
#include "platform_int.h"


//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int main(void);

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************

#if defined( ccs )		// Put at 0 for CCS
#pragma DATA_SECTION(g_pfnVectors, ".intvecs")
#define PUT_ME_AT_0
#else					// Put at 0 for gcc
#define PUT_ME_AT_0 __attribute__ ((section(".isr_vector")))
#endif

PUT_ME_AT_0
void (* const g_pfnVectors[])(void) =
{
    (void (*) (void))( SRAM_BASE + SRAM_SIZE ),
                                            // The initial stack pointer
    ResetISR,                               // The reset handler
    NmiSR,                                  // The NMI handler
    FaultISR,                               // The hard fault handler
    IntDefaultHandler,                      // The MPU fault handler
    IntDefaultHandler,                      // The bus fault handler
    IntDefaultHandler,                      // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // SVCall handler
    IntDefaultHandler,                      // Debug monitor handler
    0,                                      // Reserved
    IntDefaultHandler,                      // The PendSV handler
    SysTickIntHandler,                      // The SysTick handler
    gpioa_handler,                          // GPIO Port A
    gpiob_handler,                          // GPIO Port B
    gpioc_handler,                          // GPIO Port C
    gpiod_handler,                          // GPIO Port D
    gpioe_handler,                          // GPIO Port E
#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )
    uart0_handler,                          // UART0 Rx and Tx
#else
    IntDefaultHandler,
#endif
#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )
    uart1_handler,                          // UART1 Rx and Tx
#else
    IntDefaultHandler,                      // UART1 Rx and Tx
#endif    
    IntDefaultHandler,                      // SSI0 Rx and Tx
    IntDefaultHandler,                      // I2C0 Master and Slave
    IntDefaultHandler,                      // PWM Fault
    IntDefaultHandler,                      // PWM Generator 0
    IntDefaultHandler,                      // PWM Generator 1
    IntDefaultHandler,                      // PWM Generator 2
    IntDefaultHandler,                      // Quadrature Encoder 0
#if defined( BUILD_ADC )
    ADCIntHandler,	                    // ADC Sequence 0
    ADCIntHandler,                          // ADC Sequence 1
    ADCIntHandler,                          // ADC Sequence 2
    ADCIntHandler,                          // ADC Sequence 3
#else
    IntDefaultHandler,	                    // ADC Sequence 0
    IntDefaultHandler,                      // ADC Sequence 1
    IntDefaultHandler,                      // ADC Sequence 2
    IntDefaultHandler,                      // ADC Sequence 3
#endif
    IntDefaultHandler,                      // Watchdog timer
    tmr0_handler,                           // Timer 0 subtimer A
    IntDefaultHandler,                      // Timer 0 subtimer B
    tmr1_handler,                           // Timer 1 subtimer A
    IntDefaultHandler,                      // Timer 1 subtimer B
    tmr2_handler,                           // Timer 2 subtimer A
    IntDefaultHandler,                      // Timer 2 subtimer B
    IntDefaultHandler,                      // Analog Comparator 0
    IntDefaultHandler,                      // Analog Comparator 1
    IntDefaultHandler,                      // Analog Comparator 2
    IntDefaultHandler,                      // System Control (PLL, OSC, BO)
    IntDefaultHandler,                      // FLASH Control
    gpiof_handler,                          // GPIO Port F
    gpiog_handler,                          // GPIO Port G
    gpioh_handler,                          // GPIO Port H
#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )
    uart2_handler,                          // UART2 Rx and Tx
#else    
    IntDefaultHandler,                      // UART2 Rx and Tx
#endif    
    IntDefaultHandler,                      // SSI1 Rx and Tx
    tmr3_handler,                           // Timer 3 subtimer A
    IntDefaultHandler,                      // Timer 3 subtimer B
    IntDefaultHandler,                      // I2C1 Master and Slave
    IntDefaultHandler,                      // Quadrature Encoder 1
#if defined( BUILD_CAN )
    CANIntHandler,                          // CAN0
#else
    IntDefaultHandler,                      // CAN0
#endif
    IntDefaultHandler,                      // CAN1
    IntDefaultHandler,                      // CAN2
    EthernetIntHandler,                     // Ethernet
#if defined( FORLM3S9B92 ) || defined( FORLM3S9D92 ) || defined( FORLM4F120 )
    IntDefaultHandler,                      // Hibernate
#if defined( BUILD_USB_CDC )
    USB0DeviceIntHandler,                   // USB0
#else // #if defined( BUILD_USB_CDC )
    IntDefaultHandler,                      // USB0
#endif // #if defined( BUILD_USB_CDC )
    IntDefaultHandler,                      // PWM Generator 3
    IntDefaultHandler,                      // uDMA Software Transfer
    IntDefaultHandler,                      // uDMA Error
    IntDefaultHandler,                      // ADC1 Sequence 0
    IntDefaultHandler,                      // ADC1 Sequence 1
    IntDefaultHandler,                      // ADC1 Sequence 2
    IntDefaultHandler,                      // ADC1 Sequence 3
    IntDefaultHandler,                      // I2S0
    IntDefaultHandler,                      // External Bus Interface 0
// suppress comma unless LM4F120
#ifndef FORLM4F120 
    gpioj_handler                           // GPIO Port J    
#else
    gpioj_handler,                          // GPIO Port J    
#endif
#else // #if defined( FORLM3S9B92 ) || defined( FORLM3S9D92 ) || defined( FORLM4F120 )
    IntDefaultHandler                       // Hibernate
#endif // #if defined( FORLM3S9B92 ) || defined( FORLM3S9D92 ) || defined( FORLM4F120 )

#if defined( FORLM4F120 )
    IntDefaultHandler,                      // GPIO Port K
    IntDefaultHandler,                      // GPIO Port L
    IntDefaultHandler,                      // SSI2 Rx and Tx
    IntDefaultHandler,                      // SSI3 Rx and Tx
#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )
    uart3_handler,                          // UART3 Rx and Tx
#else
    IntDefaultHandler,
#endif
#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )
    uart4_handler,                          // UART4 Rx and Tx
#else
    IntDefaultHandler,
#endif
#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )
    uart5_handler,                          // UART5 Rx and Tx
#else
    IntDefaultHandler,
#endif
    IntDefaultHandler,                      // UART6 Rx and Tx
    IntDefaultHandler,                      // UART7 Rx and Tx
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // I2C2 Master and Slave
    IntDefaultHandler,                      // I2C3 Master and Slave
    tmr4_handler,                           // Timer 4 subtimer A
    IntDefaultHandler,                      // Timer 4 subtimer B
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    tmr5_handler,                           // Timer 5 subtimer A
    IntDefaultHandler,                      // Timer 5 subtimer B
    IntDefaultHandler,                      // Wide Timer 0 subtimer A
    IntDefaultHandler,                      // Wide Timer 0 subtimer B
    IntDefaultHandler,                      // Wide Timer 1 subtimer A
    IntDefaultHandler,                      // Wide Timer 1 subtimer B
    IntDefaultHandler,                      // Wide Timer 2 subtimer A
    IntDefaultHandler,                      // Wide Timer 2 subtimer B
    IntDefaultHandler,                      // Wide Timer 3 subtimer A
    IntDefaultHandler,                      // Wide Timer 3 subtimer B
    IntDefaultHandler,                      // Wide Timer 4 subtimer A
    IntDefaultHandler,                      // Wide Timer 4 subtimer B
    IntDefaultHandler,                      // Wide Timer 5 subtimer A
    IntDefaultHandler,                      // Wide Timer 5 subtimer B
    IntDefaultHandler,                      // FPU
    IntDefaultHandler,                      // PECI 0
    IntDefaultHandler,                      // LPC 0
    IntDefaultHandler,                      // I2C4 Master and Slave
    IntDefaultHandler,                      // I2C5 Master and Slave
    IntDefaultHandler,                      // GPIO Port M
    IntDefaultHandler,                      // GPIO Port N
    IntDefaultHandler,                      // Quadrature Encoder 2
    IntDefaultHandler,                      // Fan 0
    0,                                      // Reserved
    IntDefaultHandler,                      // GPIO Port P (Summary or P0)
    IntDefaultHandler,                      // GPIO Port P1
    IntDefaultHandler,                      // GPIO Port P2
    IntDefaultHandler,                      // GPIO Port P3
    IntDefaultHandler,                      // GPIO Port P4
    IntDefaultHandler,                      // GPIO Port P5
    IntDefaultHandler,                      // GPIO Port P6
    IntDefaultHandler,                      // GPIO Port P7
    IntDefaultHandler,                      // GPIO Port Q (Summary or Q0)
    IntDefaultHandler,                      // GPIO Port Q1
    IntDefaultHandler,                      // GPIO Port Q2
    IntDefaultHandler,                      // GPIO Port Q3
    IntDefaultHandler,                      // GPIO Port Q4
    IntDefaultHandler,                      // GPIO Port Q5
    IntDefaultHandler,                      // GPIO Port Q6
    IntDefaultHandler,                      // GPIO Port Q7
    IntDefaultHandler,                      // GPIO Port R
    IntDefaultHandler,                      // GPIO Port S
    IntDefaultHandler,                      // PWM 1 Generator 0
    IntDefaultHandler,                      // PWM 1 Generator 1
    IntDefaultHandler,                      // PWM 1 Generator 2
    IntDefaultHandler,                      // PWM 1 Generator 3
    IntDefaultHandler                       // PWM 1 Fault
#endif // FORLM4F120

};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void
ResetISR(void)
{
    unsigned long *pulSrc, *pulDest;

    //
    // Copy the data segment initializers from flash to SRAM.
    //
    pulSrc = &_etext;
    for(pulDest = &_data; pulDest < &_edata; )
    {
        *pulDest++ = *pulSrc++;
    }

    //
    // Zero fill the bss segment.  This is done with inline assembly since this
    // will clear the value of pulDest if it is not kept in a register.
    //
    __asm("    ldr     r0, =_bss\n"
          "    ldr     r1, =_ebss\n"
          "    mov     r2, #0\n"
          "    .thumb_func\n"
          "zero_loop:\n"
          "        cmp     r0, r1\n"
          "        it      lt\n"
          "        strlt   r2, [r0], #4\n"
          "        blt     zero_loop");

#if defined( FORLM4F120 )
// FIXME: incorporate FPU into eLua
    //
    // Enable the floating-point unit.  This must be done here to handle the
    // case where main() uses floating-point and the function prologue saves
    // floating-point registers (which will fault if floating-point is not
    // enabled).  Any configuration of the floating-point unit using DriverLib
    // APIs must be done here prior to the floating-point unit being enabled.
    //
    // Note that this does not use DriverLib since it might not be included in
    // this project.
    //

#if !defined( ccs )  // This will not actually work with CCS, just suppress some errors
    HWREG(NVIC_CPAC) = ((HWREG(NVIC_CPAC) &
                         ~(NVIC_CPAC_CP10_M | NVIC_CPAC_CP11_M)) |
                        NVIC_CPAC_CP10_FULL | NVIC_CPAC_CP11_FULL);

#endif // ccs
#endif // FORLM4F120

    //
    // Call the application's entry point.
    //
    main();

    while(1);
}

// Platform specific includes
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"	//TODO: Remove - duplicate
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
      UARTCharPut( UART0_BASE, '!' );
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
FaultISR(void)
{
    //
    // Enter an infinite loop.
    //
   UARTCharPut( UART0_BASE, '#' );
   UARTCharPut( UART0_BASE, '#' );
   UARTCharPut( UART0_BASE, '#' );
   UARTCharPut( UART0_BASE, '#' );
   UARTCharPut( UART0_BASE, '#' );
   while(1)
   {
   }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
      UARTCharPut( UART0_BASE, '*' );
    }
}
