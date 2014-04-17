//*****************************************************************************
//
// startup_gcc.c - Startup code for use with GNU tools.
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.0.12573 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

#if 0
// FIXME: Find macro to test for IAR compiler

// Enable the IAR extensions for this source file.
#pragma language=extended

//*****************************************************************************
//
// The entry point for the application startup code.
//
//*****************************************************************************
extern void __iar_program_start(void);

#endif 

#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

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

#include "platform_conf.h"

// Handlers in platform.c
extern void EthernetIntHandler(void);
extern void SysTickIntHandler(void);
#if defined( BUILD_ADC )
extern void ADCIntHandler(void);
#endif

#if defined( BUILD_CAN )

extern void can0_handler(void);
extern void can1_handler(void);
extern void can2_handler(void);

#endif

// Handlers from usblib (usbdevice.h)
#if defined( BUILD_USB_CDC )
extern void USB0DeviceIntHandler(void);
#endif

// Handlers in platform_int.c
#include "platform_int.h"

// From platform.c
extern const u32 uart_base[];

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

// eLua - compile with Code Composer Studuio (ccs) or gcc
#if defined( ccs )		// Put at 0 for CCS
#pragma DATA_SECTION(g_pfnVectors, ".intvecs")
#define PUT_ME_AT_0
#elif defined( __GNUC__	)			// Put at 0 for gcc
#define PUT_ME_AT_0 __attribute__ ((section(".isr_vector")))
#else

#warning Unrecognized compiler, not sure how to put vectors at 0.
#if 0

// This is startup code for IAR Embeded workbench

//*****************************************************************************
//
// A union that describes the entries of the vector table.  The union is needed
// since the first entry is the stack pointer and the remainder are function
// pointers.
//
//*****************************************************************************
typedef union
{
    void (*pfnHandler)(void);
    uint32_t ui32Ptr;
}
uVectorEntry;

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
__root const uVectorEntry __vector_table[] @ ".intvec" =
{
    { .ui32Ptr = (uint32_t)pui32Stack + sizeof(pui32Stack) },

#endif // 0

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
    IntDefaultHandler,                      // UART0 Rx and Tx
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
    ADCIntHandler,	                        // ADC Sequence 0
    ADCIntHandler,                          // ADC Sequence 1
    ADCIntHandler,                          // ADC Sequence 2
    ADCIntHandler,                          // ADC Sequence 3
#else
    IntDefaultHandler,                      // ADC Sequence 0
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
#if NUM_CAN > 0
    can0_handler,                           // CAN0
#else
    IntDefaultHandler,                      // CAN0
#endif
#if NUM_CAN > 1
    can1_handler,                           // CAN1
#else
    IntDefaultHandler,                      // CAN1
#endif
#if NUM_CAN > 2
#error Maximum of 2 CAN supported
#endif
#else
    IntDefaultHandler,                      // CAN0
    IntDefaultHandler,                      // CAN1
#endif
#if !defined(FORTM4C1294)
    0,                                      // Reserved - This was CAN2 in Stellarisware
#endif
#if defined(BUILD_UIP)
    EthernetIntHandler,                     // Ethernet
#else    
    0,                                      // Reserved
#endif
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
    0,                                      // Reserved
#if !defined(FORTM4C1294)
    0,                                      // Reserved
#endif
    gpioj_handler,                          // GPIO Port J
    gpiok_handler,                          // GPIO Port K
    gpiol_handler,                          // GPIO Port L
    IntDefaultHandler,                      // SSI2 Rx and Tx
    IntDefaultHandler,                      // SSI3 Rx and Tx
#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )
    uart3_handler,                          // UART3 Rx and Tx
#else
    IntDefaultHandler,                      // UART3 Rx and Tx
#endif
#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )
    uart4_handler,                          // UART4 Rx and Tx
#else
    IntDefaultHandler,                      // UART4 Rx and Tx
#endif
#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )
    uart5_handler,                          // UART5 Rx and Tx
#else
    IntDefaultHandler,                      // UART5 Rx and Tx
#endif
#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )
    uart6_handler,                          // UART6 Rx and Tx
#else
    IntDefaultHandler,                      // UART6 Rx and Tx
#endif
#if defined( BUILD_C_INT_HANDLERS ) || defined( BUILD_LUA_INT_HANDLERS )
    uart7_handler,                          // UART7 Rx and Tx
#else
    IntDefaultHandler,                      // UART7 Rx and Tx
#endif
#if !defined(FORTM4C1294)
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
#endif    
    IntDefaultHandler,                      // I2C2 Master and Slave
    IntDefaultHandler,                      // I2C3 Master and Slave
    tmr4_handler,                           // Timer 4 subtimer A
    IntDefaultHandler,                      // Timer 4 subtimer B
#if ! defined(FORTM4C1294)
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
#endif
    tmr5_handler,                           // Timer 5 subtimer A
    IntDefaultHandler,                      // Timer 5 subtimer B
#if ! defined(FORTM4C1294)
    tmr6_handler,                           // Wide Timer 0 subtimer A
    IntDefaultHandler,                      // Wide Timer 0 subtimer B
    tmr7_handler,                           // Wide Timer 1 subtimer A
    IntDefaultHandler,                      // Wide Timer 1 subtimer B
    tmr8_handler,                           // Wide Timer 2 subtimer A
    IntDefaultHandler,                      // Wide Timer 2 subtimer B
    tmr9_handler,                           // Wide Timer 3 subtimer A
    IntDefaultHandler,                      // Wide Timer 3 subtimer B
    tmr10_handler,                          // Wide Timer 4 subtimer A
    IntDefaultHandler,                      // Wide Timer 4 subtimer B
    tmr11_handler,                          // Wide Timer 5 subtimer A
    IntDefaultHandler,                      // Wide Timer 5 subtimer B
#endif
    IntDefaultHandler,                      // FPU
    IntDefaultHandler,                      // PECI 0
    IntDefaultHandler,                      // LPC 0
    IntDefaultHandler,                      // I2C4 Master and Slave
    IntDefaultHandler,                      // I2C5 Master and Slave
// TODO: int handlers for more PIO ports (M, N, P, Q, etc. for Tiva 129)
    gpiom_handler,                          // GPIO Port M
    gpion_handler,                          // GPIO Port N
    IntDefaultHandler,                      // Quadrature Encoder 2
    IntDefaultHandler,                      // Fan 0
#if ! defined(FORTM4C1294)
    0,                                      // Reserved
#endif
    gpiop_handler,                          // GPIO Port P (Summary or P0)
    IntDefaultHandler,                      // GPIO Port P1
    IntDefaultHandler,                      // GPIO Port P2
    IntDefaultHandler,                      // GPIO Port P3
    IntDefaultHandler,                      // GPIO Port P4
    IntDefaultHandler,                      // GPIO Port P5
    IntDefaultHandler,                      // GPIO Port P6
    IntDefaultHandler,                      // GPIO Port P7
    gpioq_handler,                          // GPIO Port Q (Summary or Q0)
    IntDefaultHandler,                      // GPIO Port Q1
    IntDefaultHandler,                      // GPIO Port Q2
    IntDefaultHandler,                      // GPIO Port Q3
    IntDefaultHandler,                      // GPIO Port Q4
    IntDefaultHandler,                      // GPIO Port Q5
    IntDefaultHandler,                      // GPIO Port Q6
    IntDefaultHandler,                      // GPIO Port Q7
    IntDefaultHandler,                      // GPIO Port R
    IntDefaultHandler,                      // GPIO Port S
#if defined(FORTM4C1294)
    IntDefaultHandler,                      // SHA/MD5 0
    IntDefaultHandler,                      // AES 0
    IntDefaultHandler,                      // DES3DES 0
    IntDefaultHandler,                      // LCD Controller 0
    IntDefaultHandler,                      // Timer 6 subtimer A
    IntDefaultHandler,                      // Timer 6 subtimer B
    IntDefaultHandler,                      // Timer 7 subtimer A
    IntDefaultHandler,                      // Timer 7 subtimer B
    IntDefaultHandler,                      // I2C6 Master and Slave
    IntDefaultHandler,                      // I2C7 Master and Slave
    IntDefaultHandler,                      // HIM Scan Matrix Keyboard 0
    IntDefaultHandler,                      // One Wire 0
    IntDefaultHandler,                      // HIM PS/2 0
    IntDefaultHandler,                      // HIM LED Sequencer 0
    IntDefaultHandler,                      // HIM Consumer IR 0
    IntDefaultHandler,                      // I2C8 Master and Slave
    IntDefaultHandler,                      // I2C9 Master and Slave
    IntDefaultHandler                       // GPIO Port T
#else
    IntDefaultHandler,                      // PWM 1 Generator 0
    IntDefaultHandler,                      // PWM 1 Generator 1
    IntDefaultHandler,                      // PWM 1 Generator 2
    IntDefaultHandler,                      // PWM 1 Generator 3
    IntDefaultHandler                       // PWM 1 Fault
#endif
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern uint32_t _etext;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss;
extern uint32_t _ebss;

#if defined( ccs )
//*****************************************************************************
//
// External declaration for the reset handler that is to be called when the
// processor is started
//
//*****************************************************************************
extern void _c_int00(void);
#endif

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

#if defined( ccs )
    //
    // Jump to the CCS C initialization routine.  This will enable the
    // floating-point unit as well, so that does not need to be done here.
    //
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");

#elif defined( __GNUC__	)
// Reset code for gcc
// TODO: Add test to be sure gcc

    uint32_t *pui32Src, *pui32Dest;

    //
    // Copy the data segment initializers from flash to SRAM.
    //
    pui32Src = &_etext;
    for(pui32Dest = &_data; pui32Dest < &_edata; )
    {
        *pui32Dest++ = *pui32Src++;
    }

    //
    // Zero fill the bss segment.
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

#endif

// FIXME: incorporate FPU into eLua
#if defined(LUA_NUMBER_INTEGRAL)

// No floating point, so turn off FP stack for ISR
  MAP_FPUStackingDisable();
  
#else
#if defined( BUILD_LUA_INT_HANDLERS )
    // FPU lazy stacking - in case use floating point in ISR
    MAP_FPULazyStackingEnable();
#else
    // TODO:  Could make this a configurable option
//    MAP_FPUStackingDisable();
#endif    

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
    HWREG(NVIC_CPAC) = ((HWREG(NVIC_CPAC) &
                         ~(NVIC_CPAC_CP10_M | NVIC_CPAC_CP11_M)) |
                        NVIC_CPAC_CP10_FULL | NVIC_CPAC_CP11_FULL);

#endif //  defined(LUA_NUMBER_INTEGRAL)

    //
    // Call the application's entry point.
    //

#if defined( __GNUC__	)   // gcc

    main();

#elif 0                   // IAR Embedded Workbench
// FIXME: Find macro to test for IAR compiler
    __iar_program_start();

#else

#warning Unrecognized compiler.

#endif // __GNUC__
}

// eLua - Platform specific includes
//#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

// FIXME: Assumes console is UART, will not give feedback for USB_CDC
#if defined( BUILD_USB_CDC )
#warning No error messages for Nmi/Fault with USB_CDC
#endif

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
#if !defined( BUILD_USB_CDC )
  MAP_UARTCharPut( uart_base[CON_UART_ID], 'N' );
  MAP_UARTCharPut( uart_base[CON_UART_ID], 'M' );
  MAP_UARTCharPut( uart_base[CON_UART_ID], 'I' );
#endif
  while(1)
    {
#if !defined( BUILD_USB_CDC )
      MAP_UARTCharPut( uart_base[CON_UART_ID], '!' );
#endif
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
#if !defined( BUILD_USB_CDC )
  MAP_UARTCharPut( uart_base[CON_UART_ID], 'F' );
  MAP_UARTCharPut( uart_base[CON_UART_ID], 'a' );
  MAP_UARTCharPut( uart_base[CON_UART_ID], 'u' );
  MAP_UARTCharPut( uart_base[CON_UART_ID], 'l' );
  MAP_UARTCharPut( uart_base[CON_UART_ID], 't' );
#endif
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
#if !defined( BUILD_USB_CDC )
  MAP_UARTCharPut( uart_base[CON_UART_ID], 'I' );
  MAP_UARTCharPut( uart_base[CON_UART_ID], 'n' );
  MAP_UARTCharPut( uart_base[CON_UART_ID], 't' );
#endif
    while(1)
    {
#if !defined( BUILD_USB_CDC )
      MAP_UARTCharPut( uart_base[CON_UART_ID], '*' );
#endif
    }
}
