/**
 * \file
 *
 * \brief Autogenerated API include file for the Atmel Software Framework (ASF)
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef ASF_H
#define ASF_H

/*
 * This file includes all API header files for the selected drivers from ASF.
 * Note: There might be duplicate includes required by more than one driver.
 *
 * The file is automatically generated and will be re-written when
 * running the ASF driver selector tool. Any changes will be discarded.
 */

// From module: ADC - Analog-to-digital Converter
#include <adc.h>

// From module: CAN - Controller Area Network
#include <can.h>

// From module: CHIPID - Chip Identifier
#include <chipid.h>

// From module: Common SAM compiler driver
#include <compiler.h>
#include <status_codes.h>

// From module: DACC - Digital-to-Analog Converter
#include <dacc.h>

// From module: Delay routines
#include <delay.h>

// From module: EEFC - Enhanced Embedded Flash Controller
#include <efc.h>

// From module: FIFO - First-In-First-Out circular buffer
#include <fifo.h>

// From module: Flash - SAM Flash Service API
#include <flash_efc.h>

// From module: GPIO - General purpose Input/Output
#include <gpio.h>

// From module: Generic board support
#include <board.h>

// From module: IOPORT - General purpose I/O service
#include <ioport.h>

// From module: Interrupt management - SAM implementation
#include <interrupt.h>

// From module: PIO - Parallel Input/Output Controller
#include <pio.h>

// From module: PMC - Power Management Controller
#include <pmc.h>
#include <sleep.h>

// From module: PWM - Pulse Width Modulation
#include <pwm.h>

// From module: Part identification macros
#include <parts.h>

// From module: RTC - Real Time Clock
#include <rtc.h>

// From module: SAM3X startup code
#include <exceptions.h>

// From module: SPI - SAM Implementation
#include <spi_master.h>
#include <spi_master.h>

// From module: SPI - Serial Peripheral Interface
#include <spi.h>

// From module: Sleep manager - SAM implementation
#include <sam/sleepmgr.h>
#include <sleepmgr.h>

// From module: Standard serial I/O (stdio) - SAM implementation
#include <stdio_serial.h>

// From module: System Clock Control - SAM3X/A implementation
#include <sysclk.h>

// From module: TC - Timer Counter
#include <tc.h>

// From module: TRNG - True Random Number Generator Register
#include <trng.h>

// From module: TWI - Two-Wire Interface - SAM implementation
#include <sam_twi/twi_master.h>
#include <sam_twi/twi_slave.h>
#include <twi_master.h>
#include <twi_slave.h>

// From module: TWI - Two-wire Interface
#include <twi.h>

// From module: UART - Univ. Async Rec/Trans
#include <uart.h>

// From module: USART - Serial interface - SAM implementation for devices with both UART and USART
#include <serial.h>

// From module: USART - Univ. Syn Async Rec/Trans
#include <usart.h>

// #ifdef BUILD_USB_CDC
// Should make it not try to build the USB_CDC material if not defined
//  Currently it will build the USB files, just presumably doesn't use them (?)

// From module: USB CDC Protocol
#include <usb_protocol_cdc.h>

// From module: USB Device CDC (Single Interface Device)
#include <udi_cdc.h>

// From module: USB Device CDC Standard I/O (stdio) - SAM implementation
#include <stdio_usb.h>

// From module: USB Device Stack Core (Common API)
#include <udc.h>
#include <udd.h>

// #endif // BUILD_USB_CDC

// From module: pio_handler support enabled
#include <pio_handler.h>

#endif // ASF_H
