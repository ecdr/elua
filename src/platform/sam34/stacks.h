// Stack size definitions

// Fixme: Look up what reasonable from ASF documentation (this was just arbitrarily coppied form STM32F4)

#ifndef __STACKS_H__
#define __STACKS_H__

#define  STACK_SIZE       4096
#define  STACK_SIZE_TOTAL ( STACK_SIZE )

#define __stack_size__  STACK_SIZE
// Fixme: __stack_size__ used by linker script - check that symbol is transferred to linker

#endif
