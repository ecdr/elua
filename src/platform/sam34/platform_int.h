// eLua platform interrupts defined in platform_int.c

#ifndef __PLATFORM_INT_H__
#define __PLATFORM_INT_H__


extern void uart0_handler(void);
extern void uart1_handler(void);
extern void uart2_handler(void);

extern void gpioa_handler(void);
extern void gpiob_handler(void);
extern void gpioc_handler(void);
extern void gpiod_handler(void);
extern void gpioe_handler(void);
extern void gpiof_handler(void);

extern void tmr0_handler(void);
extern void tmr1_handler();
extern void tmr2_handler();
extern void tmr3_handler();
extern void tmr4_handler();
extern void tmr5_handler();
extern void tmr6_handler(void);
extern void tmr7_handler(void);
extern void tmr8_handler(void);

#endif
