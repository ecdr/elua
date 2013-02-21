// eLua platform interrupts defined in platform_int.c

#ifndef __PLATFORM_INT_H__
#define __PLATFORM_INT_H__


extern void uart0_handler(void);
extern void uart1_handler(void);
extern void uart2_handler(void);
extern void uart3_handler(void);
extern void uart4_handler(void);
extern void uart5_handler(void);
extern void uart6_handler(void);
extern void uart7_handler(void);

extern void gpioa_handler(void);
extern void gpiob_handler(void);
extern void gpioc_handler(void);
extern void gpiod_handler(void);
extern void gpioe_handler(void);
extern void gpiof_handler(void);
extern void gpiog_handler(void);
extern void gpioh_handler(void);
extern void gpioj_handler(void);

extern void tmr0_handler(void);
extern void tmr1_handler();
extern void tmr2_handler();
extern void tmr3_handler();
extern void tmr4_handler();
extern void tmr5_handler();

#endif
