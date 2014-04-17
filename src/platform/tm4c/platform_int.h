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
// #if NUM_PIO > 6
extern void gpiog_handler(void);
extern void gpioh_handler(void);

extern void gpioj_handler(void);
extern void gpiok_handler(void);
extern void gpiol_handler(void);
extern void gpiom_handler(void);
extern void gpion_handler(void);

extern void gpiop_handler(void);
extern void gpioq_handler(void);
// #endif // NUM_PIO

extern void tmr0_handler(void);
extern void tmr1_handler();
extern void tmr2_handler();
extern void tmr3_handler();
extern void tmr4_handler();
extern void tmr5_handler();

#if defined( FORLM4F )
extern void tmr6_handler(void);
extern void tmr7_handler(void);
extern void tmr8_handler(void);
extern void tmr9_handler(void);
extern void tmr10_handler(void);
extern void tmr11_handler(void);
#endif // defined( FORLM4F )

#endif
