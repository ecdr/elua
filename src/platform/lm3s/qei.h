// Quadrature Encoding Interface (QEI)

#ifndef __LM3S_QEI_H__
#define __LM3S_QEI_H__

#ifdef ENABLE_QEI


#include "platform_conf.h"

/* Encoder 0 and 1 each with phase A and B */
enum valid_encoder_ids { LM3S_QEI_CH0 = 1, LM3S_QEI_CH1, LM3S_QEI_CH01 };
enum valid_encoder_phases { LM3S_QEI_PHA = 0, LM3S_QEI_PHAB };
/* Defines whether phases are switched before processing */
enum qei_swap_codes { LM3S_QEI_NO_SWAP = 0, LM3S_QEI_SWAP };
/* Defines whether an index pulse is used */
enum qei_index_codes { LM3S_QEI_NO_INDEX = 0, LM3S_QEI_INDEX };
/* Error Codes */
enum qei_error_codes { LM3S_QEI_ERR_OK = 0, LM3S_QEI_ERR_VEL_NOT_ENABLED, LM3S_QEI_ERR_ENC_NOT_ENABLED };

/* qei_flag keeps track of encoders/channels that are enabled, as well
 * as whether they are enabled for velocity measurement.
 * BIT0 Channel0 Enabled
 * BIT1 Channel1 Enabled
 * BIT2 Channel0 Velocity Enabled
 * BIT3 Channel1 Velocity Enabled
 * BIT4 Channel0 Initialized
 * BIT5 Channel1 Initialized
 * BIT6-7 Unused. */

u8 qei_flag;
#define VEL_FLAG_OFFSET 2
#define INIT_FLAG_OFFSET 4

/* vel_period is time (us) over which to measure velocity. vel_ticks is
 * this period converted into counts on the system clock. */
u32 vel_ticks;

/* Function Prototypes */

int platform_qei_exists( u8 enc_id );
void lm3s_qei_setPosition( u8 enc_id, u32 position );

void lm3s_qei_init( u8 enc_id, u8 phase, u8 swap, u8 index, u32 max_count );
void lm3s_qei_vel_init( u8 enc_id, u32 vel_period );
void lm3s_qei_enable( u8 enc_id );
void lm3s_qei_disable( u8 enc_id );
u32 lm3s_qei_get_sys_clk();
u32 lm3s_qei_getPulses( u8 enc_id );
u32 lm3s_qei_getPosition( u8 enc_id );
long lm3s_qei_getDirection( u8 enc_id );

#endif // ENABLE_QEI

#endif
