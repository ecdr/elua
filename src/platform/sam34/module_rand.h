// ****************************************************************************
// Random sequence generator - random sequence of numbers

#ifndef __MODULE_RAND_H__
#define __MODULE_RAND_H__

extern u8 platform_rand_init(void); // Return PLATFORM_ERR if problem (e.g. no random generator)
extern u32 platform_rand_next(void);

#endif // #ifndef __MODULE_RAND_H__
