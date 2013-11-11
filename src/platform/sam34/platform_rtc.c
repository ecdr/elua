// RTC platform - SAM

#include "platform_conf.h"
#include <asf.h>
#include "module_rtc.h"

//#define PLATFORM_RTC_EXTERNAL_CRYSTAL

// Maybe should give function call to select hour mode
#define PLATFORM_CLOCK_24HR 0
#define PLATFORM_CLOCK_12HR 1

#ifndef PLATFORM_CLOCK_HR_MODE
#define PLATFORM_CLOCK_HR_MODE  PLATFORM_CLOCK_12HR
#endif 

#ifdef BUILD_RTC


// Limit how long will wait for the 32khz crystal to be ready
// FIXME: Number is arbitrary at the moment, check manuals to see how long should wait
#define RTC_TIMEOUT_COUNT 50000000


static u8 rtc_initialized = 0;	// Have we set up RTC?

// void RTC_Handler( void )
  
/* Initialise the RTC subsystem:
 * Return 0 for success, non-zero if no RTC or failure
 *
 * TODO: If none, do the RTC stuff using the one-second-tick system timer (or rtt?)
 */
int platform_rtc_init( void )
{
#if PLATFORM_RTC_EXTERNAL_CRYSTAL
  u32 waitcount;
#endif //  PLATFORM_RTC_EXTERNAL_CRYSTAL

  if (rtc_initialized) return 0;

#if PLATFORM_RTC_EXTERNAL_CRYSTAL

#warning Be sure actually have 32khz crystal before using this module

  pmc_switch_sclk_to_32kxtal(PMC_OSC_XTAL);

  for (waitcount = 0; waitcount < RTC_TIMEOUT_COUNT; waitcount++)
    if (pmc_osc_is_ready_32kxtal())
      {
      rtc_set_hour_mode(RTC, PLATFORM_CLOCK_HR_MODE);    // 24 hour mode

      rtc_initialized = 1;
      return 0;
      };
  // timeout - 32k crystal never got ready
  // FIXME: see if there is any other cleanup to do (e.g. change sclk to a different source)
  // Or, if there is no recovery, can it print an error message?
  return 1;
#else
  rtc_initialized = 1;
  rtc_set_hour_mode(RTC, PLATFORM_CLOCK_HR_MODE);    // 24 hour mode
  return 0;
#endif //  PLATFORM_RTC_EXTERNAL_CRYSTAL
}


void platform_read_rtc(uint32_t * hour, uint32_t * minute, uint32_t * second, uint32_t * year, uint32_t * month, uint32_t * day, uint32_t * week)
{
  rtc_get_time(RTC, hour, minute, second);
  rtc_get_date(RTC, year, month, day, week);
}


uint32_t platform_write_rtc(uint32_t hour, uint32_t minute, uint32_t second, uint32_t year, uint32_t month, uint32_t day, uint32_t week)
{
  u32 result;
 	result = rtc_set_time (RTC, hour, minute, second);
 	result |= rtc_set_date (RTC, year, month, day, week);   // FIXME: result handling - does this make sense?
  return result;
}

#endif // BUILD_RTC
