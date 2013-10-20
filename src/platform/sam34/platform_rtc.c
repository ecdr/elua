// RTC platform - SAM#ifdef BUILD_RTCstatic u8 rtc_initialized = 0;	// Have we set up RTC?  /* Initialise the RTC subsystem: * Return 0 for success, non-zero if no RTC or failure * * TODO: If none, do the RTC stuff using the one-second-tick system timer */int platform_rtc_init(){  if (rtc_initialized) return 0;  pmc_switch_sclk_to_32kxtal(PMC_OSC_XTAL);  while (!pmc_osc_is_ready_32kxtal());  rtc_set_hour_mode(RTC, 0);    // 24 hour mode  rtc_initialized++;  return 0;}void platform_read_rtc(uint32_t * hour, uint32_t * minute, uint32_t * second, uint32_t * year, uint32_t * month, uint32_t * day, uint32_t * week);{  rtc_get_time(RTC, hour, minute, second);  rtc_get_date(RTC, year, month, day, week);}uint32_t platform_write_rtc(uint32_t hour, uint32_t minute, uint32_t second, uint32_t year, uint32_t month, uint32_t day, uint32_t week);{  u32 result; 	result = rtc_set_time (RTC, hour, minute, second); 	result |= rtc_set_date (RTC, year, month, day, week);   // FIXME: result handling - does this make sense?  return result;}#endif // BUILD_RTC