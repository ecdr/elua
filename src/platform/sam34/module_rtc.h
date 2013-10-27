// eLua module - Real Time Clock

#ifndef __MODULE_RTC_H__
#define __MODULE_RTC_H__

int platform_rtc_init(void);
void platform_read_rtc(uint32_t * hour, uint32_t * minute, uint32_t * second, uint32_t * year, uint32_t * month, uint32_t * day, uint32_t * week);
uint32_t platform_write_rtc(uint32_t hour, uint32_t minute, uint32_t second, uint32_t year, uint32_t month, uint32_t day, uint32_t week);

#endif // #ifndef __MODULE_RTC_H__
