// eLua module - Real Time Clock

// It provides two functions: platform.rtc.get() and platform.rtc.set(table).
// The first returns a table of lua-like exploded time/date fields:
//  year (1900-2099) month (1-12) day (1-13) hour (0-23) min (0-59) sec (0-59)
// The second accepts the same kind of table, replacing nil values with 0.
// Lua's isdst (daylight savings time) field is ignored and not returned 
// (so t.isdst is nil==false).

// TODO: Copied from mizar32 RTC, should split into platform part and elua module part (part way done, but 
//  Some of the mizar32 material removed (so would have to restore it to get version for mizar)
//  Documentation says there is also an RTC module in one of the other platforms, but haven't found it.
// TODO: add to other platforms - RTC hardware on lpc17xx, stm32f10x, stm32f4, maybe str7, str9, maybe on lm3

// TODO: Add day-of-week field (some devices use values 1-7 others 0-6);
// we may one day either use this or just calculate the DOW.

/* 
SAM
// Also has time, date alarms

// STM32
RTC_Init(RTC_InitTypeDef* RTC_InitStruct);
RTC_SetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
RTC_GetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
RTC_SetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);
RTC_GetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);
 */

#include "lauxlib.h"
#include "lrotable.h"
#include "platform_conf.h"

#include "module_rtc.h"

#ifdef BUILD_RTC

#define MSG_NO_RTC "No real-time clock present"


// The names of the Lua time table elements.
// In the order they are in the DS1337 register set.
static const char * const rtc_fieldnames[] = {
  "sec", "min", "hour", "wday", "day", "month", "year"
};
static const u16 rtc_field_minval[] = {
  0, 0, 0, 1, 1, 1, 1900
};
static const u16 rtc_field_maxval[] = {
  59, 59, 23, 7, 31, 12, 2099
};

// The number of fields in the above tables.
#define RTC_NFIELDS 7

// The order offsets of the fields in the DS1337 register set
#define SEC   0
#define MIN   1
#define HOUR  2
#define WDAY  3
#define DAY   4
#define MONTH 5
#define YEAR  6



// Read the time from the RTC.
static int rtc_get( lua_State *L )
{
  uint32_t hour, minute, second;
  uint32_t year, month, day, week;

  if( platform_rtc_init() )       // No RTC or init failed
    return luaL_error( L, MSG_NO_RTC );

  platform_read_rtc( &hour, &minute, &second, &year, &month, &day, &week );

  // Construct the table to return the result
  lua_createtable( L, 0, 7 );

  lua_pushstring( L, "sec" );
  lua_pushinteger( L, second );
  lua_rawset( L, -3 );

  lua_pushstring( L, "min" );
  lua_pushinteger( L, minute );
  lua_rawset( L, -3 );

  lua_pushstring( L, "hour" );
  lua_pushinteger( L, hour );
  lua_rawset( L, -3 );

  lua_pushstring( L, "wday" );
  lua_pushinteger( L, week );
  lua_rawset( L, -3 );

  lua_pushstring( L, "day" );
  lua_pushinteger( L, day );
  lua_rawset( L, -3 );

  lua_pushstring( L, "month" );
  lua_pushinteger( L, month );
  lua_rawset( L, -3 );

  lua_pushstring( L, "year" );
  lua_pushinteger( L, year );
  lua_rawset( L, -3 );

  return 1;
}

// platform.rtc.set()
// Parameter is a table containing fields with the usual Lua time field
// names.  
// Missing elements are not set and remain the same as they were.

static int rtc_set( lua_State *L )
{
  lua_Integer value;
  unsigned field;         // Which field are we handling (0-6) - FIXME: Should be an enum
  uint32_t hour, minute, second;
  uint32_t year, month, day, week;
  
  if( platform_rtc_init() )       // No RTC or init failed
    return luaL_error( L, MSG_NO_RTC );
  
  // Read the rtc so that unspecified fields remain at the same value as before
  // FIXME: This introduces an error if value not being set changes during processing
  platform_read_rtc( &hour, &minute, &second, &year, &month, &day, &week );

  // Set any values that they specified as table entries
  // TODO: May make more sense to unroll this loop, 
  //       make the fetch and check a subroutine
  //       Since the assignments are all laid out linear
  //       Would get rid of the field array and SEC, etc. constants
  for (field=0; field<RTC_NFIELDS; field++) {
    lua_getfield( L, 1, rtc_fieldnames[field] );
    switch( lua_type( L, -1 ) ) {
    case LUA_TNIL:
      // Do not set unspecified fields
      break;

    case LUA_TNUMBER:
    case LUA_TSTRING:
      value = lua_tointeger( L, -1 );
      if (value < rtc_field_minval[field] || value > rtc_field_maxval[field])
        return luaL_error( L, "Time value out of range" );

      // Special cases for some fields
      switch( field ) {
      case SEC: 
        second = value;
        break;
      case MIN: 
        minute = value;
        break;
      case HOUR:
        hour = value;
        break;
      case WDAY: 
        week = value;
        break;
      case DAY:
        day = value;
        break;
      case MONTH:
        month = value;
        break;
      case YEAR:
        year = value;
        break;
      }
      break;

    default:
      return luaL_error( L, "Time values must be numbers" );
    }
    lua_pop( L, 1 );
  }

  platform_write_rtc( hour, minute, second, year, month, day, week );

  return 0;
}


#define MIN_OPT_LEVEL 2
#include "lrodefs.h"

// platform.rtc.*() module function map
const LUA_REG_TYPE rtc_map[] =
{
  { LSTRKEY( "get" ), LFUNCVAL( rtc_get ) },
  { LSTRKEY( "set" ), LFUNCVAL( rtc_set ) },
  { LNILKEY, LNILVAL }
};

#endif // BUILD_RTC
