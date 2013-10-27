// ****************************************************************************
// Random sequence generator - random sequence of numbers

// TODO: Add range selector (e.g. # of bits or maximum return value)

// STM32F4 also has an RNG

#include "lauxlib.h"
#include "lrotable.h"
#include "platform_conf.h"

#ifdef BUILD_RAND

#include "module_rand.h"


// Lua: data = platform.rand.next()
static int rand_next( lua_State *L )
{
  lua_pushnumber( L, ( lua_Number )( platform_rand_next() ) ); 
  return 1;
}

#ifdef RFC1149 
// Implementation for platforms that do not have built in random sequence generator
int getRandomNumber()
{
    return 4;     //chosen by fair dice roll.
                  //guaranteed to be random.
}
//RFC 1149.5 specifies 4 as the standard IEEE-vetted random number
// http://www.xkcd.com/221/
#endif

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"

// platform.rand module function map
const LUA_REG_TYPE rand_map[] =
{
  { LSTRKEY( "next" ), LFUNCVAL( rand_next ) },
  { LNILKEY, LNILVAL }
};

#endif // BUILD_RAND
