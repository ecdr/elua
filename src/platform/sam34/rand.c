// ****************************************************************************
// Random sequence generator

// TODO: Add range selector (e.g. # of bits or maximum return value)

// STM32F4 also has an RNG

#include "lauxlib.h"
#include "lrotable.h"
#include "platform_conf.h"

#ifdef BUILD_RAND

#include "lrotable.h"
#include "platform_conf.h"


extern u8 platform_rand_init(); // Return PLATFORM_ERR if problem (e.g. no random generator)
extern u32 platform_rand_next();


// Lua: data = platform.rand.next()
static int rand_next( lua_State *L )
{
  lua_pushnumber( L, ( lua_Number )( platform_rand_next() ) ); 
  return 1;
}

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"

// platform.rand module function map
const LUA_REG_TYPE rand_map[] =
{
  { LSTRKEY( "next" ), LFUNCVAL( rand_next ) },
  { LNILKEY, LNILVAL }
};

#endif // BUILD_RAND
