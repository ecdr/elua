// ****************************************************************************
// Random sequence generator - random sequence of numbers

// TODO: Add range selector (e.g. # of bits, or maximum return value)
// TODO: Maybe add number available flag access?

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

// Lua: data = platform.rand.fnext()  - In range (0.0 to 1.0)
static int rand_next_f( lua_State *L )
{
  lua_pushnumber( L, ( lua_Number )( platform_rand_next() )/PLATFORM_RAND_MAX ); 
  return 1;
}


#ifdef RFC1149 
// Random number generator for platforms that do not have built in random sequence generator
// http://www.xkcd.com/221/
int getRandomNumber()
{
    return 4;     //chosen by fair dice roll.
                  //guaranteed to be random.
}
//RFC 1149.5 specifies 4 as the standard IEEE-vetted random number
#endif

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"

// platform.rand module function map
const LUA_REG_TYPE rand_map[] =
{
  { LSTRKEY( "next" ), LFUNCVAL( rand_next ) },
  { LSTRKEY( "nextf" ), LFUNCVAL( rand_next_f ) },
  { LSTRKEY( "MAX" ), LNUMVAL( PLATFORM_RAND_MAX ) },
  { LNILKEY, LNILVAL }
};

/* FIXME: Not sure if need this
LUALIB_API int luaopen_rand( lua_State *L );

LUALIB_API int luaopen_rand( lua_State *L )
{
#if LUA_OPTIMIZE_MEMORY > 0
  return 0;
#else // #if LUA_OPTIMIZE_MEMORY > 0
  luaL_register( L, AUXLIB_RAND, rand_map );

  // Set it as its own metatable
  lua_pushvalue( L, -1 );
  lua_setmetatable( L, -2 );
  
  MOD_REG_NUMBER( L, "MAX", PLATFORM_RAND_MAX );

  return 1;
#endif // #if LUA_OPTIMIZE_MEMORY > 0
}
*/

#endif // BUILD_RAND
