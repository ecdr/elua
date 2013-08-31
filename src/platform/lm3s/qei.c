// eLua Module for LM3S Quadrature Encoding Interface (QEI) Support
// qei is a platform-dependent (LM3S) module, that binds to Lua the basic API from Texas Instruments

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "platform.h"
#include "common.h"
#include "auxmods.h"
#include "lrotable.h"
#include "platform_conf.h"
#include "qei.h"

#ifdef ENABLE_QEI

static u32 vel_modifier = 0;

// Fixme: Figure how to handle errors in functions returning voids

//Lua: lm3s.qei.init( encoder_id, phase, swap, index, max_count )
static void qei_init( lua_State *L )
{
    u8 enc_id = ( u8 )luaL_checkinteger( L, 1 );
    u8 phase = ( u8 )luaL_checkinteger( L, 2 );
    u8 swap = ( u8 )luaL_checkinteger( L, 3 );
    u8 index = ( u8 )luaL_checkinteger( L, 4 );
    u32 max_count = ( u32 )luaL_checkinteger( L, 5 );

    MOD_CHECK_ID( qei, enc_id );
    if ( phase > LM3S_QEI_PHAB || swap > LM3S_QEI_SWAP || index > LM3S_QEI_INDEX)
      return luaL_error( L, "qei invalid argument" );

// Todo: Not sure if there is a range on max_count
    lm3s_qei_init( enc_id, phase, swap, index, max_count );
}

//Lua: lm3s.qei.velInit( encoder_id, vel_period, ppr, edges )
static void qei_velInit( lua_State *L )
{
    u8 enc_id = ( u8 )luaL_checkinteger( L, 1 );
    MOD_CHECK_ID( qei, enc_id );

    u32 vel_period = ( u32 )luaL_checkinteger( L, 2 );
    int ppr = ( int )luaL_checkinteger( L, 3 );
    int edges = ( int )luaL_checkinteger( L, 4 );

// ToDo: Are there any useful range checks to make on arguments?
    lm3s_qei_vel_init( enc_id, vel_period );

    qei_flag |= ( enc_id << VEL_FLAG_OFFSET );  //Sets encoder velocity flag
    u32 clk_freq = lm3s_qei_get_sys_clk();
    vel_modifier = (clk_freq * 60) / (vel_ticks * ppr * edges);
}

//Lua: lm3s.qei.enable( encoder_id )
static void qei_enable( lua_State *L )
{
    u8 enc_id = ( u8 )luaL_checkinteger( L, 1 );
    MOD_CHECK_ID( qei, enc_id );

    lm3s_qei_enable( enc_id );
    qei_flag |= enc_id;
}

//Lua: lm3s.qei.disable( encoder_id )
static void qei_disable( lua_State *L )
{
    u8 enc_id = ( u8 )luaL_checkinteger( L, 1 );
    MOD_CHECK_ID( qei, enc_id );

    lm3s_qei_disable( enc_id );
    qei_flag &= ~enc_id;
}

//Lua: vel, err = lm3s.qei.getVelPulses( encoder_id )
static int qei_getVelPulses( lua_State *L )
{
    u8 enc_id = ( u8 )luaL_checkinteger( L, 1 );
    MOD_CHECK_ID( qei, enc_id );
    int err = 0;
    if( (enc_id == LM3S_QEI_CH01) || !(qei_flag & enc_id) )
    {
        err = 2;
        lua_pushinteger( L, -1 );
        lua_pushinteger( L, err );
        return 2;
    }
    else if( !(qei_flag & (enc_id << VEL_FLAG_OFFSET) ) )
    {
        err = 1;
        lua_pushinteger( L, -1 );
        lua_pushinteger( L, err );
        return 2;
    }
    u32 pulses = lm3s_qei_getPulses( enc_id );
    lua_pushinteger( L, pulses );
    lua_pushinteger( L, err );
    return 2;
}

//Lua: rpm, err = lm3s.qei.getRPM( encoder_id )
static int qei_getRPM( lua_State *L )
{
    u8 enc_id = ( u8 )luaL_checkinteger( L, 1 );
    MOD_CHECK_ID( qei, enc_id );
    int err = 0;
    if( (enc_id == LM3S_QEI_CH01) || !(qei_flag & enc_id) )
    {
        err = 2;
        lua_pushinteger( L, -1 );
        lua_pushinteger( L, err );
        return 2;
    }
    else if( !(qei_flag & (enc_id << VEL_FLAG_OFFSET) ) )
    {
        err = 1;
        lua_pushinteger( L, -1 );
        lua_pushinteger( L, err );
        return 2;
    }
    u32 pulses = lm3s_qei_getPulses( enc_id );
    s32 rpm = pulses * vel_modifier * lm3s_qei_getDirection( enc_id );
    lua_pushinteger( L, rpm );
    lua_pushinteger( L, err );
    return 2;
}

//Lua: pos, err = lm3s.qei.getPosition( encoder_id )
static int qei_getPosition( lua_State *L )
{
    u8 enc_id = ( u8 )luaL_checkinteger( L, 1 );
    MOD_CHECK_ID( qei, enc_id );
    int err = 0;
    if( (enc_id == LM3S_QEI_CH01) || !(qei_flag & enc_id) )
    {
// FIXME: What is the magic err number 2 being returned?  Should it be one of the error enum values?
        err = 2;
        lua_pushinteger( L, -1 );
        lua_pushinteger( L, err );
        return 2;
    }
    lua_pushinteger( L, lm3s_qei_getPosition( enc_id ) );
    lua_pushinteger( L, err );
    return 2;
}

// Fixme: The returns are probably not right here - check some other code for examples
//Lua: err = lm3s.qei.setPosition( encoder_id, position)
static int qei_setPosition( lua_State *L )
{
    u8 enc_id = ( u8 )luaL_checkinteger( L, 1 );
    MOD_CHECK_ID( qei, enc_id );
    u32 position = ( u32 )luaL_checkinteger( L, 2 );
    int err = 0;
    if( (enc_id == LM3S_QEI_CH01) || !(qei_flag & enc_id) )
    {
// FIXME: What is the magic err number 2 being returned?  Should it be one of the error enum values?
        err = 2;
        lua_pushinteger( L, err );
        return 1;
    }
    lm3s_qei_setPosition( enc_id, position );
    lua_pushinteger( L, err );
    return 1;
}

// Module function map
#define MIN_OPT_LEVEL 2
#include "lrodefs.h"
const LUA_REG_TYPE qei_map[] =
{
    { LSTRKEY( "init" ), LFUNCVAL( qei_init ) },
    { LSTRKEY( "velInit" ), LFUNCVAL( qei_velInit ) },
    { LSTRKEY( "enable" ), LFUNCVAL( qei_enable ) },
    { LSTRKEY( "disable" ), LFUNCVAL( qei_disable ) },
    { LSTRKEY( "getVelPulses" ), LFUNCVAL( qei_getVelPulses ) },
    { LSTRKEY( "getRPM" ), LFUNCVAL( qei_getRPM ) },
    { LSTRKEY( "getPosition" ), LFUNCVAL( qei_getPosition ) },
    { LSTRKEY( "setPosition" ), LFUNCVAL( qei_setPosition ) },

    { LSTRKEY( "PHA" ), LNUMVAL( LM3S_QEI_PHA ) },
    { LSTRKEY( "PHAB" ), LNUMVAL( LM3S_QEI_PHAB ) },
    { LSTRKEY( "CH0" ), LNUMVAL( LM3S_QEI_CH0 ) },
    { LSTRKEY( "CH01" ), LNUMVAL( LM3S_QEI_CH01 ) },
    { LSTRKEY( "CH1" ), LNUMVAL( LM3S_QEI_CH1 ) },
    { LSTRKEY( "NO_SWAP" ), LNUMVAL( LM3S_QEI_NO_SWAP ) },
    { LSTRKEY( "SWAP" ), LNUMVAL( LM3S_QEI_SWAP ) },
    { LSTRKEY( "NO_INDEX" ), LNUMVAL( LM3S_QEI_NO_INDEX ) },
    { LSTRKEY( "INDEX" ), LNUMVAL( LM3S_QEI_INDEX ) },
    { LSTRKEY( "ERR_OK" ), LNUMVAL( LM3S_QEI_ERR_OK ) },
    { LSTRKEY( "ERR_VELOCITY_NOT_ENABLED" ), LNUMVAL( LM3S_QEI_ERR_VEL_NOT_ENABLED ) },
    { LSTRKEY( "ERR_ENCODER_NOT_ENABLED" ), LNUMVAL( LM3S_QEI_ERR_ENC_NOT_ENABLED ) },

    { LNILKEY, LNILVAL }
};

/*endif ENABLE_QEI*/
#endif
