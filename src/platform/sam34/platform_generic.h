// Generic platform configuration file

#ifndef __PLATFORM_GENERIC_H__
#define __PLATFORM_GENERIC_H__

// TODO: Why is this setting here, rather than in board.lua file (e.g. systimer=true) or in CPU file? 
// Could do away with this file entirely on several platforms if move this one setting
#define PLATFORM_HAS_SYSTIMER

#ifndef debug
#define debug(a) printf(a)
#endif

// Macro for waiting - loop at the moment, but make it easier to slow down, use low power, etc.
#define WAIT_WHILE( cond ) while( cond );
// FIXME: Busy waiting (should do low power/sleep and wake on interrupt, or do something useful)

#define BUILD_SHELL_REFLASH

#if defined(BUILD_SHELL_REFLASH)
#define PLATFORM_SHELL_COMMANDS { "reflash", platform_shell_reflash },

#define PLATFORM_SHELL_HELP SHELL_HELP( reflash );

#define PLATFORM_SHELL_HELP_DATA  SHELL_INFO( reflash ),

#define PLATFORM_SHELL_FUNC   SHELL_FUNC( platform_shell_reflash );

#endif // BUILD_SHELL_REFLASH

#endif // #ifndef __PLATFORM_GENERIC_H__

