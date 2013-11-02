// Generic platform configuration file

#ifndef __PLATFORM_GENERIC_H__
#define __PLATFORM_GENERIC_H__

// TODO: Why is this setting here, rather than in board.lua file (e.g. systimer=true) or in CPU file? 
// Could do away with this file entirely on several platforms if move this one setting
#define PLATFORM_HAS_SYSTIMER

#endif // #ifndef __PLATFORM_GENERIC_H__

