-- This is the platform specific board configuration file
-- SAM
-- It is used by the generic board configuration system (config/)

module( ..., package.seeall )
local comps = require "components"
local at = require "attributes"

-- Add specific components to the 'components' table
function add_platform_components( t, board, cpu )
  t.cdc = comps.cdc_uart()
  t.sam34_rand = { macro = "BUILD_RAND" }
  t.sam34_rtc = { macro = "BUILD_RTC" }
end

-- Add specific configuration to the 'configs' table
function add_platform_configs( t, board, cpu )
end

-- Return an array of all the available platform modules for the given cpu
function get_platform_modules( board, cpu )
  return {
    rand = { guards = { "BUILD_RAND" }, lib = '"rand"', open = false },
    rtc = { guards = { "BUILD_RTC" }, lib = '"rtc"', open = false }
  }
end

