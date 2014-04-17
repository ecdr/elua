-- This is the platform specific board configuration file
-- It is used by the generic board configuration system (config/)

module( ..., package.seeall )
local comps = require "components"
local at = require "attributes"

-- Add specific components to the 'components' table
function add_platform_components( t, board, cpu )
  t.cdc = comps.cdc_uart()
  board = board:upper()
  t.lm3s_pio = { macro = 'ENABLE_LM3S_GPIO' }
end

-- Add specific configuration to the 'configs' table
function add_platform_configs( t, board, cpu )
  t.tm4c_adc_timers = {
    attrs = {
      first_timer = at.int_attr( 'ADC_TIMER_FIRST_ID', 0 ),
      num_timers = at.int_attr( 'ADC_NUM_TIMERS', 0 )
    },
    required = { first_timer = 0, num_timers = "NUM_TIMER" }
  }
  t.tm4c_unlock_nmi = { macro = "PIO_UNLOCK_NMI" }
end

-- Return an array of all the available platform modules for the given cpu
function get_platform_modules( board, cpu )
  m = { pio = { guards = { 'ENABLE_LM3S_GPIO' }, lib = '"pio"', map = "lm3s_pio_map", open = false } }
  board = board:upper()
  return m
end

