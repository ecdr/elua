-- EK-TM4C123G build configuration
-- CPU lm4f230h5qr same as TM4C123GH6PM

-- ToDo: 2 CAN
-- ToDo: 2 PWM (16 channels)
	-- ToDo: PWM needs support for built-in PWM
-- ToDo: QEI

return {
  cpu = 'tm4c123g',
  components = {
    sercon = { uart = 0, speed = 115200 },
    wofs = true,
    romfs = true,
    shell = true,
    term = { lines = 25, cols = 80 },
    linenoise = { shell_lines = 3, lua_lines = 10 }, -- was 10/50 on mbed
    cints = true,
    lm3s_pio = true,
    rpc = { uart = 0, speed = 115200 },
    adc = { buf_size = 2 },
--    comp = true,
    xmodem = true,
  },
  config = {
    vtmr = { num = 4, freq = 4 },
    tm4c_unlock_nmi = true, 	-- Allow use of PF0 and PD7 as GPIO pins
--    tm4c_target = { revision = B1 },
  },
  modules = {
    generic = { 'all', '-i2c', '-net' },
	-- ToDo: i2c needs code for lm3 etc (8962, LM4F, )
    platform = 'all'
  },
  macros = { 
     { "PART_TM4C123GH6PM", "" },	-- Needed for Tivaware PIN_MAP.
     { "TARGET_IS_TM4C123_RA0", ""}, -- Replacement for BLIZZARD_Rxx
     { "UART_ALT_CLOCK", ""},     -- Alternative clock for serial port, don't know if need for this processor
--     { "BUILD_COMP", ""},
 },
  build = {
    target = "lualong"			-- Integer only to conserve memory
  }
}
