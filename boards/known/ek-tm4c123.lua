-- EK-TM4C123G build configuration
-- CPU lm4f230h5qr same as TM4C123GH6PM

-- Should make it inherit from lm4F120 but not sure how to handle macros 
-- local t = dofile( "boards/known/ek-lm4f120.lua" )
-- t.cpu = 'tm4c123'	 				-- Fixme: Not implemented yet
-- t.macros =  { "PART_LM4F230H5QR", "" },	-- Needed for PIN_MAP.	-- FixMe: How do I override PART from LM4F120
-- modules add pwm
-- return t

return {
  cpu = 'lm4f230',
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
    xmodem = true,
  },
  config = {
    vtmr = { num = 4, freq = 4 },
  },
  modules = {
    generic = { 'all', '-i2c', '-net' },
	-- ToDo: i2c needs code for lm3 etc (8962, LM4F, )
    platform = 'all'
  },
  macros = { { "PART_LM4F230H5QR", "" },	-- Needed for Stellarisware PIN_MAP.
--    { "PART_TM4C123GH6PM", "" },	-- Needed for Tivaware PIN_MAP.
		 { "PIO_UNLOCK_NMI", ""} 	-- Allow use of PF0 and PD7 as GPIO pins
 },
  build = {
    target = "lualong"			-- Integer only to conserve memory
  }
}

-- ToDo: 2 CAN
-- ToDo: 2 PWM (16 channels)
	-- ToDo: PWM needs support for built-in PWM
-- ToDo: QEI
