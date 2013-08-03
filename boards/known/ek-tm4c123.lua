-- EK-TM4C123 build configuration
-- CPU same as lm4f230h5qr

-- Should make it inherit from lm4F120 but not sure how to handle macros 
-- local t = dofile( "boards/known/ek-lm4f120.lua" )
-- t.cpu = 'tm4c123'	 				-- Fixme: Not implemented yet
-- t.macros =  { "PART_LM4F230H5QR", "" },	-- Needed for PIN_MAP.	-- FixMe: How do I override PART from LM4F120
-- return t

return {
  cpu = 'lm4f120',	-- Fixme: CPU 'tm4c123'
  components = {
    sercon = { uart = 0, speed = 115200 },
    wofs = true,
    romfs = true,
    shell = true,
    term = { lines = 25, cols = 80 },
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
    generic = { 'all', '-i2c', '-net', '-pwm' },
	-- ToDo: PWM needs support for built-in PWM
	-- ToDo: i2c needs code for lm3 etc (8962, LM4F, )
    platform = 'all', '-pwm'
  },
  macros = { { "PART_LM4F230H5QR", "" },	-- Needed for PIN_MAP.
		 { "PIO_UNLOCK_NMI", ""} 	-- Allow use of PF0 and PD7 as GPIO pins
 },
  build = {
    target = "lualong"			-- Integer only to conserve memory
  }
}

-- ToDo: Need to add cpu to platform code (CAN ports, PWMs)
-- ToDo: 2 CAN
-- ToDo: 2 PWM
-- ToDo: QEI
