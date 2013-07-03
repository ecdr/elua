-- EK-LM4F120 build configuration

return {
  cpu = 'lm4f120',
  components = {
    sercon = { uart = 0, speed = 115200 },
    wofs = true,
    romfs = true,
    shell = true,
    term = { lines = 25, cols = 80 },
    cints = true,
--    lm3s_disp = false,
    lm3s_pio = true,
    rpc = { uart = 0, speed = 115200 },
    adc = { buf_size = 2 },	-- Not sure about this
    xmodem = true,
  },
  config = {
    vtmr = { num = 4, freq = 4 },
  },
  modules = {
    generic = { 'all', '-i2c', '-net', '-pwm', '-adc', '-can' },
	-- ToDo: PWM use timers instead 
	-- ToDo: ADC not tested
	-- ToDo: CAN not tested
	-- ToDo: i2c needs code for lm3 etc (8962, LM4F, )
    platform = 'all', '-pwm'
  },
  macros = { { "PART_LM4F120H5QR", "" } }	
-- Needed for PIN_MAP.
}

