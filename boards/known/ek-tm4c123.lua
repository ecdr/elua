-- EK-TM4C123 build configuration
-- CPU same as lm4f230h5qr

return {
  cpu = 'lm4f120',
  components = {
    sercon = { uart = 0, speed = 115200 },
    wofs = true,
    romfs = true,
    shell = true,
    term = { lines = 25, cols = 80 },
    cints = true,
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
	-- ToDo: ADC not tested
	-- ToDo: CAN not tested
	-- ToDo: PWM use timers instead
	-- ToDo: i2c needs code for lm3 etc (8962, LM4F, )
    platform = 'all', '-pwm'
  },
  macros = { { "PART_LM4F230H5QR", "" } }
-- Needed for PIN_MAP.
}

-- ToDo: Need to add cpu to platform code (CAN ports, PWMs)
-- ToDo: 2 CAN
-- ToDo: 2 PWM
