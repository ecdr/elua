-- EK-LM4F120 build configuration

return {
  cpu = 'lm4f120',
  components = {
    sercon = { uart = 0, speed = 115200 },
    wofs = true,
    romfs = true,
    shell = true,
    term = { lines = 25, cols = 80 },
-- Fixme: Testing, I thought it had line editor before, maybe this is why gone?
--    linenoise = { shell_lines = 3, lua_lines = 10 }, -- was 10/50 on mbed
    cints = true,
    lm3s_pio = true,
    rpc = { uart = 0, speed = 115200 },
    adc = { buf_size = 2 },	-- Not sure about this
    xmodem = true,
-- MMCFS works (No MMC slot on board, but can use boosterpack)
--    mmcfs = { spi = 0, cs_port = 0, cs_pin = 6 },
  },
  config = {
    vtmr = { num = 4, freq = 4 },
  },
  modules = {
    generic = { 'all', '-i2c', '-net', '-pwm', '-can' },
	-- ToDo: CAN not tested
	-- ToDo: PWM needs code to use timers instead
	-- ToDo: i2c needs code for lm3 etc (8962, LM4F, )
    platform = 'all', '-pwm'
  },
-- Needed for PIN_MAP.
  macros = { { "PART_LM4F120H5QR", "" } },
  build = {
    target = "lualong"			-- Integer only to conserve memory
  }
}

