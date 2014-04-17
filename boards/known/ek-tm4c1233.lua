-- EK-TM4C1233 (EK-LM4F120) build configuration
-- CPU same as the TM4C1233H6PM

return {
  cpu = 'tm4c1233',
  components = {
    sercon = { uart = 0, speed = 115200 },
    wofs = true,
    romfs = true,
    shell = true,
    term = { lines = 25, cols = 80 },
-- Fixme: Testing, I thought it had line editor before, maybe this is why gone?
    linenoise = { shell_lines = 3, lua_lines = 10 }, -- was 10/50 on mbed
    cints = true,
    luaints = true,	-- Not set by default, think have it working now
    lm3s_pio = true,

    rpc = { uart = 0, speed = 115200 },
    adc = { buf_size = 2 },	-- Not sure about this
    xmodem = true,
-- MMCFS works (No MMC slot on board, but can use boosterpack)
-- ToDo: Was pin 6 for earlier trial, pin 3 replaces FSS  I think code doesn't use FSS, but check that
--    mmcfs = { spi = 0, cs_port = 0, cs_pin = 3 },	
  },
  config = {
--    vtmr = { num = 4, freq = 4 },
    vtmr = false,
    tm4c_unlock_nmi = true, 	-- Allow use of PF0 and PD7 as GPIO pins
--    tm4c_target = { revision = A3 },
  },
  modules = {
    generic = { 'all', '-i2c', '-net', '-pwm', '-mmc' }, -- '+bitarray' does not work
	-- ToDo: CAN tested in loopback, needs testing with transceiver
	-- ToDo: PWM needs code to use timers instead
	-- ToDo: i2c needs code for lm3 etc (8962, LM4F, )
    platform = {'all', '-pwm', '-mmc'},
  },
  macros = { -- { "PART_LM4F120H5QR", "" },	-- Needed for Stellaris PIN_MAP.
     { "PART_TM4C1233H6PM", ""}, -- Needed for Tivaware PIN_MAP.
     { "TARGET_IS_TM4C123_RA0", ""},  -- Todo: make platform component to handle target revisions
     { "UART_ALT_CLOCK", ""},     
--     { "BUILD_COMP", ""},
  },
  build = {
    target = "lualong"			-- Integer only to conserve memory
  }
}

