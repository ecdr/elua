-- EK-LM4F120 build configuration - Stellaris launchpad
-- CPU same as the TM4C1233H6PM

return {
  cpu = 'lm4f120',
  components = {
    sercon = { uart = 0, speed = 115200 },
    wofs = true,
    romfs = true,
    shell = true,
    shellinfo = true,
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
  },
  modules = {
    generic = { 'all', '-i2c', '-net', '-pwm', '-mmc', '-comp' }, -- '+bitarray' does not work
	-- ToDo: CAN tested in loopback, needs testing with transceiver
	-- ToDo: PWM needs code to use timers instead
	-- ToDo: i2c needs code for lm3 etc (8962, LM4F, )
    platform = {'all', '-pwm', '-mmc'},
  },
  macros = { 
    { "PART_LM4F120H5QR", "" },	-- Needed for Stellarisware PIN_MAP.
--  { "PART_TM4C1233H6PM", "" },	-- Needed for Tivaware PIN_MAP.  
    { "PLATFORM_SHELL_INFO", "" },
    { "PIO_UNLOCK_NMI", ""} 	-- Allow use of PF0 and PD7 as GPIO pins
  },
  build = {
    target = "lualong"			-- Integer only to conserve memory
  }
}

