-- SAM3x8e build configuration

return {
  cpu = 'sam3x8e',
  components = {
--    cdc = true,     -- enable USB CDC    
--      cdc = { buf_size = 128 }, -- buf doesn't work yet
--      cdc = { vid = 0x03EB, pid = 0x2404 }          -- Idea, not implemented
--    sercon = { uart = "cdc", speed = 115200 },  -- set serial console uart to "cdc" (use native USB port for console)
    sercon = { uart = 0, speed = 115200 },
    wofs = false,		-- See cpu file
    romfs = true,
    shell = true,
    term = { lines = 25, cols = 80 },
    linenoise = { shell_lines = 10, lua_lines = 50 },
--    cints = true,
--    luaints = true,
    cints = false,
    luaints = false,
    can = true,
    rpc = { uart = 0, speed = 115200 },
    adc = { buf_size = 2 },
    xmodem = true,
    sam34_rand = true,
    sam34_rtc = true,
--    sam34_rtc = {crystal = yes, hour_mode = PLATFORM_CLOCK_24HR } -- attributes don't seem to work yet
  },
  config = {
      vtmr = { num = 4, freq = 10 },   -- Frequency should be same as systickhz
  },
  modules = {
    generic = { 'all', '-net', '-mmc' },
    platform = {'all', '-mmc'},
  },
  macros = { -- { "SPI0_NPCS_PIN1", "" }, -- Select which pin to use for SPI0 NPCS (define for PA29)
	 { "SHELL_SHOW_INFO", ""},
--   { "USB_CDC_STDIO", ""}       -- Use CDC_STDIO (rather than USB_CDC), does not work
  },
  build = {
  }
}

