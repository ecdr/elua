-- SAM3x8e build configuration

return {
  cpu = 'sam3x8e',
  components = {
--    sercon = { uart = "cdc", speed = 115200 },
    sercon = { uart = 0, speed = 115200 },
    wofs = false,		-- See cpu file
    romfs = true,
    shell = true,
    term = { lines = 25, cols = 80 },
    linenoise = { shell_lines = 10, lua_lines = 50 },
    cints = false,
    luaints = false,
    can = true,
    rpc = { uart = 0, speed = 115200 },
    adc = { buf_size = 2 },
    xmodem = true,
    sam34_rand = true,
    sam34_rtc = true,
  },
  config = {
      vtmr = { num = 4, freq = 6 },   -- Frequency should be same as systickhz
  },
  modules = {
    generic = { 'all', '-net', '-mmc' },
    platform = {'all', '-mmc'},
  },
  macros = { -- { "SPI0_NPCS_PIN1", "" }, -- Select which pin to use for SPI0 NPCS (define for PA29)
--		 { "", ""}
  },
  build = {
  }
}

