-- SAM3x8e build configuration

return {
  cpu = 'sam3x8e',
  components = {
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
  },
  config = {
      vtmr = { num = 4, freq = 4 },
  },
  modules = {
    generic = { 'all', '-i2c', '-net', '-mmc' },
    platform = {'all', '-mmc'},
  },
--  macros = { { "", "" },
--		 { "", ""}
--  },
  build = {
  }
}

