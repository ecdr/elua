-- EK-TM4C1294XL build configuration
-- CPU tm4c1294ncpdt
-- TODO: Is there an equivalent lm4f CPU?

-- has network
-- has 256kb RAM, so can use float

return {
  cpu = 'tm4c1294',
  components = {
    sercon = { uart = 0, speed = 115200 },  -- CAN uses same pins as uart 0, use uart 4 if using CAN
--    sercon = { uart = "cdc", speed = 115200 },
--    cdc = { buf_size = 128 },
    wofs = true,
    romfs = true,
    shell = true,
    term = { lines = 25, cols = 80 },
    linenoise = { shell_lines = 10, lua_lines = 50 },
    cints = true,
    luaints = true,
    lm3s_pio = true,
    rpc = { uart = 0, speed = 115200 },
    adc = { buf_size = 2 },
--    tcpip = { ip = "192.168.1.100", dns = "192.168.1.1", gw = "192.168.1.1", netmask = "255.255.255.0" },
--    dns = true,
--    dhcp = true,
    xmodem = true,
--    mmcfs = { spi = 2, cs_port = 7, cs_pin = 2 }, -- SSI 2 on Booster pack 1 (TODO: what chip select makes sense?)
--    mmcfs = { spi = 3, cs_port = 15, cs_pin = 5 }, -- SSI 3 on Booster pack 2 (TODO: what chip select makes sense?)
  },
  config = {
    vtmr = { num = 4, freq = 4 },
    clocks = { cpu = 120000000 },
--    tm4c_target = { revision = A1 },
  },
  modules = {
    generic = { 'all', '-i2c', '-net' },
    platform = { 'all', '-net' }
  },
  macros = { 
     { "PART_TM4C1294NCPDT", "" },	-- Needed for PIN_MAP.
     { "TARGET_IS_TM4C129_RA0", ""}, -- RA1 is other choice so far
--     { "UART_ALT_CLOCK", ""}, -- Use alternate clock for UART
     { "DEBUG", ""}, -- Extra checks
  },
}
