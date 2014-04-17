-- Configuration file for the TM4C microcontrollers

addi( sf( 'src/platform/%s/inc', platform ) )
addi( sf( 'src/platform/%s/driverlib', platform ) )
local cpu = comp.cpu:upper()
local board = comp.board:upper()


-- Only include USB headers/paths for boards which support it
-- TODO: TM4F123 has USB, but may not have enough memory to use it in eLua
-- TODO: USB should have a single flag to control it, rather than constantly testing for a list of boards.
-- TODO: (or board == 'EK-LM4F120' or board == 'EK-TM4C123' )
if cpu == 'LM3S9B92' or cpu == 'LM3S9D92' then
  addi( sf( 'src/platform/%s/usblib', platform ) )
  addi( sf( 'src/platform/%s/usblib/device', platform ) )
end

specific_files = "startup_gcc.c platform.c platform_int.c lm3s_pio.c"
local fwlib_files = utils.get_files( "src/platform/" .. platform .. "/driverlib", ".*%.c$" )


-- ToDo: USB test again
if cpu == 'LM3S9B92' or cpu == 'LM3S9D92' then
   fwlib_files = fwlib_files .. " " .. utils.get_files( "src/platform/" .. platform .. "/usblib", ".*%.c$" ) 
   fwlib_files = fwlib_files .. " " .. utils.get_files( "src/platform/" .. platform .. "/usblib/device", ".*%.c$" )
   specific_files = specific_files .. "  usb_serial_structs.c"
end

if board == 'EK-TM4C1294' then
   ldscript = "tm4c129.ld"
else
   ldscript = "tm4c.ld"
end

-- Prepend with path
specific_files = fwlib_files .. " " .. utils.prepend_path( specific_files, "src/platform/" .. platform )
specific_files = specific_files .. " src/platform/cortex_utils.s src/platform/arm_cortex_interrupts.c"
ldscript = sf( "src/platform/%s/%s", platform, ldscript )

--if board == 'EK-LM4F120' or board == "EK-TM4C123" then
addm{ "FOR" .. comp.cpu:upper(), 'gcc', 'CORTEX_M4' }
--end

-- FIXME: Need to figure how to get this to adapt appropriately (or why isn't getting into cpu_tm4c123.h )
--if cpu == 'TM4C123G' then
--  addm{ "PART_TM4C123GH6PM" }
--elseif cpu == 'TM4C1233' then
--  addm{ "PART_TM4C1233H6PM" }
--elseif cpu == 'TM4C1294' then
--  addm{ "PART_TM4C1294NCPDT" }
--else
--  -- unknown cpu, todo: give a warning
--end 

-- Standard GCC flags
addcf{ '-ffunction-sections', '-fdata-sections', '-fno-strict-aliasing', '-Wall' }
addlf{ '-nostartfiles', '-nostdlib', '-T', ldscript, '-Wl,--gc-sections', '-Wl,--allow-multiple-definition' }
addaf{ '-x', 'assembler-with-cpp', '-Wall' }
addlib{ 'c','gcc','m' }

local target_flags

-- Todo: turn on FPU
--if board == 'EK-LM4F120' or board == "EK-TM4C123" then
target_flags =  {'-mcpu=cortex-m4','-mthumb' }
--end

-- Configure general flags for target
addcf{ target_flags, '-mlittle-endian' }
addlf{ target_flags, '-Wl,-e,ResetISR', '-Wl,-static' }
addaf( target_flags )

-- Toolset data
tools.tm4c = {}

-- Array of file names that will be checked against the 'prog' target; their absence will force a rebuild
tools.tm4c.prog_flist = { output .. ".bin" }

-- We use 'gcc' as the assembler
toolset.asm = toolset.compile

