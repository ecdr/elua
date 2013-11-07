-- Configuration file for SAM3/4 microcontrollers

addi( sf( 'src/platform/%s/config', platform ) )
-- FIXME: Should automatically generate list of where the headers are - the list works, but not robust when add new drivers

addi( sf( 'src/platform/%s/ASF/common/boards' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/clock' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/gpio' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/ioport' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/serial/sam_uart' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/serial' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/sleepmgr' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/spi/sam_spi' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/spi' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/twi' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/delay' , platform ) )   -- busy wait delay
addi( sf( 'src/platform/%s/ASF/common/services/fifo' , platform ) )    -- general small fifos
addi( sf( 'src/platform/%s/ASF/common/utils' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/utils/stdio/stdio_serial', platform ) )
addi( sf( 'src/platform/%s/ASF/sam/boards' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/boards/arduino_due_x' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/adc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/can' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/chipid' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/dacc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/efc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/pio' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/pmc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/pwm' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/rtc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/spi' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/tc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/trng' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/twi' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/uart' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/usart' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/services/flash_efc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/utils' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/utils/cmsis/sam3x/include' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/utils/cmsis/sam3x/source/templates' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/utils/header_files' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/utils/preprocessor' , platform ) )
addi( sf( 'src/platform/%s/ASF/thirdparty/CMSIS/Include' , platform ) )
addi( sf( 'src/platform/%s/ASF/thirdparty/CMSIS/Lib/GCC' , platform ) )

-- TODO: If not building USB then don't include these directories
addi( sf( 'src/platform/%s/ASF/common/services/usb' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/usb/class/cdc' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/usb/class/cdc/device' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/usb/udc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/uotghs' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/utils/stdio/stdio_usb' , platform ) )

local cpu = comp.cpu:upper()
local board = comp.board:upper()

-- Working modules
specific_files = "module_rand.c module_rtc.c platform_rtc.c platform.c platform_int.c"
-- Under construction
-- specific_files = "sam_pio.c module_dac.c" .. specific_files


-- Dig through ASF to find source files

local fwlib_files = utils.get_files( "src/platform/" .. platform .. "/ASF", ".*%.c$", false )

ldscript = "ASF/sam/utils/linker_scripts/sam3x/sam3x8/gcc/flash.ld"

-- Prepend with path
specific_files = fwlib_files .. " " .. utils.prepend_path( specific_files, "src/platform/" .. platform )
specific_files = specific_files .. " src/platform/cortex_utils.s src/platform/arm_cortex_interrupts.c"
ldscript = sf( "src/platform/%s/%s", platform, ldscript )

addm{ "FOR" .. comp.cpu:upper(), 'gcc', 'CORTEX_M3' }

-- Standard GCC flags
addcf{ '-ffunction-sections', '-fdata-sections', '-fno-strict-aliasing', '-Wall' }

addcf{ '-std=gnu99'}									-- From ASF makefile, library uses C99 features

addcf{ '-g' }                         -- Debugging flags (-g3 - include macros)
-- addcf{ '-O' }        -- ?? for final versions?

addcf{ '-Wstrict-prototypes', '-Wmissing-prototypes' }			-- From ASF makefile
--addcf{ '-Werror-implicit-function-declaration', '-Wpointer-arith' }	-- From ASF makefile

-- From ASF makefile - a bunch more warnings
--addcf{ '-Wformat=2', '-Wmissing-format-attribute', '-Wno-format-nonliteral'}
--addcf{ '-Wswitch', '-Wunused', '-Wundef',  '-Wwrite-strings' }
--addcf{ '-Wmissing-declarations', '-Wno-deprecated-declarations', '-Wredundant-decls' }
--addcf{ '-Wpacked', '-Wnested-externs' } 
--addcf{ '-Wunreachable-code' }

-- addcf{ '-Wshadow' }
-- addcf{ '-Winline', '--param max-inline-insns-single=500' }

-- Don't see a lot of point in checking for these - used a bunch and not obvious alternative
-- addcf{ '-Wlong-long' }   -- This one is used several places
-- addcf{ '-Wfloat-equal' } -- There are cases where this is reasonable (how do you tell compiler this one ok?)
-- addcf{ '-Wsign-compare' } -- Lots of these - check later
-- addcf{ '-Wcast-align' }  -- several of these - haven't looked at

-- addcf{ '-Wbad-function-cast' } -- Not sure what bad about these (they seem to work, no obvious other way to do)
-- addcf{ '-Waggregate-return' } -- I See no point in flagging this

-- End of extra warnings

-- addcf{ '-Wextra'} -- another batch of warning, from GCC manual


addlf{ '-nostartfiles', '-nostdlib', '-T', ldscript, '-Wl,--gc-sections', '-Wl,--allow-multiple-definition' }
addaf{ '-x', 'assembler-with-cpp', '-Wall' }
addlib{ 'c','gcc','m' }

-- FIXME: From ASF makefile - need to define part
-- FIXME: From AtmelStudio - board=ARDUINO_DUE_X - should be contingent on eLua board spec
--   Would be better if specify in board file (but not sure how to set a compiler flag in there)
local target_flags =  {'-mcpu=cortex-m3','-mthumb','-D=__SAM3X8E__','-DBOARD=ARDUINO_DUE_X' }

-- Configure general flags for target
addcf{ target_flags, '-mlittle-endian' }
addlf{ target_flags, '-Wl,--entry=Reset_Handler', '-Wl,-static' }
addaf( target_flags )

-- Toolset data
tools.sam34 = {}

-- Array of file names that will be checked against the 'prog' target; their absence will force a rebuild
tools.sam34.prog_flist = { output .. ".bin" }

-- We use 'gcc' as the assembler
toolset.asm = toolset.compile

