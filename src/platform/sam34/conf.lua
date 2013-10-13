-- Configuration file for SAM3/4 microcontrollers

addi( sf( 'src/platform/%s/config', platform ) )
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
addi( sf( 'src/platform/%s/ASF/common/services/usb' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/usb/class/cdc' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/usb/class/cdc/device' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/services/usb/udc' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/utils' , platform ) )
addi( sf( 'src/platform/%s/ASF/common/utils/stdio/stdio_serial', platform ) )
addi( sf( 'src/platform/%s/ASF/sam/boards' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/boards/arduino_due_x' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/utils' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/utils/cmsis/sam3x/include' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/utils/cmsis/sam3x/source/templates' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/utils/header_files' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/utils/preprocessor' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/pio' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/pmc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/adc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/can' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/dacc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/pwm' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/spi' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/tc' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/trng' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/twi' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/uart' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/uotghs' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/chipid' , platform ) )
addi( sf( 'src/platform/%s/ASF/sam/drivers/usart' , platform ) )
addi( sf( 'src/platform/%s/ASF/thirdparty/CMSIS/Include' , platform ) )
addi( sf( 'src/platform/%s/ASF/thirdparty/CMSIS/Lib/GCC' , platform ) )
local cpu = comp.cpu:upper()
local board = comp.board:upper()



specific_files = "ASF/sam/utils/cmsis/sam3x/source/templates/gcc/startup_sam3x.c platform.c platform_int.c"

-- Dig through ASF to find source files

local fwlib_files = utils.get_files( "src/platform/" .. platform .. "/ASF", ".*%.c$", false )

ldscript = "ASF/sam/utils/liker_scripts/sam3x/sam3x8/gcc/lm3s.ld"

-- Prepend with path
specific_files = fwlib_files .. " " .. utils.prepend_path( specific_files, "src/platform/" .. platform )
specific_files = specific_files .. " src/platform/cortex_utils.s src/platform/arm_cortex_interrupts.c"
ldscript = sf( "src/platform/%s/%s", platform, ldscript )

addm{ "FOR" .. comp.cpu:upper(), 'gcc', 'CORTEX_M3' }

-- Standard GCC flags
addcf{ '-ffunction-sections', '-fdata-sections', '-fno-strict-aliasing', '-Wall' }
--addcf{ '-Wstrict-prototypes', '-Wmissing-prototypes' }			-- From ASF makefile
--addcf{ '-Werror-implicit-function-declaration', '-Wpointer-arith' }	-- From ASF makefile
--addcf{ '-std=gnu99'}									-- From ASF makefile

-- From ASF makefile - a bunch more warnings
-- -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
-- -Wmain -Wparentheses
-- -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
-- -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
-- -Wshadow -Wbad-function-cast -Wwrite-strings
-- -Wsign-compare -Waggregate-return
-- -Wmissing-declarations
-- -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
-- -Wpacked -Wredundant-decls -Wnested-externs -Winline -Wlong-long
-- -Wunreachable-code
-- -Wcast-align
-- --param max-inline-insns-single=500

addlf{ '-nostartfiles', '-nostdlib', '-T', ldscript, '-Wl,--gc-sections', '-Wl,--allow-multiple-definition' }
addaf{ '-x', 'assembler-with-cpp', '-Wall' }
addlib{ 'c','gcc','m' }

-- FIXME: From ASF makefile - need to define part
local target_flags =  {'-mcpu=cortex-m3','-mthumb', '-D=__SAM3X8E__' }

-- Configure general flags for target
addcf{ target_flags, '-mlittle-endian' }
addlf{ target_flags, '-Wl,-e,ResetISR', '-Wl,-static' }
addaf( target_flags )

-- Toolset data
tools.sam34 = {}

-- Array of file names that will be checked against the 'prog' target; their absence will force a rebuild
tools.sam34.prog_flist = { output .. ".bin" }

-- We use 'gcc' as the assembler
toolset.asm = toolset.compile

