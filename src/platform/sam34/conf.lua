-- Configuration file for SAM3/4 microcontrollers

addi( sf( 'src/platform/%s/config', platform ) )
addi( sf( 'src/platform/%s/ASF', platform ) )
local cpu = comp.cpu:upper()
local board = comp.board:upper()


specific_files = "ASF/sam/utils/cmsis/sam3x/source/templates/gcc/startup_sam3x.c startup_sam3x.c platform.c platform_int.c"
local fwlib_files = utils.get_files( "src/platform/" .. platform .. "/drivers", ".*%.c$" )

-- Fixme: Needs a lot more directories (or some wildcards) to get all the fwlib files

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

