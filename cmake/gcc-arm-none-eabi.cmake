set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Accept PATH, ARM_TOOLCHAIN_ROOT, or a standard STM32CubeCLT installation.
file(GLOB _stm32_clt_bins
    "/opt/ST/STM32CubeCLT_*/GNU-tools-for-STM32/bin"
    "C:/ST/STM32CubeCLT_*/GNU-tools-for-STM32/bin")
list(SORT _stm32_clt_bins COMPARE NATURAL ORDER DESCENDING)
find_program(ARM_GCC arm-none-eabi-gcc
    HINTS "$ENV{ARM_TOOLCHAIN_ROOT}/bin" ${_stm32_clt_bins}
    REQUIRED)
get_filename_component(ARM_TOOLCHAIN_BIN "${ARM_GCC}" DIRECTORY)
set(TOOLCHAIN_PREFIX "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-")
if(WIN32)
    set(ARM_TOOL_SUFFIX ".exe")
else()
    set(ARM_TOOL_SUFFIX "")
endif()
set(CMAKE_C_COMPILER "${ARM_GCC}")
set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++${ARM_TOOL_SUFFIX}")
set(CMAKE_OBJCOPY "${TOOLCHAIN_PREFIX}objcopy${ARM_TOOL_SUFFIX}")
set(CMAKE_SIZE "${TOOLCHAIN_PREFIX}size${ARM_TOOL_SUFFIX}")

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS_INIT "${TARGET_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")
set(CMAKE_ASM_FLAGS_INIT "${TARGET_FLAGS} -x assembler-with-cpp -MMD -MP")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS_INIT "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT} -T \"${CMAKE_SOURCE_DIR}/STM32F407xx_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
