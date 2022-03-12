include(CMakeForceCompiler)

set(triplet arm-none-eabi-)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(arch cortex-m4)

set(CMAKE_C_COMPILER ${triplet}gcc)
set(CMAKE_CXX_COMPILER ${triplet}g++)

set(CMAKE_ASM_FLAGS "-mcpu=${arch} -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(COMMON_C_CXX_FLAGS
    "-mcpu=${arch} -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-common -fsigned-char -fmessage-length=0 -Wall -Wextra -Wshadow -Og -ffunction-sections -fdata-sections -g3"
)
set(CMAKE_C_FLAGS "${COMMON_C_CXX_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${COMMON_C_CXX_FLAGS} -std=c++17")

set(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/CMSIS/Device/STM32F429ZITx_FLASH.ld)
set(CMAKE_CXX_FLAGS_INIT ${CMAKE_C_FLAGS})
set(CMAKE_C_FLAGS_INIT ${CMAKE_CXX_FLAGS})
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -specs=nosys.specs -specs=nano.specs -T${LINKER_SCRIPT}")
