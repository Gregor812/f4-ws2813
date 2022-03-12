project(CMSIS C ASM)

set(C_SOURCES
    "${PROJECT_SOURCE_DIR}/Device/system_stm32f4xx.c"
)
set(ASM_SOURCES
    "${PROJECT_SOURCE_DIR}/Device/startup_stm32f429xx.s"
)
set(SOURCES ${C_SOURCES} ${ASM_SOURCES})

add_library(F4-CMSIS ${SOURCES})

target_include_directories(F4-CMSIS PUBLIC
    "${PROJECT_SOURCE_DIR}/Include"
    "${PROJECT_SOURCE_DIR}/Device/Include"
)
