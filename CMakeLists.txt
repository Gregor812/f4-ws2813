cmake_minimum_required(VERSION 3.10)

project(F429_Display VERSION 1.0
    LANGUAGES C ASM CXX)

message(STATUS "Building project\n\t${PROJECT_NAME}")
message(STATUS "Source directory\n\t${PROJECT_SOURCE_DIR}")
message(STATUS "Build directory\n\t${PROJECT_BINARY_DIR}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

message(STATUS "Using toolchain file\n\t${CMAKE_TOOLCHAIN_FILE}")

add_definitions(-DSTM32F429xx)

add_subdirectory(CMSIS)
add_subdirectory(Core)
add_subdirectory(Peripherals)
add_subdirectory(Application)

set(TARGET_NAME firmware)
set(EXE_NAME ${TARGET_NAME}.elf)

add_executable(${EXE_NAME})

target_include_directories(${EXE_NAME} PUBLIC
    "${PROJECT_SOURCE_DIR}/Application"
    "${PROJECT_SOURCE_DIR}"
)

target_sources(
    ${EXE_NAME}
    PRIVATE
    ${PROJECT_SOURCE_DIR}/main.cpp
)

message(STATUS "Project config file\n\t${PROJECT_SOURCE_DIR}/config.h")

configure_file(config.h.in ${PROJECT_SOURCE_DIR}/config.h)

target_link_libraries(${EXE_NAME} PUBLIC
   Application
)

set(LINKER_MAP_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)

message(STATUS "Linker map file\n\t${LINKER_MAP_FILE}")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${LINKER_MAP_FILE}")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${TARGET_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${TARGET_NAME}.bin)

message(STATUS "Build artifacts\n\t${PROJECT_BINARY_DIR}/${EXE_NAME}\n\t${HEX_FILE}\n\t${BIN_FILE}")

add_custom_command(TARGET ${EXE_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${EXE_NAME}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${EXE_NAME}> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}")
