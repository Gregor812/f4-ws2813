project(Peripherals CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CXX_SOURCES
    "${PROJECT_SOURCE_DIR}/Random/Random.cpp"
)
set(SOURCES ${CXX_SOURCES})

add_library(Peripherals ${SOURCES})

target_include_directories(Peripherals PUBLIC
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/Gpio"
    "${PROJECT_SOURCE_DIR}/Random"
    "${PROJECT_SOURCE_DIR}/../CMSIS/Include"
    "${PROJECT_SOURCE_DIR}/../CMSIS/Device/Include"
)

target_link_libraries(Peripherals PUBLIC
   F4-CMSIS
)
