project(Application CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CXX_SOURCES
    "${PROJECT_SOURCE_DIR}/App.cpp"
)
set(SOURCES ${CXX_SOURCES})

add_library(Application ${SOURCES})

target_include_directories(Application PUBLIC
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/Ili9341"
    "${PROJECT_SOURCE_DIR}/../CMSIS/Include"
    "${PROJECT_SOURCE_DIR}/../CMSIS/Device/Include"
    "${PROJECT_SOURCE_DIR}/../Core"
    "${PROJECT_SOURCE_DIR}/../Peripherals"
)

target_link_libraries(Application PUBLIC
   F4-CMSIS
   Core
   Peripherals
)
