project(Core CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CXX_SOURCES
    "${PROJECT_SOURCE_DIR}/Systick/Systick.cpp"
)
set(SOURCES ${CXX_SOURCES})

add_library(Core ${SOURCES})

target_include_directories(Core PUBLIC
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/Systick"
    "${PROJECT_SOURCE_DIR}/../CMSIS/Include"
    "${PROJECT_SOURCE_DIR}/../CMSIS/Device/Include"
)

target_link_libraries(Core PUBLIC
   F4-CMSIS
)
