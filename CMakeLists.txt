cmake_minimum_required(VERSION 3.16)
project(HaldexController VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Static library — core + logging, shared by the app and the tests
add_library(haldex_core STATIC
    src/core/haldexController.cpp
    src/logging/telemetry.cpp
    src/logging/diagnostics.cpp
)
target_include_directories(haldex_core PUBLIC src)
target_compile_options(haldex_core PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W3 /O2>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -O2>
)

# Main executable
add_executable(haldex src/main.cpp)
target_link_libraries(haldex PRIVATE haldex_core)

# Tests — require git for FetchContent of GTest
find_program(GIT_EXECUTABLE git)
if(GIT_EXECUTABLE)
    enable_testing()
    add_subdirectory(tests)
else()
    message(WARNING "git not found — test target (haldex_tests) is disabled. Install Git to enable tests.")
endif()
